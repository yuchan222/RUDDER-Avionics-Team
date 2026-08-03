"""
RUDDER 2026 — GCS version6 (version6 펌웨어 전용, 기존 gcs_v5.py는 보존)

gcs_v5.py에서 검증된 구조를 유지하면서 2026-07-30 GPT 기술검토(2차) 대응으로
다음을 추가/변경 (자세한 내용은 2026-Rudder/version6/GPT_REVIEW_RESPONSE_0730.md 참고):

  1. 재연결 시 last_pkt 초기화 — 이전 로켓/이전 비행의 마지막 상태가 안 남게
  2. 재연결 실패 시 죽은 link 객체가 "연결됨"으로 남지 않게 순서 정리
  3. eject_state 표시 문구 정정 ("사출 완료"→"사출 명령 실행됨, 물리적 확인 아님")
  4. 그래프 발사 기준선 1000mg→2000mg (실제 임계값과 일치)
  5. cmd_rx_count 도움말 문구 정정 (거부된 명령도 포함됨을 명시)

2026-08-02 테스트 발사 피드백 반영 (패킷 47B→48B, version2~5와 바이트 호환 깨짐):
  6. eject_state를 사출 사유코드로 확장 (0=없음/1=주사출/2=보조사출/3=비상명령) — 표시에 반영
  7. system_status 여유비트로 발사 감지 경로(가속도/고도/수동) 기록 — 타임라인에 표시
  8. last_cmd 필드 추가 (최근 수신 명령 바이트, 사후분석용 — SD 로그에만 기록)

실행:  python -m streamlit run gcs_v6.py
"""

import streamlit as st
import time
import struct
import threading
import queue
import csv
import pathlib
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List

# ─── 상수 (version6 Config.h / Packet.h와 동기화) ────────────────────────────
FLIGHT_MODES = {0: "대기", 1: "준비", 2: "비행", 3: "낙하", 4: "착륙"}
MODE_COLORS  = {0: "#555", 1: "#e67e22", 2: "#2980b9", 3: "#27ae60", 4: "#8e44ad"}

STATUS_BMP, STATUS_IMU, STATUS_SD, STATUS_INA = 0x01, 0x02, 0x04, 0x08
STATUS_BASELINE_BAD = 0x10
STATUS_LOG_CLOSED = 0x20
STATUS_LAUNCH_ACCEL = 0x40   # 발사 감지 경로 — 가속도
STATUS_LAUNCH_ALT   = 0x80   # 발사 감지 경로 — 고도 (둘 다 0이면 수동 강제진입)

EJECT_REASON = {0: None, 1: "주 사출 (정점통과)", 2: "보조 사출 (10초 타이머)", 3: "지상국 비상 명령"}

ALT_INVALID = -2147483648   # Config.h의 ALT_INVALID(INT32_MIN)와 동일 — BMP 실패 표시

CMD_SET_STANDBY  = 0x0B
CMD_SET_ARMED    = 0x16
CMD_FORCE_FLIGHT = 0x2D
CMD_FORCE_EJECT  = 0x42
CMD_FORCE_LAND   = 0x51

BAUD          = 57600
RETRY_GAP_S   = 0.3     # 비상사출 자동 재전송 간격
MAX_RETRY_S   = 30.0    # 이 시간 넘도록 확인 안 되면 자동 중단 (무한 재전송 방지)
FAST_TICK_S   = 0.3     # 수치/재전송 엔진 갱신 주기
CHART_TICK_S  = 1.0     # 그래프 갱신 주기 (깜빡임 완화)
HISTORY_MAX   = 700

# ─── 명령 패킷 (0x3C 0x3C | CMD×4 | CRC8) ───────────────────────────────────
def _crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def cmd_packet(cmd: int) -> bytes:
    body = bytes([0x3C, 0x3C, cmd, cmd, cmd, cmd])
    return body + bytes([_crc8(body)])


# ─── 텔레메트리 패킷 (48B, 2026-08-02: last_cmd 1바이트 추가로 47B→48B) ─────
PKT_FMT    = '<HBHBHIhhhhhhihihhBBBHBHH'
PKT_SIZE   = struct.calcsize(PKT_FMT)       # 48
CRC_OFFSET = 44


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Pkt:
    seq: int = 0
    ms: int = 0
    acc: List[int] = field(default_factory=lambda: [0, 0, 0])
    gyro: List[int] = field(default_factory=lambda: [0, 0, 0])
    pressure: int = 0
    bmp_temp: int = 0
    altitude_cm: int = 0
    voltage_mv: int = 0
    current_ma: int = 0
    flight_mode: int = 0
    eject_state: int = 0
    system_status: int = 0
    cmd_rx_count: int = 0
    last_cmd: int = 0
    rx_time: float = 0.0


def parse_raw(raw: bytes):
    """(Pkt|None, crc_ok) — STX/ETX 불일치는 None, CRC 불일치는 (None, False)"""
    try:
        v = struct.unpack_from(PKT_FMT, raw)
        if v[0] != 0xAA55 or v[-1] != 0x55AA:
            return None, True
        if _crc16(raw[:CRC_OFFSET]) != v[22]:
            return None, False          # 도착했지만 손상됨
        return Pkt(
            seq=v[2], ms=v[5],
            acc=list(v[6:9]), gyro=list(v[9:12]),
            pressure=v[12], bmp_temp=v[13], altitude_cm=v[14],
            voltage_mv=v[15], current_ma=v[16],
            flight_mode=v[17], eject_state=v[18], system_status=v[19],
            cmd_rx_count=v[20], last_cmd=v[21], rx_time=time.time(),
        ), True
    except Exception:
        return None, True


# ─── SiK 시리얼 링크 (수신 스레드 + 명령 송신) ───────────────────────────────
class SikLink:
    def __init__(self, port: str):
        self._port = port
        self._ser = None
        self._q: queue.Queue = queue.Queue(maxsize=500)
        self._running = False
        self._thread = None
        self.crc_bad = 0

    def start(self):
        import serial
        self._ser = serial.Serial(self._port, BAUD, timeout=0.05)
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._ser:
            self._ser.close()

    def send_cmd(self, cmd: int):
        if self._ser and self._ser.is_open:
            self._ser.write(cmd_packet(cmd))

    def read_packet(self) -> Optional[Pkt]:
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def _rx_loop(self):
        buf = b''
        STX = struct.pack('<H', 0xAA55)
        while self._running:
            try:
                buf += self._ser.read(256)
                while len(buf) >= PKT_SIZE:
                    idx = buf.find(STX)
                    if idx < 0:
                        buf = buf[-4:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < PKT_SIZE:
                        break
                    raw, buf = buf[:PKT_SIZE], buf[PKT_SIZE:]
                    pkt, crc_ok = parse_raw(raw)
                    if pkt:
                        try:
                            self._q.put_nowait(pkt)
                        except queue.Full:
                            pass
                    elif not crc_ok:
                        self.crc_bad += 1
            except Exception:
                time.sleep(0.05)


def list_ports() -> List[tuple]:
    try:
        import serial.tools.list_ports
        return [(f"{p.device} — {p.description[:35]}", p.device)
                for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)]
    except Exception:
        return []


# ─── CSV 로그 ────────────────────────────────────────────────────────────────
LOG_DIR = pathlib.Path(__file__).parent / 'logs'
LOG_FIELDS = ['rx_time', 'seq', 'ms', 'flight_mode', 'eject_state', 'altitude_cm',
              'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
              'pressure', 'bmp_temp', 'voltage_mv', 'current_ma',
              'system_status', 'cmd_rx_count', 'last_cmd']


def open_log():
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"gcs_v6_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(path, 'w', newline='', encoding='utf-8')
    w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
    w.writeheader()
    return path, f, w


def write_log(pkt: Pkt):
    w = st.session_state.get('log_writer')
    f = st.session_state.get('log_file')
    if not w:
        return
    w.writerow({
        'rx_time': f'{pkt.rx_time:.3f}', 'seq': pkt.seq, 'ms': pkt.ms,
        'flight_mode': pkt.flight_mode, 'eject_state': pkt.eject_state,
        'altitude_cm': pkt.altitude_cm,
        'acc_x': pkt.acc[0], 'acc_y': pkt.acc[1], 'acc_z': pkt.acc[2],
        'gyro_x': pkt.gyro[0], 'gyro_y': pkt.gyro[1], 'gyro_z': pkt.gyro[2],
        'pressure': pkt.pressure, 'bmp_temp': pkt.bmp_temp,
        'voltage_mv': pkt.voltage_mv, 'current_ma': pkt.current_ma,
        'system_status': hex(pkt.system_status), 'cmd_rx_count': pkt.cmd_rx_count,
        'last_cmd': hex(pkt.last_cmd),
    })
    if f:
        f.flush()


# ─── Streamlit ───────────────────────────────────────────────────────────────
st.set_page_config(page_title="RUDDER GCS v6", layout="wide", page_icon="🚀")


def _init():
    if st.session_state.get('ready'):
        return
    st.session_state.ready = True
    st.session_state.link = None
    st.session_state.last_seq = -1
    st.session_state.lost = 0
    st.session_state.total = 0
    st.session_state.rate_ts = []
    st.session_state.last_pkt = None
    st.session_state.t_hist = []
    st.session_state.alt_hist = []
    st.session_state.acc_hist = []
    st.session_state.timeline = {'launch': None, 'launch_reason': None, 'eject': None, 'land': None}
    st.session_state.prev_mode = -1
    # 비상사출 재전송 엔진
    st.session_state.eject_phase = 0          # 0=대기 1=확인대기 2=재전송중
    st.session_state.eject_click_t = 0.0
    st.session_state.retry_sent = 0
    # 강제비행 진입 확인 (되돌릴 수 없는 명령이라 비상사출과 동일하게 확인 단계를 둠)
    st.session_state.ff_confirm  = False
    st.session_state.ff_click_t  = 0.0
    st.session_state.chart_epoch = 0   # 재연결마다 증가 — 그래프 위젯을 완전히 새로 그리게 함
    path, f, w = open_log()
    st.session_state.log_path = path
    st.session_state.log_file = f
    st.session_state.log_writer = w


def _reset_flight_data():
    st.session_state.last_pkt = None   # 이전 로켓/이전 비행의 마지막 상태가 새 연결에 남지 않게
    st.session_state.last_seq = -1
    st.session_state.lost = 0
    st.session_state.total = 0
    st.session_state.rate_ts = []
    st.session_state.t_hist = []
    st.session_state.alt_hist = []
    st.session_state.acc_hist = []
    st.session_state.timeline = {'launch': None, 'launch_reason': None, 'eject': None, 'land': None}
    st.session_state.prev_mode = -1
    # 비상사출/강제비행 확인 상태도 재연결 시 반드시 초기화
    # (이전 세션의 재전송이 남아있으면, 새로 연결되는 즉시 밀린 명령이 나갈 수 있음)
    st.session_state.eject_phase = 0
    st.session_state.retry_sent = 0
    st.session_state.ff_confirm = False
    st.session_state.chart_epoch = st.session_state.get('chart_epoch', 0) + 1


def _sidebar():
    with st.sidebar:
        st.markdown("### 🔌 연결")
        ports = list_ports()
        disp = [d for d, _ in ports] or ["(포트 없음)"]
        devs = [dev for _, dev in ports]
        sel = st.selectbox("지상측 SiK 포트", disp)
        if st.button("연결", type="primary", use_container_width=True) and devs:
            old = st.session_state.get('link')
            if old:
                old.stop()
            st.session_state.link = None   # 새 연결 성공 전까지는 죽은 링크가 남지 않게
            new_link = SikLink(devs[disp.index(sel)])
            try:
                new_link.start()
                st.session_state.link = new_link
                _reset_flight_data()
                st.toast(f"연결됨: {devs[disp.index(sel)]}", icon="✅")
            except Exception as e:
                new_link.stop()
                st.error(f"연결 실패: {e}")
        st.caption("상태: " + ("🟢 연결됨" if st.session_state.get('link') else "⚪ 미연결"))
        st.divider()
        st.caption(f"로그: `{pathlib.Path(st.session_state.log_path).name}`")
        st.caption(f"패킷 크기: {PKT_SIZE} bytes")


# ─── 데이터 드레인 + 수치 + 명령 패널 (0.3초 부분 갱신) ─────────────────────
@st.fragment(run_every=FAST_TICK_S)
def _panel():
    link: Optional[SikLink] = st.session_state.get('link')
    now = time.time()

    # ── 수신 처리 ──────────────────────────────────────────────
    if link:
        while True:
            pkt = link.read_packet()
            if pkt is None:
                break
            # 재부팅 감지 (장치시간 역행 → 새 비행)
            if st.session_state.t_hist and pkt.ms / 1000.0 < st.session_state.t_hist[-1] - 5.0:
                _reset_flight_data()
                st.toast("보드 재시작 감지 — 데이터 초기화", icon="🔄")
            if st.session_state.last_seq >= 0:
                gap = (pkt.seq - ((st.session_state.last_seq + 1) & 0xFFFF)) & 0xFFFF
                if 0 < gap < 0x8000:
                    st.session_state.lost += gap
            st.session_state.last_seq = pkt.seq
            st.session_state.total += 1
            st.session_state.rate_ts.append(now)
            st.session_state.last_pkt = pkt

            st.session_state.t_hist.append(pkt.ms / 1000.0)
            st.session_state.alt_hist.append(
                pkt.altitude_cm / 100.0 if pkt.altitude_cm != ALT_INVALID else None)
            st.session_state.acc_hist.append(pkt.acc[0])   # X축 = 발사 감지축 (2026-07-30 실측 확인)
            if len(st.session_state.t_hist) > HISTORY_MAX:
                st.session_state.t_hist.pop(0)
                st.session_state.alt_hist.pop(0)
                st.session_state.acc_hist.pop(0)

            # 타임라인 (모드가 실제 사건과 1:1이라 그대로 신뢰 가능)
            tl, pm = st.session_state.timeline, st.session_state.prev_mode
            if pkt.flight_mode == 2 and pm in (0, 1, -1) and tl['launch'] is None:
                tl['launch'] = pkt.rx_time
                if pkt.system_status & STATUS_LAUNCH_ACCEL:
                    tl['launch_reason'] = "가속도 경로"
                elif pkt.system_status & STATUS_LAUNCH_ALT:
                    tl['launch_reason'] = "고도 경로"
                else:
                    tl['launch_reason'] = "수동 강제진입"
            if pkt.flight_mode == 3 and pm != 3 and tl['eject'] is None:
                tl['eject'] = pkt.rx_time
            if pkt.flight_mode == 4 and pm != 4 and tl['land'] is None:
                tl['land'] = pkt.rx_time
            st.session_state.prev_mode = pkt.flight_mode

            write_log(pkt)

    st.session_state.rate_ts = [t for t in st.session_state.rate_ts if now - t < 1.0]
    rate = len(st.session_state.rate_ts)
    last: Optional[Pkt] = st.session_state.last_pkt

    # ── 모드 배지 + 센서 상태 ───────────────────────────────────
    mode = last.flight_mode if last else 0
    s = last.system_status if last else 0
    c_badge, c_num = st.columns([2, 5])
    with c_badge:
        st.markdown(
            f'<div style="background:{MODE_COLORS.get(mode, "#555")};border-radius:12px;'
            f'padding:18px 10px;text-align:center;">'
            f'<span style="color:#fff;font-size:44px;font-weight:900;">'
            f'{FLIGHT_MODES.get(mode, "?")}</span></div>',
            unsafe_allow_html=True)
        if last:
            s = last.system_status
            def dot(ok, name):
                return f'<span style="color:{"#2ecc71" if ok else "#e74c3c"};">●</span>{name}'
            st.markdown(
                dot(s & STATUS_BMP, 'BMP') + ' ' + dot(s & STATUS_IMU, 'IMU') + ' ' +
                dot(s & STATUS_SD, 'SD') + ' ' + dot(s & STATUS_INA, 'INA') + ' ' +
                dot(not (s & STATUS_BASELINE_BAD), '기준압'),
                unsafe_allow_html=True)
            servo_on = mode != 0
            st.markdown(
                f'<div style="background:{"#1e7e34" if servo_on else "#3a3a3a"};'
                f'border-radius:10px;padding:10px 8px;text-align:center;margin-top:8px;">'
                f'<span style="color:#fff;font-size:24px;font-weight:900;">'
                f'⚡ 서보 전원 {"ON" if servo_on else "OFF"}</span></div>',
                unsafe_allow_html=True)
            st.caption("MOSFET 신호(명령) 기준 — 실제 전류 실측 아님")
            if last.eject_state:
                reason = EJECT_REASON.get(last.eject_state, f"알 수 없음({last.eject_state})")
                st.error(f"🪂 사출 명령 실행됨 — {reason} (펌웨어 기준, 실제 낙하산 전개 확인 아님)")

    with c_num:
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("수신율", f"{rate} pkt/s")
        m2.metric("손실 누적", st.session_state.lost)
        m3.metric("손상(CRC)", link.crc_bad if link else 0)
        m4.metric("총 수신", st.session_state.total)
        if last:
            n1, n2, n3, n4, n5 = st.columns(5)
            alt = f"{last.altitude_cm / 100:.1f} m" if last.altitude_cm != ALT_INVALID else "BMP 오류"
            n1.metric("고도", alt)
            n2.metric("전압", f"{last.voltage_mv / 1000:.2f} V" if last.voltage_mv != -1 else "—")
            n3.metric("X가속도", f"{last.acc[0]} mg", help="발사 감지축 (2026-07-30 실측 확인)")
            n4.metric("명령 수신", last.cmd_rx_count,
                      help="로켓이 지금까지 수신한 유효 형식(CRC 통과) 명령 프레임 수 — "
                           "모드가 안 맞아 거부된 명령도 포함, '적용됨'의 의미 아님")
            if not (s & STATUS_SD):
                sd_text = "미장착"
            elif s & STATUS_LOG_CLOSED:
                sd_text = "종료"
            elif 1 <= mode <= 3:
                sd_text = "기록중"
            else:
                sd_text = "대기"
            n5.metric("SD로그", sd_text)

    st.divider()

    # ── 명령 패널 ──────────────────────────────────────────────
    st.subheader("🎛️ 모드 제어")
    connected = link is not None
    mode_now = last.flight_mode if last else -1
    b1, b2, b3, b4 = st.columns(4)
    if b1.button("⚪ 준비→대기 전환", use_container_width=True,
                 disabled=not connected or mode_now != 1):
        link.send_cmd(CMD_SET_STANDBY)
    if b2.button("🟠 대기→준비 전환 (기준압 수집)", use_container_width=True,
                 disabled=not connected or mode_now != 0):
        link.send_cmd(CMD_SET_ARMED)
    if b3.button("🔵 준비→비행 강제진입", use_container_width=True,
                 disabled=not connected or mode_now != 1,
                 help="센서 무관 강제 진입 — 발사감지 실패 시 백업. 10초 타이머 즉시 시작됨. 되돌릴 수 없음"):
        st.session_state.ff_confirm = True
        st.session_state.ff_click_t = now
    if b4.button("🟣 낙하→착륙 강제전환 (SD 닫기)", use_container_width=True,
                 disabled=not connected or mode_now != 3):
        link.send_cmd(CMD_FORCE_LAND)

    if st.session_state.ff_confirm:
        remain = 5.0 - (now - st.session_state.ff_click_t)
        if remain <= 0:
            st.session_state.ff_confirm = False
        else:
            st.error(f"⚠️ 비행 강제진입은 되돌릴 수 없습니다 — {remain:.1f}초 안에 확인")
            fc1, fc2 = st.columns(2)
            if fc1.button("🔴 강제진입 확인", use_container_width=True):
                link.send_cmd(CMD_FORCE_FLIGHT)
                st.session_state.ff_confirm = False
            if fc2.button("취소", use_container_width=True, key="ff_cancel"):
                st.session_state.ff_confirm = False

    st.caption("버튼은 현재 모드와 맞을 때만 활성화됩니다. 클릭 후 위 모드 배지가 "
               "바뀌는지로 적용 여부 확인 (모드 배지 = 로켓이 실제 보고하는 상태)")

    # ── 비상 사출 (2단계 확인 + 자동 재전송) ───────────────────
    st.subheader("🚨 비상 사출")
    phase = st.session_state.eject_phase

    if phase == 0:
        if st.button("⚠️ 비상 사출", type="primary", use_container_width=True,
                     disabled=not connected):
            st.session_state.eject_phase = 1
            st.session_state.eject_click_t = now
    elif phase == 1:
        remain = 5.0 - (now - st.session_state.eject_click_t)
        if remain <= 0:
            st.session_state.eject_phase = 0
        else:
            st.error(f"⚠️ {remain:.1f}초 안에 [실행 확인] — 확인 즉시 자동 재전송 시작")
            cc1, cc2 = st.columns(2)
            if cc1.button("🔴 실행 확인", use_container_width=True):
                st.session_state.eject_phase = 2
                st.session_state.retry_sent = 0
                st.session_state.last_retry_t = 0.0
                st.session_state.retry_start_t = now
            if cc2.button("취소", use_container_width=True):
                st.session_state.eject_phase = 0
    else:  # phase 2: 자동 재전송 중
        confirmed = (last.eject_state != 0) if last else False
        timed_out = (now - st.session_state.get('retry_start_t', now)) > MAX_RETRY_S

        if confirmed:
            st.success(f"✅ 로켓이 명령 수신 확인! (전송 {st.session_state.retry_sent}회 만에)")
            if st.button("확인 (닫기)", use_container_width=True):
                st.session_state.eject_phase = 0
        elif timed_out:
            st.error(f"⏱️ {MAX_RETRY_S:.0f}초 동안 로켓 확인이 안 됐습니다 — 자동 중단됨. "
                     f"링크 상태를 확인하고 필요하면 다시 시도하세요.")
            if st.button("확인 (닫기)", use_container_width=True, key="ff_timeout_close"):
                st.session_state.eject_phase = 0
        else:
            if link and now - st.session_state.get('last_retry_t', 0) >= RETRY_GAP_S:
                link.send_cmd(CMD_FORCE_EJECT)
                st.session_state.retry_sent += 1
                st.session_state.last_retry_t = now
            st.warning(f"📡 자동 재전송 중... 전송 {st.session_state.retry_sent}회 · "
                       f"로켓 사출 확인 대기 (eject_state={last.eject_state if last else '—'})")
            if st.button("🛑 재전송 중단", use_container_width=True):
                st.session_state.eject_phase = 0

    st.divider()
    _timeline()


def _timeline():
    tl = st.session_state.timeline
    def fmt(t):
        return datetime.fromtimestamp(t).strftime('%H:%M:%S') if t else '—'
    c1, c2, c3 = st.columns(3)
    launch_label = "🚀 발사(비행 진입)"
    if tl.get('launch_reason'):
        launch_label += f" · {tl['launch_reason']}"
    c1.metric(launch_label, fmt(tl['launch']))
    c2.metric("🪂 사출(낙하 진입)", fmt(tl['eject']))
    c3.metric("🛬 착륙", fmt(tl['land']))


# ─── 그래프 (1초 갱신 — 깜빡임 완화) ────────────────────────────────────────
@st.fragment(run_every=CHART_TICK_S)
def _charts():
    ts = st.session_state.t_hist
    if not ts:
        st.info("데이터 수신 대기 중…")
        return
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=["고도 [m]", "X축 가속도 [mg] (발사 감지축)"],
                        vertical_spacing=0.12)
    fig.add_trace(go.Scatter(x=ts, y=st.session_state.alt_hist, mode='lines',
                             line=dict(color='#3498db', width=2), connectgaps=False),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=ts, y=st.session_state.acc_hist, mode='lines',
                             line=dict(color='#e74c3c', width=1.5)),
                  row=2, col=1)
    fig.add_hline(y=2000, line_dash='dash', line_color='#888',
                  annotation_text='발사감지 +2g', row=2, col=1)
    fig.update_layout(height=430, margin=dict(t=40, b=20, l=60, r=20),
                      showlegend=False)
    fig.update_xaxes(title_text='로켓 시간 [s]', row=2, col=1)
    st.plotly_chart(fig, use_container_width=True,
                    key=f"v3_chart_{st.session_state.chart_epoch}")


def main():
    _init()
    st.title("🚀 RUDDER GCS v6")
    _sidebar()
    _panel()
    _charts()


if __name__ == '__main__':
    main()
