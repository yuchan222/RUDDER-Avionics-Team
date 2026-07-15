"""
RUDDER 2026 GCS — Streamlit 기반 지상국 소프트웨어

작년 실패 반영 (gpt_1차코드_피드백.txt):
  - SEQ 기반 패킷 손실 카운터 (#12 중복수신 대응)
  - ACK 수신 표시 (#6 CRC/ACK 미보장 대응)
  - 비상 사출 2단계 확인 (#13 오입력 방지)
  - 수신 패킷 전체 CSV 로그 저장
  - BMP 실패(-1) 처리 (그래프 끊김으로 표시)
  - 데이터 소스 추상화: VirtualFlightSource → SerialSikSource 교체 가능
"""

import streamlit as st
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time
import math
import random
import threading
import queue
import csv
import struct
import pathlib
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List
from abc import ABC, abstractmethod

# ─── 상수 (Config.h / ModeManager.h 동기화) ──────────────────────────────────
FLIGHT_MODES = {0: "대기", 1: "Armed", 2: "비행", 3: "사출완료", 4: "착지"}
MODE_COLORS  = {
    0: "#555",
    1: "#e67e22",
    2: "#2980b9",
    3: "#27ae60",
    4: "#8e44ad",
}
STATUS_BMP = 0x01
STATUS_IMU = 0x02
STATUS_SD  = 0x04
STATUS_INA = 0x08

HISTORY_MAX  = 700          # 보관 최대 포인트 (~140s @ 5Hz)
REFRESH_MS   = 500          # UI 갱신 주기 [ms] (200→500: 깜빡임 빈도 완화)

# ─── 송신 명령 (CommandRx.h 동기화) ──────────────────────────────────────────
# 프로토콜: 0x3C 0x3C | CMD×4 | CRC8(앞 6바이트) = 7바이트
CMD_SET_STANDBY = 0x0B
CMD_SET_ARMED   = 0x16
CMD_FORCE_EJECT = 0x42
CMD_SYSRESET    = 0x63


def _crc8(data: bytes) -> int:
    """CommandRx.cpp와 동일한 CRC8 (poly 0x07, init 0xFF)"""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _cmd_packet(cmd: int) -> bytes:
    """비행코드 CommandRx가 파싱하는 7바이트 명령 패킷 생성"""
    body = bytes([0x3C, 0x3C, cmd, cmd, cmd, cmd])
    return body + bytes([_crc8(body)])

# ─── 패킷 포맷 (Packet.h __attribute__((packed)) 동일) ────────────────────────
#   stx   from  seq   id    len   ms      acc[3]        gyro[3]
#   H     B     H     B     H     I       hhh           hhh
#   pressure  bmp_temp  altitude_cm  voltage_mv  current_ma
#   i         h         i            h           h
#   flight_mode  eject_state  system_status  pkt_no  crc16  etx
#   B            B            B              H       H      H
PKT_FMT  = '<HBHBHIhhhhhhihihhBBBHHH'
PKT_SIZE = struct.calcsize(PKT_FMT)   # 47 bytes


@dataclass
class Pkt:
    stx:           int = 0xAA55
    from_id:       int = 1
    seq:           int = 0
    id:            int = 1
    length:        int = 0
    ms:            int = 0
    acc:           List[int] = field(default_factory=lambda: [0, 0, 0])  # [mg]
    gyro:          List[int] = field(default_factory=lambda: [0, 0, 0])  # [0.1 dps]
    pressure:      int = 101325   # [Pa]
    bmp_temp:      int = 2500     # [0.01 °C]
    altitude_cm:   int = 0        # [cm], -1 = BMP 실패
    voltage_mv:    int = 7400     # [mV]
    current_ma:    int = 500      # [mA]
    flight_mode:   int = 0        # 0~4
    eject_state:   int = 0        # 0=대기, 1=사출완료
    system_status: int = 0x0F    # STATUS_BMP|IMU|SD|INA
    pkt_no:        int = 0
    crc16:         int = 0
    etx:           int = 0x55AA
    rx_time:       float = 0.0    # GCS 수신 시각 (epoch)
    ack_received:  bool = False


# ─── 데이터 소스 추상 클래스 ──────────────────────────────────────────────────
class DataSource(ABC):
    @abstractmethod
    def start(self): ...

    @abstractmethod
    def stop(self): ...

    @abstractmethod
    def read_packet(self) -> Optional[Pkt]: ...

    @abstractmethod
    def send_ack(self): ...

    @abstractmethod
    def send_force_eject(self): ...


# ─── 가상 비행 시뮬레이터 ─────────────────────────────────────────────────────
class VirtualFlightSource(DataSource):
    """
    Config.h 기반 가상 비행 (2026-06-30 팀 합의 수치):
      최고고도 271m / 8.51s / 총비행 68s
      텔레메트리 5Hz (TELEM_INTERVAL_MS=200)
      패킷 손실 5% 시뮬레이션
    """
    MAX_ALT_M  = 271.0
    APOGEE_T   = 8.51
    FLIGHT_DUR = 68.0
    # 시나리오: 0~5s 대기, 5~10s Armed(baseline), 10s~ 발사
    T_ARMED  = 5.0
    T_LAUNCH = 10.0

    def __init__(self):
        self._q: queue.Queue    = queue.Queue(maxsize=100)
        self._running           = False
        self._thread: Optional[threading.Thread] = None
        self._seq               = 0
        self._pkt_no            = 0
        self._ack_pending       = False
        self._force_eject       = False

    def start(self):
        self._running = True
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def send_ack(self):
        self._ack_pending = True

    def send_force_eject(self):
        self._force_eject = True

    def read_packet(self) -> Optional[Pkt]:
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def _profile(self, tf: float):
        """발사 후 tf초의 (고도[m], Z가속도[mg]) 반환"""
        T, H = self.APOGEE_T, self.MAX_ALT_M
        if tf <= 0:
            return 0.0, 1000
        if tf <= T:
            alt = H * (1 - ((tf - T) / T) ** 2)
            if tf < 5.0:
                # 부스트: 최대 ~3.5g
                acc_z = 1000 + int(2500 * math.sin(tf / 5.0 * math.pi))
            else:
                # 코스트: 드래그+중력
                acc_z = -int(200 + 80 * (tf - 5.0))
        else:
            ratio = max(0.0, (self.FLIGHT_DUR - tf) / (self.FLIGHT_DUR - T))
            alt   = H * ratio ** 0.55
            if tf > self.FLIGHT_DUR - 2:
                acc_z = 1000 + random.randint(-120, 120)  # 착지 충격
            else:
                acc_z = -int(80 + 30 * random.random())   # 낙하산 하강
        return max(0.0, alt), max(-32768, min(32767, acc_z))

    def _loop(self):
        ejected = False
        while self._running:
            el = time.monotonic() - self._t0
            tf = el - self.T_LAUNCH   # 발사 후 경과 시간

            # 모드 전환
            if el < self.T_ARMED:
                mode = 0
            elif el < self.T_LAUNCH:
                mode = 1
            elif tf >= self.FLIGHT_DUR:
                mode = 4
            else:
                mode = 2

            if self._force_eject and not ejected:
                ejected = True

            alt_m, acc_z = self._profile(tf)

            # 자동 사출 (정점 도달 후)
            if mode == 2 and tf >= self.APOGEE_T and not ejected:
                ejected = True

            if ejected and mode == 2:
                mode = 3

            self._seq    = (self._seq + 1) & 0xFFFF
            self._pkt_no = (self._pkt_no + 1) & 0xFFFF
            ack, self._ack_pending = self._ack_pending, False

            alt_cm = int(alt_m * 100) + random.randint(-20, 20)

            pkt = Pkt(
                seq          = self._seq,
                ms           = int(el * 1000),
                acc          = [
                    random.randint(-60, 60),
                    random.randint(-60, 60),
                    acc_z + random.randint(-50, 50),
                ],
                gyro         = [random.randint(-5, 5) for _ in range(3)],
                pressure     = int(101325 - alt_m * 12.2),
                bmp_temp     = int(2500 - alt_m * 0.65),
                altitude_cm  = alt_cm,
                voltage_mv   = 7400 - int(el * 0.4),
                current_ma   = 500 + random.randint(-20, 20),
                flight_mode  = mode,
                eject_state  = int(ejected),
                system_status = 0x0F,
                pkt_no       = self._pkt_no,
                rx_time      = time.time(),
                ack_received = ack,
            )

            # 5% 패킷 손실 시뮬레이션
            if random.random() > 0.05:
                try:
                    self._q.put_nowait(pkt)
                except queue.Full:
                    pass

            time.sleep(0.2)  # 5Hz


# ─── pyserial SiK 소스 ────────────────────────────────────────────────────────
class SerialSikSource(DataSource):
    """
    실제 SiK 라디오 수신. 사이드바에서 포트 선택 후 자동 전환.
    STX=0xAA55 기준으로 패킷 동기화, PKT_FMT 그대로 파싱.
    """
    def __init__(self, port: str = 'COM3', baud: int = 57600):
        self._port  = port
        self._baud  = baud
        self._ser   = None
        self._q: queue.Queue = queue.Queue(maxsize=200)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        import serial
        self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._ser:
            self._ser.close()

    def send_ack(self):
        # 비행코드에 ACK/Ping 명령이 없어 실기체 연결에서는 미지원 (UI에서 숨김)
        pass

    def send_force_eject(self):
        # CommandRx 프로토콜에 맞는 7바이트 패킷 (버그 수정: 기존 0xF0 0xFF는 파싱 불가)
        if self._ser and self._ser.is_open:
            self._ser.write(_cmd_packet(CMD_FORCE_EJECT))

    def read_packet(self) -> Optional[Pkt]:
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def _rx_loop(self):
        buf = b''
        STX_BYTES = struct.pack('<H', 0xAA55)
        while self._running:
            try:
                buf += self._ser.read(256)
                while len(buf) >= PKT_SIZE:
                    idx = buf.find(STX_BYTES)
                    if idx < 0:
                        buf = buf[-4:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < PKT_SIZE:
                        break
                    raw, buf = buf[:PKT_SIZE], buf[PKT_SIZE:]
                    pkt = _parse_raw(raw)
                    if pkt:
                        try:
                            self._q.put_nowait(pkt)
                        except queue.Full:
                            pass
            except Exception:
                time.sleep(0.05)


def _parse_raw(raw: bytes) -> Optional[Pkt]:
    """raw bytes → Pkt. STX/ETX 불일치 시 None."""
    try:
        v = struct.unpack_from(PKT_FMT, raw)
        if v[0] != 0xAA55 or v[-1] != 0x55AA:
            return None
        return Pkt(
            stx=v[0], from_id=v[1], seq=v[2], id=v[3], length=v[4], ms=v[5],
            acc=list(v[6:9]), gyro=list(v[9:12]),
            pressure=v[12], bmp_temp=v[13], altitude_cm=v[14],
            voltage_mv=v[15], current_ma=v[16],
            flight_mode=v[17], eject_state=v[18], system_status=v[19],
            pkt_no=v[20], crc16=v[21],
            rx_time=time.time(),
        )
    except Exception:
        return None


# ─── 로그 관리 ────────────────────────────────────────────────────────────────
_LOG_DIR    = pathlib.Path(__file__).parent / 'logs'
_LOG_FIELDS = [
    'rx_time', 'seq', 'pkt_no', 'ms',
    'flight_mode', 'eject_state', 'altitude_cm',
    'acc_x', 'acc_y', 'acc_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'pressure', 'bmp_temp', 'voltage_mv', 'current_ma',
    'system_status', 'ack_received', 'crc16',
]


def _open_log():
    _LOG_DIR.mkdir(exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = _LOG_DIR / f'gcs_{ts}.csv'
    f    = open(path, 'w', newline='', encoding='utf-8')
    w    = csv.DictWriter(f, fieldnames=_LOG_FIELDS)
    w.writeheader()
    f.flush()
    return str(path), f, w


def _write_log(pkt: Pkt):
    w: Optional[csv.DictWriter] = st.session_state.get('log_writer')
    f = st.session_state.get('log_file')
    if w is None:
        return
    w.writerow({
        'rx_time':       f'{pkt.rx_time:.3f}',
        'seq':           pkt.seq,
        'pkt_no':        pkt.pkt_no,
        'ms':            pkt.ms,
        'flight_mode':   pkt.flight_mode,
        'eject_state':   pkt.eject_state,
        'altitude_cm':   pkt.altitude_cm,
        'acc_x':         pkt.acc[0],
        'acc_y':         pkt.acc[1],
        'acc_z':         pkt.acc[2],
        'gyro_x':        pkt.gyro[0],
        'gyro_y':        pkt.gyro[1],
        'gyro_z':        pkt.gyro[2],
        'pressure':      pkt.pressure,
        'bmp_temp':      pkt.bmp_temp,
        'voltage_mv':    pkt.voltage_mv,
        'current_ma':    pkt.current_ma,
        'system_status': hex(pkt.system_status),
        'ack_received':  pkt.ack_received,
        'crc16':         hex(pkt.crc16),
    })
    if f:
        f.flush()


# ─── COM 포트 감지 ────────────────────────────────────────────────────────────
def _list_serial_ports() -> List[tuple]:
    """감지된 시리얼 포트 목록. [(표시명, 장치명), ...]"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [
            (f"{p.device} — {p.description[:35]}", p.device)
            for p in sorted(ports, key=lambda p: p.device)
        ]
    except Exception:
        return []


def _reset_flight_data():
    """소스 전환 시 비행 데이터 전체 초기화"""
    st.session_state.t_list     = []
    st.session_state.alt_list   = []
    st.session_state.acc_list   = []
    st.session_state.last_seq   = -1
    st.session_state.lost_pkts  = 0
    st.session_state.total_pkts = 0
    st.session_state.rate_ts    = []
    st.session_state.last_pkt   = None
    st.session_state.timeline   = {'launch_t': None, 'eject_t': None, 'land_t': None}
    st.session_state.prev_mode  = -1
    st.session_state.prev_eject = 0


def _switch_source(port: Optional[str], display: str):
    """현재 소스 정지 → 새 소스 시작 → 데이터 초기화"""
    old_src = st.session_state.get('source')
    if old_src:
        old_src.stop()

    if port is None:
        src = VirtualFlightSource()
        info = '가상 데이터 (시뮬레이션)'
    else:
        src = SerialSikSource(port=port, baud=57600)
        info = display

    try:
        src.start()
        st.session_state.source          = src
        st.session_state.connection_info = info
        st.toast(f"연결됨: {info}", icon="✅")
    except Exception as e:
        st.error(f"연결 실패 ({port}): {e}")
        fallback = VirtualFlightSource()
        fallback.start()
        st.session_state.source          = fallback
        st.session_state.connection_info = '가상 데이터 (연결 실패 — 폴백)'

    _reset_flight_data()


# ─── Streamlit 페이지 설정 ────────────────────────────────────────────────────
st.set_page_config(
    page_title="RUDDER GCS 2026",
    layout="wide",
    page_icon="🚀",
)


def _init():
    """세션 최초 1회 초기화"""
    if st.session_state.get('ready'):
        return
    st.session_state.ready = True

    src = VirtualFlightSource()
    src.start()
    st.session_state.source          = src
    st.session_state.connection_info = '가상 데이터 (시뮬레이션)'
    st.session_state.t_list:    List[float]           = []
    st.session_state.alt_list:  List[Optional[float]] = []
    st.session_state.acc_list:  List[float]           = []
    st.session_state.last_seq   = -1
    st.session_state.lost_pkts  = 0
    st.session_state.total_pkts = 0
    st.session_state.rate_ts:   List[float]           = []
    st.session_state.last_pkt:  Optional[Pkt]         = None
    st.session_state.eject_phase    = 0
    st.session_state.eject_click_t  = 0.0
    # 비행 타임라인
    st.session_state.timeline   = {'launch_t': None, 'eject_t': None, 'land_t': None}
    st.session_state.prev_mode  = -1   # 직전 flight_mode (이벤트 엣지 감지용)
    st.session_state.prev_eject = 0    # 직전 eject_state

    path, f, w = _open_log()
    st.session_state.log_path   = path
    st.session_state.log_file   = f
    st.session_state.log_writer = w


# ─── UI 컴포넌트 ──────────────────────────────────────────────────────────────
def _sidebar_connection():
    """사이드바 — COM 포트 선택 및 연결"""
    with st.sidebar:
        st.markdown("### 🔌 연결 설정")

        detected = _list_serial_ports()

        # 옵션 목록 구성: 표시명 → 포트값
        disp_list: List[str]          = ["가상 데이터 (시뮬레이션)"]
        port_map:  dict[str, Optional[str]] = {"가상 데이터 (시뮬레이션)": None}

        if detected:
            for disp, dev in detected:
                label = f"🔌 {disp}"
                disp_list.append(label)
                port_map[label] = dev

        # 감지 안 된 COM1~COM10 수동 추가
        detected_devs = {dev for _, dev in detected}
        for i in range(1, 11):
            dev = f"COM{i}"
            if dev not in detected_devs:
                disp_list.append(dev)
                port_map[dev] = dev

        selected_disp = st.selectbox(
            "포트 선택",
            disp_list,
            index=0,
            key="port_select",
            help="감지된 포트는 🔌 아이콘으로 표시됩니다",
        )

        info = st.session_state.get('connection_info', '가상 데이터 (시뮬레이션)')
        st.caption(f"현재 연결: **{info}**")

        if st.button("연결", use_container_width=True, key="connect_btn", type="primary"):
            port_val = port_map.get(selected_disp)
            _switch_source(port_val, selected_disp)
            st.rerun()

        st.divider()
        log_name = pathlib.Path(st.session_state.get('log_path', 'none')).name
        st.caption(f"로그 파일: `{log_name}`")
        st.caption(f"PKT 크기: {PKT_SIZE} bytes")


def _mode_badge(mode: int):
    name  = FLIGHT_MODES.get(mode, "?")
    color = MODE_COLORS.get(mode, "#555")
    st.markdown(
        f'<div style="background:{color};border-radius:12px;padding:22px 10px;'
        f'text-align:center;margin-bottom:8px">'
        f'<span style="color:#fff;font-size:52px;font-weight:900;'
        f'letter-spacing:2px;text-shadow:0 2px 8px rgba(0,0,0,.4)">'
        f'{name}</span></div>',
        unsafe_allow_html=True,
    )


def _sensor_dots(status: int):
    def dot(ok: bool, label: str) -> str:
        c = '#2ecc71' if ok else '#e74c3c'
        return f'<span style="color:{c};font-size:20px">●</span> {label}'

    st.markdown(
        dot(bool(status & STATUS_BMP), 'BMP') + ' &nbsp; ' +
        dot(bool(status & STATUS_IMU), 'IMU') + ' &nbsp; ' +
        dot(bool(status & STATUS_SD),  'SD')  + ' &nbsp; ' +
        dot(bool(status & STATUS_INA), 'INA'),
        unsafe_allow_html=True,
    )


def _graphs():
    ts  = st.session_state.t_list
    alt = st.session_state.alt_list
    acc = st.session_state.acc_list

    if not ts:
        st.info("데이터 수신 대기 중…")
        return

    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True,
        subplot_titles=["고도 [m]", "Z축 가속도 [mg]"],
        vertical_spacing=0.12, row_heights=[0.55, 0.45],
    )
    fig.add_trace(
        go.Scatter(x=ts, y=alt, mode='lines', name='고도',
                   line=dict(color='#3498db', width=2),
                   connectgaps=False),   # BMP 실패(-1→None) 구간 끊김
        row=1, col=1,
    )
    fig.add_trace(
        go.Scatter(x=ts, y=acc, mode='lines', name='acc_z',
                   line=dict(color='#e74c3c', width=1.5)),
        row=2, col=1,
    )
    # 1g 기준선
    fig.add_hline(y=1000, line_dash='dash', line_color='#888',
                  annotation_text='1g (1000mg)', row=2, col=1)

    # 타임라인 이벤트 수직선
    tl = st.session_state.timeline
    launch_t = tl.get('launch_t')
    t0 = st.session_state.t_list[0] if st.session_state.t_list else 0
    events_on_graph = [
        (tl.get('launch_t'), '🚀 발사', '#f39c12'),
        (tl.get('eject_t'),  '💥 사출', '#27ae60'),
        (tl.get('land_t'),   '🛬 착지', '#8e44ad'),
    ]
    for evt_t, label, color in events_on_graph:
        if evt_t is not None and launch_t is not None:
            # ms 기준 x축에 맞춰 장치 ms 추정 (rx_time → device ms는 근사값)
            pass  # 그래프 vline은 device ms 단위가 필요해 생략

    fig.update_layout(
        height=520,
        margin=dict(t=40, b=20, l=60, r=20),
        showlegend=False,
        plot_bgcolor='#111',
        paper_bgcolor='#0e1117',
        font=dict(color='#eee'),
    )
    fig.update_xaxes(title_text='경과 시간 [s]', row=2, col=1,
                     gridcolor='#333', showgrid=True)
    fig.update_yaxes(gridcolor='#333', showgrid=True)
    # key 고정: fragment 갱신 시 차트 컴포넌트를 재사용해 깜빡임 방지
    st.plotly_chart(fig, use_container_width=True, key="live_chart")


def _eject_ui(src: DataSource):
    """비상 사출 버튼 — 2단계 확인 (오입력 방지, 피드백 #13)"""
    phase = st.session_state.eject_phase

    if phase == 0:
        if st.button("⚠️ 비상 사출", use_container_width=True,
                     type="primary", key="eject_arm"):
            st.session_state.eject_phase    = 1
            st.session_state.eject_click_t  = time.time()
            st.rerun()
    else:
        elapsed   = time.time() - st.session_state.eject_click_t
        remaining = 5.0 - elapsed

        if remaining <= 0:
            st.session_state.eject_phase = 0
            st.rerun()

        st.error(f"⚠️ {remaining:.1f}초 안에 확인 버튼 클릭 — 취소하려면 [취소]")
        c1, c2 = st.columns(2)

        if c1.button("🔴 사출 실행 확인", use_container_width=True,
                     key="eject_confirm"):
            src.send_force_eject()
            st.session_state.eject_phase = 0
            st.toast("비상 사출 명령 전송됨!", icon="🚨")

        if c2.button("취소", use_container_width=True, key="eject_cancel"):
            st.session_state.eject_phase = 0
            st.rerun()


def _timeline():
    """비행 타임라인 — 발사/사출/착지 시각 표시"""
    tl       = st.session_state.get('timeline', {})
    launch_t = tl.get('launch_t')
    eject_t  = tl.get('eject_t')
    land_t   = tl.get('land_t')

    def wall(t: Optional[float]) -> str:
        """epoch → HH:MM:SS.ms"""
        if not t:
            return '—'
        dt = datetime.fromtimestamp(t)
        return dt.strftime('%H:%M:%S.') + f'{dt.microsecond // 1000:03d}'

    def rel(t: Optional[float]) -> str:
        """발사 기준 상대 시각 (T+Xs)"""
        if t is None or launch_t is None:
            return ''
        return f' &nbsp; T+{t - launch_t:.2f}s'

    st.markdown("#### 비행 타임라인")

    events = [
        ('🚀', '발사 감지', launch_t, '#f39c12'),
        ('💥', '사출',      eject_t,  '#27ae60'),
        ('🛬', '착지',      land_t,   '#8e44ad'),
    ]
    for icon, label, t, color in events:
        done      = t is not None
        bar_color = color if done else '#444'
        time_html = (f'<code>{wall(t)}</code><span style="color:{color}">'
                     f'{rel(t)}</span>') if done else '<code>—</code>'
        st.markdown(
            f'<div style="border-left:4px solid {bar_color};'
            f'padding:8px 0 8px 12px;margin:5px 0;border-radius:0 4px 4px 0;">'
            f'<b>{icon} {label}</b>: {time_html}</div>',
            unsafe_allow_html=True,
        )

    if launch_t:
        elapsed = time.time() - launch_t
        st.caption(f"현재 비행 시간: **T+{elapsed:.1f}s**")
    else:
        st.caption("발사 감지 대기 중…")


# ─── 대시보드 (fragment: 페이지 전체 rerun 없이 부분 갱신 → 깜빡임 제거) ──────
@st.fragment(run_every=REFRESH_MS / 1000.0)
def _dashboard():
    src: DataSource = st.session_state.source
    now = time.time()

    # ── 패킷 드레인 & 처리 ───────────────────────────────────────────────────
    while True:
        pkt = src.read_packet()
        if pkt is None:
            break

        # 보드 재부팅 감지: 장치시간(ms)이 크게 뒤로 점프하면 아두이노 리셋으로
        # 간주하고 그래프/타임라인/카운터를 새 비행 기준으로 초기화
        t_sec = pkt.ms / 1000.0
        if st.session_state.t_list and t_sec < st.session_state.t_list[-1] - 5.0:
            _reset_flight_data()
            st.toast("보드 재시작 감지 — 그래프/타임라인 초기화", icon="🔄")

        # SEQ 기반 손실 카운터 (피드백 #12: 누락·중복 모두 추적)
        if st.session_state.last_seq >= 0:
            expected = (st.session_state.last_seq + 1) & 0xFFFF
            if pkt.seq != expected:
                gap = (pkt.seq - expected) & 0xFFFF
                if gap < 0x8000:   # 역방향 래핑 방지
                    st.session_state.lost_pkts += gap

        st.session_state.last_seq    = pkt.seq
        st.session_state.last_pkt    = pkt
        st.session_state.total_pkts += 1
        st.session_state.rate_ts.append(now)

        st.session_state.t_list.append(t_sec)
        # altitude_cm == -1 은 BMP 실패 → None으로 변환 (그래프 공백)
        alt_m = pkt.altitude_cm / 100.0 if pkt.altitude_cm != -1 else None
        st.session_state.alt_list.append(alt_m)
        st.session_state.acc_list.append(pkt.acc[2])

        _write_log(pkt)

        # ── 비행 타임라인 이벤트 감지 (엣지 트리거) ──────────────────────────
        prev_mode  = st.session_state.prev_mode
        prev_eject = st.session_state.prev_eject
        tl         = st.session_state.timeline

        # 발사: 이전 모드가 0 또는 1이었다가 2로 전환
        if pkt.flight_mode == 2 and prev_mode in (0, 1, -1) and tl['launch_t'] is None:
            tl['launch_t'] = pkt.rx_time

        # 사출: eject_state 0→1 첫 전환
        if pkt.eject_state == 1 and prev_eject == 0 and tl['eject_t'] is None:
            tl['eject_t'] = pkt.rx_time

        # 착지: mode 4 첫 진입
        if pkt.flight_mode == 4 and prev_mode != 4 and tl['land_t'] is None:
            tl['land_t'] = pkt.rx_time

        st.session_state.prev_mode  = pkt.flight_mode
        st.session_state.prev_eject = pkt.eject_state

        # 히스토리 상한 유지 (오래된 앞쪽 제거)
        if len(st.session_state.t_list) > HISTORY_MAX:
            st.session_state.t_list.pop(0)
            st.session_state.alt_list.pop(0)
            st.session_state.acc_list.pop(0)

    # 1초 윈도우 레이트 계산
    st.session_state.rate_ts = [t for t in st.session_state.rate_ts
                                 if now - t < 1.0]
    pkt_rate = len(st.session_state.rate_ts)
    last: Optional[Pkt] = st.session_state.last_pkt

    # ── 헤더 ─────────────────────────────────────────────────────────────────
    hc1, hc2 = st.columns([4, 2])
    hc1.markdown("## 🚀 RUDDER 2026 지상국 (GCS)")
    hc2.caption(f"로그 저장 중: `{pathlib.Path(st.session_state.log_path).name}`")
    st.divider()

    # ── 상단: 모드 배지 + 텔레메트리 수치 ───────────────────────────────────
    col_mode, col_tele = st.columns([2, 5])

    with col_mode:
        _mode_badge(last.flight_mode if last else 0)
        if last and last.eject_state:
            st.error("🔻 사출 완료")
        if last:
            _sensor_dots(last.system_status)

    with col_tele:
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("패킷 수신율", f"{pkt_rate} pkt/s",
                  help="최근 1초 이내 수신 패킷 수")
        m2.metric("패킷 손실",   f"{st.session_state.lost_pkts} 개",
                  help="SEQ 번호 기반 누락 패킷 수 (작년 실패 반영)")
        m3.metric("총 수신",     f"{st.session_state.total_pkts} 개")
        ack_label = "✅ 수신됨" if (last and last.ack_received) else "⏳ 대기"
        m4.metric("ACK", ack_label)

        if last:
            n1, n2, n3, n4 = st.columns(4)
            alt_str = (f"{last.altitude_cm / 100:.1f} m"
                       if last.altitude_cm != -1 else "BMP 오류")
            n1.metric("고도",   alt_str)
            n2.metric("전압",   f"{last.voltage_mv / 1000:.2f} V")
            n3.metric("전류",   f"{last.current_ma} mA")
            n4.metric("온도",   f"{last.bmp_temp / 100:.1f} °C")

    st.divider()

    # ── 실시간 그래프 ─────────────────────────────────────────────────────────
    _graphs()
    st.divider()

    # ── 하단: 비상 제어 + 패킷 상세 + 타임라인 ───────────────────────────────
    col_ctrl, col_pkt = st.columns([1, 2])

    with col_ctrl:
        st.subheader("비상 제어")
        _eject_ui(src)
        st.write("")
        if isinstance(src, VirtualFlightSource):
            if st.button("통신 확인 (Ping)", use_container_width=True):
                src.send_ack()
            st.caption("로켓과 통신 상태 확인 (시뮬레이션 전용)")
        else:
            st.caption("Ping은 비행코드에 해당 명령이 없어 실기체 연결에서는 비활성화")

    with col_pkt:
        if last:
            st.subheader("마지막 수신 패킷")
            st.code(
                f"SEQ={last.seq}  PKT_NO={last.pkt_no}  MS={last.ms / 1000:.2f}s\n"
                f"acc   X={last.acc[0]:+6d}  Y={last.acc[1]:+6d}  Z={last.acc[2]:+6d} [mg]\n"
                f"gyro  X={last.gyro[0]:+5d}  Y={last.gyro[1]:+5d}  Z={last.gyro[2]:+5d} [0.1dps]\n"
                f"pressure={last.pressure} Pa  bmp_temp={last.bmp_temp / 100:.2f} °C\n"
                f"MODE={last.flight_mode} ({FLIGHT_MODES[last.flight_mode]})  "
                f"EJECT={last.eject_state}  STATUS=0x{last.system_status:02X}\n"
                f"SEQ 손실 누적={st.session_state.lost_pkts}  "
                f"총 수신={st.session_state.total_pkts}",
                language="text",
            )

        st.divider()
        _timeline()


# ─── 메인 ─────────────────────────────────────────────────────────────────────
def main():
    _init()
    _sidebar_connection()
    _dashboard()   # fragment: run_every 주기로 자동 부분 갱신


if __name__ == '__main__':
    main()
