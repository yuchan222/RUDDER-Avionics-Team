"""
RUDDER 2026 — GCS 최종본 (피그마 디자인 + v6 전체 기능, Python/Tkinter)

gcs_v6.py(Streamlit)의 실제 통신·명령·로깅 기능을 그대로 옮기고,
화면은 gcs_figma_design.html 레이아웃으로 재구성한 버전.
version_final 펌웨어(48바이트 패킷) 전용 — 구버전 로켓과는 안 맞음.
gcs_figma.py에서 이어서 나온 최종본 (버그 수정 다수: 비상사출 완료/타임아웃 후
위젯 참조 오류로 전체 화면이 멈추던 문제 등).

실행: python gcs_final.py   (Streamlit 아님, 그냥 python으로 실행)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import struct
import threading
import queue
import csv
import pathlib
import time
import math
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List

# ── Color palette ─────────────────────────────────────────────────────────
BG       = "#080b10"
SURFACE  = "#0d1117"
SURFACE2 = "#161b22"
BORDER   = "#21262d"
BORDER2  = "#30363d"
TEXT     = "#e6edf3"
MUTED    = "#8b949e"
GREEN    = "#3fb950"
GREEN_DIM= "#1a3a1f"
RED      = "#f85149"
RED_DIM  = "#3d1a1a"
AMBER    = "#d29922"
CYAN     = "#58a6ff"
CYAN_DIM = "#0d2a4a"

MONO_SM = ("Courier", 9)

# ═══════════════════════════════════════════════════════════════════════════
#  통신/패킷 계층 (gcs_v6.py와 동일 — version6 펌웨어 48바이트 패킷)
# ═══════════════════════════════════════════════════════════════════════════
FLIGHT_MODES = {0: "대기", 1: "준비", 2: "비행", 3: "낙하", 4: "착륙"}
MODE_COLORS  = {0: MUTED, 1: AMBER, 2: CYAN, 3: GREEN, 4: "#c678dd"}

STATUS_BMP, STATUS_IMU, STATUS_SD, STATUS_INA = 0x01, 0x02, 0x04, 0x08
STATUS_BASELINE_BAD = 0x10
STATUS_LOG_CLOSED = 0x20
STATUS_LAUNCH_ACCEL = 0x40
STATUS_LAUNCH_ALT   = 0x80

EJECT_REASON = {0: None, 1: "주 사출 (정점통과)", 2: "보조 사출 (10초 타이머)", 3: "지상국 비상 명령"}

ALT_INVALID = -2147483648

CMD_SET_STANDBY  = 0x0B
CMD_SET_ARMED    = 0x16
CMD_FORCE_FLIGHT = 0x2D
CMD_FORCE_EJECT  = 0x42
CMD_FORCE_LAND   = 0x51

CMD_NAMES = {
    CMD_SET_STANDBY: "대기 전환",
    CMD_SET_ARMED: "준비 전환",
    CMD_FORCE_FLIGHT: "강제 비행진입",
    CMD_FORCE_EJECT: "비상 사출",
    CMD_FORCE_LAND: "강제 착륙",
}

BAUD          = 57600
RETRY_GAP_S   = 0.3
MAX_RETRY_S   = 30.0
HISTORY_MAX   = 700

PKT_FMT    = '<HBHBHIhhhhhhihihhBBBHBhhHH'   # 52바이트 — version8 전용 (tilt_xy/tilt_xz 추가)
PKT_SIZE   = struct.calcsize(PKT_FMT)
CRC_OFFSET = 48


def _crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cmd_packet(cmd: int) -> bytes:
    body = bytes([0x3C, 0x3C, cmd, cmd, cmd, cmd])
    return body + bytes([_crc8(body)])


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
    tilt_xy: int = 0    # [0.01 deg] X-Y 평면 기울기 (상보필터, 실험용)
    tilt_xz: int = 0    # [0.01 deg] X-Z 평면 기울기
    rx_time: float = 0.0


def parse_raw(raw: bytes):
    """(Pkt|None, crc_ok) — STX/ETX 불일치는 None, CRC 불일치는 (None, False)"""
    try:
        v = struct.unpack_from(PKT_FMT, raw)
        if v[0] != 0xAA55 or v[-1] != 0x55AA:
            return None, True
        if _crc16(raw[:CRC_OFFSET]) != v[24]:
            return None, False
        return Pkt(
            seq=v[2], ms=v[5],
            acc=list(v[6:9]), gyro=list(v[9:12]),
            pressure=v[12], bmp_temp=v[13], altitude_cm=v[14],
            voltage_mv=v[15], current_ma=v[16],
            flight_mode=v[17], eject_state=v[18], system_status=v[19],
            cmd_rx_count=v[20], last_cmd=v[21],
            tilt_xy=v[22], tilt_xz=v[23], rx_time=time.time(),
        ), True
    except Exception:
        return None, True


class SikLink:
    """지상측 SiK 시리얼 링크 — 수신 스레드 + 명령 송신 (gcs_v6.py와 동일)"""
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


FAKE_PORT = "FAKE"


def list_ports():
    ports = [("🧪 가상 데이터 (테스트용, 하드웨어 불필요)", FAKE_PORT)]
    try:
        import serial.tools.list_ports
        ports += [(f"{p.device} — {p.description[:30]}", p.device)
                   for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)]
    except Exception:
        pass
    return ports


class FakeLink:
    """가상 데이터 — 실제 시리얼 없이 GCS 테스트용.
    version6 펌웨어의 모드전환·사출 로직(Config.h 상수 기준)을 최소 재현한다.
    실제 명령(모드버튼/비상사출)에도 그대로 반응해서, 하드웨어 없이도 UI 전체 흐름을 테스트할 수 있음.
    """
    SAFE_LOCK_MS = 4000
    BACKUP_TIMER_MS = 10000
    MIN_APOGEE_ALT_M = 30.0
    APOGEE_DROP_M = 3.0

    def __init__(self, port=None):
        self._q: queue.Queue = queue.Queue(maxsize=500)
        self._running = False
        self._thread = None
        self.crc_bad = 0
        self._mode = 0
        self._ejected = False
        self._eject_reason = 0
        self._launch_ms = None
        self._launch_manual = False
        self._peak_alt = 0.0
        self._t0 = None
        self._seq = 0
        self._cmd_rx = 0
        self._last_cmd = 0

    def start(self):
        self._running = True
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def send_cmd(self, cmd: int):
        self._cmd_rx += 1
        self._last_cmd = cmd
        if cmd == CMD_SET_ARMED and self._mode == 0:
            self._mode = 1
            self._ejected = False
            self._eject_reason = 0
            self._launch_ms = None
            self._peak_alt = 0.0
        elif cmd == CMD_SET_STANDBY and self._mode == 1:
            self._mode = 0
        elif cmd == CMD_FORCE_FLIGHT and self._mode == 1:
            self._mode = 2
            self._launch_ms = self._now_ms()
            self._launch_manual = True
        elif cmd == CMD_FORCE_EJECT and self._mode in (1, 2, 3):
            if not self._ejected:
                self._ejected = True
                self._eject_reason = 3   # 지상국 비상 명령
            self._mode = 3
        elif cmd == CMD_FORCE_LAND and self._mode == 3:
            self._mode = 4

    def read_packet(self) -> Optional['Pkt']:
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None

    def _now_ms(self):
        return int((time.time() - self._t0) * 1000)

    def _loop(self):
        while self._running:
            now_ms = self._now_ms()
            alt_m, accx = 0.0, 0

            if self._mode == 2 and self._launch_ms is not None:
                dt = (now_ms - self._launch_ms) / 1000.0
                if dt < 7.0:
                    alt_m = 250.0 * (1 - math.cos(min(dt / 7.0, 1.0) * math.pi / 2))
                    accx = int(2500 * math.exp(-dt))
                else:
                    alt_m = max(0.0, 250.0 - (dt - 7.0) * 40.0)
                if alt_m > self._peak_alt:
                    self._peak_alt = alt_m

                since_launch = now_ms - self._launch_ms
                if not self._ejected:
                    if since_launch >= self.BACKUP_TIMER_MS:
                        self._ejected, self._eject_reason, self._mode = True, 2, 3
                    elif (since_launch >= self.SAFE_LOCK_MS
                          and self._peak_alt >= self.MIN_APOGEE_ALT_M
                          and (self._peak_alt - alt_m) >= self.APOGEE_DROP_M):
                        self._ejected, self._eject_reason, self._mode = True, 1, 3
            elif self._mode == 3 and self._launch_ms is not None:
                dt = (now_ms - self._launch_ms) / 1000.0
                alt_m = max(0.0, self._peak_alt - max(0.0, dt - 7.0) * 45.0)

            # 가상 기울기 — 대기중엔 거의 0, 낙하(텀블링) 중엔 크게 흔들리게 (검증용)
            amp = 20.0 if self._mode == 3 else (3.0 if self._mode == 2 else 0.5)
            tilt_xy = amp * math.sin(now_ms / 400.0)
            tilt_xz = amp * math.cos(now_ms / 550.0)

            pkt = Pkt(
                seq=self._seq, ms=now_ms,
                acc=[accx, 0, 0], gyro=[0, 0, 0],
                pressure=101325, bmp_temp=2500,
                altitude_cm=int(alt_m * 100),
                voltage_mv=7500, current_ma=120,
                flight_mode=self._mode, eject_state=self._eject_reason,
                system_status=(STATUS_BMP | STATUS_IMU | STATUS_SD | STATUS_INA
                               | (STATUS_LAUNCH_ALT if self._mode >= 2 and not self._launch_manual else 0)),
                cmd_rx_count=self._cmd_rx, last_cmd=self._last_cmd,
                tilt_xy=int(tilt_xy * 100), tilt_xz=int(tilt_xz * 100),
                rx_time=time.time(),
            )
            self._seq += 1
            try:
                self._q.put_nowait(pkt)
            except queue.Full:
                pass
            time.sleep(0.02)   # 50Hz


# ─── CSV 로그 (gcs_v6.py와 동일 필드) ────────────────────────────────────────
LOG_DIR = pathlib.Path(__file__).parent / 'logs'
LOG_FIELDS = ['rx_time', 'seq', 'ms', 'flight_mode', 'eject_state', 'altitude_cm',
              'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
              'pressure', 'bmp_temp', 'voltage_mv', 'current_ma',
              'system_status', 'cmd_rx_count', 'last_cmd']


def open_log():
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"gcs_final_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(path, 'w', newline='', encoding='utf-8')
    w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
    w.writeheader()
    return path, f, w


# ═══════════════════════════════════════════════════════════════════════════
#  UI 위젯
# ═══════════════════════════════════════════════════════════════════════════
class DataCard(tk.Frame):
    def __init__(self, parent, label, unit="", color=CYAN, **kw):
        super().__init__(parent, bg=SURFACE2, bd=0, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        tk.Label(self, text=label.upper(), font=("Arial", 7, "bold"),
                 bg=SURFACE2, fg=MUTED, anchor="w").pack(fill="x", padx=6, pady=(5, 0))
        row = tk.Frame(self, bg=SURFACE2)
        row.pack(fill="x", padx=6, pady=(0, 5))
        self._val = tk.Label(row, text="—", font=("Courier", 15, "bold"),
                              bg=SURFACE2, fg=color, anchor="w")
        self._val.pack(side="left")
        self._unit = tk.Label(row, text=unit, font=("Courier", 8),
                               bg=SURFACE2, fg=MUTED, anchor="sw")
        self._unit.pack(side="left", padx=(2, 0))
        self._default_color = color

    def set(self, value, color=None):
        self._val.config(text=str(value), fg=color or self._default_color)


class Sparkline(tk.Canvas):
    """격자선 + 시간축(발사 후 경과초) + 현재값(우상단 큰 숫자) 표시가 있는 그래프"""
    def __init__(self, parent, color=CYAN, unit="", fmt="{:.1f}", **kw):
        super().__init__(parent, bg=SURFACE2, bd=0, highlightthickness=0, **kw)
        self._color = color
        self._unit = unit
        self._fmt = fmt

    def update_data(self, data, times=None):
        self.delete("all")
        self.update_idletasks()
        w, h = self.winfo_width(), self.winfo_height()
        if w < 2 or h < 2 or len(data) < 2:
            return
        vals = [v for v in data if v is not None]
        if not vals:
            return
        mn, mx = min(vals), max(vals)
        rng = mx - mn if mx != mn else 1
        pad_l, pad_r, pad_t, pad_b = 34, 8, 8, 24

        def to_xy(i, v):
            x = pad_l + (i / (len(data) - 1)) * (w - pad_l - pad_r)
            y = (h - pad_b) - ((v - mn) / rng) * (h - pad_t - pad_b)
            return x, y

        # 격자선 4단계 (min~max 사이) + 값 라벨
        for frac in (0.0, 1/3, 2/3, 1.0):
            gy = (h - pad_b) - frac * (h - pad_t - pad_b)
            self.create_line(pad_l, gy, w - pad_r, gy, fill=BORDER, dash=(2, 3))
            gv = mn + frac * rng
            self.create_text(pad_l - 4, gy, text=self._fmt.format(gv), font=("Courier", 7),
                              fill=MUTED, anchor="e")

        # 시간축 (발사 감지 후 경과초, 5구간) — 잘 보이게 흰색 + 큰 글씨
        # 양끝은 좌/우로 안 잘리게 anchor를 sw/se로, 중간 3개는 중앙정렬(s)
        if times:
            for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
                idx = min(int(frac * (len(times) - 1)), len(times) - 1)
                tx = pad_l + frac * (w - pad_l - pad_r)
                anchor = "sw" if frac == 0.0 else ("se" if frac == 1.0 else "s")
                self.create_text(tx, h - 4, text=f"{times[idx]:+.1f}s", font=("Courier", 9, "bold"),
                                  fill=TEXT, anchor=anchor)

        pts = []
        for i, v in enumerate(data):
            if v is None:
                continue
            pts.extend(to_xy(i, v))
        if len(pts) >= 4:
            fill_pts = pts[:] + [pts[-2], h - pad_b, pts[0], h - pad_b]
            self.create_polygon(fill_pts, fill=self._color, stipple="gray25", outline="")
            self.create_line(pts, fill=self._color, width=1.5, smooth=False)

        # 현재값 (우상단, 크게)
        cur = vals[-1]
        self.create_text(w - pad_r, 4, text=f"{self._fmt.format(cur)}{self._unit}",
                          font=("Courier", 13, "bold"), fill=self._color, anchor="ne")


class AttitudeIndicator(tk.Canvas):
    """로켓 자세 — 2D 로켓 실루엣이 실제 tilt_xy/tilt_xz 값에 따라 기울어짐.
    (실제 Fusion360 OBJ 3D 모델은 Three.js/WebGL이 필요해서 Tkinter엔 못 넣음 —
    v7의 브라우저 기반 3D뷰와는 별개. 여기선 2D 실루엣 회전으로 근사함.)
    version8 실험 기능 — 자세추정 자체가 완성되기 전까지는 참고용."""

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=SURFACE2, bd=0, highlightthickness=0, **kw)
        self._xy = 0.0   # roll처럼 사용 (2D 평면 내 회전)
        self._xz = 0.0   # pitch처럼 사용 (수직 축소로 앞뒤 기울임 표현)

    def update_tilt(self, tilt_xy, tilt_xz):
        self._xy = tilt_xy
        self._xz = tilt_xz
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 10 or h < 10:
            return

        # 상단: 실측 각도 텍스트 (원/실루엣 밖 고정 위치라 절대 안 잘림)
        self.create_text(w / 2, 6, text=f"XY {self._xy:+.1f}°   XZ {self._xz:+.1f}°",
                          font=("Courier", 10, "bold"), fill=TEXT, anchor="n")

        cx, cy = w / 2, h / 2 + 8
        roll = math.radians(self._xy)
        pitch = math.radians(self._xz)
        bw, bh = min(w, h) * 0.14, min(w, h) * 0.42

        def transform(px, py):
            rx = px * math.cos(roll) - py * math.sin(roll)
            ry = px * math.sin(roll) + py * math.cos(roll)
            ry *= math.cos(pitch)   # 앞/뒤로 기울면 짧아 보이게 (원근 근사)
            return cx + rx, cy + ry

        corners = [(-bw/2, -bh/2), (bw/2, -bh/2), (bw/2, bh/2), (-bw/2, bh/2)]
        pts = []
        for px, py in corners:
            pts.extend(transform(px, py))

        # 기준 십자선 (정자세 참고용)
        ref = min(w, h) * 0.42
        self.create_line(cx - ref, cy, cx + ref, cy, fill=BORDER, dash=(2, 4))
        self.create_line(cx, cy - ref, cx, cy + ref, fill=BORDER, dash=(2, 4))

        # 그림자 + 몸통 (x,y 각각 +3만큼 오프셋 — 이전엔 x끼리/y끼리 따로 모아서
        # create_polygon에 잘못된 좌표 순서로 들어가 이상한 삼각형이 그려지던 버그)
        self.create_polygon([c + 3 for c in pts], fill="#050810", outline="")
        self.create_polygon(pts, fill=SURFACE, outline=BORDER2, width=1)

        # 노즈콘
        tip = transform(0, -bh/2 - bw * 0.7)
        left = transform(-bw/2, -bh/2)
        right = transform(bw/2, -bh/2)
        self.create_polygon(tip[0], tip[1], left[0], left[1], right[0], right[1],
                             fill=CYAN_DIM, outline=CYAN, width=1)

        # 핀 (좌우)
        for side in (-1, 1):
            f1 = transform(side * bw/2, bh/2 - bw*0.3)
            f2 = transform(side * (bw/2 + bw*0.8), bh/2 + bw*0.5)
            f3 = transform(side * bw/2, bh/2)
            self.create_polygon(f1[0], f1[1], f2[0], f2[1], f3[0], f3[1],
                                 fill=SURFACE2, outline=BORDER2)


# ═══════════════════════════════════════════════════════════════════════════
#  메인 앱
# ═══════════════════════════════════════════════════════════════════════════
class RocketGCS(tk.Tk):
    MODES = ["대기", "준비", "비행", "낙하", "착륙"]
    SENSOR_NAMES = ["BMP", "MPU", "SD", "INA", "기준압"]

    def __init__(self):
        super().__init__()
        self.title("Rocket GCS — 최종본 (version_final)")
        self.configure(bg=BG)
        self.geometry("1280x780")
        self.resizable(True, True)

        self.link = None   # SikLink 또는 FakeLink
        self.last_pkt: Optional[Pkt] = None
        self.last_seq = -1
        self.lost = 0
        self.total = 0
        self.rate_ts: List[float] = []
        self.t_hist: List[float] = []
        self.alt_hist: List[Optional[float]] = []
        self.acc_hist: List[int] = []
        self.timeline = {'launch': None, 'launch_reason': None, 'eject': None, 'land': None}
        self.prev_mode = -1
        self.launch_ref_ms = None      # 발사 감지 시점 ms — 그래프 시간축 0초 기준
        self.graph_frozen = False      # 착륙하면 그래프 그대로 고정(더 안 밀려나게)
        self.prev_cmd_rx_count = 0     # 명령 수신 카운트 변화 감지용 (이벤트 로그에 명령명 표시)

        self.eject_phase = 0          # 0=없음 1=1차확인대기(다이얼로그가 처리) 2=재전송중
        self.retry_sent = 0
        self.retry_start_t = 0.0
        self.last_retry_t = 0.0

        self.log_path, self.log_file, self.log_writer = open_log()

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ── UI 빌드 ───────────────────────────────────────────────────────────
    def _build_ui(self):
        self._build_topbar()
        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True, padx=6, pady=6)
        main.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=3)   # 왼쪽: 2x2 정사각형 그리드 (고도/가속도/자세/이벤트로그)
        main.columnconfigure(1, weight=2)   # 오른쪽: 나머지(데이터카드/센서/서보모드/타임라인/비상사출)

        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        self._build_grid(left)

        right = tk.Frame(main, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        self._build_right(right)

    def _panel(self, parent, title="", color=MUTED):
        f = tk.Frame(parent, bg=SURFACE, bd=0, highlightthickness=1, highlightbackground=BORDER)
        if title:
            h = tk.Frame(f, bg=SURFACE2)
            h.pack(fill="x")
            tk.Label(h, text=f"◆ {title.upper()}", font=("Arial", 7, "bold"),
                     bg=SURFACE2, fg=color, anchor="w").pack(side="left", padx=8, pady=4)
        return f

    # ── 상단바 ────────────────────────────────────────────────────────────
    def _build_topbar(self):
        bar = tk.Frame(self, bg=SURFACE, bd=0, highlightthickness=1, highlightbackground=BORDER)
        bar.pack(fill="x", side="top")

        tk.Label(bar, text="◈ ROCKET GCS", font=("Courier", 12, "bold"),
                 bg=SURFACE, fg=CYAN).pack(side="left", padx=12, pady=6)
        tk.Frame(bar, bg=BORDER2, width=1).pack(side="left", fill="y", pady=4, padx=4)

        self._build_conn_panel(bar)

        rx_frame = tk.Frame(bar, bg=SURFACE)
        rx_frame.pack(side="left", padx=8)
        tk.Label(rx_frame, text="수신율", font=("Arial", 9), bg=SURFACE, fg=MUTED).pack(side="left")
        self.rx_lbl = tk.Label(rx_frame, text="—", font=("Courier", 10, "bold"), bg=SURFACE, fg=CYAN)
        self.rx_lbl.pack(side="left", padx=(4, 0))

        tk.Frame(bar, bg=BORDER2, width=1).pack(side="left", fill="y", pady=4, padx=4)

        mode_frame = tk.Frame(bar, bg=SURFACE)
        mode_frame.pack(side="left", padx=14, pady=6)
        self.mode_labels = {}
        for i, m in enumerate(self.MODES):
            if i > 0:
                tk.Label(mode_frame, text="→", font=("Courier", 12), bg=SURFACE,
                         fg=BORDER2).pack(side="left", padx=2)
            lbl = tk.Label(mode_frame, text=m.upper(), font=("Arial", 13, "bold"),
                           bg=SURFACE2, fg=MUTED, padx=16, pady=8, bd=0,
                           highlightthickness=2, highlightbackground=BORDER2)
            lbl.pack(side="left")
            self.mode_labels[i] = lbl

        self.clock_lbl = tk.Label(bar, text="", font=MONO_SM, bg=SURFACE, fg=MUTED)
        self.clock_lbl.pack(side="right", padx=12)

    def _build_conn_panel(self, parent):
        frame = tk.Frame(parent, bg=SURFACE)
        frame.pack(side="left", padx=4)

        self.conn_toggle_btn = tk.Button(
            frame, text="»", font=("Courier", 11, "bold"), bg=SURFACE2, fg=CYAN, bd=0,
            activebackground=BORDER, activeforeground=CYAN, cursor="hand2", padx=8, pady=3,
            command=self._toggle_conn_panel)
        self.conn_toggle_btn.pack(side="left")

        self.conn_dot = tk.Label(frame, text="●", font=("Arial", 10), bg=SURFACE, fg=RED)
        self.conn_dot.pack(side="left", padx=(6, 2))
        self.conn_status_lbl = tk.Label(frame, text="DISC", font=("Courier", 10, "bold"),
                                         bg=SURFACE, fg=RED)
        self.conn_status_lbl.pack(side="left")

        self.conn_expand = tk.Frame(frame, bg=SURFACE)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(self.conn_expand, textvariable=self.port_var,
                                     width=16, font=MONO_SM, state="readonly")
        self.port_cb.pack(side="left", padx=(8, 4))
        self._refresh_ports()

        self.conn_btn = tk.Button(
            self.conn_expand, text="연결", font=("Courier", 9, "bold"), bg=GREEN_DIM, fg=GREEN,
            bd=0, activebackground=GREEN_DIM, cursor="hand2", padx=8, pady=2,
            highlightthickness=1, highlightbackground=GREEN, command=self._toggle_connection)
        self.conn_btn.pack(side="left")

    def _refresh_ports(self):
        ports = list_ports()
        self._port_devs = [dev for _, dev in ports]
        disp = [d for d, _ in ports] or ["(포트 없음)"]
        self.port_cb['values'] = disp
        if disp:
            self.port_cb.current(0)

    def _toggle_conn_panel(self):
        if self.conn_expand.winfo_ismapped():
            self.conn_expand.pack_forget()
            self.conn_toggle_btn.config(text="»")
        else:
            self._refresh_ports()
            self.conn_expand.pack(side="left")
            self.conn_toggle_btn.config(text="«")

    def _toggle_connection(self):
        if self.link:
            self.link.stop()
            self.link = None
            self._log_event("연결 끊김")
            self.conn_dot.config(fg=RED)
            self.conn_status_lbl.config(text="DISC", fg=RED)
            self.conn_btn.config(text="연결", bg=GREEN_DIM, fg=GREEN, highlightbackground=GREEN)
            return
        idx = self.port_cb.current()
        if idx < 0 or idx >= len(self._port_devs):
            messagebox.showerror("연결 실패", "연결할 포트가 없습니다.")
            return
        dev = self._port_devs[idx]
        new_link = FakeLink(dev) if dev == FAKE_PORT else SikLink(dev)
        try:
            new_link.start()
        except Exception as e:
            messagebox.showerror("연결 실패", str(e))
            return
        self.link = new_link
        self._reset_flight_data()
        self._log_event(f"연결됨 — {dev}")
        self.conn_dot.config(fg=GREEN)
        self.conn_status_lbl.config(text="CONN", fg=GREEN)
        self.conn_btn.config(text="끊기", bg=RED_DIM, fg=RED, highlightbackground=RED)

    def _reset_flight_data(self):
        self.last_pkt = None
        self.last_seq = -1
        self.lost = 0
        self.total = 0
        self.rate_ts = []
        self.t_hist = []
        self.alt_hist = []
        self.acc_hist = []
        self.timeline = {'launch': None, 'launch_reason': None, 'eject': None, 'land': None}
        self.prev_mode = -1
        self.eject_phase = 0
        self.launch_ref_ms = None
        self.graph_frozen = False
        self.prev_cmd_rx_count = 0

    def _log_event(self, msg: str, color=None):
        if not hasattr(self, 'event_log_text'):
            return
        ts = datetime.now().strftime('%H:%M:%S')
        tag = ()
        if color:
            tag = (f"c_{color}",)
            if tag[0] not in self.event_log_text.tag_names():
                self.event_log_text.tag_config(tag[0], foreground=color)
        self.event_log_text.config(state='normal')
        self.event_log_text.insert('end', f"[{ts}] {msg}\n", tag)
        n_lines = int(self.event_log_text.index('end-1c').split('.')[0])
        if n_lines > 300:
            self.event_log_text.delete('1.0', f"{n_lines - 300}.0")
        self.event_log_text.see('end')
        self.event_log_text.config(state='disabled')

    def _log_mode_transition(self, prev_mode: int, pkt: 'Pkt'):
        """모드 전환을 이벤트 로그에 기록 — 수동/자동 사유를 같이 남겨서
        '수동으로 전환했는데 어디 기록되는지 모르겠다' 문제를 해결."""
        names = self.MODES
        frm = names[prev_mode] if 0 <= prev_mode < len(names) else str(prev_mode)
        to = names[pkt.flight_mode] if 0 <= pkt.flight_mode < len(names) else str(pkt.flight_mode)
        reason = ""
        if pkt.flight_mode == 2:
            if pkt.system_status & STATUS_LAUNCH_ACCEL:
                reason = " (자동 · 가속도 경로)"
            elif pkt.system_status & STATUS_LAUNCH_ALT:
                reason = " (자동 · 고도 경로)"
            else:
                reason = " (수동 강제진입)"
        elif pkt.flight_mode == 3:
            r = EJECT_REASON.get(pkt.eject_state)
            reason = f" ({r})" if r else ""
        elif pkt.flight_mode in (0, 1, 4):
            reason = " (수동)"
        self._log_event(f"모드: {frm} → {to}{reason}", color=CYAN)

    # ── 왼쪽: 2x2 정사각형 그리드 (1고도그래프 2가속도그래프 4로켓자세 5이벤트로그) ─
    def _build_grid(self, parent):
        parent.columnconfigure(0, weight=1, uniform="sq")
        parent.columnconfigure(1, weight=1, uniform="sq")
        parent.rowconfigure(0, weight=1, uniform="sq")
        parent.rowconfigure(1, weight=1, uniform="sq")

        alt_panel = self._panel(parent, "고도 그래프", CYAN)
        alt_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 3), pady=(0, 3))
        self.alt_spark = Sparkline(alt_panel, color=CYAN, unit="m", fmt="{:.1f}")
        self.alt_spark.pack(fill="both", expand=True, padx=4, pady=4)

        ax_panel = self._panel(parent, "가속도 X 그래프 (발사감지축)", AMBER)
        ax_panel.grid(row=0, column=1, sticky="nsew", padx=(3, 0), pady=(0, 3))
        self.ax_spark = Sparkline(ax_panel, color=AMBER, unit="mg", fmt="{:.0f}")
        self.ax_spark.pack(fill="both", expand=True, padx=4, pady=4)

        att_panel = self._panel(parent, "로켓 자세 (실험용)", CYAN)
        att_panel.grid(row=1, column=0, sticky="nsew", padx=(0, 3), pady=(3, 0))
        self.attitude = AttitudeIndicator(att_panel)
        self.attitude.pack(fill="both", expand=True, padx=4, pady=4)

        self._build_event_log(parent, row=1, col=1)

    # ── 오른쪽: 데이터카드/센서/서보+모드/타임라인/비상사출 (나머지 전부) ────
    def _build_right(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=0)   # 데이터 카드
        parent.rowconfigure(1, weight=1)   # 센서 상태 — 남는 공간 절반 흡수 (여백 없애기)
        parent.rowconfigure(2, weight=0)   # 서보 + 모드제어
        parent.rowconfigure(3, weight=1)   # 타임라인 — 남는 공간 나머지 절반 흡수
        parent.rowconfigure(4, weight=0)   # 비상사출 + 발사이벤트

        self._build_datacards(parent)
        self._build_sensor_panel(parent)
        self._build_ctrl_row(parent)
        self._build_timeline_panel(parent)
        self._build_emer_row(parent)

    def _build_datacards(self, parent):
        """오른쪽 칼럼이 좁아져서 6개를 2열×3행으로 배치"""
        row = tk.Frame(parent, bg=BG)
        row.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        row.columnconfigure((0, 1), weight=1, uniform="card")

        specs = [
            ("고도",   "m",    CYAN),
            ("전압",   "V",    GREEN),
            ("가속X",  "mg",   AMBER),
            ("가속Y",  "mg",   AMBER),
            ("가속Z",  "mg",   AMBER),
            ("SD로그", "",     GREEN),
        ]
        self.dcards = {}
        for i, (lbl, unit, clr) in enumerate(specs):
            c = DataCard(row, lbl, unit, clr)
            c.grid(row=i // 2, column=i % 2, sticky="nsew", padx=2, pady=2)
            self.dcards[lbl] = c

    def _build_sensor_panel(self, parent):
        sens_panel = self._panel(parent, "센서 상태", MUTED)
        sens_panel.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
        sens_row = tk.Frame(sens_panel, bg=SURFACE)
        sens_row.pack(fill="both", expand=True, padx=8, pady=(0, 6))
        self.sensor_frames = {}
        for name in self.SENSOR_NAMES:
            f = tk.Frame(sens_row, bg=RED_DIM, bd=0, highlightthickness=1, highlightbackground=RED)
            f.pack(side="left", padx=3, pady=2)
            dot = tk.Label(f, text="●", font=("Arial", 9), bg=RED_DIM, fg=RED)
            dot.pack(side="left", padx=(4, 2), pady=2)
            lbl = tk.Label(f, text=name, font=("Courier", 9, "bold"), bg=RED_DIM, fg=RED)
            lbl.pack(side="left", padx=(0, 4), pady=2)
            self.sensor_frames[name] = (f, dot, lbl)

    def _build_ctrl_row(self, parent):
        ctrl_row = tk.Frame(parent, bg=BG)
        ctrl_row.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        ctrl_row.columnconfigure(0, weight=0)
        ctrl_row.columnconfigure(1, weight=1)

        srv_panel = self._panel(ctrl_row, "서보 전원", MUTED)
        srv_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        srv_inner = tk.Frame(srv_panel, bg=SURFACE)
        srv_inner.pack(padx=8, pady=6)
        self.servo_lbl = tk.Label(srv_inner, text="OFF", font=("Courier", 12, "bold"),
                                   bg=SURFACE2, fg=MUTED, width=6, pady=6,
                                   highlightthickness=1, highlightbackground=BORDER2)
        self.servo_lbl.pack()
        tk.Label(srv_inner, text="MOSFET 신호 기준", font=("Arial", 6), bg=SURFACE,
                 fg=MUTED).pack(pady=(2, 0))

        mode_panel = self._panel(ctrl_row, "모드 제어", MUTED)
        mode_panel.grid(row=0, column=1, sticky="nsew")
        mode_inner = tk.Frame(mode_panel, bg=SURFACE)
        mode_inner.pack(padx=8, pady=6, fill="x")
        self.mode_btns = {}
        btn_specs = [
            ("⚪ 준비→대기", CMD_SET_STANDBY, 1, False),
            ("🟠 대기→준비", CMD_SET_ARMED, 0, False),
            ("🔵 준비→비행", CMD_FORCE_FLIGHT, 1, True),   # 되돌릴 수 없음 → 확인 필요
            ("🟣 낙하→착륙", CMD_FORCE_LAND, 3, False),
        ]
        for i, (label, cmd, req_mode, confirm) in enumerate(btn_specs):
            btn = tk.Button(mode_inner, text=label, font=("Arial", 8, "bold"), bg=SURFACE2,
                            fg=MUTED, bd=0, highlightthickness=1, highlightbackground=BORDER2,
                            padx=6, pady=5, cursor="hand2", state="disabled",
                            command=lambda c=cmd, cf=confirm: self._send_mode_cmd(c, cf))
            btn.grid(row=i // 2, column=i % 2, padx=2, pady=2, sticky="ew")
            self.mode_btns[cmd] = (btn, req_mode)
        mode_inner.columnconfigure(0, weight=1)
        mode_inner.columnconfigure(1, weight=1)

    def _build_timeline_panel(self, parent):
        tl_panel = self._panel(parent, "타임라인", CYAN)
        tl_panel.grid(row=3, column=0, sticky="nsew", pady=(0, 6))
        tl_inner = tk.Frame(tl_panel, bg=SURFACE)
        tl_inner.pack(fill="both", expand=True, padx=6, pady=6)
        tl_inner.columnconfigure((0, 1, 2), weight=1, uniform="tl")
        tl_inner.rowconfigure(0, weight=1)
        self.tl_vals = {}
        self.tl_sub = {}
        for i, (lbl, clr) in enumerate([("발사시각", CYAN), ("사출시각", AMBER), ("착륙시각", GREEN)]):
            card = tk.Frame(tl_inner, bg=SURFACE2, bd=0, highlightthickness=1, highlightbackground=BORDER)
            card.grid(row=0, column=i, sticky="nsew", padx=2)
            tk.Label(card, text=lbl.upper(), font=("Arial", 7, "bold"), bg=SURFACE2, fg=MUTED,
                     anchor="w").pack(fill="x", padx=6, pady=(3, 0))
            v = tk.Label(card, text="--:--:--", font=("Courier", 10, "bold"), bg=SURFACE2, fg=clr, anchor="w")
            v.pack(fill="x", padx=6, pady=(0, 0))
            sub = tk.Label(card, text="", font=("Arial", 7), bg=SURFACE2, fg=MUTED, anchor="w")
            sub.pack(fill="x", padx=6, pady=(0, 3))
            self.tl_vals[lbl] = v
            self.tl_sub[lbl] = sub

    def _build_event_log(self, parent, row=4, col=0):
        """모드전환(수동/자동 구분)·명령수신·사출·연결 이력을 시간순으로 표시."""
        log_panel = self._panel(parent, "이벤트 로그", MUTED)
        log_panel.grid(row=row, column=col, sticky="nsew",
                        padx=(3, 0) if col else (0, 0), pady=(3, 0) if row else (0, 6))
        log_inner = tk.Frame(log_panel, bg=SURFACE)
        log_inner.pack(fill="both", expand=True, padx=6, pady=6)
        scrollbar = tk.Scrollbar(log_inner)
        scrollbar.pack(side="right", fill="y")
        self.event_log_text = tk.Text(
            log_inner, bg=SURFACE2, fg=TEXT, font=("Courier", 8), bd=0,
            highlightthickness=0, wrap="word", state="disabled",
            yscrollcommand=scrollbar.set)
        self.event_log_text.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.event_log_text.yview)

    def _build_emer_row(self, parent):
        row = tk.Frame(parent, bg=BG)
        row.grid(row=4, column=0, sticky="ew")
        row.columnconfigure(0, weight=3)   # 비상사출 — 더 넓게(안전 관련 최우선 요소)
        row.columnconfigure(1, weight=2)   # 발사 이벤트 — 상대적으로 축소

        emer_panel = self._panel(row, "비상 사출", RED)
        emer_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        self.emer_inner = tk.Frame(emer_panel, bg=SURFACE)
        self.emer_inner.pack(fill="both", expand=True, padx=10, pady=10)
        self._build_emer_idle()

        tl2_panel = self._panel(row, "발사 이벤트", MUTED)
        tl2_panel.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        tl2_inner = tk.Frame(tl2_panel, bg=SURFACE)
        tl2_inner.pack(fill="both", expand=True, padx=10, pady=10)
        tk.Label(tl2_inner, text="경과시간 (발사 후 · 착륙 시 정지)", font=("Arial", 8, "bold"),
                 bg=SURFACE, fg=MUTED, anchor="w", wraplength=160, justify="left").pack(fill="x")
        self.uptime_lbl = tk.Label(tl2_inner, text="--:--:--", font=("Courier", 20, "bold"),
                                    fg=CYAN, bg=SURFACE)
        self.uptime_lbl.pack(pady=(4, 0), anchor="w")

        tk.Frame(tl2_inner, bg=BORDER, height=1).pack(fill="x", pady=12)
        tk.Label(tl2_inner, text="명령 누적 수신", font=("Arial", 8, "bold"),
                 bg=SURFACE, fg=MUTED, anchor="w").pack(fill="x")
        self.cmdrx_lbl = tk.Label(tl2_inner, text="—", font=("Courier", 22, "bold"),
                                   fg=CYAN, bg=SURFACE)
        self.cmdrx_lbl.pack(pady=(4, 0), anchor="w")
        tk.Label(tl2_inner, text="(상세 내역은 이벤트 로그 참고)", font=("Arial", 7),
                 bg=SURFACE, fg=MUTED, anchor="w", wraplength=160, justify="left").pack(fill="x")

    # ── 비상 사출 UI (2단계 확인 + 자동 재전송, gcs_v6.py 로직 그대로) ──────
    def _build_emer_idle(self):
        for w in self.emer_inner.winfo_children():
            w.destroy()
        self.emer_btn = tk.Button(
            self.emer_inner, text="⚠  비상 사출", font=("Courier", 16, "bold"), bg="#7a0000",
            fg="white", activebackground="#9a0000", activeforeground="white", bd=0,
            highlightthickness=2, highlightbackground=RED, cursor="hand2", pady=18,
            wraplength=260, state="disabled", command=self._emer_click)
        self.emer_btn.pack(fill="x", pady=(0, 10))
        self.emer_status = tk.Label(self.emer_inner, text="", font=("Courier", 12, "bold"),
                                     bg=SURFACE, fg=RED, wraplength=260, justify="center")
        self.emer_status.pack(fill="x")

    def _emer_click(self):
        if not self.link or self.eject_phase != 0:
            return
        dlg = tk.Toplevel(self)
        dlg.title("비상 사출 확인")
        dlg.configure(bg=SURFACE)
        dlg.resizable(False, False)
        dlg.grab_set()
        dlg.geometry("380x180")
        dlg.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() - 380) // 2
        y = self.winfo_y() + (self.winfo_height() - 180) // 2
        dlg.geometry(f"380x180+{x}+{y}")

        tk.Label(dlg, text="⚠  비상 사출을 정말 실행하시겠습니까?", font=("Courier", 11, "bold"),
                 bg=SURFACE, fg=RED, wraplength=340, justify="center").pack(pady=(18, 8))
        tk.Label(dlg, text="이 작업은 되돌릴 수 없습니다. 확인 즉시 자동 재전송이 시작됩니다.",
                 font=("Arial", 10), bg=SURFACE, fg=TEXT, wraplength=340,
                 justify="center").pack(pady=4)

        btn_row = tk.Frame(dlg, bg=SURFACE)
        btn_row.pack(pady=14)

        def on_confirm():
            dlg.destroy()
            self._start_eject_retry()

        tk.Button(btn_row, text="취소", font=("Arial", 10, "bold"), bg=SURFACE2, fg=MUTED, bd=0,
                  highlightthickness=1, highlightbackground=BORDER2, padx=20, pady=6,
                  cursor="hand2", command=dlg.destroy).pack(side="left", padx=6)
        tk.Button(btn_row, text="🔴 실행 확인", font=("Courier", 10, "bold"), bg=RED_DIM, fg=RED,
                  bd=0, highlightthickness=1, highlightbackground=RED, padx=20, pady=6,
                  cursor="hand2", command=on_confirm).pack(side="left", padx=6)

    def _start_eject_retry(self):
        self.eject_phase = 2
        self.retry_sent = 0
        self.retry_start_t = time.time()
        self.last_retry_t = 0.0
        for w in self.emer_inner.winfo_children():
            w.destroy()
        self.emer_progress_lbl = tk.Label(self.emer_inner, text="📡 자동 재전송 중...",
                                           font=("Courier", 10, "bold"), bg=SURFACE, fg=AMBER,
                                           wraplength=260, justify="center")
        self.emer_progress_lbl.pack(pady=(10, 6))
        tk.Button(self.emer_inner, text="🛑 재전송 중단", font=("Arial", 9, "bold"), bg=SURFACE2,
                  fg=MUTED, bd=0, highlightthickness=1, highlightbackground=BORDER2, padx=10,
                  pady=6, cursor="hand2", command=self._stop_eject_retry).pack(fill="x")

    def _stop_eject_retry(self):
        self.eject_phase = 0
        self._build_emer_idle()
        self._sync_emer_button_state()

    def _eject_confirmed_success(self):
        self.eject_phase = 0
        last = self.last_pkt
        reason = EJECT_REASON.get(last.eject_state) if last else None
        if self.retry_sent == 0:
            # 내가 보내기 전에 이미 eject_state가 세팅돼 있었다는 뜻 — 자동사출이 먼저 됐던 경우.
            # "내 명령이 통했다"처럼 오해되지 않게 문구를 정직하게 다르게 표시.
            detail = f"이미 사출 상태였음 — {reason}" if reason else "이미 사출 상태였음"
        else:
            detail = f"전송 {self.retry_sent}회 만에 확인 — {reason}" if reason else f"전송 {self.retry_sent}회 만에 확인"
        tk.Label(self.emer_inner, text=f"✅ 로켓 확인됨\n({detail})",
                 font=("Courier", 10, "bold"), bg=SURFACE, fg=GREEN, wraplength=260,
                 justify="center").pack(pady=(14, 4))
        tk.Label(self.emer_inner, text="펌웨어 기준 — 실제 낙하산 전개 확인 아님", font=("Arial", 7),
                 bg=SURFACE, fg=MUTED, wraplength=260, justify="center").pack(pady=(0, 8))
        tk.Button(self.emer_inner, text="확인 (닫기)", font=("Arial", 9, "bold"), bg=SURFACE2,
                  fg=MUTED, bd=0, highlightthickness=1, highlightbackground=BORDER2, padx=10,
                  pady=6, cursor="hand2", command=self._build_emer_idle).pack(fill="x")

    def _eject_timed_out(self):
        self.eject_phase = 0
        for w in self.emer_inner.winfo_children():
            w.destroy()
        tk.Label(self.emer_inner, text=f"⏱️ {MAX_RETRY_S:.0f}초 동안\n확인 안 됨 — 중단",
                 font=("Courier", 9, "bold"), bg=SURFACE, fg=RED, wraplength=260,
                 justify="center").pack(pady=(14, 8))
        tk.Button(self.emer_inner, text="확인 (닫기)", font=("Arial", 9, "bold"), bg=SURFACE2,
                  fg=MUTED, bd=0, highlightthickness=1, highlightbackground=BORDER2, padx=10,
                  pady=6, cursor="hand2", command=self._build_emer_idle).pack(fill="x")

    def _sync_emer_button_state(self):
        if self.eject_phase != 0:
            return
        # 사출 완료/타임아웃 처리 직후엔 emer_btn이 destroy된 상태로 잠깐 남아있을 수 있음
        # (사용자가 "확인" 눌러야 _build_emer_idle()로 재생성됨) — 이때 .config()를 부르면
        # TclError로 죽어서 _tick() 전체가 멈추는 버그가 있었음. winfo_exists()로 방지.
        if not self.emer_btn.winfo_exists():
            return
        mode = self.last_pkt.flight_mode if self.last_pkt else -1
        can_eject = bool(self.link) and mode not in (0, 4)
        self.emer_btn.config(state="normal" if can_eject else "disabled")

    # ── 모드 제어 명령 (되돌릴 수 없는 것만 확인창) ─────────────────────────
    def _send_mode_cmd(self, cmd, needs_confirm):
        if not self.link:
            return
        if not needs_confirm:
            self.link.send_cmd(cmd)
            return
        dlg = tk.Toplevel(self)
        dlg.title("비행 강제진입 확인")
        dlg.configure(bg=SURFACE)
        dlg.resizable(False, False)
        dlg.grab_set()
        dlg.geometry("360x160")
        dlg.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() - 360) // 2
        y = self.winfo_y() + (self.winfo_height() - 160) // 2
        dlg.geometry(f"360x160+{x}+{y}")
        tk.Label(dlg, text="⚠️ 비행 강제진입은 되돌릴 수 없습니다", font=("Courier", 10, "bold"),
                 bg=SURFACE, fg=RED, wraplength=320, justify="center").pack(pady=(18, 10))

        def on_confirm():
            self.link.send_cmd(cmd)
            dlg.destroy()

        btn_row = tk.Frame(dlg, bg=SURFACE)
        btn_row.pack(pady=10)
        tk.Button(btn_row, text="취소", font=("Arial", 10, "bold"), bg=SURFACE2, fg=MUTED, bd=0,
                  highlightthickness=1, highlightbackground=BORDER2, padx=20, pady=6,
                  cursor="hand2", command=dlg.destroy).pack(side="left", padx=6)
        tk.Button(btn_row, text="🔴 강제진입 확인", font=("Courier", 10, "bold"), bg=RED_DIM,
                  fg=RED, bd=0, highlightthickness=1, highlightbackground=RED, padx=20, pady=6,
                  cursor="hand2", command=on_confirm).pack(side="left", padx=6)

    def _on_close(self):
        if self.link:
            self.link.stop()
        try:
            self.log_file.close()
        except Exception:
            pass
        self.destroy()

    # ── 메인 루프 (100ms — 데이터 드레인 + 화면 갱신) ───────────────────────
    def _tick(self):
        now = time.time()

        if self.link:
            while True:
                pkt = self.link.read_packet()
                if pkt is None:
                    break
                if self.last_pkt and pkt.ms < self.last_pkt.ms - 5000:
                    self._reset_flight_data()
                    self._log_event("보드 재시작 감지 — 데이터 초기화")
                if self.last_seq >= 0:
                    gap = (pkt.seq - ((self.last_seq + 1) & 0xFFFF)) & 0xFFFF
                    if 0 < gap < 0x8000:
                        self.lost += gap
                self.last_seq = pkt.seq
                self.total += 1
                self.rate_ts.append(now)

                pm = self.prev_mode

                # 명령 수신 감지 — "명령 누적 수신 3"이 정확히 뭐였는지 이벤트 로그에 남김
                if pkt.cmd_rx_count != self.prev_cmd_rx_count and self.prev_cmd_rx_count is not None:
                    name = CMD_NAMES.get(pkt.last_cmd, f"알 수 없음(0x{pkt.last_cmd:02X})")
                    self._log_event(f"명령수신: {name} (누적 {pkt.cmd_rx_count}회)", color=AMBER)
                self.prev_cmd_rx_count = pkt.cmd_rx_count

                # 모드 전환 감지 — 이벤트 로그에 수동/자동 사유와 함께 기록
                if pkt.flight_mode != pm and pm != -1:
                    self._log_mode_transition(pm, pkt)

                # 발사 감지되는 순간 그래프 히스토리 초기화 (시간축 0초 기준점)
                if pkt.flight_mode == 2 and pm in (0, 1, -1) and self.launch_ref_ms is None:
                    self.t_hist, self.alt_hist, self.acc_hist = [], [], []
                    self.launch_ref_ms = pkt.ms
                    self.graph_frozen = False

                # 착륙하면 그래프를 그 상태로 고정 (더 이상 안 밀려나서 비행 프로파일이 남음)
                if pkt.flight_mode == 4 and pm != 4:
                    self.graph_frozen = True

                self.last_pkt = pkt

                if not self.graph_frozen:
                    t_val = ((pkt.ms - self.launch_ref_ms) / 1000.0
                             if self.launch_ref_ms is not None else pkt.ms / 1000.0)
                    self.t_hist.append(t_val)
                    self.alt_hist.append(pkt.altitude_cm / 100.0 if pkt.altitude_cm != ALT_INVALID else None)
                    self.acc_hist.append(pkt.acc[0])
                    if self.launch_ref_ms is None and len(self.t_hist) > HISTORY_MAX:
                        self.t_hist.pop(0); self.alt_hist.pop(0); self.acc_hist.pop(0)

                # 타임라인
                tl = self.timeline
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
                self.prev_mode = pkt.flight_mode

                self._write_log(pkt)

        self.rate_ts = [t for t in self.rate_ts if now - t < 1.0]
        rate = len(self.rate_ts)
        last = self.last_pkt

        # ── 비상사출 자동 재전송 엔진 ───────────────────────────
        if self.eject_phase == 2:
            confirmed = bool(last and last.eject_state != 0)
            timed_out = (now - self.retry_start_t) > MAX_RETRY_S
            if confirmed:
                self._eject_confirmed_success()
            elif timed_out:
                self._eject_timed_out()
            else:
                if self.link and now - self.last_retry_t >= RETRY_GAP_S:
                    self.link.send_cmd(CMD_FORCE_EJECT)
                    self.retry_sent += 1
                    self.last_retry_t = now
                if hasattr(self, 'emer_progress_lbl'):
                    self.emer_progress_lbl.config(
                        text=f"📡 재전송 중... {self.retry_sent}회\n확인 대기")

        # ── 상단바 ──────────────────────────────────────────────
        self.rx_lbl.config(text=f"{rate} pkt/s")
        cur_mode = last.flight_mode if last else -1
        for i, lbl in self.mode_labels.items():
            if i == cur_mode:
                lbl.config(bg=CYAN_DIM, fg=MODE_COLORS.get(i, CYAN), highlightbackground=CYAN)
            else:
                lbl.config(bg=SURFACE2, fg=MUTED, highlightbackground=BORDER2)
        self.clock_lbl.config(text=datetime.now().strftime("%H:%M:%S"))

        # ── 데이터 카드 + 센서 상태 ─────────────────────────────
        if last:
            s = last.system_status
            alt_txt = f"{last.altitude_cm/100:.1f}" if last.altitude_cm != ALT_INVALID else "오류"
            self.dcards["고도"].set(alt_txt, RED if last.altitude_cm == ALT_INVALID else CYAN)
            self.dcards["전압"].set(f"{last.voltage_mv/1000:.2f}" if last.voltage_mv != -1 else "—",
                                    RED if 0 < last.voltage_mv < 7000 else GREEN)
            self.dcards["가속X"].set(last.acc[0])
            self.dcards["가속Y"].set(last.acc[1])
            self.dcards["가속Z"].set(last.acc[2])

            if not (s & STATUS_SD):
                sd_text = "미장착"
            elif s & STATUS_LOG_CLOSED:
                sd_text = "종료"
            elif 1 <= cur_mode <= 3:
                sd_text = "기록중"
            else:
                sd_text = "대기"
            self.dcards["SD로그"].set(sd_text, GREEN if sd_text == "기록중" else MUTED)

            sensor_ok = {
                "BMP": bool(s & STATUS_BMP), "MPU": bool(s & STATUS_IMU),
                "SD": bool(s & STATUS_SD), "INA": bool(s & STATUS_INA),
                "기준압": not bool(s & STATUS_BASELINE_BAD),
            }
            for name, ok in sensor_ok.items():
                f, dot, lbl = self.sensor_frames[name]
                bg = GREEN_DIM if ok else RED_DIM
                fg = GREEN if ok else RED
                f.config(bg=bg, highlightbackground=fg)
                dot.config(bg=bg, fg=fg)
                lbl.config(bg=bg, fg=fg)

            servo_on = cur_mode not in (0, -1)
            self.servo_lbl.config(text="ON" if servo_on else "OFF",
                                   bg=GREEN_DIM if servo_on else SURFACE2,
                                   fg=GREEN if servo_on else MUTED,
                                   highlightbackground=GREEN if servo_on else BORDER2)

            self.cmdrx_lbl.config(text=str(last.cmd_rx_count))
            self.attitude.update_tilt(last.tilt_xy / 100.0, last.tilt_xz / 100.0)

            if last.eject_state and self.eject_phase == 0 and self.emer_status.winfo_exists():
                reason = EJECT_REASON.get(last.eject_state, f"알 수 없음({last.eject_state})")
                self.emer_status.config(text=f"🪂 사출됨 — {reason}")

        self.alt_spark.update_data(self.alt_hist, self.t_hist)
        self.ax_spark.update_data(self.acc_hist, self.t_hist)

        # 모드 버튼 활성/비활성
        for cmd, (btn, req_mode) in self.mode_btns.items():
            enabled = bool(self.link) and cur_mode == req_mode
            btn.config(state="normal" if enabled else "disabled")
        self._sync_emer_button_state()

        # 타임라인
        def fmt(t):
            return datetime.fromtimestamp(t).strftime('%H:%M:%S') if t else '--:--:--'
        self.tl_vals["발사시각"].config(text=fmt(self.timeline['launch']))
        self.tl_sub["발사시각"].config(text=self.timeline.get('launch_reason') or "")
        self.tl_vals["사출시각"].config(text=fmt(self.timeline['eject']))
        self.tl_vals["착륙시각"].config(text=fmt(self.timeline['land']))

        # 경과시간 — 발사 후 기준, 착륙하면 그 시점 값으로 정지 (그래프 freeze와 동일 시점)
        if last and self.launch_ref_ms is not None and not self.graph_frozen:
            elapsed_s = max(0, (last.ms - self.launch_ref_ms) // 1000)
            hh, rem = divmod(elapsed_s, 3600)
            mm, ss = divmod(rem, 60)
            self.uptime_lbl.config(text=f"{hh:02d}:{mm:02d}:{ss:02d}")

        self.after(100, self._tick)

    def _write_log(self, pkt: Pkt):
        self.log_writer.writerow({
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
        self.log_file.flush()


if __name__ == "__main__":
    app = RocketGCS()
    app.mainloop()
