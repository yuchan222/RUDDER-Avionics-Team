"""
RUDDER 2026 — SD 로그 디코더

비행 후 SD카드의 LOG*.RAW (47바이트 바이너리 DataPacket 연속 기록)를
CSV로 변환하고 비행 요약을 출력한다.

사용법:
  python decode_sd_log.py LOG00001.RAW              # CSV 변환 + 요약
  python decode_sd_log.py LOG00001.RAW --plot       # + 고도/가속도 그래프 PNG 저장
  python decode_sd_log.py E:\\                       # 폴더를 주면 안의 LOG*.RAW 전부 처리

출력:
  입력과 같은 위치에 LOG00001.csv (+ --plot 시 LOG00001.png)

패킷 포맷: Packet.h __attribute__((packed)) 와 동일 (총 52 bytes, Rudder2026 펌웨어)
CRC16   : Logger.cpp와 동일 (CCITT, init 0xFFFF, poly 0x1021, crc16 필드 직전까지)

2026-08-02: version6에 last_cmd 필드 추가되며 47B→48B로 변경됨.
2026-08-05: version8(현 Rudder2026)에 tilt_xy/tilt_xz(자세추정) 필드 추가되며 48B→52B로 변경됨.
version_final 이하(48B 이하) 로그는 이 버전으로 디코딩하면 깨짐 — 필요하면 git
히스토리에서 이전 버전의 스크립트를 참고할 것.
"""

import argparse
import csv
import pathlib
import struct
import sys

PKT_FMT   = '<HBHBHIhhhhhhihihhBBBHBhhHH'
PKT_SIZE  = struct.calcsize(PKT_FMT)          # 52
CRC_OFFSET = 48                               # offsetof(DataPacket, crc16)
STX        = 0xAA55
ETX        = 0x55AA
STX_BYTES  = struct.pack('<H', STX)
ALT_INVALID = -2147483648   # Config.h의 ALT_INVALID(INT32_MIN)와 동일 — BMP 실패 표시
GAP_WARN_MS = 2000          # 연속 패킷 간 이 이상 시간차 나면 기록공백으로 경고

FLIGHT_MODES = {0: 'Standby', 1: 'Armed', 2: 'Flight', 3: 'Ejected', 4: 'Landed'}
EJECT_REASONS = {0: '-', 1: 'Primary(apogee)', 2: 'Backup(10s timer)', 3: 'Manual(ground cmd)'}

FIELDS = [
    'seq', 'pkt_no', 'ms', 't_sec',
    'flight_mode', 'eject_state',
    'acc_x_mg', 'acc_y_mg', 'acc_z_mg',
    'gyro_x_ddps', 'gyro_y_ddps', 'gyro_z_ddps',
    'pressure_pa', 'bmp_temp_c', 'altitude_m',
    'voltage_v', 'current_ma', 'system_status', 'last_cmd',
    'tilt_xy_deg', 'tilt_xz_deg', 'crc_ok',
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_file(path: pathlib.Path, make_plot: bool) -> None:
    raw = path.read_bytes()
    print(f'\n=== {path.name} ({len(raw):,} bytes) ===')

    rows, crc_bad, etx_bad = [], 0, 0
    i = 0
    while True:
        i = raw.find(STX_BYTES, i)
        if i < 0 or i + PKT_SIZE > len(raw):
            break
        chunk = raw[i:i + PKT_SIZE]
        v = struct.unpack(PKT_FMT, chunk)
        if v[-1] != ETX:                      # ETX 불일치 → 1바이트 밀고 재동기화
            etx_bad += 1
            i += 1
            continue
        crc_ok = crc16_ccitt(chunk[:CRC_OFFSET]) == v[24]
        if not crc_ok:
            crc_bad += 1

        alt_cm = v[14]
        rows.append({
            'seq': v[2], 'pkt_no': v[20], 'ms': v[5], 't_sec': round(v[5] / 1000.0, 3),
            'flight_mode': v[17], 'eject_state': v[18],
            'acc_x_mg': v[6], 'acc_y_mg': v[7], 'acc_z_mg': v[8],
            'gyro_x_ddps': v[9], 'gyro_y_ddps': v[10], 'gyro_z_ddps': v[11],
            'pressure_pa': v[12], 'bmp_temp_c': round(v[13] / 100.0, 2),
            'altitude_m': round(alt_cm / 100.0, 2) if alt_cm != ALT_INVALID else '',   # ALT_INVALID = BMP 실패
            'voltage_v': round(v[15] / 1000.0, 3), 'current_ma': v[16],
            'system_status': f'0x{v[19]:02X}', 'last_cmd': f'0x{v[21]:02X}',
            'tilt_xy_deg': round(v[22] / 100.0, 2), 'tilt_xz_deg': round(v[23] / 100.0, 2),
            'crc_ok': int(crc_ok),
        })
        i += PKT_SIZE

    if not rows:
        print('  [!] 유효한 패킷이 없습니다 (빈 로그이거나 다른 형식)')
        return

    # ── CSV 저장 ─────────────────────────────────────────────
    out_csv = path.with_suffix('.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    # ── 비행 요약 ────────────────────────────────────────────
    t0, t1 = rows[0]['t_sec'], rows[-1]['t_sec']
    alts   = [r['altitude_m'] for r in rows if r['altitude_m'] != '']
    accz   = [r['acc_z_mg'] for r in rows]
    modes_seen = []
    for r in rows:
        if not modes_seen or modes_seen[-1][0] != r['flight_mode']:
            modes_seen.append((r['flight_mode'], r['t_sec']))
    eject_row = next((r for r in rows if r['eject_state'] != 0), None)

    # 연속 패킷 간 기록공백(시간 점프) 탐지 — SD 기록이 끊겼던 구간을 바로 알 수 있게
    gaps = []
    for a, b in zip(rows, rows[1:]):
        dt_ms = b['ms'] - a['ms']
        if dt_ms >= GAP_WARN_MS:
            gaps.append((a['t_sec'], b['t_sec'], dt_ms / 1000.0))

    print(f'  패킷 {len(rows):,}개  (CRC 오류 {crc_bad}, 동기화 실패 {etx_bad})')
    print(f'  기록 구간: {t0:.2f}s ~ {t1:.2f}s  (총 {t1 - t0:.2f}s)')
    if alts:
        print(f'  최고 고도: {max(alts):.2f} m   /  BMP 실패 샘플: {len(rows) - len(alts)}개')
    print(f'  최대 Z가속도: {max(accz)} mg  /  최소: {min(accz)} mg')

    # ── 전압 스파이크/드롭 탐지 ──────────────────────────────
    volts = [(r['t_sec'], r['voltage_v']) for r in rows if r['voltage_v'] > 0]
    if volts:
        vmin = min(v for _, v in volts)
        vmax = max(v for _, v in volts)
        print(f'  전압 범위: {vmin:.3f}V ~ {vmax:.3f}V')
        jumps = []
        for (ta, va), (tb, vb) in zip(volts, volts[1:]):
            if abs(vb - va) >= 0.5:   # 샘플 간 0.5V 이상 급변 — 스파이크/드롭 의심
                jumps.append((ta, tb, va, vb))
        if jumps:
            print(f'  [!] 전압 급변 {len(jumps)}건 (샘플 간 0.5V 이상):')
            for ta, tb, va, vb in jumps:
                arrow = '급상승' if vb > va else '급하락'
                print(f'      {ta:.2f}s→{tb:.2f}s   {va:.3f}V → {vb:.3f}V  ({arrow})')
        else:
            print('  전압 급변(0.5V 이상 순간 변화) 없음')
    print(f'  모드 전환: ' + '  ->  '.join(
        f'{FLIGHT_MODES.get(m, m)}@{t:.1f}s' for m, t in modes_seen))
    if eject_row is not None:
        reason = EJECT_REASONS.get(eject_row['eject_state'], f"unknown({eject_row['eject_state']})")
        print(f"  사출 시각: {eject_row['t_sec']:.2f}s  [{reason}]")
    if gaps:
        print(f'  [!] 기록공백 {len(gaps)}건 (연속 패킷 간 {GAP_WARN_MS/1000:.0f}초 이상 점프):')
        for g0, g1, dur in gaps:
            print(f'      {g0:.2f}s -> {g1:.2f}s  (공백 {dur:.1f}초)')
    print(f'  CSV 저장: {out_csv}')

    # ── 그래프 (선택) ────────────────────────────────────────
    if make_plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('  [!] matplotlib 미설치 — "pip install matplotlib" 후 다시 시도')
            return
        ts = [r['t_sec'] for r in rows]
        alt_series = [(r['altitude_m'] if r['altitude_m'] != '' else None) for r in rows]
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
        ax1.plot(ts, alt_series, lw=1.2)
        ax1.set_ylabel('Altitude [m]')
        ax1.grid(alpha=.3)
        ax2.plot(ts, accz, lw=0.9, color='tab:red')
        ax2.axhline(1000, ls='--', lw=0.8, color='gray')
        ax2.set_ylabel('Acc Z [mg]')
        ax2.set_xlabel('Time [s]')
        ax2.grid(alpha=.3)
        if eject_row is not None:
            eject_t = eject_row['t_sec']
            for ax in (ax1, ax2):
                ax.axvline(eject_t, color='tab:green', ls=':', lw=1.2)
            ax1.set_title(f'{path.name}  (eject @ {eject_t:.2f}s)')
        else:
            ax1.set_title(path.name)
        out_png = path.with_suffix('.png')
        fig.tight_layout()
        fig.savefig(out_png, dpi=130)
        print(f'  그래프 저장: {out_png}')


def main() -> None:
    ap = argparse.ArgumentParser(description='RUDDER SD 로그(LOG*.RAW) -> CSV 디코더')
    ap.add_argument('target', help='LOG*.RAW 파일 또는 폴더(예: SD카드 드라이브 E:\\)')
    ap.add_argument('--plot', action='store_true', help='고도/가속도 그래프 PNG 저장')
    args = ap.parse_args()

    target = pathlib.Path(args.target)
    if target.is_dir():
        files = sorted(target.glob('LOG*.RAW')) + sorted(target.glob('LOG*.raw'))
        if not files:
            sys.exit(f'폴더에 LOG*.RAW 파일이 없습니다: {target}')
    elif target.is_file():
        files = [target]
    else:
        sys.exit(f'경로를 찾을 수 없습니다: {target}')

    for f in files:
        decode_file(f, args.plot)


if __name__ == '__main__':
    main()
