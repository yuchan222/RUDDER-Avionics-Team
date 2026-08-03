"""
RUDDER 2026 — SiK 라디오 장거리(레인지) 테스트

두 대의 노트북에 SiK 라디오를 하나씩 USB로 꽂고, 양쪽에서 이 스크립트를
동시에 실행하면 서로 번호 붙은 핑을 주고받으며 수신율(손실률)을 실시간으로
보여준다. 아두이노 불필요 — SiK는 USB로 꽂으면 그 자체로 시리얼 포트가 됨.

사용법 (양쪽 노트북에서 각각):
  python sik_range_test.py --port COM7 --name yuchan
  python sik_range_test.py --port COM5 --name kiuk

포트 확인이 안 되면 --port 없이 실행 → 감지된 포트 목록을 보여주고 종료.

화면에 뜨는 숫자:
  TX      : 내가 지금까지 보낸 핑 개수
  RX      : 상대에게서 받은 핑 개수
  손실률   : 최근 20개 윈도우 기준 수신 실패율(%) — 이게 핵심 지표
  RTT     : 내가 보낸 핑에 대한 상대 응답(에코) 왕복 시간 [ms]

Ctrl+C로 종료.
"""

import argparse
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit('pyserial이 필요합니다: pip install pyserial')

BAUD = 57600          # BAUD_TELEMETRY와 동일
PING_INTERVAL = 0.5   # 초당 2회
WINDOW = 20            # 손실률 계산 윈도우 (최근 N개 TX 기준)


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print('감지된 시리얼 포트가 없습니다. SiK가 USB로 꽂혀 있는지 확인하세요.')
        return
    print('감지된 포트:')
    for p in ports:
        print(f'  {p.device} — {p.description}')


def main():
    ap = argparse.ArgumentParser(description='SiK 라디오 장거리 테스트 (무전기 모드)')
    ap.add_argument('--port', help='이 노트북에 꽂힌 SiK의 COM 포트 (예: COM7)')
    ap.add_argument('--name', default='node', help='내 이름표 (상대 화면에 표시됨)')
    ap.add_argument('--interval', type=float, default=PING_INTERVAL, help='핑 전송 주기(초)')
    args = ap.parse_args()

    if not args.port:
        list_ports()
        return

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    print(f'[{args.name}] {args.port} 열림 @ {BAUD}bps. 상대 노트북에서도 실행하세요.')
    print('Ctrl+C로 종료\n')

    my_seq = 0
    rx_count = 0
    tx_recent = []      # 최근 전송한 seq들의 ack 수신 여부 (윈도우)
    pending = {}        # seq -> 보낸 시각(초) — RTT 계산용
    last_rtt_ms = None
    lock = threading.Lock()

    def rx_loop():
        nonlocal rx_count, last_rtt_ms
        buf = b''
        while True:
            buf += ser.read(256)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                try:
                    text = line.decode('ascii', errors='ignore').strip()
                except Exception:
                    continue
                if not text:
                    continue
                parts = text.split(',')
                with lock:
                    if parts[0] == 'PING' and len(parts) == 3:
                        # 상대 핑 수신 → 그대로 ACK 에코
                        rx_count += 1
                        ser.write(f'ACK,{parts[1]},{parts[2]}\n'.encode('ascii'))
                        print(f'  <- PING from {parts[1]} seq={parts[2]}  (echo 보냄)')
                    elif parts[0] == 'ACK' and len(parts) == 3 and parts[1] == args.name:
                        seq = int(parts[2])
                        if seq in pending:
                            last_rtt_ms = (time.monotonic() - pending.pop(seq)) * 1000.0
                            for i, (s, ok) in enumerate(tx_recent):
                                if s == seq:
                                    tx_recent[i] = (s, True)
                                    break

    threading.Thread(target=rx_loop, daemon=True).start()

    try:
        while True:
            with lock:
                my_seq += 1
                seq = my_seq
                pending[seq] = time.monotonic()
                tx_recent.append((seq, False))
                if len(tx_recent) > WINDOW:
                    old_seq, _ = tx_recent.pop(0)
                    pending.pop(old_seq, None)

                loss_pct = 0.0
                if len(tx_recent) >= 3:
                    acked = sum(1 for _, ok in tx_recent if ok)
                    loss_pct = 100.0 * (1 - acked / len(tx_recent))

            ser.write(f'PING,{args.name},{seq}\n'.encode('ascii'))

            rtt_str = f'{last_rtt_ms:.0f}ms' if last_rtt_ms is not None else '—'
            print(f'[{args.name}] TX={seq:4d}  RX={rx_count:4d}  '
                  f'손실률(최근{WINDOW}개)={loss_pct:5.1f}%  RTT={rtt_str}')

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\n종료.')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
