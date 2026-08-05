#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>

// 52바이트 텔레메트리 패킷 (2026-08-05: 자세추정 필드 tilt_xy/tilt_xz 4바이트
// 추가 — 48B였던 version_final에서 더 늘어남. GCS/decode_sd_log.py도 함께
// 갱신 필요, version_final 이하와는 바이트 호환 안 됨 — version8 실험 전용.
// 구 pkt_no 필드는 cmd_rx_count(유효 명령 누적 수신 횟수)로 재활용.
struct __attribute__((packed)) DataPacket {
  uint16_t stx     = 0xAA55;
  uint8_t  from    = 1;
  uint16_t seq     = 0;          // 패킷 일련번호 (GCS 손실률 계산용)
  uint8_t  id      = 1;
  uint16_t len     = 0;          // finalizePacket()에서 채움
  uint32_t ms;                   // 부팅 후 경과 [ms]
  int16_t  acc[3];               // [mg]
  int16_t  gyro[3];              // [0.1 dps]
  int32_t  pressure;             // [Pa], -1 = BMP 실패
  int16_t  bmp_temp;             // [0.01 °C]
  int32_t  altitude_cm;          // 지면 기준 상대고도 [cm], ALT_INVALID(Config.h) = BMP 실패
  int16_t  voltage_mv;           // 배터리 전압 [mV] (INA219), -1 = 미장착
  int16_t  current_ma;           // 전류 [mA], -1 = 미장착
  uint8_t  flight_mode;          // 0대기 1준비 2비행 3낙하 4착륙
  uint8_t  eject_state;          // 사출 사유코드 (Config.h EJECT_REASON_*) — 0=미사출
  uint8_t  system_status;        // STATUS_BMP|IMU|SD|INA|BASELINE_BAD|LOG_CLOSED|LAUNCH_ACCEL|LAUNCH_ALT
  uint16_t cmd_rx_count = 0;     // (구 pkt_no 자리) 유효 명령 누적 수신 횟수
  uint8_t  last_cmd = 0;         // 가장 최근 수신한 명령 바이트 (사후분석용)
  int16_t  tilt_xy = 0;          // [0.01 deg] X-Y 평면 기울기 (상보필터, 실험용)
  int16_t  tilt_xz = 0;          // [0.01 deg] X-Z 평면 기울기 (상보필터, 실험용)
  uint16_t crc16;
  uint16_t etx     = 0x55AA;
};

#endif
