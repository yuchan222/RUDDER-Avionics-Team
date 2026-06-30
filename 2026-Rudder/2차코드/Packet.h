#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>

struct __attribute__((packed)) DataPacket {
  uint16_t stx     = 0xAA55;
  uint8_t  from    = 1;
  uint16_t seq     = 0;
  uint8_t  id      = 1;
  uint16_t len     = 0;          // finalizePacket()에서 sizeof(*this)로 채움
  uint32_t ms;
  int16_t  acc[3];               // [mg] (×1000)
  int16_t  gyro[3];              // [0.1 dps] (×10) — ±500 dps 범위 int16_t 안전
  int32_t  pressure;             // [Pa]
  int16_t  bmp_temp;             // [0.01 °C]
  int32_t  altitude_cm;          // 지면 기준 상대 고도 [cm], -1 = BMP 실패
  int16_t  voltage_mv;           // 배터리 전압 [mV] (INA219), -1 = 미사용
  int16_t  current_ma;           // 전류 [mA] (INA219), -1 = 미사용
  uint8_t  flight_mode;          // 현재 비행 모드 (0~4)
  uint8_t  eject_state;          // 0=대기, 1=사출완료
  uint8_t  system_status;        // STATUS_BMP|STATUS_IMU|STATUS_SD|STATUS_INA
  uint16_t pkt_no  = 0;
  uint16_t crc16;
  uint16_t etx     = 0x55AA;
};

// 삭제: ConfirmPacket, RxPacket1 — 미사용 구조체 제거 (#16)
// 삭제: quat[4]                  — MPU6050 자세추정 미사용 (#17)

#endif
