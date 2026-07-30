#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>

// 47바이트 텔레메트리 패킷 — version2와 바이트 배치 100% 동일 (GCS/디코더 호환)
// 변경점 딱 하나: 구 pkt_no 필드(seq와 항상 같은 값이라 중복이었음)를
// cmd_rx_count(유효 명령 누적 수신 횟수)로 재활용.
// → GCS가 "내가 보낸 비상사출 명령을 로켓이 몇 번 받았는지" 확인 가능 (자동 재전송의 근거)
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
  uint8_t  eject_state;          // 0=대기, 1=사출완료
  uint8_t  system_status;        // STATUS_BMP|IMU|SD|INA
  uint16_t cmd_rx_count = 0;     // (구 pkt_no 자리) 유효 명령 누적 수신 횟수
  uint16_t crc16;
  uint16_t etx     = 0x55AA;
};

#endif
