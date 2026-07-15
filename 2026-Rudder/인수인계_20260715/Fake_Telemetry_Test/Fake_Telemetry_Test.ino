// 가짜 텔레메트리 송신기 — 센서 없이 SiK ↔ GCS 통신 체인 테스트용
//
// 배선: SiK TX → D0(RX1), SiK RX → D1(TX1), SiK 5V/GND
// PC쪽: 지상측 SiK를 USB에 꽂고 GCS 사이드바에서 해당 COM 포트 선택
//
// 가상 시나리오(GCS VirtualFlightSource와 동일):
//   0~5s 대기 → 5~10s Armed → 10s 발사 → 18.5s 정점/사출 → 78s 착지

#include <Arduino.h>
#include <math.h>

// ── Packet.h와 완전히 동일한 구조 (47 bytes) ─────────────────────────────
struct __attribute__((packed)) DataPacket {
  uint16_t stx     = 0xAA55;
  uint8_t  from    = 1;
  uint16_t seq     = 0;
  uint8_t  id      = 1;
  uint16_t len     = 0;
  uint32_t ms;
  int16_t  acc[3];
  int16_t  gyro[3];
  int32_t  pressure;
  int16_t  bmp_temp;
  int32_t  altitude_cm;
  int16_t  voltage_mv;
  int16_t  current_ma;
  uint8_t  flight_mode;
  uint8_t  eject_state;
  uint8_t  system_status;
  uint16_t pkt_no  = 0;
  uint16_t crc16;
  uint16_t etx     = 0x55AA;
};

static DataPacket pkt;

// Logger.cpp와 동일한 CRC16-CCITT
static uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (uint16_t)(crc << 1);
  }
  return crc;
}

// 가상 비행 프로파일: 발사 후 tf초의 (고도 m, Z가속도 mg)
static void profile(float tf, float &altM, int16_t &accZ) {
  const float T = 8.51f, H = 271.0f, DUR = 68.0f;
  if (tf <= 0)      { altM = 0; accZ = 1000; return; }
  if (tf <= T) {
    float r = (tf - T) / T;
    altM = H * (1.0f - r * r);
    accZ = (tf < 5.0f) ? (int16_t)(1000 + 2500 * sinf(tf / 5.0f * PI))
                       : (int16_t)(-200 - 80 * (tf - 5.0f));
  } else if (tf < DUR) {
    float ratio = (DUR - tf) / (DUR - T);
    altM = H * powf(ratio, 0.55f);
    accZ = (int16_t)(-80 - random(0, 30));
  } else {
    altM = 0;
    accZ = 1000 + random(-100, 100);   // 착지
  }
}

void setup() {
  Serial.begin(115200);          // USB 디버그 출력
  Serial1.begin(57600);          // SiK (BAUD_TELEMETRY)
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Serial.println("=== 가짜 텔레메트리 송신 시작 (5Hz) ===");
  Serial.print("packet size = "); Serial.println(sizeof(DataPacket));  // 47이어야 함
}

void loop() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 200) return;   // 5Hz
  lastMs = now;

  float el = now / 1000.0f;         // 부팅 후 경과 초
  float tf = el - 10.0f;            // 발사 후 경과 초

  uint8_t mode;
  bool    ejected = false;
  if      (el < 5.0f)   mode = 0;
  else if (el < 10.0f)  mode = 1;
  else if (tf >= 68.0f) mode = 4;
  else                  mode = 2;

  float   altM; int16_t accZ;
  profile(tf, altM, accZ);
  if (mode >= 2 && tf >= 8.51f) { ejected = true; if (mode == 2) mode = 3; }

  pkt.seq++;
  pkt.pkt_no++;
  pkt.ms          = now;
  pkt.acc[0]      = random(-60, 60);
  pkt.acc[1]      = random(-60, 60);
  pkt.acc[2]      = accZ;
  pkt.gyro[0]     = random(-5, 5);
  pkt.gyro[1]     = random(-5, 5);
  pkt.gyro[2]     = random(-5, 5);
  pkt.pressure    = (int32_t)(101325 - altM * 12.2f);
  pkt.bmp_temp    = (int16_t)(2500 - altM * 0.65f);
  pkt.altitude_cm = (int32_t)(altM * 100) + random(-20, 20);
  pkt.voltage_mv  = 7400 - (int16_t)(el * 0.4f);
  pkt.current_ma  = 500 + random(-20, 20);
  pkt.flight_mode   = mode;
  pkt.eject_state   = ejected ? 1 : 0;
  pkt.system_status = 0x0F;
  pkt.len   = sizeof(DataPacket);
  pkt.crc16 = crc16((uint8_t*)&pkt, offsetof(DataPacket, crc16));

  Serial1.write((const uint8_t*)&pkt, sizeof(pkt));

  // USB로도 상태 출력 (SiK 없이도 송신 동작 확인 가능)
  Serial.print("[TX] seq="); Serial.print(pkt.seq);
  Serial.print(" mode=");    Serial.print(mode);
  Serial.print(" alt=");     Serial.print(altM, 1);
  Serial.print("m eject=");  Serial.println(pkt.eject_state);
}
