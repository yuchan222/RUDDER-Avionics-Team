// 4채널 레벨시프터 테스트 — I2C 신호를 시프터에 관통시켜 검증
//
// 시프터는 I2C 주소가 없으므로, 센서 하나(MPU 등)를 시프터 건너편에 두고
// 그 센서가 인식되는지로 "신호가 양방향으로 통과하는가"를 판정한다.
//
// 배선:
//   아두이노 A4 → HV1        아두이노 A5 → HV2
//   아두이노 5V → HV         아두이노 GND → GND(HV쪽)
//   아두이노 3.3V → LV       (LV쪽 GND는 보드 내부 공통이라 생략 가능)
//   센서 SDA → LV1           센서 SCL → LV2
//   센서 VCC → 5V/3.3V       센서 GND → GND
//
// 판정: 아래 출력에서 센서 주소가 [O]로 뜨면 시프터 정상!

#include <Wire.h>

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Wire.begin();
  Serial.println("=== 레벨시프터 관통 테스트 ===");
  Serial.println("센서(MPU/BMP/INA 아무거나)를 시프터 LV1/LV2 건너편에 연결하세요.");
}

void loop() {
  uint8_t found = 0;
  Serial.println("---------------------------------------------");

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (probe(addr)) {
      Serial.print("[O] 0x");
      Serial.print(addr, HEX);
      if      (addr == 0x68 || addr == 0x69) Serial.print("  (MPU-6050)");
      else if (addr == 0x76 || addr == 0x77) Serial.print("  (BMP388)");
      else if (addr == 0x40)                 Serial.print("  (INA219)");
      Serial.println("  ← 시프터 통과 성공!");
      found++;
    }
  }

  if (found == 0)
    Serial.println("[X] 감지 없음 — HV/LV 전원(5V/3.3V), HV1↔LV1 채널 배선 확인");

  delay(2000);
}
