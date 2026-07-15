// INA219 단독 테스트 — 연결 상태 + 실시간 전압/전류 출력
// 배선: VCC→5V, GND→GND, SDA→A4, SCL→A5
// 참고: 전압/전류 값은 VIN+/VIN- 단자에 측정 대상(배터리 등)을 연결해야 유효.
//       아무것도 안 물리면 0V/0mA 근처가 나오는 게 정상 (I2C 인식만 확인하면 됨)

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x40);
static bool s_inited = false;

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Wire.begin();
  Serial.println("=== INA219 단독 테스트 ===");
}

void loop() {
  if (!s_inited) {
    if (!probe(0x40)) {
      Serial.println("[X] I2C 미인식 (0x40) — VCC/GND/SDA(A4)/SCL(A5) 배선 확인");
    } else if (ina219.begin()) {
      Serial.println("[O] INA219 초기화 성공! 주소 0x40");
      s_inited = true;
    } else {
      Serial.println("[!] 0x40 응답은 있으나 init 실패 — 납땜 확인");
    }
    delay(1000);
    return;
  }

  Serial.print("[INA] bus=");   Serial.print(ina219.getBusVoltage_V(), 3);
  Serial.print(" V  shunt=");   Serial.print(ina219.getShuntVoltage_mV(), 2);
  Serial.print(" mV  current=");Serial.print(ina219.getCurrent_mA(), 1);
  Serial.print(" mA  power=");  Serial.print(ina219.getPower_mW(), 1);
  Serial.println(" mW");

  if (!probe(0x40)) {
    Serial.println("[X] 연결 끊김 감지 — 재탐색 시작");
    s_inited = false;
  }
  delay(500);
}
