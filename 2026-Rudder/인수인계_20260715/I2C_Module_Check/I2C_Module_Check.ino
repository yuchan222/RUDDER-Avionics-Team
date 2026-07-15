// I2C 모듈 통합 체커 — 한 번 업로드해두고 센서를 하나씩 꽂으면서 확인
// 2초마다 전체 스캔 후 모듈별 O/X 표시. 재업로드 불필요.

#include <Wire.h>

struct ModuleDef {
  const char *name;
  uint8_t     addr1;
  uint8_t     addr2;   // 대체 주소 (없으면 0)
};

// version2 Config.h와 동일한 주소
static const ModuleDef MODULES[] = {
  { "MPU-6050 (IMU)   ", 0x68, 0x69 },   // AD0=HIGH면 0x69
  { "BMP388  (기압계) ", 0x76, 0x77 },   // SDO에 따라 0x76/0x77
  { "INA219  (전류계) ", 0x40, 0x00 },
};
static const uint8_t N_MODULES = sizeof(MODULES) / sizeof(MODULES[0]);

static bool probe(uint8_t addr) {
  if (addr == 0) return false;
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Wire.begin();
  Serial.println("=== I2C 모듈 통합 체커 (2초마다 자동 스캔) ===");
}

void loop() {
  Serial.println("---------------------------------------------");

  uint8_t foundCount = 0;
  for (uint8_t i = 0; i < N_MODULES; i++) {
    bool hit1 = probe(MODULES[i].addr1);
    bool hit2 = !hit1 && probe(MODULES[i].addr2);

    Serial.print(MODULES[i].name);
    if (hit1 || hit2) {
      Serial.print("  [O] 인식됨  0x");
      Serial.println(hit1 ? MODULES[i].addr1 : MODULES[i].addr2, HEX);
      foundCount++;
    } else {
      Serial.println("  [X] 없음");
    }
  }

  // 목록에 없는 주소가 잡히면 알려줌 (배선 실수/주소 변경 감지용)
  for (uint8_t addr = 1; addr < 127; addr++) {
    bool known = false;
    for (uint8_t i = 0; i < N_MODULES; i++)
      if (addr == MODULES[i].addr1 || addr == MODULES[i].addr2) known = true;
    if (!known && probe(addr)) {
      Serial.print("기타 장치            [?] 발견     0x");
      Serial.println(addr, HEX);
    }
  }

  Serial.print(">> ");
  Serial.print(foundCount);
  Serial.print(" / ");
  Serial.print(N_MODULES);
  Serial.println(" 모듈 인식");

  delay(2000);
}
