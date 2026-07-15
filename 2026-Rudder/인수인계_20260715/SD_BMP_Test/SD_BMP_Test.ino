#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define PIN_SD_CS    10
#define BMP388_ADDR  0x76   // 실패 시 0x77 자동재시도

SdFat            sd;
Adafruit_BMP3XX  bmp;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // R4 Minima USB CDC 연결 대기

  Serial.println();
  Serial.println("=== SD / BMP388 연결 테스트 ===");

  // ── SD 테스트 ──────────────────────────────────────────
  if (sd.begin(PIN_SD_CS, SD_SCK_MHZ(4))) {
    Serial.println("[OK] SD init");
    FsFile f = sd.open("test.txt", O_WRONLY | O_CREAT | O_TRUNC);
    if (f) {
      f.println("hello");
      f.close();
      Serial.println("[OK] SD write (test.txt 생성됨)");
    } else {
      Serial.println("[ERR] SD write 실패");
    }
  } else {
    Serial.println("[ERR] SD init 실패 — CS/MOSI/MISO/SCK/VCC/GND 배선 확인");
  }

  // ── BMP388 테스트 ──────────────────────────────────────
  Wire.begin();
  if (bmp.begin_I2C(BMP388_ADDR) || bmp.begin_I2C(0x77)) {
    Serial.println("[OK] BMP388 init (0x76 또는 0x77)");
  } else {
    Serial.println("[ERR] BMP388 init 실패 — SDA/SCL/VCC/GND, CSB/SDO 배선 확인");
  }

  Serial.println("=== 테스트 끝, 1초마다 BMP 값 출력 시작 ===");
}

void loop() {
  // 초기화 실패해도 루프가 무조건 뭔가 찍도록 하트비트 보장
  // (USB CDC 연결 타이밍 때문에 setup() 로그를 놓쳐도 생사 확인 가능하게)
  if (bmp.performReading()) {
    Serial.print("[BMP] Pressure="); Serial.print(bmp.pressure);
    Serial.print(" Pa  Temp=");      Serial.print(bmp.temperature);
    Serial.println(" C");
  } else {
    Serial.println("[BMP] read 실패 (init 안 된 상태) — 그래도 보드는 살아있음");
  }
  delay(1000);
}
