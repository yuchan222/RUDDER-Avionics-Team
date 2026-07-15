// SD카드 단독 테스트 — 연결 상태 + 실시간 쓰기 검증
// 배선: CS→10, MOSI→11, MISO→12, SCK→13, VCC→5V, GND→GND

#include <SPI.h>
#include <SdFat.h>

#define PIN_SD_CS 10

SdFat  sd;
static bool     s_inited     = false;
static uint32_t s_writeCount = 0;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Serial.println("=== SD카드 단독 테스트 ===");
}

void loop() {
  if (!s_inited) {
    if (sd.begin(PIN_SD_CS, SD_SCK_MHZ(4))) {
      Serial.println("[O] SD init 성공!");
      s_inited     = true;
      s_writeCount = 0;
    } else {
      Serial.println("[X] SD init 실패 — CS(10)/MOSI(11)/MISO(12)/SCK(13)/VCC/GND 배선, 카드 삽입/포맷(FAT32) 확인");
    }
    delay(1000);
    return;
  }

  // 1초마다 실제 쓰기 테스트 (쓰기 실패 시 재탐색 모드로 복귀)
  FsFile f = sd.open("SDTEST.TXT", O_WRONLY | O_CREAT | O_APPEND);
  if (f) {
    f.print("write test #");
    f.println(++s_writeCount);
    f.close();
    Serial.print("[SD] write OK  count=");
    Serial.println(s_writeCount);
  } else {
    Serial.println("[X] 쓰기 실패 — 연결 끊김/카드 탈락 의심, 재탐색 시작");
    s_inited = false;
  }
  delay(1000);
}
