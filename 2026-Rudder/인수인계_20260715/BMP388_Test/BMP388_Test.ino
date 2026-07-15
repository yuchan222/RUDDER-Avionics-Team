// BMP388 단독 테스트 — 연결 상태 + 실시간 기압/온도 출력
// 배선(I2C 4핀 헤더): VCC→5V(또는 3.3V), GND→GND, SDA→A4, SCL→A5
// 인식 안 되면: 왼쪽 줄(SPI측)의 CS 핀을 VCC에 연결해볼 것 (I2C 모드 고정)

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
static bool    s_inited    = false;
static uint8_t s_foundAddr = 0;

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Wire.begin();
  Serial.println("=== BMP388 단독 테스트 ===");
}

void loop() {
  if (!s_inited) {
    // 1단계: I2C 버스에서 주소 응답 확인 (0x76 → 0x77 순서)
    s_foundAddr = probe(0x76) ? 0x76 : (probe(0x77) ? 0x77 : 0);

    if (s_foundAddr == 0) {
      Serial.println("[X] I2C 미인식 — VCC/GND/SDA(A4)/SCL(A5) 배선, CS→VCC 연결 확인");
    } else if (bmp.begin_I2C(s_foundAddr)) {
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      Serial.print("[O] BMP388 초기화 성공! 주소 0x");
      Serial.println(s_foundAddr, HEX);
      s_inited = true;
    } else {
      Serial.print("[!] 주소 0x"); Serial.print(s_foundAddr, HEX);
      Serial.println(" 응답은 있으나 init 실패 — 전원 전압/납땜 확인");
    }
    delay(1000);
    return;
  }

  // 2단계: 실시간 값 출력 (손으로 센서를 가볍게 덮으면 온도가 서서히 올라야 정상)
  if (bmp.performReading()) {
    Serial.print("[BMP] pressure="); Serial.print(bmp.pressure, 0);
    Serial.print(" Pa  temp=");      Serial.print(bmp.temperature, 2);
    Serial.println(" C");
  } else {
    Serial.println("[!] read 실패");
  }

  // 연결 끊김 감지 → 재탐색 모드로 복귀
  if (!probe(s_foundAddr)) {
    Serial.println("[X] 연결 끊김 감지 — 재탐색 시작");
    s_inited = false;
  }
  delay(500);
}
