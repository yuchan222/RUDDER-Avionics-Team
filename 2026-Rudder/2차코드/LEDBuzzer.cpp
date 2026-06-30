#include "LEDBuzzer.h"

static uint8_t s_sysStatus   = 0xFF;  // 기본값: 모두 OK (init 전 안전값)
static bool    s_beepEnabled = true;

static void shiftOut595(uint8_t data) {
  digitalWrite(PIN_RCLK, LOW);
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(PIN_SRCLK, LOW);
    digitalWrite(PIN_SER, (data >> i) & 1 ? HIGH : LOW);
    digitalWrite(PIN_SRCLK, HIGH);
  }
  digitalWrite(PIN_RCLK, HIGH);
}

void initLEDBuzzer() {
  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_BUZ,   OUTPUT);
  digitalWrite(PIN_BUZ, HIGH);
  shiftOut595(0x00);
}

void setSystemStatus(uint8_t status) { s_sysStatus = status; }
void setBeepEnabled(bool en)         { s_beepEnabled = en;   }

void setLED(uint8_t bits) { shiftOut595(bits); }

void beep(uint16_t ms) {
  if (!s_beepEnabled) return;   // 비행 중 blocking beep 차단 (#20)
  digitalWrite(PIN_BUZ, LOW);
  delay(ms);
  digitalWrite(PIN_BUZ, HIGH);
}

void updateLEDBuzzer(uint8_t mode) {
  static uint32_t lastMs     = 0;
  static bool     blinkState = false;
  uint32_t now = millis();

  switch (mode) {
    case 0:  // 대기 — 전원 LED 0.5초 blink
      if (now - lastMs >= 500) {
        blinkState = !blinkState;
        setLED(blinkState ? LED_PWR : 0x00);
        lastMs = now;
      }
      break;

    case 1: {  // Armed — 실제 센서 초기화 상태 반영 (#19)
      uint8_t leds = LED_PWR | LED_ARMED;
      if (s_sysStatus & STATUS_BMP) leds |= LED_BMP;
      if (s_sysStatus & STATUS_IMU) leds |= LED_IMU;
      if (s_sysStatus & STATUS_SD)  leds |= LED_SD;
      setLED(leds);
      break;
    }

    case 2:  // 비행 중 — ARMED+FLT 빠른 blink (200ms)
      if (now - lastMs >= 200) {
        blinkState = !blinkState;
        setLED(blinkState ? (LED_ARMED | LED_FLT) : LED_PWR);
        lastMs = now;
      }
      break;

    case 3:  // 사출 완료 — EJ LED 고정
      setLED(LED_PWR | LED_EJ);
      break;

    case 4:  // 착지 — LED 전부 OFF
      setLED(0x00);
      break;
  }
}
