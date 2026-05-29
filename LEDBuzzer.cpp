//LEDBuzzer.cpp
#include "LEDBuzzer.h"
#include <Arduino.h>


// 내부 상태 변수 -----------------------
static AvionicsMode lastMode = M0;
static uint32_t tLed = 0, tBuz = 0;
static uint8_t ledByte = 0, ledSeq = 0;
static bool ledState = false, buzState = false;
static uint8_t m5Blink = 0;     // 셧다운 LED 깜빡임 카운트
const uint16_t TONE_MS = 1000;  // 모든 tone() 길이 = 1 s
static uint8_t buzCount = 0;    // M0·M1용 카운터
static const uint8_t seqBits[3] = { LED_R, LED_G, LED_Y };

/* ---------- 초기화 ---------- */
void ledBuzBegin() {
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_RCLK, OUTPUT);
  pinMode(PIN_BUZ, OUTPUT);
  digitalWrite(PIN_BUZ, HIGH);  // 부저 무음(Active-LOW)
  shift595(0);                  // LED OFF
}

/* ---------- 헬퍼 ---------- */
inline void shift595(uint8_t data) {
  shiftOut(PIN_SER, PIN_SRCLK, MSBFIRST, data);
  digitalWrite(PIN_RCLK, HIGH);
  digitalWrite(PIN_RCLK, LOW);
}

/* ─── LED/BUZ 핼퍼 ─── */
inline void ledOn(uint8_t bit) {
  ledByte |= (1U << bit);
  shift595(ledByte);
}
inline void ledOff(uint8_t bit) {
  ledByte &= ~(1U << bit);
  shift595(ledByte);
}
inline void ledOffAll() {
  ledByte = 0;
  shift595(0);
}
inline void ledTgl(uint8_t bit) {
  ledByte ^= (1U << bit);
  shift595(ledByte);
}
inline void ledSet(uint8_t b) {
  ledByte = _BV(b);
  shift595(ledByte);
}

inline void buzOn() {
  digitalWrite(PIN_BUZ, LOW);
  buzState = true;
}  // LOW = ON
inline void buzOff() {
  digitalWrite(PIN_BUZ, HIGH);
  buzState = false;
}
inline void buzPlay1s() {
  buzOn();
  tBuz = millis();
}  // 1 s 경과 시점 추적

/* ─── 주기 갱신 FSM (1~10 ms 주기로 호출) ─── */
void ledBuzUpdate(AvionicsMode curMode) {
  const uint32_t now = millis();

  /* ─── 초기화 블록 ──── */
  if (curMode != lastMode) {
    ledOffAll();
    buzOff();

    tLed = tBuz = now;
    ledSeq = m5Blink = 0;
    buzCount = 0;
    ledState = buzState = false;  // 상태 플래그 리셋
    lastMode = curMode;
  }
  /* ─ LED 패턴 (2 Hz 깜빡임 예시) ─ */
    switch (curMode) {
        /* ───────── MODE 0 : 점검 ───────── R LED 1 Hz + 1 회 Beep */
      case M0:
        //Serial.println("LED11");
        if (now - tLed >= 1000) {
          tLed = now;
          ledTgl(LED_R);
          
        }
        if (buzCount == 0) {
          buzPlay1s();
          ++buzCount;
        }
        break;


      /* ───────── MODE 1 : 기동 ───────── Y LED ON + Beep 2 회 */
      case M1:
        
        if (!ledByte) ledSet(LED_Y);               // LED 상시 ON
        if (buzCount < 2 && now - tBuz >= 1000) {  // 1 s 간격 두 번
          buzPlay1s();
          tBuz = now;
          ++buzCount;
        }
        break;


      /* ───────── MODE 2 : 비행 ───────── G LED ON + 5 s 주기 Beep */
      case M2:
        
        if (!ledByte) ledSet(LED_G);  // LED ON
        if (now - tBuz >= 5000) {     // 5 s 주기 1 s 삐
          buzPlay1s();
          tBuz = now;
        }
        break;

      /* ───────── MODE 3 : 사출 ───────── G↔R 1 Hz + 3 s 주기 Beep */
      case M3:
        Serial.println("LED33");
        if (now - tLed >= 1000) {
          tLed = now;
          ledByte = (ledByte == _BV(LED_G) ? _BV(LED_R) : _BV(LED_G));
          shift595(ledByte);
        }
        if (now - tBuz >= 3000) {  // 3 s 주기 1 s 삐
          buzPlay1s();
          tBuz = now;
        }
        break;

      /* ───────── MODE 4 : 회수 ───────── R→G→Y 1 Hz + 2 s 주기 Beep */
      case M4:
        if (now - tLed >= 1000) {
          tLed = now;
          ledSeq = (ledSeq + 1) % 3;
          ledSet(seqBits[ledSeq]);  // R→G→Y 순환
        }
        if (now - tBuz >= 2000) {  // 2 s 주기 1 s 삐
          buzPlay1s();
          tBuz = now;
        }
        break;

      /* ───────── MODE 5 : 셧다운 ───────── 전 LED 1 s 간격 3 번 깜빡 */
      case M5:
        if (m5Blink < 6 && now - tLed >= 1000) {  // ON+OFF = 2토글 = 1회
          tLed = now;
          ledTgl(LED_R);
          ledTgl(LED_G);
          ledTgl(LED_Y);
          ++m5Blink;
        }
        if (m5Blink >= 6) ledOffAll();  // 세 번 끝나면 모두 OFF
        buzOff();                     
        break;
    }
    if (buzState && (now - tBuz >= 1000)) {
      buzOff();
    }
  lastMode = curMode;
}