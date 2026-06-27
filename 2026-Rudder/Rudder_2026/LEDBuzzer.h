#ifndef LED_BUZZER_H
#define LED_BUZZER_H

#include <Arduino.h>
#include "Config.h"

// ── 74HC595 출력 비트 ─────────────────────────────────────────────────────
#define LED_PWR   0x01   // 전원
#define LED_SD    0x02   // SD 카드 준비
#define LED_IMU   0x04   // IMU OK
#define LED_BMP   0x08   // BMP388 OK
#define LED_ARMED 0x10   // Armed
#define LED_FLT   0x20   // 비행 중
#define LED_EJ    0x40   // 사출 완료

void initLEDBuzzer();
void updateLEDBuzzer(uint8_t mode);  // 모드별 LED/부저 패턴 (매 루프 호출)
void setLED(uint8_t bits);
void beep(uint16_t ms);

#endif
