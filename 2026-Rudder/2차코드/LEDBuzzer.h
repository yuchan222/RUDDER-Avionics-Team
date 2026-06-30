#ifndef LED_BUZZER_H
#define LED_BUZZER_H

#include <Arduino.h>
#include "Config.h"

#define LED_PWR   0x01
#define LED_SD    0x02
#define LED_IMU   0x04
#define LED_BMP   0x08
#define LED_ARMED 0x10
#define LED_FLT   0x20
#define LED_EJ    0x40

void initLEDBuzzer();
void setSystemStatus(uint8_t status);   // setup() 후 호출 → Mode 1 LED에 반영 (#19)
void setBeepEnabled(bool en);           // false: 비행 중 blocking beep 차단 (#20)
void updateLEDBuzzer(uint8_t mode);
void setLED(uint8_t bits);
void beep(uint16_t ms);

#endif
