#include "ServoControl.h"
#include <Arduino.h>

ServoControl::ServoControl(int pin) : servoPin(pin) {}

void ServoControl::begin() {
  servo.attach(servoPin);
}

void ServoControl::open() {
  servo.write(OPEN_ANGLE);
}

void ServoControl::close() {
  servo.write(CLOSE_ANGLE);
}

void ServoControl::chattering(uint8_t times, uint16_t delay_ms) {
  for (uint8_t i = 0; i < times; i++) {
    servo.write(OPEN_ANGLE + CHATTER_RANGE);
    delay(delay_ms);
    servo.write(OPEN_ANGLE - CHATTER_RANGE);
    delay(delay_ms);
  }
  // 마지막에 open 위치로 복귀
  servo.write(OPEN_ANGLE);
}
