#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Servo.h>

class ServoControl {
public:
  ServoControl(int pin);
  void begin();
  void open();
  void close();
  void chattering(uint8_t times = 5, uint16_t delay_ms = 100);  // 챠터링 함수

private:
  static const int OPEN_ANGLE = 50;
  static const int CLOSE_ANGLE = 100;
  static const int CHATTER_RANGE = 10;  // ±10도 진동 범위

  int servoPin;
  Servo servo;
};

#endif
