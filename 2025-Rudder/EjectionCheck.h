#ifndef EJECTIONCHECK_H
#define EJECTIONCHECK_H

#include <Arduino.h>
#include <DueTimer.h>
#include "ServoControl.h"  // 반드시 먼저 include
class CommandRx;
extern CommandRx commandReceiver;

extern ServoControl myServo;

class CountdownTimer {
public:
  CountdownTimer(uint8_t timerId, const char* name, float seconds);

  void start();
  void stop();
  void update();
  bool isRunning() const;
  void printStatus() const;

private:
  uint8_t _timerId;
  const char* _name;
  volatile uint8_t _ticks;       // 남은 카운트
  uint8_t _initialTicks;         // 초기 카운트값 (재시작용)
  volatile bool _running;
};

// ISR 중계 함수
void timer6ISR();
void timer7ISR();

// 전역 변수 선언 (Nose 각도 [deg])
extern uint8_t nose_angle_deg;

// Nose 각도 계산 및 출력 함수
void Noseangle();
// 이 함수는 loop()에서 반복 호출되며, nose_angle_deg를 감시합니다
void angleCheck();
void ZaccCheck();

// 외부에서 사용할 전역 객체
extern CountdownTimer timer6;
extern CountdownTimer timer7;

#endif
