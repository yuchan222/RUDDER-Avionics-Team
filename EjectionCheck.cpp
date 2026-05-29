#include "EjectionCheck.h"
#include <math.h>
#include "SensorManager.h"
#include "DebugSerial.h"
#include "CommandRx.h"
//초기 노즈 각도

uint8_t nose_angle_deg = 0;
// 임계 조건 상수
const int angleThreshold = 90;            // 45도 이상이면 경고
const unsigned long holdTimeMs = 1000;    // 3초 이상 유지 시 경고

// 상태 추적용 내부 변수
unsigned long angleStartTime = 0;
bool angleConditionActive = false;
bool angleMessageSent = false;

const int16_t zAccThreshold = 2000;              // 예: 2g (단위: mg)
const unsigned long zHoldTimeMs = 300;          // 0.3초 이상 지속 시 경고

unsigned long zAccStartTime = 0;
bool zAccConditionActive = false;
bool zAccMessageSent = false;



// ───────────────────────────────
// 전역 타이머 객체 정의 (Timer6 → Timer1, Timer7 → Timer2)
CountdownTimer timer6(6, "Timer1", 22.0);   // 5초
CountdownTimer timer7(7, "Timer2", 12.0);   // 3.5초

// ISR 함수 정의
void timer6ISR() { timer6.update(); }
void timer7ISR() { timer7.update(); }

// 생성자
CountdownTimer::CountdownTimer(uint8_t timerId, const char* name, float seconds)
  : _timerId(timerId), _name(name), _running(false) {
  _initialTicks = static_cast<uint8_t>(seconds * 2);  // 0.5초 단위
  _ticks = _initialTicks;
}

void CountdownTimer::start() {
  if (_initialTicks == 0) {
    Serial.print("[");
    Serial.print(_name);
    Serial.println("] Initial duration is zero. Aborting.");
    return;
  }

  _ticks = _initialTicks;
  _running = true;

  switch (_timerId) {
    case 6: Timer6.attachInterrupt(timer6ISR).start(500000); break;
    case 7: Timer7.attachInterrupt(timer7ISR).start(500000); break;
  }

  Serial.print("[");
  Serial.print(_name);
  Serial.println("] Countdown started");
  myServo.close();
}

void CountdownTimer::stop() {
  _running = false;
  switch (_timerId) {
    case 6: Timer6.stop(); break;
    case 7: Timer7.stop(); break;
  }

  //Serial.print("[");
  //Serial.print(_name);
  //Serial.println("] Countdown stopped manually");
}

void CountdownTimer::update() {
  if (_running && _ticks > 0) {
    _ticks--;

    Serial.print("[");
    Serial.print(_name);
    Serial.print("] Countdown: ");
    Serial.print(_ticks * 0.5, 1);  // 소수 한 자리 출력
    Serial.println(" sec");
    commandReceiver.sendRxPacket1(0x6E, _ticks * 0.5, _ticks * 0.5);//타이머

    if (_ticks == 0) {
      _running = false;
      switch (_timerId) {
        case 6: Timer6.stop(); break;
        case 7: Timer7.stop(); break;
      }
      Serial.print("[");
      Serial.print(_name);
      Serial.println("] DONE!");
      myServo.open();
    }
  }
}

bool CountdownTimer::isRunning() const {
  return _running;
}

void CountdownTimer::printStatus() const {
  Serial.print("[");
  Serial.print(_name);
  Serial.print("] Running: ");
  Serial.print(_running ? "YES" : "NO");
  Serial.print(", Time Left: ");
  Serial.print(_ticks * 0.5, 1);
  Serial.println(" sec");
}

//int nose_angle_deg = 0;

// 외부에서 주기적으로 갱신되는 최신 센서 데이터
extern DataPacket p;

void Noseangle() {
    float w = p.quat[0] / 10000.0f;
    float x = p.quat[1] / 10000.0f;
    float y = p.quat[2] / 10000.0f;
    float z = p.quat[3] / 10000.0f;

    float vx = 2.0f * (x * z - w * y);
    float vy = 2.0f * (y * z + w * x);
    float vz = 1.0f - 2.0f * (x * x + y * y);

    float len = sqrt(vx * vx + vy * vy + vz * vz);
    if (len < 1e-6f) {
        nose_angle_deg = 0;
        return;
    }

    float angle_rad = acos(vz / len);
    nose_angle_deg = (int)(angle_rad * 180.0f / M_PI + 0.5f);  // 반올림

    //Serial.print("Nose Angle [deg]: ");
    //Serial.println(nose_angle_deg);
}
// nose_angle_deg 값을 감시하여 일정 시간 이상 threshold 초과 시 경고 출력
void angleCheck() {
  unsigned long now = millis();

  if (nose_angle_deg >= angleThreshold) {
    if (!angleConditionActive) {
      angleStartTime = now;
      angleConditionActive = true;
    }

    if ((now - angleStartTime >= holdTimeMs) && !angleMessageSent) {
      Serial.print("[ALERT] Nose Angle exceeded ");
      Serial.print(angleThreshold);
      Serial.println(" degrees for more than 3 seconds!");
      myServo.open();
      angleMessageSent = true;
    }
  } else {
    angleConditionActive = false;
    angleMessageSent = false;
  }
}
// ─────────────── Z축 가속도 감시 ───────────────
void ZaccCheck() {
  unsigned long now = millis();

  if (p.acc[2] >= zAccThreshold) {
    if (!zAccConditionActive) {
      zAccStartTime = now;
      zAccConditionActive = true;
    }

    if ((now - zAccStartTime >= zHoldTimeMs) && !zAccMessageSent) {
      Serial.print("[ALERT] Z-Acceleration exceeded ");
      Serial.print(zAccThreshold);
      Serial.println(" mg for more than 0.3 seconds!");
      timer7.start();
      zAccMessageSent = true;
    }
  } else {
    zAccConditionActive = false;
    //zAccMessageSent = false; 주석을 할경우 한번 실행되면 다시는 실행 안함
  }
}





/*

void Noseangle() {
    float w = p.quat[0] / 10000.0f;
    float x = p.quat[1] / 10000.0f;
    float y = p.quat[2] / 10000.0f;
    float z = p.quat[3] / 10000.0f;

    float vx = 2.0f * (x * z - w * y);
    float vy = 2.0f * (y * z + w * x);
    float vz = 1.0f - 2.0f * (x * x + y * y);

    float len = sqrt(vx * vx + vy * vy + vz * vz);
    if (len < 1e-6f) {
        nose_angle_deg = 0;
        return;
    }

    float angle_rad = acos(vz / len);
    nose_angle_deg = (int)(angle_rad * 180.0f / M_PI + 0.5f);  // 반올림

    Serial.print("Nose Angle [deg]: ");
    Serial.println(nose_angle_deg);
}

*/











