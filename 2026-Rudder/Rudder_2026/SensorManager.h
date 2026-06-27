#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>   // Adafruit MPU6050 라이브러리
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Packet.h"
#include "Config.h"

// ── 공개 함수 ─────────────────────────────────────────────────────────────
void clearI2CBus();              // UNO R4 Minima: A4(SDA)=18, A5(SCL)=19
void initSensors();
void collectBaseline();          // Mode 0→1 전환 시 지상 기준 기압 평균화
void readSensors(DataPacket &p);

// BMP388 고도 계산 (EjectionLogic에서 사용)
float getAltitude();             // [m], baseline 기준 상대 고도
float getBaselinePressure();     // [Pa]
// float getBaselineTemp();      // TODO: 세종대 식(9) 온도 보정 시 활성화

// ── 외부 센서 객체 ────────────────────────────────────────────────────────
extern Adafruit_MPU6050 mpu;
extern Adafruit_BMP3XX  bmp;

// ── 참고: 쿼터니언이 필요하면 MadgwickAHRS 또는 MahonyAHRS 라이브러리를  ──
// ── SensorManager.cpp 내부에 추가하고 updateAHRS()를 readSensors() 끝에  ──
// ── 호출하면 됩니다. 2026 사출 판단은 BMP388 고도 기반이므로 기본 제외.  ──

#endif
