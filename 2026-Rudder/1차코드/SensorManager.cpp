#include "SensorManager.h"
#include <math.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP3XX  bmp;

static float s_baselinePa  = 101325.0f;
static float s_baselineAlt = 0.0f;
// static float s_baselineTempC = 15.0f;  // TODO: 세종대 식(9) 온도 보정 시 활성화

// ── I2C 버스 클리어 (UNO R4 Minima: SDA=A4=18, SCL=A5=19) ──────────────
void clearI2CBus() {
  pinMode(I2C_SDA_PIN, OUTPUT);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(5);
}

// ── 센서 초기화 ──────────────────────────────────────────────────────────
void initSensors() {
  Wire.end();
  delay(10);
  Wire.begin();

  // MPU-6050
  if (!mpu.begin(IMU_ADDR)) {
    Serial.println("[ERR] MPU6050 init fail");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU6050");
  }

  // BMP388
  if (!bmp.begin_I2C(BMP388_ADDR)) {
    Serial.println("[ERR] BMP388 init fail");
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("[OK] BMP388");
  }
}

// ── 지상 기준 기압 평균화 (Mode 0→1 전환 시 한 번 호출) ─────────────────
void collectBaseline() {
  double sum = 0.0;
  // double sum_temp = 0.0;  // TODO: 온도 보정 활성화 시
  int    cnt = 0;
  Serial.println("[BMP] baseline 수집 중...");
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    if (bmp.performReading()) {
      sum += bmp.pressure;
      // sum_temp += bmp.temperature;  // TODO: 온도 보정 활성화 시
      cnt++;
    }
    delay(20);
  }
  if (cnt > 0) {
    s_baselinePa = (float)(sum / cnt);
    // s_baselineTempC = (float)(sum_temp / cnt);  // TODO: 온도 보정 활성화 시
  } else {
    s_baselinePa = 101325.0f;
    Serial.println("[WARN] baseline 수집 실패 → 기본값 사용");
  }
  s_baselineAlt = 44330.0f * (1.0f - powf(s_baselinePa / 101325.0f, 1.0f / 5.255f));
  Serial.print("[BMP] baseline Pa="); Serial.print(s_baselinePa, 1);
  Serial.print("  Alt=");            Serial.print(s_baselineAlt, 1); Serial.println(" m");
}

// ── 고도 계산 (baseline 기준 상대 고도) ──────────────────────────────────
float getAltitude() {
  if (!bmp.performReading()) return 0.0f;
  float absAlt = 44330.0f * (1.0f - powf(bmp.pressure / 101325.0f, 1.0f / 5.255f));
  return absAlt - s_baselineAlt;
}

float getBaselinePressure() { return s_baselinePa; }
// float getBaselineTemp() { return s_baselineTempC; }  // TODO: 온도 보정 활성화 시

// ── 센서 읽기 → DataPacket 채우기 ────────────────────────────────────────
void readSensors(DataPacket &p) {
  sensors_event_t accelEv, gyroEv, tempEv;
  mpu.getEvent(&accelEv, &gyroEv, &tempEv);

  // acc: m/s² → mg (×1000 / 9.80665)
  p.acc[0] = (int16_t)(accelEv.acceleration.x / 9.80665f * 1000.0f);
  p.acc[1] = (int16_t)(accelEv.acceleration.y / 9.80665f * 1000.0f);
  p.acc[2] = (int16_t)(accelEv.acceleration.z / 9.80665f * 1000.0f);

  // gyro: rad/s → 0.01 dps (× 180/π × 100)
  p.gyro[0] = (int16_t)(gyroEv.gyro.x * (18000.0f / (float)M_PI));
  p.gyro[1] = (int16_t)(gyroEv.gyro.y * (18000.0f / (float)M_PI));
  p.gyro[2] = (int16_t)(gyroEv.gyro.z * (18000.0f / (float)M_PI));

  // quat: 2026 기본 미사용 (Madgwick 추가 시 여기서 채울 것)
  p.quat[0] = p.quat[1] = p.quat[2] = p.quat[3] = 0;

  // BMP388
  if (bmp.performReading()) {
    p.pressure  = (int32_t)bmp.pressure;
    p.bmp_temp  = (int16_t)(bmp.temperature * 100.0f);
    float alt   = 44330.0f * (1.0f - powf(bmp.pressure / 101325.0f, 1.0f / 5.255f))
                  - s_baselineAlt;
    p.altitude_cm = (int16_t)(alt * 100.0f);
  } else {
    p.pressure    = -1;
    p.bmp_temp    = -1;
    p.altitude_cm = -1;
  }

  p.ms = millis();
}
