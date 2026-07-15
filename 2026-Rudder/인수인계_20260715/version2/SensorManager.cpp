#include "SensorManager.h"
#include <math.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP3XX  bmp;
Adafruit_INA219  ina219(INA219_ADDR);

static float   s_baselinePa    = 101325.0f;
static float   s_baselineAlt   = 0.0f;
static float   s_baselineTempC = 15.0f;
static uint8_t s_sensorStatus  = 0;

// ── 세종대 식(9) 온도 보정 고도 계산 (내부 전용) ─────────────────────────
static float computeAltitude(float pressurePa) {
  const float L  = 0.0065f;
  const float Rs = 287.058f;
  const float g  = 9.80665f;
  float T_ref = s_baselineTempC + 273.15f;
  return (T_ref / L) * (powf(pressurePa / s_baselinePa, -Rs * L / g) - 1.0f);
}

// ── I2C 버스 클리어 ───────────────────────────────────────────────────────
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
    s_sensorStatus |= STATUS_IMU;
    Serial.println("[OK] MPU6050");
  }

  // BMP388: 0x76 실패 시 0x77 자동재시도 (#23)
  if (!bmp.begin_I2C(BMP388_ADDR) && !bmp.begin_I2C(0x77)) {
    Serial.println("[ERR] BMP388 init fail (tried 0x76 & 0x77)");
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    s_sensorStatus |= STATUS_BMP;
    Serial.println("[OK] BMP388");
  }

  // INA219 (#9)
  if (!ina219.begin()) {
    Serial.println("[ERR] INA219 init fail");
  } else {
    s_sensorStatus |= STATUS_INA;
    Serial.println("[OK] INA219");
  }
}

// ── 지상 기준 기압/온도 평균화 ────────────────────────────────────────────
void collectBaseline() {
  double sum      = 0.0;
  double sum_temp = 0.0;
  int    cnt = 0;
  Serial.println("[BMP] baseline 수집 중...");
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    if (bmp.performReading()) {
      sum      += bmp.pressure;
      sum_temp += bmp.temperature;
      cnt++;
    }
    delay(20);
  }
  if (cnt > 0) {
    s_baselinePa    = (float)(sum / cnt);
    s_baselineTempC = (float)(sum_temp / cnt);
  } else {
    s_baselinePa    = 101325.0f;
    Serial.println("[WARN] baseline 수집 실패 → 기본값 사용");
  }
  s_baselineAlt = 44330.0f * (1.0f - powf(s_baselinePa / 101325.0f, 1.0f / 5.255f));
  Serial.print("[BMP] Pa="); Serial.print(s_baselinePa, 1);
  Serial.print("  T=");     Serial.print(s_baselineTempC, 2);
  Serial.print(" C  Alt="); Serial.print(s_baselineAlt, 1); Serial.println(" m");
}

float   getBaselinePressure() { return s_baselinePa;    }
float   getBaselineTemp()     { return s_baselineTempC; }
uint8_t getSystemStatus()     { return s_sensorStatus;  }

// ── 센서 읽기 → DataPacket 채우기 ────────────────────────────────────────
void readSensors(DataPacket &p) {
  sensors_event_t accelEv, gyroEv, tempEv;
  mpu.getEvent(&accelEv, &gyroEv, &tempEv);

  p.acc[0] = (int16_t)(accelEv.acceleration.x / 9.80665f * 1000.0f);
  p.acc[1] = (int16_t)(accelEv.acceleration.y / 9.80665f * 1000.0f);
  p.acc[2] = (int16_t)(accelEv.acceleration.z / 9.80665f * 1000.0f);

  // 0.1 dps 단위 (1800/π ≈ 572.96): ±500 dps → ±5000 → int16_t 안전 (#8)
  p.gyro[0] = (int16_t)(gyroEv.gyro.x * (1800.0f / (float)M_PI));
  p.gyro[1] = (int16_t)(gyroEv.gyro.y * (1800.0f / (float)M_PI));
  p.gyro[2] = (int16_t)(gyroEv.gyro.z * (1800.0f / (float)M_PI));

  if (bmp.performReading()) {
    p.pressure    = (int32_t)bmp.pressure;
    p.bmp_temp    = (int16_t)(bmp.temperature * 100.0f);
    p.altitude_cm = (int32_t)(computeAltitude(bmp.pressure) * 100.0f);
  } else {
    p.pressure    = -1;
    p.bmp_temp    = -1;
    p.altitude_cm = -1;   // BMP 실패 sentinel
  }

  if (s_sensorStatus & STATUS_INA) {
    p.voltage_mv = (int16_t)(ina219.getBusVoltage_V() * 1000.0f);
    p.current_ma = (int16_t)(ina219.getCurrent_mA());
  } else {
    p.voltage_mv = -1;
    p.current_ma = -1;
  }

  p.ms = millis();
}
