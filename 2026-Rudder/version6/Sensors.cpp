#include "Sensors.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_INA219.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════
//  IMU: 레지스터 직접제어 드라이버 (이상혁 방식 + 측정범위 설정)
//  - 라이브러리의 WHO_AM_I 검사를 하지 않으므로 클론 칩(0x72 등)도 동작
//  - WHO_AM_I는 부팅 시 로그로만 남김 (판정에 사용 안 함)
// ═══════════════════════════════════════════════════════════════════════════

static bool s_imuOk    = false;
static bool s_bmpOk    = false;   // 최근 읽기 성공 여부 (매 readSensors마다 갱신 — 상태점 최신성 유지)
static bool s_bmpInited = false;  // 부팅 시 초기화 성공 여부 (고정, 읽기 재시도 가능 여부 판단용)
static bool s_inaOk    = false;

static Adafruit_BMP3XX  s_bmp;
static Adafruit_INA219  s_ina(INA219_ADDR);

static float s_baselinePa    = 101325.0f;
static float s_baselineTempC = 15.0f;

// ── IMU 레지스터 유틸 ─────────────────────────────────────────────────────
static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

static bool imuInit() {
  // 정체 기록 (판정 안 함)
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x75);                       // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;   // ACK 없음 = 미연결
  Wire.requestFrom(IMU_ADDR, 1, true);
  uint8_t whoami = Wire.available() ? Wire.read() : 0xFF;
  Serial.print("[IMU] WHO_AM_I=0x"); Serial.print(whoami, HEX);
  Serial.println(" (기록용 — 어떤 값이든 계속 진행)");

  if (!imuWriteReg(0x6B, 0x80)) return false;   // 전체 리셋
  delay(100);
  if (!imuWriteReg(0x6B, 0x01)) return false;   // 슬립 해제 + PLL 클럭
  if (!imuWriteReg(0x6C, 0x00)) return false;   // 전축 활성화
  if (!imuWriteReg(0x1C, 0x10)) return false;   // 가속도 ±8g (2g 임계 검사에 필수)
  if (!imuWriteReg(0x1B, 0x08)) return false;   // 자이로 ±500dps
  if (!imuWriteReg(0x1A, 0x04)) return false;   // DLPF ≈20Hz (노이즈 완화)
  delay(50);
  return true;
}

// acc[mg], gyro[0.1dps]로 변환해서 반환. 실패 시 false
static bool imuRead(int16_t acc[3], int16_t gyro[3]) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(IMU_ADDR, 14, true);
  if (Wire.available() != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++)
    raw[i] = (int16_t)(Wire.read() << 8 | Wire.read());
  // raw[0..2]=acc, raw[3]=temp, raw[4..6]=gyro

  for (int i = 0; i < 3; i++) {
    acc[i]  = (int16_t)((int32_t)raw[i]     * 1000 / 4096);   // ±8g → mg
    gyro[i] = (int16_t)((int32_t)raw[4 + i] * 20   / 131);    // ±500dps → 0.1dps
  }
  return true;
}

// ── 세종대 식(9) 온도보정 고도 [m] ────────────────────────────────────────
static float computeAltitude(float pressurePa) {
  const float L  = 0.0065f;
  const float Rs = 287.058f;
  const float g  = 9.80665f;
  float T_ref = s_baselineTempC + 273.15f;
  return (T_ref / L) * (powf(pressurePa / s_baselinePa, -Rs * L / g) - 1.0f);
}

// ── 공개 함수 ─────────────────────────────────────────────────────────────
void initSensors() {
  Wire.begin();
  Wire.setClock(100000);

  s_imuOk = imuInit();
  Serial.println(s_imuOk ? "[OK] IMU (레지스터 직접제어, ±8g/±500dps)"
                         : "[ERR] IMU 초기화 실패");

  s_bmpOk = s_bmp.begin_I2C(BMP388_ADDR) || s_bmp.begin_I2C(0x77);
  s_bmpInited = s_bmpOk;   // 초기화 성공 여부는 고정 기록 (읽기 재시도 가능 여부 판단용)
  if (s_bmpOk) {
    s_bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);   // 온도는 정밀도 불필요 — 측정시간 단축
    s_bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    s_bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    s_bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("[OK] BMP388");
  } else {
    Serial.println("[ERR] BMP388 초기화 실패 (0x76/0x77 모두)");
  }

  s_inaOk = s_ina.begin();
  Serial.println(s_inaOk ? "[OK] INA219" : "[ERR] INA219 초기화 실패");
}

bool collectBaseline() {
  if (!s_bmpInited) {
    Serial.println("[WARN] BMP 없음 — 기준압 기본값(101325Pa) 사용, 고도 신뢰 불가");
    return false;
  }
  double sumPa = 0.0, sumT = 0.0;
  int cnt = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    if (s_bmp.performReading()) {
      sumPa += s_bmp.pressure;
      sumT  += s_bmp.temperature;
      cnt++;
    }
    delay(20);
  }
  if (cnt > 0) {
    s_baselinePa    = (float)(sumPa / cnt);
    s_baselineTempC = (float)(sumT / cnt);
    Serial.print("[BMP] 기준압 "); Serial.print(s_baselinePa, 1);
    Serial.print(" Pa / ");        Serial.print(s_baselineTempC, 1);
    Serial.print(" C  (성공 ");    Serial.print(cnt);
    Serial.print("/");             Serial.print(BASELINE_SAMPLES);
    Serial.println(")");
  } else {
    Serial.println("[WARN] 기준압 수집 실패 — 기본값 유지");
  }
  return cnt >= BASELINE_MIN_OK;
}

void readSensors(DataPacket &p) {
  // IMU — 읽기 실패 시 상태비트만 끄고 이전 값 유지 (일시 끊김에 관대)
  int16_t acc[3], gyro[3];
  if (imuRead(acc, gyro)) {
    for (int i = 0; i < 3; i++) { p.acc[i] = acc[i]; p.gyro[i] = gyro[i]; }
    s_imuOk = true;
  } else {
    s_imuOk = false;
  }

  // BMP — 초기화된 경우 매번 읽기 시도(재연결 시 회복 가능), 실패 시 ALT_INVALID sentinel
  bool bmpRead = s_bmpInited && s_bmp.performReading();
  if (bmpRead) {
    p.pressure    = (int32_t)s_bmp.pressure;
    p.bmp_temp    = (int16_t)(s_bmp.temperature * 100.0f);
    p.altitude_cm = (int32_t)(computeAltitude(s_bmp.pressure) * 100.0f);
  } else {
    p.pressure    = ALT_INVALID;
    p.bmp_temp    = -1;
    p.altitude_cm = ALT_INVALID;
  }
  s_bmpOk = bmpRead;   // 이번 읽기의 실제 성공 여부로 갱신 (상태점이 항상 최신 반영)

  // INA — 미장착 시 -1
  if (s_inaOk) {
    p.voltage_mv = (int16_t)(s_ina.getBusVoltage_V() * 1000.0f);
    p.current_ma = (int16_t)(s_ina.getCurrent_mA());
  } else {
    p.voltage_mv = -1;
    p.current_ma = -1;
  }

  p.ms = millis();
}

uint8_t sensorStatus() {
  uint8_t s = 0;
  if (s_bmpOk) s |= STATUS_BMP;
  if (s_imuOk) s |= STATUS_IMU;
  if (s_inaOk) s |= STATUS_INA;
  return s;
}
