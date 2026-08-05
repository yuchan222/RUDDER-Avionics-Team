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
static uint16_t s_imuZeroCount = 0;   // 가속도·자이로 전부 0 연속 카운트 (고착 감지용)

// ── 자세추정 (상보필터, 실험용 — version8) ──────────────────────────────
static float    s_tiltXY = 0.0f;      // [deg] X-Y 평면 기울기
static float    s_tiltXZ = 0.0f;      // [deg] X-Z 평면 기울기
static uint32_t s_lastAttMs = 0;
static bool     s_attInit = false;
static uint8_t s_imuAddr = IMU_ADDR;   // 실제 응답하는 주소로 부팅 시 자동 확정됨 (AD0 핀 상태에 따라 0x68/0x69)
static bool s_bmpOk    = false;   // 최근 읽기 성공 여부 (매 readSensors마다 갱신 — 상태점 최신성 유지)
static bool s_bmpInited = false;  // 부팅 시 초기화 성공 여부 (고정, 읽기 재시도 가능 여부 판단용)
static bool s_inaOk    = false;   // 최근 읽기 성공 여부 (매 readSensors마다 갱신)
static bool s_inaInited = false;  // 부팅 시 초기화 성공 여부 (고정)

static Adafruit_BMP3XX  s_bmp;
static Adafruit_INA219  s_ina(INA219_ADDR);

static float s_baselinePa    = 101325.0f;
static float s_baselineTempC = 15.0f;

// ── IMU 레지스터 유틸 ─────────────────────────────────────────────────────
static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(s_imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

// 그 주소에 ACK가 오는지만 빠르게 확인 (레지스터 안 건드림)
static bool imuPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

static bool imuInit() {
  // AD0 핀 상태에 따라 0x68 또는 0x69로 응답할 수 있어 둘 다 시도
  // (배선 작업 중 AD0 연결이 바뀌면 주소가 튈 수 있음 — 2026-07-31 추가)
  const uint8_t altAddr = (IMU_ADDR == 0x68) ? 0x69 : 0x68;
  if (imuPing(IMU_ADDR)) {
    s_imuAddr = IMU_ADDR;
  } else if (imuPing(altAddr)) {
    s_imuAddr = altAddr;
    Serial.print("[IMU] 0x"); Serial.print(IMU_ADDR, HEX);
    Serial.print(" 무응답 — 0x"); Serial.print(altAddr, HEX);
    Serial.println("에서 발견됨 (AD0 핀 배선 확인 권장, Config.h는 안 고쳐도 동작함)");
  } else {
    Serial.print("[IMU] 0x"); Serial.print(IMU_ADDR, HEX);
    Serial.print("·0x"); Serial.print(altAddr, HEX);
    Serial.println(" 둘 다 무응답 — 배선/전원 확인 필요");
    return false;
  }

  // 정체 기록 (판정 안 함)
  Wire.beginTransmission(s_imuAddr);
  Wire.write(0x75);                       // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;   // ACK 없음 = 미연결
  Wire.requestFrom(s_imuAddr, 1, true);
  uint8_t whoami = Wire.available() ? Wire.read() : 0xFF;
  Serial.print("[IMU] 주소 0x"); Serial.print(s_imuAddr, HEX);
  Serial.print(" WHO_AM_I=0x"); Serial.print(whoami, HEX);
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
  Wire.beginTransmission(s_imuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(s_imuAddr, 14, true);
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
  s_inaInited = s_inaOk;   // 초기화 성공 여부는 고정 기록 (읽기 재시도 가능 여부 판단용)
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
    // 발사축 부호 보정 — 텔레메트리 값 자체를 "+ = 로켓 상승 방향"으로 통일
    // (2026-07-31 실측: 이 장착 방향에서 원시 X축은 위로 갈수록 더 음수가 됨)
    p.acc[LAUNCH_AXIS] *= LAUNCH_SIGN;
    s_imuOk = true;

    // I2C ACK는 정상인데 값만 0,0,0으로 고착되는 증상 감지 (2026-08-02 실비행 확인)
    // — 정지 상태에서도 중력 성분 때문에 가속도 3축이 동시에 정확히 0일 수는 없음
    bool allZero = (acc[0] == 0 && acc[1] == 0 && acc[2] == 0
                 && gyro[0] == 0 && gyro[1] == 0 && gyro[2] == 0);
    if (allZero) {
      if (++s_imuZeroCount >= IMU_ZERO_RESET_COUNT) {
        Serial.println("[IMU] 0,0,0 고착 감지 — 재초기화 시도");
        imuInit();
        s_imuZeroCount = 0;
      }
    } else {
      s_imuZeroCount = 0;
    }

    // 상보필터 — 자이로 적분(빠른 변화) + 가속도 atan2(느린 드리프트 보정)를 blend.
    // X축이 로켓 세로축이라 "X를 기준"으로 한 두 기울기각을 씀 (Config.h 주석 참고).
    // Y/Z 부호는 아직 미확정 — 실측 전이라 원시값 그대로 사용 중.
    uint32_t nowMs = millis();
    float dt = s_attInit ? (nowMs - s_lastAttMs) / 1000.0f : 0.0f;
    s_lastAttMs = nowMs;
    s_attInit = true;
    if (dt > 0.0f && dt < 0.5f) {   // 첫 호출이나 비정상적으로 큰 dt는 건너뜀 (튐 방지)
      float gyroY_dps = p.gyro[1] / 10.0f;   // X-Z 평면 회전
      float gyroZ_dps = p.gyro[2] / 10.0f;   // X-Y 평면 회전
      float accAngleXY = atan2f((float)p.acc[1], (float)p.acc[0]) * 180.0f / PI;
      float accAngleXZ = atan2f((float)p.acc[2], (float)p.acc[0]) * 180.0f / PI;
      s_tiltXY = ATTITUDE_ALPHA * (s_tiltXY + gyroZ_dps * dt) + (1.0f - ATTITUDE_ALPHA) * accAngleXY;
      s_tiltXZ = ATTITUDE_ALPHA * (s_tiltXZ + gyroY_dps * dt) + (1.0f - ATTITUDE_ALPHA) * accAngleXZ;
    }
    p.tilt_xy = (int16_t)(s_tiltXY * 100.0f);
    p.tilt_xz = (int16_t)(s_tiltXZ * 100.0f);
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

  // INA — 매 읽기마다 I2C 응답 + 값 타당성 재확인 (한번 붙으면 영원히 정상 고정되던 문제 수정)
  bool inaRead = false;
  if (s_inaInited) {
    Wire.beginTransmission(INA219_ADDR);
    if (Wire.endTransmission() == 0) {
      float v = s_ina.getBusVoltage_V();
      float i = s_ina.getCurrent_mA();
      if (isfinite(v) && isfinite(i) && v > 0.0f && v < 20.0f) {
        p.voltage_mv = (int16_t)(v * 1000.0f);
        p.current_ma = (int16_t)i;
        inaRead = true;
      }
    }
  }
  if (!inaRead) {
    p.voltage_mv = -1;
    p.current_ma = -1;
  }
  s_inaOk = inaRead;   // 이번 읽기의 실제 성공 여부로 갱신 (상태점이 항상 최신 반영)

  p.ms = millis();
}

uint8_t sensorStatus() {
  uint8_t s = 0;
  if (s_bmpOk) s |= STATUS_BMP;
  if (s_imuOk) s |= STATUS_IMU;
  if (s_inaOk) s |= STATUS_INA;
  return s;
}
