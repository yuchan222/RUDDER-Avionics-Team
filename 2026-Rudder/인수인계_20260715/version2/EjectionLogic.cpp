#include "EjectionLogic.h"
#include <math.h>

static Servo   *s_servo        = nullptr;
static bool     s_launched     = false;
static bool     s_ejected      = false;
static bool     s_safeLockDone = false;

static uint32_t s_launchMs    = 0;
static uint32_t s_zaccStartMs = 0;
static bool     s_zaccActive  = false;

static float    s_peakAlt   = 0.0f;
static int      s_dropCount = 0;

static uint32_t s_landStableMs = 0;
static int32_t  s_lastAltCm   = 0;

static void doEject() {
  if (s_servo) s_servo->write(SERVO_EJECT_DEG);
  s_ejected = true;
}

void initEjection(Servo &servo) {
  s_servo = &servo;
  s_servo->write(SERVO_CLOSED_DEG);
}

bool updateEjection(const DataPacket &p) {
  if (s_ejected) return false;

  int16_t  zacc  = p.acc[2];
  uint32_t now   = millis();
  bool     bmpOk = (p.altitude_cm != -1);
  float    altM  = bmpOk ? (float)p.altitude_cm / 100.0f : 0.0f;

  // ── 1. 발사 감지 (#3: BMP 고장 시 zacc 단독 경로) ────────────────────
  // BMP 정상: zacc 300ms 유지 AND 고도 ≥ 2m (이중 확인)
  // BMP 고장: zacc 300ms 유지만으로 판단 (단독 경로)
  if (!s_launched) {
    bool zaccCond = (zacc >= ZACC_THRESH_MG);
    bool bmpCond  = !bmpOk || (altM >= LAUNCH_ALT_THRESH_M);

    if (zaccCond) {
      if (!s_zaccActive) { s_zaccActive = true; s_zaccStartMs = now; }
    } else {
      s_zaccActive = false;
    }

    if (s_zaccActive && bmpCond && (now - s_zaccStartMs >= ZACC_HOLD_MS)) {
      s_launched  = true;
      s_launchMs  = now;
      s_peakAlt   = altM;
      s_dropCount = 0;
      if (bmpOk) Serial.println("[EJ] LAUNCH DETECTED");
      else       Serial.println("[EJ] LAUNCH DETECTED (zacc only, BMP fail)");
    }
    return false;
  }

  // ── 2. Safe Lock: 발사 후 3초 사출 금지 ─────────────────────────────
  if (!s_safeLockDone) {
    if (now - s_launchMs >= SAFE_LOCK_MS) {
      s_safeLockDone = true;
      Serial.println("[EJ] Safe Lock released");
    }
    if (bmpOk && altM > s_peakAlt) s_peakAlt = altM;
    return false;
  }

  // ── 3. 주 사출: BMP 정상 시 정점 판단 ───────────────────────────────
  if (bmpOk) {
    if (altM > s_peakAlt) { s_peakAlt = altM; s_dropCount = 0; }

    bool descending = (s_peakAlt - altM >= APOGEE_DROP_M)
                   && (s_peakAlt >= MIN_APOGEE_ALT_M);

    if (descending) {
      if (++s_dropCount >= APOGEE_DROP_COUNT) {
        Serial.println("[EJ] APOGEE -> EJECT (primary)");
        doEject();
        return true;
      }
    } else {
      s_dropCount = 0;
    }
  }

  // ── 4. 보조 사출: 발사 후 10초 경과 단독 조건 (#2: descending 제거) ──
  // BMP 고장 여부·하강 여부 무관하게 타이머 만료 시 반드시 사출
  if (now - s_launchMs >= (uint32_t)(TIMER_A_SEC * 1000.0f)) {
    Serial.println("[EJ] TIMEOUT -> EJECT (backup)");
    doEject();
    return true;
  }

  return false;
}

// ── 착지 감지: 사출 후 고도 안정 + 가속도 ≈ 1g 5초 유지 (#24) ──────────
bool checkLanded(const DataPacket &p) {
  if (!s_ejected) return false;
  if (p.altitude_cm == -1) return false;   // BMP 실패 시 착지 판단 보류

  float ax = (float)p.acc[0];
  float ay = (float)p.acc[1];
  float az = (float)p.acc[2];
  float netAccMg = sqrtf(ax*ax + ay*ay + az*az);

  bool altStable = (abs(p.altitude_cm - s_lastAltCm) < LAND_ALT_CM_THRESH);
  bool accStable = (netAccMg >= LAND_ACC_MIN_MG && netAccMg <= LAND_ACC_MAX_MG);

  if (altStable && accStable) {
    if (s_landStableMs == 0) s_landStableMs = millis();
    if (millis() - s_landStableMs >= LAND_STABLE_MS) return true;
  } else {
    s_landStableMs = 0;
  }
  s_lastAltCm = p.altitude_cm;
  return false;
}

void forceEject() {
  if (!s_ejected) {
    Serial.println("[EJ] FORCE EJECT (emergency)");
    doEject();
  }
}

bool isLaunched() { return s_launched; }
bool isEjected()  { return s_ejected;  }
