#include "EjectionLogic.h"

static Servo   *s_servo        = nullptr;
static bool     s_launched     = false;
static bool     s_ejected      = false;
static bool     s_safeLockDone = false;

static uint32_t s_launchMs    = 0;
static uint32_t s_zaccStartMs = 0;
static bool     s_zaccActive  = false;

static float    s_peakAlt   = 0.0f;
static int      s_dropCount = 0;

static void doEject() {
  if (s_servo) s_servo->write(90);
  s_ejected = true;
}

void initEjection(Servo &servo) {
  s_servo = &servo;
  s_servo->write(0);
}

bool updateEjection(const DataPacket &p) {
  if (s_ejected) return false;

  // BMP 읽기 실패 루프 스킵
  if (p.altitude_cm == -1) return false;

  float    altM = p.altitude_cm / 100.0f;
  int16_t  zacc = p.acc[2];    // [mg], 로켓 추력 방향(Z+) 기준
  uint32_t now  = millis();

  // ── 1. 발사 감지: Z-acc >= 2000mg (300ms 유지) AND BMP 고도 >= 2m ─────
  if (!s_launched) {
    bool zaccCond = (zacc >= ZACC_THRESH_MG);
    bool bmpCond  = (altM >= LAUNCH_ALT_THRESH_M);

    if (zaccCond) {
      if (!s_zaccActive) {
        s_zaccActive  = true;
        s_zaccStartMs = now;
      }
    } else {
      s_zaccActive = false;
    }

    if (s_zaccActive && bmpCond && (now - s_zaccStartMs >= ZACC_HOLD_MS)) {
      s_launched  = true;
      s_launchMs  = now;
      s_peakAlt   = altM;
      s_dropCount = 0;
      Serial.println("[EJ] LAUNCH DETECTED");
    }
    return false;
  }

  // ── 2. Safe Lock: 발사 후 3초간 사출 금지 (연소 중 오작동 방지) ──────────
  if (!s_safeLockDone) {
    if (now - s_launchMs >= SAFE_LOCK_MS) {
      s_safeLockDone = true;
      Serial.println("[EJ] Safe Lock released");
    }
    if (altM > s_peakAlt) s_peakAlt = altM;
    return false;
  }

  // ── 최고 고도 갱신 ─────────────────────────────────────────────────────
  if (altM > s_peakAlt) {
    s_peakAlt   = altM;
    s_dropCount = 0;
  }

  // ── 3. 주 사출: 최고점 -3m 이상 하강이 5회 연속 확인 ──────────────────
  bool descending = (s_peakAlt - altM >= APOGEE_DROP_M)
                 && (s_peakAlt        >= MIN_APOGEE_ALT_M);

  if (descending) {
    if (++s_dropCount >= APOGEE_DROP_COUNT) {
      Serial.println("[EJ] APOGEE -> EJECT (primary)");
      doEject();
      return true;
    }
  } else {
    s_dropCount = 0;
  }

  // ── 4. 보조 사출: 발사 후 12초 경과 + 하강 경향 동시 확인 ────────────
  uint32_t timerA_ms = (uint32_t)(TIMER_A_SEC * 1000.0f);
  if ((now - s_launchMs >= timerA_ms) && descending) {
    Serial.println("[EJ] TIMEOUT+DESC -> EJECT (backup)");
    doEject();
    return true;
  }

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
