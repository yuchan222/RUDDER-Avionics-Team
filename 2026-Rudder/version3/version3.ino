// ═══════════════════════════════════════════════════════════════════════════
//  RUDDER 2026 — version3 비행 소프트웨어
//
//  설계 원칙 (2026-07-22 팀 합의):
//   1. 모드 = 실제 사건과 1:1 대응 (모드만 봐도 로켓이 뭘 하는지 안다)
//   2. 모든 모드 전환에 수동 명령 백업 (센서가 죽어도 사람이 진행 가능)
//   3. 발사감지 OR 조건 (가속도 2g·0.3s 또는 고도 10m — 한 센서만 살아도 감지)
//   4. 사출 로직은 비행 모드에서만 → 대기/준비는 구조적으로 사출 불가
//   5. 비상사출은 모드·잠금 전부 무시 (유일한 방어선 = GCS 2단계 확인)
//   6. 사출 후 서보 목표각 반복 재명령 (일시 실패 대비)
//
//  ┌─────────┐ SET_ARMED ┌─────────┐ 발사감지(OR) ┌─────────┐
//  │ 0 대기  │──────────>│ 1 준비  │─────────────>│ 2 비행  │
//  │  SAFE   │<──────────│  ARMED  │  FORCE_FLIGHT│ FLIGHT  │
//  └─────────┘SET_STANDBY└─────────┘              └────┬────┘
//       ▲                                  주사출(정점) │ 보조사출(10s)
//       │                                               v
//  ┌─────────┐  착륙감지  ┌─────────┐             ┌─────────┐
//  │ 4 착륙  │<───────────│ 3 낙하  │<────────────│  사출!  │
//  │ LANDED  │ FORCE_LAND │ DESCENT │  FORCE_EJECT(어디서든)
//  └─────────┘            └─────────┘
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include "Config.h"
#include "Packet.h"
#include "Sensors.h"
#include "Logger.h"

// ── 모드 정의 ─────────────────────────────────────────────────────────────
enum Mode : uint8_t {
  MODE_SAFE    = 0,   // 대기: 명령 대기만. 사출 로직 없음
  MODE_ARMED   = 1,   // 준비: 기준압 수집 완료, 50Hz 계측+로깅+발사감지
  MODE_FLIGHT  = 2,   // 비행: 발사 감지됨! 3초 후 주사출 검사, 10초 보조사출
  MODE_DESCENT = 3,   // 낙하: 사출 완료, 로깅 지속 + 착륙감지 + 서보 재명령
  MODE_LANDED  = 4,   // 착륙: SD 닫기, 부저 비콘, 텔레메트리는 계속
};

// ── 수신 명령 (7바이트: 0x3C 0x3C | CMD×4 | CRC8) ────────────────────────
#define CMD_SET_STANDBY  0x0B   // → 대기
#define CMD_SET_ARMED    0x16   // → 준비 (기준압 수집 포함)
#define CMD_FORCE_FLIGHT 0x2D   // → 비행 강제 진입 (센서 무관, 타이머 시작)
#define CMD_FORCE_EJECT  0x42   // 즉시 사출 (모드·잠금 무시) → 낙하
#define CMD_FORCE_LAND   0x51   // → 착륙 강제 (SD 닫기)
#define CMD_SYSRESET     0x63   // 소프트 리셋 (대기 모드에서만)

// ── 전역 상태 ─────────────────────────────────────────────────────────────
static Servo      g_servo;
static DataPacket g_pkt;
static uint8_t    g_mode        = MODE_SAFE;
static bool       g_ejected     = false;
static uint32_t   g_launchMs    = 0;      // 발사 감지 시각
static uint16_t   g_cmdRxCount  = 0;      // 유효 명령 누적 수신 (패킷으로 보고)
static bool       g_sdOk        = false;

// 발사감지 상태
static uint32_t   g_zaccStartMs = 0;
static bool       g_zaccActive  = false;
static uint8_t    g_altHitCount = 0;

// 주사출 상태
static float      g_peakAltM    = 0.0f;
static uint8_t    g_dropCount   = 0;

// 착륙감지 상태
static uint32_t   g_landStableMs = 0;
static int32_t    g_lastAltCm    = 0;

// 착륙 처리 1회 플래그
static bool       g_logClosed   = false;

// ═══════════════════════════════════════════════════════════════════════════
//  사출 (한 곳에서만 실행 — 어디서 호출되든 동일 동작)
// ═══════════════════════════════════════════════════════════════════════════
static void doEject(const char *reason) {
  if (!g_ejected) {
    g_ejected = true;
    g_servo.write(SERVO_EJECT_DEG);
    Serial.print("[EJECT] "); Serial.println(reason);
  }
  g_mode = MODE_DESCENT;
}

// ═══════════════════════════════════════════════════════════════════════════
//  모드 전환 (진입 시 필요한 초기화를 한 곳에)
// ═══════════════════════════════════════════════════════════════════════════
static void enterArmed() {
  collectBaseline();          // 약 1초 블로킹 — 발사 전 지상에서만 호출됨
  g_zaccActive  = false;
  g_altHitCount = 0;
  g_peakAltM    = 0.0f;
  g_dropCount   = 0;
  g_mode = MODE_ARMED;
  Serial.println("[MODE] 1 준비 (발사감지 대기)");
}

static void enterFlight(const char *how) {
  g_launchMs = millis();
  g_mode = MODE_FLIGHT;
  Serial.print("[MODE] 2 비행 — 발사 감지 ("); Serial.print(how); Serial.println(")");
}

static void enterLanded() {
  g_mode = MODE_LANDED;
  Serial.println("[MODE] 4 착륙");
}

// ═══════════════════════════════════════════════════════════════════════════
//  명령 수신 (0x3C 0x3C | CMD×4 다수결 | CRC8)
// ═══════════════════════════════════════════════════════════════════════════
static uint8_t crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0xFF;
  for (uint8_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (c << 1) ^ 0x07 : (uint8_t)(c << 1);
  }
  return c;
}

static void handleCommand(uint8_t cmd) {
  g_cmdRxCount++;   // 유효 명령이면 종류 무관 카운트 (GCS 재전송 확인용)

  switch (cmd) {
    case CMD_SET_STANDBY:
      g_mode = MODE_SAFE;
      if (!g_ejected) g_servo.write(SERVO_CLOSED_DEG);
      Serial.println("[MODE] 0 대기");
      break;
    case CMD_SET_ARMED:
      enterArmed();
      break;
    case CMD_FORCE_FLIGHT:
      enterFlight("수동 강제");
      break;
    case CMD_FORCE_EJECT:
      doEject("지상국 비상 명령");   // 모드·잠금 무시, 어디서든
      break;
    case CMD_FORCE_LAND:
      enterLanded();
      break;
    case CMD_SYSRESET:
      if (g_mode == MODE_SAFE) NVIC_SystemReset();
      break;
    default:
      g_cmdRxCount--;   // 모르는 명령은 카운트 취소
      break;
  }
}

static void pollCommands() {
  static uint8_t buf[8];
  static uint8_t pos = 0;

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (pos == 0 && b != 0x3C) continue;
    if (pos == 1 && b != 0x3C) { pos = 0; continue; }
    buf[pos++] = b;
    if (pos >= 7) {
      pos = 0;
      // 다수결: CMD 4개 중 2개 이상 일치
      uint8_t cmd = 0; bool ok = false;
      for (uint8_t i = 2; i < 6 && !ok; i++)
        for (uint8_t j = i + 1; j < 6 && !ok; j++)
          if (buf[i] == buf[j]) { cmd = buf[i]; ok = true; }
      if (ok && buf[6] == crc8(buf, 6)) handleCommand(cmd);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  발사 감지 (준비 모드, OR 조건)
// ═══════════════════════════════════════════════════════════════════════════
static void checkLaunch(uint32_t now) {
  bool imuOk = (sensorStatus() & STATUS_IMU);
  bool bmpOk = (g_pkt.altitude_cm != -1);

  // 경로 A: Z축 가속도 2g 이상 0.3초 연속
  if (imuOk && g_pkt.acc[2] >= LAUNCH_ZACC_MG) {
    if (!g_zaccActive) { g_zaccActive = true; g_zaccStartMs = now; }
    if (now - g_zaccStartMs >= LAUNCH_ZACC_MS) { enterFlight("가속도 2g/0.3s"); return; }
  } else {
    g_zaccActive = false;
  }

  // 경로 B: 고도 10m 이상 5회 연속
  if (bmpOk && g_pkt.altitude_cm >= (int32_t)(LAUNCH_ALT_M * 100)) {
    if (++g_altHitCount >= LAUNCH_ALT_COUNT) { enterFlight("고도 10m"); return; }
  } else {
    g_altHitCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  주 사출 + 보조 사출 (비행 모드)
// ═══════════════════════════════════════════════════════════════════════════
static void checkEjection(uint32_t now) {
  uint32_t sinceLaunch = now - g_launchMs;

  // 보조 사출: 발사 10초 후 묻지도 따지지도 않고 (센서·잠금 무관)
  if (sinceLaunch >= BACKUP_TIMER_MS) {
    doEject("보조 사출 (10초 타이머)");
    return;
  }

  // Safe Lock: 발사 후 3초간 주 사출 검사 시작 안 함 (정점 추적은 계속)
  bool bmpOk = (g_pkt.altitude_cm != -1);
  float altM = bmpOk ? g_pkt.altitude_cm / 100.0f : 0.0f;
  if (bmpOk && altM > g_peakAltM) { g_peakAltM = altM; g_dropCount = 0; }

  if (sinceLaunch < SAFE_LOCK_MS) return;

  // 주 사출: 최고고도 30m 이상 + 정점 대비 3m 하강 5회 연속
  if (bmpOk && g_peakAltM >= MIN_APOGEE_ALT_M
            && (g_peakAltM - altM) >= APOGEE_DROP_M) {
    if (++g_dropCount >= APOGEE_DROP_COUNT)
      doEject("주 사출 (정점 통과)");
  } else if (bmpOk) {
    g_dropCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  착륙 감지 (낙하 모드)
// ═══════════════════════════════════════════════════════════════════════════
static void checkLanding(uint32_t now) {
  if (g_pkt.altitude_cm == -1) return;   // BMP 실패 시 판단 보류

  float ax = g_pkt.acc[0], ay = g_pkt.acc[1], az = g_pkt.acc[2];
  float netMg = sqrtf(ax * ax + ay * ay + az * az);

  bool altStable = (labs(g_pkt.altitude_cm - g_lastAltCm) < LAND_ALT_CM_THRESH);
  bool accStable = (netMg >= LAND_ACC_MIN_MG && netMg <= LAND_ACC_MAX_MG);

  if (altStable && accStable) {
    if (g_landStableMs == 0) g_landStableMs = now;
    if (now - g_landStableMs >= LAND_STABLE_MS) enterLanded();
  } else {
    g_landStableMs = 0;
  }
  g_lastAltCm = g_pkt.altitude_cm;
}

// ═══════════════════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(BAUD_USB);
  Serial1.begin(BAUD_TELEMETRY);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // USB CDC 연결 대기

  g_servo.attach(PIN_SERVO);
  g_servo.write(SERVO_CLOSED_DEG);

  initSensors();
  g_sdOk = initSD();
  if (!g_sdOk) Serial.println("[ERR] SD 초기화 실패 — 로깅 없이 진행");

  Serial.print("[BOOT] version3  mode=0 대기  status=0x");
  Serial.println(sensorStatus() | (g_sdOk ? STATUS_SD : 0), HEX);
  // 재부팅 시 항상 대기 모드에서 시작 (모드 저장/복원 없음 — 단순·안전)
}

void loop() {
  uint32_t now = millis();

  pollCommands();

  // ── 50Hz: 계측 + 모드 로직 + 로깅 ─────────────────────────────────────
  static uint32_t lastSensorMs = 0;
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;

    readSensors(g_pkt);
    g_pkt.flight_mode   = g_mode;
    g_pkt.eject_state   = g_ejected ? 1 : 0;
    g_pkt.system_status = sensorStatus() | (g_sdOk ? STATUS_SD : 0);
    g_pkt.cmd_rx_count  = g_cmdRxCount;

    switch (g_mode) {
      case MODE_ARMED:   checkLaunch(now);   break;
      case MODE_FLIGHT:  checkEjection(now); break;
      case MODE_DESCENT: checkLanding(now);  break;
      default: break;
    }

    // 준비~낙하 구간만 SD 기록 (발사 전 데이터부터 착륙까지)
    if (g_mode >= MODE_ARMED && g_mode <= MODE_DESCENT) {
      finalizePacket(g_pkt);
      writePacket(g_pkt);
    }
  }

  // ── 5Hz: 텔레메트리 (모든 모드에서 항상 — GCS가 로켓 상태를 항상 봄) ──
  // seq는 여기(전송 시점)에서만 증가 — GCS 손실률 계산이 정확해짐
  // (SD 로그에는 같은 seq가 최대 10행 반복되지만 ms로 구분되고, 디코더 손실
  //  계산은 gap>1만 세므로 영향 없음)
  static uint32_t lastTelemMs = 0;
  if (now - lastTelemMs >= TELEM_INTERVAL_MS) {
    lastTelemMs = now;
    g_pkt.seq++;
    finalizePacket(g_pkt);
    Serial1.write((const uint8_t*)&g_pkt, sizeof(g_pkt));
  }

  // ── 사출 후 서보 목표각 반복 재명령 (일시 실패 대비) ──────────────────
  static uint32_t lastServoMs = 0;
  if (g_ejected && now - lastServoMs >= SERVO_RECMD_MS) {
    lastServoMs = now;
    g_servo.write(SERVO_EJECT_DEG);
  }

  // ── 착륙 후 SD 마무리 (1회) ───────────────────────────────────────────
  if (g_mode == MODE_LANDED && !g_logClosed) {
    flushLog();
    closeLog();
    g_logClosed = true;
    Serial.println("[SD] 로그 저장 완료 — 전원 분리 안전");
  }

  // ── 대기 모드 하트비트 (1초, 생존 확인용) ─────────────────────────────
  static uint32_t lastBeatMs = 0;
  if (g_mode == MODE_SAFE && now - lastBeatMs >= 1000) {
    lastBeatMs = now;
    Serial.print("[대기] status=0x");
    Serial.println(g_pkt.system_status, HEX);
  }
}
