// ═══════════════════════════════════════════════════════════════════════════
//  RUDDER 2026 — version6 비행 소프트웨어
//
//  설계 원칙 (2026-07-22 팀 합의):
//   1. 모드 = 실제 사건과 1:1 대응 (모드만 봐도 로켓이 뭘 하는지 안다)
//   2. 모든 모드 전환에 수동 명령 백업 (센서가 죽어도 사람이 진행 가능)
//   3. 발사감지 OR 조건 (가속도 2g·0.3s 또는 고도 10m — 한 센서만 살아도 감지)
//   4. 사출 로직은 비행 모드에서만 → 대기/준비는 구조적으로 사출 불가
//   5. 비상사출은 준비·비행·낙하 중엔 잠금 무시하고 무조건 실행
//      단, 대기·착륙 상태에서는 거부 (오작동/지연패킷으로 인한 오사출 방지)
//   6. 사출 후 서보 목표각 반복 재명령 (일시 실패 대비)
//   7. 모드 전환 명령은 정해진 출발 모드에서만 허용 (오작동·오전송·지연도착 방지)
//      단, FORCE_EJECT만 예외 — 모드·잠금 전부 무시
//
//  ┌─────────┐ SET_ARMED ┌─────────┐ 발사감지(OR) ┌─────────┐
//  │ 0 대기  │──────────>│ 1 준비  │─────────────>│ 2 비행  │
//  │  SAFE   │<──────────│  ARMED  │  FORCE_FLIGHT│ FLIGHT  │
//  └─────────┘SET_STANDBY└─────────┘              └────┬────┘
//       ▲                                  주사출(정점) │ 보조사출(10s)
//       │                                               v
//  ┌─────────┐ FORCE_LAND ┌─────────┐             ┌─────────┐
//  │ 4 착륙  │<───────────│ 3 낙하  │<────────────│  사출!  │
//  │ LANDED  │  (수동전용,│ DESCENT │  FORCE_EJECT(준비·비행·낙하중)
//  └─────────┘  자동판정없음)└───────┘
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
  MODE_FLIGHT  = 2,   // 비행: 발사 감지됨! SAFE_LOCK_MS 후 주사출 검사, 10초 보조사출
  MODE_DESCENT = 3,   // 낙하: 사출 완료, 로깅 지속 + 서보 재명령 (착륙은 자동판정 없음, FORCE_LAND로만)
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
static uint8_t    g_ejectReason = EJECT_REASON_NONE;   // 사출 사유 (패킷으로 보고)
static uint8_t    g_launchReasonBits = 0;               // STATUS_LAUNCH_ACCEL/ALT (패킷으로 보고)
static uint32_t   g_launchMs    = 0;      // 발사 감지 시각
static uint16_t   g_cmdRxCount  = 0;      // 유효 명령 누적 수신 (패킷으로 보고)
static uint8_t    g_lastCmd     = 0;      // 가장 최근 수신한 명령 바이트 (패킷으로 보고)
static bool       g_sdOk        = false;
static bool       g_baselineOk  = false;  // 마지막 ARMED 진입 시 기준압 수집 성공 여부 (아직 수집 전엔 false가 맞음)
static uint8_t    g_servoRecmdLeft = 0;   // 사출 후 남은 재명령 횟수 (무기한 반복 방지)

// 발사감지 상태
static uint32_t   g_zaccStartMs = 0;
static bool       g_zaccActive  = false;
static uint8_t    g_altHitCount = 0;

// 주사출 상태
static float      g_peakAltM    = 0.0f;
static uint8_t    g_dropCount   = 0;
static uint8_t    g_peakConfirmCount = 0;   // 최고고도 갱신 연속 확인 카운터 (단일 스파이크 방지)

// 착륙 처리 1회 플래그
static bool       g_logClosed   = false;
static uint32_t   g_landedMs    = 0;      // 착륙 진입 시각 (서보 복귀 유지시간 계산용)
static bool       g_servoOffDone = false; // 착륙 후 전원차단 완료 여부 (1회만 실행)

// ═══════════════════════════════════════════════════════════════════════════
//  사출 (한 곳에서만 실행 — 어디서 호출되든 동일 동작)
// ═══════════════════════════════════════════════════════════════════════════
static void doEject(uint8_t reasonCode, const char *reasonStr) {
  if (!g_ejected) {
    g_ejected = true;
    g_ejectReason = reasonCode;
    g_servo.write(SERVO_EJECT_DEG);
    g_servoRecmdLeft = SERVO_RECMD_COUNT;
    Serial.print("[EJECT] "); Serial.println(reasonStr);
  }
  g_mode = MODE_DESCENT;
}

// 서보 전원 완전 차단 — 착륙(회수) 후 통전 유지할 이유가 없어 무조건 호출
static void servoSafeOff() {
  if (g_servo.attached()) g_servo.detach();
  pinMode(PIN_SERVO, INPUT);
  digitalWrite(PIN_ARM_EN, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════
//  모드 전환 (진입 시 필요한 초기화를 한 곳에)
// ═══════════════════════════════════════════════════════════════════════════
static void enterArmed() {
  g_baselineOk = collectBaseline();   // 약 1초 블로킹 — 발사 전 지상에서만 호출됨
  if (!g_baselineOk) Serial.println("[경고] 기준압 수집 실패 — 고도 신뢰 불가, 그래도 준비 모드 진입");
  g_zaccActive  = false;
  g_altHitCount = 0;
  g_peakAltM    = 0.0f;
  g_dropCount   = 0;
  g_peakConfirmCount = 0;
  g_launchReasonBits = 0;
  g_mode = MODE_ARMED;
  Serial.println("[MODE] 1 준비 (발사감지 대기)");
}

static void enterFlight(uint32_t now, const char *how) {
  g_launchMs = now;   // loop()의 now를 그대로 사용 — millis() 재호출 시 값이 갈라지는 것 방지
  g_mode = MODE_FLIGHT;
  Serial.print("[MODE] 2 비행 — 발사 감지 ("); Serial.print(how); Serial.println(")");
}

static void enterLanded() {
  g_mode = MODE_LANDED;
  g_landedMs = millis();
  g_servoOffDone = false;
  g_servo.write(SERVO_CLOSED_DEG);   // 닫힘 위치로 복귀 시도 — SERVO_CLOSE_HOLD_MS 동안 전원 유지 후 차단
  Serial.println("[MODE] 4 착륙 — 서보 복귀 중");
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

static void handleCommand(uint8_t cmd, uint32_t now) {
  g_cmdRxCount++;   // 유효 명령이면 종류 무관 카운트 (GCS 재전송 확인용)
  g_lastCmd = cmd;  // 사후분석용 — 거부된 명령이라도 어떤 시도였는지는 남김

  switch (cmd) {
    case CMD_SET_STANDBY:
      if (g_mode != MODE_ARMED) { Serial.println("[거부] 대기 전환 — 준비 상태에서만 가능"); break; }
      g_mode = MODE_SAFE;
      if (!g_ejected) g_servo.write(SERVO_CLOSED_DEG);
      Serial.println("[MODE] 0 대기");
      break;
    case CMD_SET_ARMED:
      if (g_mode != MODE_SAFE) { Serial.println("[거부] 준비 전환 — 대기 상태에서만 가능"); break; }
      enterArmed();
      break;
    case CMD_FORCE_FLIGHT:
      if (g_mode != MODE_ARMED) { Serial.println("[거부] 비행 강제진입 — 준비 상태에서만 가능"); break; }
      enterFlight(now, "수동 강제");
      break;
    case CMD_FORCE_EJECT:
      if (g_mode == MODE_SAFE || g_mode == MODE_LANDED) {
        Serial.println("[거부] 비상사출 — 대기·착륙 상태에서는 무시");
        break;
      }
      doEject(EJECT_REASON_MANUAL, "지상국 비상 명령");   // 준비·비행·낙하 중엔 그대로 무조건 실행
      break;
    case CMD_FORCE_LAND:
      if (g_mode != MODE_DESCENT) { Serial.println("[거부] 착륙 강제전환 — 낙하 상태에서만 가능"); break; }
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

static void pollCommands(uint32_t now) {
  static uint8_t buf[8];
  static uint8_t pos = 0;

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (pos == 0 && b != 0x3C) continue;
    if (pos == 1 && b != 0x3C) { pos = 0; continue; }
    buf[pos++] = b;
    if (pos >= 7) {
      pos = 0;
      // 4개 완전 일치해야 채택 (2:2처럼 모호한 프레임은 거부)
      bool allSame = buf[2] == buf[3] && buf[2] == buf[4] && buf[2] == buf[5];
      if (allSame && buf[6] == crc8(buf, 6)) handleCommand(buf[2], now);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  발사 감지 (준비 모드, OR 조건)
// ═══════════════════════════════════════════════════════════════════════════
static void checkLaunch(uint32_t now) {
  bool imuOk = (sensorStatus() & STATUS_IMU);
  bool bmpOk = (g_pkt.altitude_cm != ALT_INVALID) && g_baselineOk;   // 기준압 자체가 불량이면 고도 경로 안 씀

  // 경로 A: 세로축(LAUNCH_AXIS) 가속도 2g 이상 0.3초 연속
  // 부호 보정은 Sensors.cpp readSensors()에서 이미 적용됨 — g_pkt.acc는 항상 "+ = 상승"
  if (imuOk && g_pkt.acc[LAUNCH_AXIS] >= LAUNCH_ZACC_MG) {
    if (!g_zaccActive) { g_zaccActive = true; g_zaccStartMs = now; }
    if (now - g_zaccStartMs >= LAUNCH_ZACC_MS) {
      g_launchReasonBits = STATUS_LAUNCH_ACCEL;
      enterFlight(now, "가속도 2g/0.3s");
      return;
    }
  } else {
    g_zaccActive = false;
  }

  // 경로 B: 고도 10m 이상 5회 연속
  if (bmpOk && g_pkt.altitude_cm >= (int32_t)(LAUNCH_ALT_M * 100)) {
    if (++g_altHitCount >= LAUNCH_ALT_COUNT) {
      g_launchReasonBits = STATUS_LAUNCH_ALT;
      enterFlight(now, "고도 10m");
      return;
    }
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
    doEject(EJECT_REASON_BACKUP, "보조 사출 (10초 타이머)");
    return;
  }

  // Safe Lock: 발사 후 SAFE_LOCK_MS(현재 4초)간 주 사출 검사 시작 안 함 (정점 추적은 계속)
  bool bmpOk = (g_pkt.altitude_cm != ALT_INVALID) && g_baselineOk;   // 기준압 자체가 불량이면 고도 경로 안 씀
  float altM = bmpOk ? g_pkt.altitude_cm / 100.0f : 0.0f;

  // 최고고도 갱신은 3회 연속 확인돼야 반영 — 단일 샘플 스파이크가 영구 오염시키는 것 방지
  // BMP 실패 시 "연속" 카운터를 리셋해 띄엄띄엄한 샘플이 조건을 채우는 것 방지
  if (bmpOk) {
    if (altM > g_peakAltM) {
      if (++g_peakConfirmCount >= PEAK_CONFIRM_COUNT) {
        g_peakAltM = altM;
        g_dropCount = 0;
        g_peakConfirmCount = 0;
      }
    } else {
      g_peakConfirmCount = 0;
    }
  } else {
    g_peakConfirmCount = 0;
    g_dropCount = 0;
  }

  if (sinceLaunch < SAFE_LOCK_MS) return;

  // 주 사출: 최고고도 30m 이상 + 정점 대비 3m 하강 5회 연속
  if (bmpOk && g_peakAltM >= MIN_APOGEE_ALT_M
            && (g_peakAltM - altM) >= APOGEE_DROP_M) {
    if (++g_dropCount >= APOGEE_DROP_COUNT)
      doEject(EJECT_REASON_PRIMARY, "주 사출 (정점 통과)");
  } else if (bmpOk) {
    g_dropCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  setup / loop
//  (착륙은 자동판정하지 않음 — 회수 후 GCS에서 FORCE_LAND로 수동 전환.
//   단일 고도/가속도 임계값 기반 자동판정은 지속 하강 중에도 "안정"으로
//   오판할 수 있어 제거함. DESCENT 상태에서 위험한 동작은 이미 없으므로
//   착륙 판정이 늦어도 안전에 지장 없음.)
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  pinMode(PIN_ARM_EN, OUTPUT);
  digitalWrite(PIN_ARM_EN, LOW);   // 부팅 즉시 서보 전원 차단 (기본 안전 — 그 무엇보다 먼저)

  Serial.begin(BAUD_USB);
  Serial1.begin(BAUD_TELEMETRY);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // USB CDC 연결 대기

  g_servo.attach(PIN_SERVO);
  g_servo.write(SERVO_CLOSED_DEG);

  initSensors();
  g_sdOk = initSD();
  if (!g_sdOk) Serial.println("[ERR] SD 초기화 실패 — 로깅 없이 진행");

  Serial.print("[BOOT] version6  mode=0 대기  status=0x");
  Serial.println(sensorStatus() | (g_sdOk ? STATUS_SD : 0), HEX);
  // 재부팅 시 항상 대기 모드에서 시작 (모드 저장/복원 없음 — 단순·안전)
}

void loop() {
  uint32_t now = millis();

  pollCommands(now);
  // 서보 전원(MOSFET) — 상태기반: 매 루프 현재 모드로 재확정 (전환지점 하나하나 챙길 필요 없음)
  // SAFE는 통전 불필요. LANDED는 닫힘 복귀에 필요한 SERVO_CLOSE_HOLD_MS 동안만 유지하고
  // 그 뒤(g_servoOffDone)엔 차단 (무기한 통전 방지는 유지, 복귀에 필요한 시간만 예외)
  bool landedPowerOff = (g_mode == MODE_LANDED && g_servoOffDone);
  digitalWrite(PIN_ARM_EN, (g_mode == MODE_SAFE || landedPowerOff) ? LOW : HIGH);

  // ── 50Hz: 계측 + 모드 로직 + 로깅 ─────────────────────────────────────
  static uint32_t lastSensorMs = 0;
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;

    readSensors(g_pkt);

    // 상태기계 먼저 실행 → 이 샘플에서 전환이 일어나면 아래 패킷화가 전환
    // "이후"의 최종 상태를 담게 됨 (사건 발생 샘플이 이전 모드로 기록되던 것 방지)
    switch (g_mode) {
      case MODE_ARMED:  checkLaunch(now);   break;
      case MODE_FLIGHT: checkEjection(now); break;
      // MODE_DESCENT: 자동판정 없음 — FORCE_LAND(수동)로만 착륙 전환
      default: break;
    }

    g_pkt.flight_mode   = g_mode;
    g_pkt.eject_state   = g_ejectReason;
    // SD 상태는 isLogReady()로 매번 실제 확인 (쓰기 실패 시 그 즉시 반영됨 — 초기화 성공에 영구 고정 안 됨)
    g_pkt.system_status = sensorStatus() | (isLogReady() ? STATUS_SD : 0)
                         | (g_baselineOk ? 0 : STATUS_BASELINE_BAD)
                         | (g_logClosed ? STATUS_LOG_CLOSED : 0)
                         | g_launchReasonBits;
    g_pkt.cmd_rx_count  = g_cmdRxCount;
    g_pkt.last_cmd      = g_lastCmd;

    // 준비~낙하 구간만 SD 기록 (발사 전 데이터부터 착륙까지)
    if (g_mode >= MODE_ARMED && g_mode <= MODE_DESCENT) {
      finalizePacket(g_pkt);
      writePacket(g_pkt);   // 실패하면 isLogReady()가 다음 주기부터 false로 반영됨
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

  // ── 사출 후 서보 목표각 재명령 (일시 실패 대비, 최대 SERVO_RECMD_COUNT회) ──
  static uint32_t lastServoMs = 0;
  if (g_ejected && g_mode != MODE_LANDED && g_servoRecmdLeft > 0
      && now - lastServoMs >= SERVO_RECMD_MS) {
    lastServoMs = now;
    g_servo.write(SERVO_EJECT_DEG);
    g_servoRecmdLeft--;
  }

  // ── 착륙 후 서보 복귀 유지시간 경과하면 전원 차단 (1회) ──────────────────
  if (g_mode == MODE_LANDED && !g_servoOffDone && now - g_landedMs >= SERVO_CLOSE_HOLD_MS) {
    servoSafeOff();
    g_servoOffDone = true;
    Serial.println("[SERVO] 닫힘 복귀 완료 — 전원 차단");
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
