#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include "Config.h"
#include "Packet.h"
#include "SensorManager.h"
#include "EjectionLogic.h"
#include "ModeManager.h"
#include "CommandRx.h"
#include "Logger.h"
#include "LEDBuzzer.h"

static Servo       myServo;
static DataPacket  pkt;
static ModeManager mode;
static CommandRx   cmdRx(Serial1);

static uint8_t  g_sysStatus   = 0;
static uint32_t g_lastSensorMs = 0;
static uint32_t g_lastTelemMs  = 0;

// ── 명령 처리 (#12, #13, #14) ────────────────────────────────────────────
static void handleCommand(uint8_t cmd) {
  uint8_t m = mode.get();
  switch (cmd) {
    case CMD_SET_STANDBY:
      mode.set(0);
      break;
    case CMD_SET_ARMED:
      mode.set(1);
      break;
    case CMD_FORCE_EJECT:
      if (m >= 1) {          // Armed 이상에서만 비상 사출 허용 (#13)
        forceEject();
        mode.set(3);
      }
      break;
    case CMD_SYSRESET:
      if (m == 0) NVIC_SystemReset();  // Standby 상태에서만 리셋 허용 (#14)
      break;
    default:
      break;
  }
}

// ── setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_USB);
  // UNO R4 Minima는 네이티브 USB CDC 시리얼 — 모니터 연결 전에 출력하면
  // 버퍼링 없이 유실됨. 최대 3초간 연결을 기다린 뒤 진행 (타임아웃 시에도
  // 계속 부팅하도록 하여 PC 미연결 상태에서도 보드가 멈추지 않게 함).
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  cmdRx.begin(BAUD_TELEMETRY);

  initLEDBuzzer();

  myServo.attach(PIN_SERVO);
  myServo.write(SERVO_CLOSED_DEG);

  clearI2CBus();
  initSensors();
  initEjection(myServo);

  // 센서 상태 수집 (#10)
  g_sysStatus = getSystemStatus();

  // SD 초기화 (#11)
  if (initSD()) {
    g_sysStatus |= STATUS_SD;
    mode.begin();      // mode.txt 로드 (Mode≥2이면 자동으로 Mode 0) (#21)
    openLogFile();
  } else {
    Serial.println("[ERR] SD init failed — mode defaults to 0");
  }

  setSystemStatus(g_sysStatus);   // LED에 실제 센서 상태 반영 (#19)

  Serial.print("[BOOT] mode="); Serial.print(mode.get());
  Serial.print("  status=0x"); Serial.println(g_sysStatus, HEX);

#ifdef TEST_SKIP_IMU_CHECK
  Serial.println("[WARN] ***** TEST MODE: IMU check skipped — NOT FOR FLIGHT *****");
#endif
}

// ── loop ──────────────────────────────────────────────────────────────────
void loop() {
  if (cmdRx.update()) handleCommand(cmdRx.lastCmd());

  uint8_t  m   = mode.get();
  uint32_t now = millis();
  updateLEDBuzzer(m);

  switch (m) {
    // ── Mode 0: 대기 ──────────────────────────────────────────────────────
    case 0: {
      // 하트비트: Mode 0에서는 원래 출력이 없어 보드 생사 확인이 불가능했음.
      // USB CDC 연결 타이밍과 무관하게 주기적으로 찍어서 "출력이 아예 없다"를
      // "아직 대기 중이다"와 구분할 수 있게 함.
      static uint32_t lastHeartbeatMs = 0;
      if (now - lastHeartbeatMs >= 1000) {
        lastHeartbeatMs = now;
        Serial.print("[MODE0] standby  status=0x");
        Serial.println(g_sysStatus, HEX);
      }
      break;
    }

    // ── Mode 1: Armed ─────────────────────────────────────────────────────
    case 1:
      Serial.println("[MODE1] Armed: collecting baseline...");
      collectBaseline();
      // 핵심 센서(BMP + IMU) 실패 시 Mode 2 진입 금지 (#10)
#ifdef TEST_SKIP_IMU_CHECK
      if (!(g_sysStatus & STATUS_BMP)) {
        Serial.println("[ERR] BMP fail — back to Standby (TEST MODE: IMU check skipped)");
#else
      if (!(g_sysStatus & STATUS_BMP) || !(g_sysStatus & STATUS_IMU)) {
        Serial.println("[ERR] Critical sensor fail — back to Standby");
#endif
        mode.set(0);
      } else {
        setBeepEnabled(false);   // 비행 중 blocking beep 차단 (#20)
        mode.next();             // → Mode 2
      }
      break;

    // ── Mode 2: 비행 (발사 감지 + 사출 판단 + 로깅) ───────────────────────
    case 2:
      // 센서 50Hz (#1)
      if (now - g_lastSensorMs >= SENSOR_INTERVAL_MS) {
        g_lastSensorMs = now;

        readSensors(pkt);
        pkt.pkt_no++;
        pkt.seq++;
        pkt.flight_mode   = m;
        pkt.eject_state   = isEjected() ? 1 : 0;
        pkt.system_status = g_sysStatus;
        finalizePacket(pkt);   // len + CRC16 계산 (#5, #6)
        writePacket(pkt);

        if (updateEjection(pkt)) mode.set(3);
      }

      // 텔레메트리 5Hz (#1)
      if (now - g_lastTelemMs >= TELEM_INTERVAL_MS) {
        g_lastTelemMs = now;
        Serial1.write((const uint8_t*)&pkt, sizeof(pkt));
      }
      break;

    // ── Mode 3: 사출 완료 (로깅 계속 + 착지 감지) ─────────────────────────
    case 3:
      if (now - g_lastSensorMs >= SENSOR_INTERVAL_MS) {
        g_lastSensorMs = now;

        readSensors(pkt);
        pkt.pkt_no++;
        pkt.seq++;
        pkt.flight_mode   = m;
        pkt.eject_state   = 1;
        pkt.system_status = g_sysStatus;
        finalizePacket(pkt);
        writePacket(pkt);

        // 착지 자동 감지 → Mode 4 전환 (#24)
        if (checkLanded(pkt)) {
          Serial.println("[MODE3] Landing detected -> Mode 4");
          mode.set(4);
        }
      }

      if (now - g_lastTelemMs >= TELEM_INTERVAL_MS) {
        g_lastTelemMs = now;
        Serial1.write((const uint8_t*)&pkt, sizeof(pkt));
      }
      break;

    // ── Mode 4: 착지 (SD 닫기 후 대기) ────────────────────────────────────
    case 4:
      flushLog();
      closeLog();
      Serial.println("[MODE4] Landed. Power off or reset.");
      while (true) {
        updateLEDBuzzer(4);
        delay(100);
      }
  }
}
