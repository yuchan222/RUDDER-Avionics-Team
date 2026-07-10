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

// ── 전역 객체 ─────────────────────────────────────────────────────────────
static Servo       myServo;
static DataPacket  pkt;
static ModeManager mode;
static CommandRx   cmdRx(Serial1);

static uint32_t telCounter = 0;

// ── 명령 처리 ─────────────────────────────────────────────────────────────
static void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_MODE_NEXT:
      mode.next();
      break;
    case CMD_MODE_PREV:
      mode.prev();
      break;
    case CMD_EJECT:
      forceEject();
      mode.set(3);   // 비상 사출 시 즉시 mode 3으로
      break;
    case CMD_SYSRESET:
      NVIC_SystemReset();
      break;
    default:
      break;
  }
}

// ── setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_USB);
  cmdRx.begin(BAUD_TELEMETRY);

  initLEDBuzzer();

  myServo.attach(PIN_SERVO);
  myServo.write(0);

  clearI2CBus();
  initSensors();
  initEjection(myServo);

  if (initSD()) {
    mode.begin();
    openLogFile();
  } else {
    Serial.println("[ERR] SD init failed — mode defaults to 0");
  }

  Serial.print("[BOOT] mode=");
  Serial.println(mode.get());
}

// ── loop ──────────────────────────────────────────────────────────────────
void loop() {
  // 명령 수신 (전 모드 공통)
  if (cmdRx.update()) {
    handleCommand(cmdRx.lastCmd());
  }

  uint8_t m = mode.get();
  updateLEDBuzzer(m);

  switch (m) {
    // ── Mode 0: 대기 ──────────────────────────────────────────────────────
    case 0:
      // CMD_MODE_NEXT 수신 시 Mode 1로 전환
      break;

    // ── Mode 1: Armed ─────────────────────────────────────────────────────
    // collectBaseline() 호출(약 1초 blocking) 후 자동으로 Mode 2 전환
    case 1:
      Serial.println("[MODE1] Armed: collecting baseline...");
      collectBaseline();
      mode.next();   // -> Mode 2
      break;

    // ── Mode 2: 비행 (발사 감지 + 사출 판단 + 로깅) ───────────────────────
    case 2:
      readSensors(pkt);
      pkt.pkt_no++;
      pkt.seq++;
      writePacket(pkt);

      if (updateEjection(pkt)) {
        mode.set(3);
      }

      // 텔레메트리: 100루프마다 1회 전송
      if (++telCounter >= 100) {
        telCounter = 0;
        Serial1.write((const uint8_t*)&pkt, sizeof(pkt));
      }
      break;

    // ── Mode 3: 사출 완료 (로깅 계속) ─────────────────────────────────────
    case 3:
      readSensors(pkt);
      pkt.pkt_no++;
      pkt.seq++;
      writePacket(pkt);

      if (++telCounter >= 100) {
        telCounter = 0;
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
