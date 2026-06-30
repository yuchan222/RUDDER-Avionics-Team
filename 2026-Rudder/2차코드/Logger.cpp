#include "Logger.h"
#include <stddef.h>

SdFat sd;

static FsFile    s_file;
static uint32_t  s_pktCount = 0;
static bool      s_logReady = false;

static uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (uint16_t)(crc << 1);
  }
  return crc;
}

bool initSD() {
  s_logReady = sd.begin(PIN_SD_CS, SD_SCK_MHZ(4));
  return s_logReady;
}

bool isLogReady() { return s_logReady; }

bool openLogFile() {
  if (!s_logReady) return false;
  char fname[16];
  for (uint16_t n = 1; n <= 99999; n++) {
    snprintf(fname, sizeof(fname), "LOG%05u.RAW", n);
    if (!sd.exists(fname)) break;
  }
  s_file = sd.open(fname, O_WRONLY | O_CREAT | O_TRUNC);
  if (!s_file) return false;
  s_file.preAllocate((uint32_t)FILE_PREALLOC_MB * 1024UL * 1024UL);
  s_pktCount = 0;
  Serial.print("[SD] log: "); Serial.println(fname);
  return true;
}

// len + CRC16 계산 — SD 저장과 텔레메트리 송신 전에 반드시 호출 (#5, #6)
void finalizePacket(DataPacket &p) {
  p.len   = (uint16_t)sizeof(DataPacket);
  p.crc16 = crc16((uint8_t*)&p, (uint16_t)offsetof(DataPacket, crc16));
}

void writePacket(const DataPacket &p) {
  if (!s_logReady) return;   // SD 실패 시 무시 (#11)
  s_file.write((const uint8_t*)&p, sizeof(p));
  if (++s_pktCount % SAMPLES_SYNC == 0) s_file.sync();
}

void flushLog()  { if (s_logReady) s_file.sync();  }
void closeLog()  { if (s_logReady) s_file.close(); }
