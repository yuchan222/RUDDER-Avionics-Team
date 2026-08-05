#include "Logger.h"
#include <SdFat.h>
#include <stddef.h>

static SdFat     s_sd;
static FsFile    s_file;
static uint32_t  s_pktCount = 0;
static bool      s_ready    = false;

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
  if (!s_sd.begin(PIN_SD_CS, SD_SCK_MHZ(4))) {
    s_ready = false;
    return false;
  }
  char fname[16];
  // uint16_t 최댓값(65535)이 99999보다 작아서 n<=99999가 사실상 항상 참이던
  // 버그 수정 — uint32_t로 바꿔 의도한 범위를 실제로 커버하게 함
  for (uint32_t n = 1; n <= 99999UL; n++) {
    snprintf(fname, sizeof(fname), "LOG%05lu.RAW", (unsigned long)n);
    if (!s_sd.exists(fname)) break;
  }
  s_file = s_sd.open(fname, O_WRONLY | O_CREAT | O_TRUNC);
  if (!s_file) {
    s_ready = false;
    return false;
  }
  s_file.preAllocate((uint32_t)FILE_PREALLOC_MB * 1024UL * 1024UL);
  s_pktCount = 0;
  s_ready    = true;
  Serial.print("[SD] log: "); Serial.println(fname);
  return true;
}

bool isLogReady() { return s_ready; }

void finalizePacket(DataPacket &p) {
  p.len   = (uint16_t)sizeof(DataPacket);
  p.crc16 = crc16((const uint8_t*)&p, (uint16_t)offsetof(DataPacket, crc16));
}

bool writePacket(const DataPacket &p) {
  if (!s_ready) return false;
  if (s_file.write((const uint8_t*)&p, sizeof(p)) != sizeof(p)) {
    s_ready = false;   // 쓰기 실패 즉시 반영 — isLogReady()가 바로 false로 바뀜
    return false;
  }
  if (++s_pktCount % SAMPLES_SYNC == 0 && !s_file.sync()) {
    s_ready = false;
    return false;
  }
  return true;
}

void flushLog() { if (s_ready) s_file.sync();  }
void closeLog() { if (s_ready) { s_file.close(); s_ready = false; } }
