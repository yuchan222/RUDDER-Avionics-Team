#include "Logger.h"

//SdFat sd;
SdFile file2;
bool logging = false;

uint32_t pktCnt = 0;
uint32_t startMillis;

uint16_t crc16_update(uint16_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  return crc;
}

void makeFilename(char* buf) {
  for (uint16_t n = 1; n < 9999; n++) {
    sprintf(buf, "LOG%05u.RAW", n);
    if (!sd.exists(buf)) return;
  }
  strcpy(buf, "LOGFULL.RAW");
}

void initSD() {
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(18))) {
    Serial.println("SD fail");
    //while (1);
    finalValue = 0xB4; //180
  }
  Serial.println("Init SD Done");
}

void initSDlog() {
  char fname[16];
  makeFilename(fname);
  if (!file2.open(fname, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("file open fail");
    finalValue = 0xB7; //183
    //while (1);
  }
  #if FILE_PREALLOC_MB > 0
    file2.preAllocate((uint32_t)FILE_PREALLOC_MB * 1024UL * 1024UL);
  #endif
  Serial.print("log file → ");
  Serial.println(fname);
  Serial.println("Init SDlog Done");
}

void writePacket(const DataPacket &p) {
  DataPacket copy = p;
  copy.len = sizeof(DataPacket);
  copy.pkt_no = ++pktCnt;
  copy.ms = millis() - startMillis;

  uint16_t crc = 0xFFFF;
  const uint8_t* b = (uint8_t*)&copy;
  for (size_t i = 0; i < sizeof(DataPacket) - 4; i++)
    crc = crc16_update(crc, b[i]);
  copy.crc16 = crc;

  file2.write((uint8_t*)&copy, sizeof(DataPacket));
  if ((pktCnt % SAMPLES_SYNC) == 0) file2.sync();
}

void startLogging() {
  Serial.println(F(">>> LOGGING START"));
  logging = true;
  pktCnt = 0;
  startMillis = millis();
}

void stopLogging() {
  logging = false;
  file2.sync();
  file2.close();
  Serial.println(F(">>> LOGGING STOP — file closed"));
}
