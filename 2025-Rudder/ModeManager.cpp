#include "ModeManager.h"
#include "Logger.h"
#include <Arduino.h>


SdFile modeFile;
bool ModeManager::initSD() {
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(18))) {
    Serial.println("[SD] 초기화 실패");
    return false;
  }
  Serial.println("[SD] 초기화 성공");
  return true;
}//미사용

bool ModeManager::save() {
  modeFile.close();  // 혹시 열려있다면 닫기
  if (!modeFile.open(modeFileName, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("[SD] 파일 열기 실패 (쓰기)");
    finalValue = 	0xBA; //186
    return false;
  }
  modeFile.println(mode);
  modeFile.close();
  Serial.print("[SD] 모드 저장됨: ");
  Serial.println(mode);
  return true;
}

bool ModeManager::load() {
  modeFile.close();  // 혹시 열려있다면 닫기

  if (!sd.exists(modeFileName)) {
    Serial.println("[SD] mode.txt 없음 → 기본값 사용");
    return false;
  }

  if (!modeFile.open(modeFileName, O_READ)) {
    Serial.println("[SD] 파일 열기 실패 (읽기)");
    finalValue = 		0xBD; //189
    return false;
  }

  String str = "";
  while (modeFile.available()) {
    char c = modeFile.read();
    if (c == '\n') break;
    str += c;
  }
  modeFile.close();

  int loaded = str.toInt();
  if (loaded >= MIN_MODE && loaded <= MAX_MODE) {
    mode = loaded;
    Serial.print("[SD] 불러온 모드 값: ");
    Serial.println(mode);
    return true;
  } else {
    Serial.println("[SD] 불러온 모드가 범위 초과 → 기본값 유지");
    return false;
  }
}

void ModeManager::setMode(int newMode) {
  if (newMode >= MIN_MODE && newMode <= MAX_MODE) {
    mode = newMode;
    Serial.print("[ModeManager] Mode set to: ");
    Serial.println(mode);
  } else {
    Serial.print("[ModeManager] Invalid mode: ");
    Serial.print(newMode);
    Serial.println(" (Allowed: 0~6)");
  }
}

int ModeManager::getMode() const {
  return mode;
}

void ModeManager::nextMode() {
  if (mode < MAX_MODE) {
    mode++;
    Serial.print("[ModeManager] Mode increased to: ");
    Serial.println(mode);
  } else {
    Serial.println("[ModeManager] Already at MAX_MODE (6)");
  }
}

void ModeManager::prevMode() {
  if (mode > MIN_MODE) {
    mode--;
    Serial.print("[ModeManager] Mode decreased to: ");
    Serial.println(mode);
  } else {
    Serial.println("[ModeManager] Already at MIN_MODE (0)");
  }
}


//modeManager.setMode(2);  // 모드 설정
//Serial.println(modeManager.getMode());  // 모드 확인