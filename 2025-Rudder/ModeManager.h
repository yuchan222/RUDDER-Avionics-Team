#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <Arduino.h>
#include <SdFat.h>
#include "CommandRx.h"

extern SdFat sd;
extern SdFile modeFile;
extern uint8_t finalValue;
class ModeManager {
private:
  int mode = 0;
  static const int MIN_MODE = 0;
  static const int MAX_MODE = 6;

  const char* modeFileName = "mode.txt";

public:
  bool initSD();               // SD 카드 초기화
  bool save();                 // 현재 모드 저장
  bool load();                 // 저장된 모드 불러오기

  void setMode(int newMode);  // 수동 설정
  int getMode() const;        // 현재 모드 확인

  void nextMode();            // 다음 모드로 증가
  void prevMode();            // 이전 모드로 감소
};

#endif
