#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <Arduino.h>

// 모드 정의
// 0: 대기  (Standby)
// 1: Armed (baseline 수집 → 자동 mode 2 전환)
// 2: 비행  (발사 감지 + 사출 판단 + 로깅)
// 3: 사출 완료 (로깅 계속)
// 4: 착지  (SD 닫기)

class ModeManager {
public:
  void    begin();          // SD mode.txt에서 모드 로드
  void    save();           // SD mode.txt에 모드 저장
  uint8_t get()  const;
  void    set(uint8_t m);
  void    next();
  void    prev();

private:
  uint8_t _mode = 0;
};

#endif
