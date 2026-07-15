#include "ModeManager.h"
#include "Logger.h"

void ModeManager::begin() {
  if (!sd.exists("mode.txt")) return;
  FsFile f = sd.open("mode.txt", O_RDONLY);
  if (!f) return;
  int v = f.parseInt();
  f.close();
  // 재부팅 시 비행/사출/착지 모드(2,3,4) 복원 금지 → Mode 0 강제 (#21)
  if (v < 0 || v > 4 || v >= 2) v = 0;
  _mode = (uint8_t)v;
}

void ModeManager::save() {
  FsFile f = sd.open("mode.txt", O_WRONLY | O_CREAT | O_TRUNC);
  if (!f) return;
  f.print(_mode);
  f.close();
}

uint8_t ModeManager::get() const { return _mode; }

void ModeManager::set(uint8_t m) {
  if (m > 4) m = 4;
  _mode = m;
  save();
}

void ModeManager::next() { set(_mode + 1); }
void ModeManager::prev() { if (_mode > 0) set(_mode - 1); }
