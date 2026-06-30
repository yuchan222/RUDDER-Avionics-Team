# CHANGELOG — 1차코드 → 2차코드

> 기준일: 2026-06-30  
> 피드백 출처: `gpt_1차코드_피드백.txt` (25개 항목)  
> 적용: 24개 / 보류: 1개 (#4 물리 ARM 스위치, 하드웨어 결정 필요)

---

## Config.h

| 항목 | 변경 내용 |
|------|-----------|
| #22 | `SERVO_CLOSED_DEG 0`, `SERVO_EJECT_DEG 90` 추가 — 서보 각도 Config로 분리 |
| #9  | `INA219_ADDR 0x40` 추가 |
| #1  | `SENSOR_INTERVAL_MS 20`, `TELEM_INTERVAL_MS 200` 추가 — 루프 주기 상수화 |
| #24 | `LAND_STABLE_MS`, `LAND_ALT_CM_THRESH`, `LAND_ACC_MIN/MAX_MG` 추가 — 착지 감지 임계값 |
| #10/#19 | `STATUS_BMP/IMU/SD/INA` 비트 상수 추가 — 시스템 상태 비트필드 |
| 1단계 | `TIMER_A_SEC` 유지 (10.0f, 시뮬레이션 반영값) |

---

## Packet.h

| 항목 | 변경 내용 |
|------|-----------|
| #17 | `quat[4]` 삭제 — MPU6050에서 미사용, 8바이트 절약 |
| #7  | `altitude_cm`: `int16_t` → `int32_t` — ±327m 상한 오버플로우 방지 |
| #8  | `gyro` 단위 주석: `0.01 dps` → `0.1 dps` (코드 변경과 일치) |
| #9  | `voltage_mv (int16_t)`, `current_ma (int16_t)` 추가 — INA219 전압/전류 |
| #25 | `flight_mode (uint8_t)`, `eject_state (uint8_t)`, `system_status (uint8_t)` 추가 |
| #16 | `ConfirmPacket`, `RxPacket1` 삭제 — 미사용 구조체 제거 |
| #5  | `len` 필드: `finalizePacket()` 호출 시 채워짐 (기본값 0 유지) |

---

## SensorManager.h / SensorManager.cpp

| 항목 | 변경 내용 |
|------|-----------|
| #18 | `getAltitude()` 헤더에서 제거 → `static computeAltitude()` 내부함수로 격리 |
| #9  | `Adafruit_INA219 ina219` 추가, `initSensors()`에서 초기화 |
| #10 | `s_sensorStatus` 비트플래그 추가, `STATUS_BMP/IMU/INA` 초기화 성공 시 세팅 |
| #23 | BMP388 `0x76` 실패 시 `0x77` 자동재시도 |
| #8  | 자이로 변환 계수: `18000/π` → `1800/π` (0.01 dps → 0.1 dps) |
| 2단계 | `s_baselineTempC` 활성화, `collectBaseline()`에서 온도 평균화 |
| 2단계 | `getAltitude()` → 세종대 식(9) 온도보정 공식 적용 (`computeAltitude`) |
| #9  | `readSensors()`에서 `voltage_mv`, `current_ma` 채우기 추가 |

---

## EjectionLogic.h / EjectionLogic.cpp

| 항목 | 변경 내용 |
|------|-----------|
| #22 | `s_servo->write(90)` → `SERVO_EJECT_DEG`, `write(0)` → `SERVO_CLOSED_DEG` |
| #3  | 발사 감지: BMP 고장 시(`altitude_cm == -1`) zacc 단독 경로 추가<br>기존: zacc AND BMP 고도 필수 → 변경: BMP 고장이면 zacc만으로 판단 |
| #2  | 보조 사출 조건 수정: `descending` 조건 제거<br>변경: 발사 후 `TIMER_A_SEC` 경과 시 **무조건** 사출 |
| #24 | `checkLanded()` 추가 — 사출 후 고도 변화 < 2m AND 가속도 800~1200mg 5초 유지 시 `true` |

---

## CommandRx.h

| 항목 | 변경 내용 |
|------|-----------|
| #12 | `CMD_MODE_NEXT (0x0B)` → `CMD_SET_STANDBY (0x0B)` |
| #12 | `CMD_MODE_PREV (0x16)` → `CMD_SET_ARMED (0x16)` |
| #12 | `CMD_EJECT (0x42)` → `CMD_FORCE_EJECT (0x42)` |
| #15 | `CMD_TIMER_A (0x4D)`, `CMD_STOP (0x58)` 삭제 |

> 바이트 값은 유지되므로 지상국 하드웨어 변경 불필요

---

## Logger.h / Logger.cpp

| 항목 | 변경 내용 |
|------|-----------|
| #5/#6 | `finalizePacket(DataPacket &p)` 추가 — `len` + CRC16 계산, SD 저장·텔레메트리 전에 호출 |
| #11 | `isLogReady()` 추가, `initSD()` 결과를 `s_logReady`에 저장 |
| #11 | `writePacket()` 내부에 `if (!s_logReady) return` guard 추가 |
| #6  | `writePacket()`에서 CRC 계산 코드 제거 — `finalizePacket()` 책임으로 분리 |

---

## LEDBuzzer.h / LEDBuzzer.cpp

| 항목 | 변경 내용 |
|------|-----------|
| #19 | `setSystemStatus(uint8_t)` 추가 — Mode 1 LED가 실제 센서 초기화 결과 반영 |
| #20 | `setBeepEnabled(bool)` 추가 — `false` 시 `beep()` 즉시 반환 (blocking 차단) |
| #19 | Mode 1 LED: 기존 전부 ON → `s_sysStatus` 비트에 따라 BMP/IMU/SD LED 선택적 ON |

---

## ModeManager.cpp

| 항목 | 변경 내용 |
|------|-----------|
| #21 | `begin()`에서 복원 모드 제한: `v >= 2` (비행/사출/착지) → 강제 Mode 0 |

---

## Rudder_2026.ino

| 항목 | 변경 내용 |
|------|-----------|
| #1  | 루프 주기 millis 기반 분리: `SENSOR_INTERVAL_MS(20ms)`, `TELEM_INTERVAL_MS(200ms)` |
| #10 | `g_sysStatus` 전역 플래그로 센서 상태 관리, `setup()`에서 합산 |
| #10 | Mode 1에서 `STATUS_BMP` or `STATUS_IMU` 미충족 시 Mode 0 강제 복귀 |
| #20 | Mode 1 → Mode 2 전환 시 `setBeepEnabled(false)` 호출 |
| #12 | `handleCommand()`: `CMD_SET_STANDBY/ARMED` 으로 직접 모드 이동 |
| #13 | `CMD_FORCE_EJECT`: `mode.get() >= 1` 조건 추가 |
| #14 | `CMD_SYSRESET`: `mode.get() == 0` 조건 추가 |
| #25 | Mode 2/3 루프에서 `flight_mode`, `eject_state`, `system_status` 패킷에 기록 |
| #5/#6 | `finalizePacket(pkt)` 호출 후 SD 저장 + 텔레메트리 전송 |
| #24 | Mode 3에서 `checkLanded(pkt)` 호출 → `true` 시 Mode 4 자동 전환 |
| #19 | `setup()`에서 `setSystemStatus(g_sysStatus)` 호출 |

---

## 보류 항목

| # | 내용 | 이유 |
|---|------|------|
| #4 | 통신 두절 시 자동 Mode2 진입 | 물리 ARM 스위치 또는 타임아웃 설계가 필요 — 하드웨어 결정 후 3차코드에 반영 |
