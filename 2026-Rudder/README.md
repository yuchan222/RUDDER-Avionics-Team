# RUDDER 2026 전자부 코드

> Arduino UNO R4 Minima 기반 로켓 비행 컴퓨터 (BMP388 기압 고도 사출 판단)

---

## 파일 구조

```
Rudder_2026/
├── Rudder_2026.ino   — 메인 스케치, FSM 루프
├── Config.h          — 핀 매핑 / 통신 설정 / 사출 임계값
├── Packet.h          — 통신 패킷 구조체 정의
├── SensorManager.h/cpp  — MPU-6050, BMP388 초기화·읽기, 고도 계산
├── EjectionLogic.h/cpp  — 발사 감지, Safe Lock, 정점 판단, 사출 명령
├── ModeManager.h/cpp    — 비행 모드 FSM (0~4), SD 저장·복원
├── CommandRx.h/cpp      — SiK 텔레메트리 수신, 이중 헤더 프로토콜
├── Logger.h/cpp         — SdFat 이진 로그 (LOGxxxxx.RAW)
└── LEDBuzzer.h/cpp      — 74HC595 LED 드라이버, 능동 부저
```

---

## 모드 FSM

| 모드 | 이름 | 동작 |
|:---:|---|---|
| 0 | 대기 | 전원 LED 점멸, 명령 수신 대기 |
| 1 | Armed | Baseline 기압 수집 (~1초) 후 자동으로 Mode 2 전환 |
| 2 | 비행 | 발사 감지 → Safe Lock → 정점 판단 → 사출. 센서 읽기 + 로깅 + 텔레메트리 |
| 3 | 사출 완료 | 로깅 + 텔레메트리 계속 |
| 4 | 착지 | SD 닫기 후 무한 대기 (전원 OFF 또는 리셋) |

---

## 2025 → 2026 주요 변경사항

| 항목 | 2025 | 2026 |
|---|---|---|
| MCU | Arduino Due (Cortex-M3, 84MHz) | Arduino UNO R4 Minima (RA4M1, 48MHz) |
| IMU | MPU-9250 (6축 + 지자기, Mahony AHRS) | MPU-6050 (가속도 + 자이로만, 자세 추정 없음) |
| 사출 주 판단 | 노즈 기울기 ≥90° 또는 Z-가속도 | BMP388 기압 고도 정점 감지 |
| 타이머 구현 | DueTimer 하드웨어 ISR | millis() 기반 소프트웨어 타이머 |
| 발사 감지 | Z-가속도 단일 조건 | Z-가속도 **AND** BMP388 고도 증가 (이중 확인) |
| 추가 하드웨어 | — | INA219 전류·전압 모니터링, I2C/UART 레벨 시프터, UBEC 5A |
| I2C 핀 | SDA=20, SCL=21 | SDA=A4(18), SCL=A5(19) |
| 74HC595 핀 | 22 / 23 / 24 | 4(DS) / 5(SHCP) / 6(STCP) |

---

## 보고서 기반 수정 사항

### Config.h — 사출 임계값 근거

| 상수 | 값 | 근거 |
|---|---|---|
| `APOGEE_DROP_M` | 3.0 m | 2026 보고서: 최고점 대비 3m 이상 하강 시 정점 판단 |
| `APOGEE_DROP_COUNT` | 5회 | 2026 보고서: 5회 연속 확인으로 노이즈 제거 |
| `SAFE_LOCK_MS` | 3000 ms | 2026 보고서: 연소 시간 ~2초 + 여유 1초 |
| `TIMER_A_SEC` | 12.0 s | 2026 보고서: 시뮬레이션 정점 도달 10.2초 + 여유 |
| `LAUNCH_ALT_THRESH_M` | 2.0 m | 발사 감지 이중 조건: BMP 고도 증가 임계 (자체 설정) |
| `MIN_APOGEE_ALT_M` | 30.0 m | 보고서 미명시 — 지면 반사 노이즈 방지용 자체 추가값 |

### Packet.h

- **`seq` 필드 추가**: 시퀀스 번호로 패킷 누락 감지 (세종대 2025 보고서 참고)
- **`mag[3]` 제거**: MPU-6050에 자기 센서 없음 → 항상 0이던 불필요 필드 삭제

### SensorManager.cpp

- **고도 계산**: `h = 44330 × (1 − (P/101325)^(1/5.255)) − baselineAlt`
  - 발사 전 Mode 1에서 50샘플 평균 기압을 `baselineAlt`로 저장 → 상대 고도 사용
  - 온도 보정 수식 (세종대 식 9)은 팀 협의 후 추가 예정 (코드 내 TODO 주석 참고)

### EjectionLogic.cpp

- **발사 감지 이중 조건**: `Z-acc ≥ 2000 mg (300ms 유지)` **AND** `BMP 고도 ≥ 2m` — 둘 다 충족해야 발사 확정
- **Safe Lock**: 발사 감지 후 3초간 사출 명령 차단 (연소 중 오발 방지)
- **보조 사출**: 12초 타이머 단독으로는 사출 안 함 — **하강 경향이 동시에 확인되는 경우에만** 작동

---

## TODO (팀원별 추가 작업)

### 전자 / 소프트웨어

- [ ] **INA219 전류·전압 모니터링** 구현
  - `SensorManager.cpp`에 초기화·읽기 추가
  - `DataPacket`에 `voltage_mv`, `current_ma` 필드 추가
- [ ] **세종대 식(9) 온도 보정 고도 공식** 적용 여부 팀 협의
  - `H = H_ref + (T_ref / L) × ((P/P_ref)^(−RL/g) − 1)`
  - `L=0.0065, R=287.05, g=9.80665`
  - 오차 3.5% → 0.7% 개선 효과
- [ ] **Madgwick / Mahony 자세 추정** 필요 여부 팀 협의 (현재 `quat[4]` = 0 고정)
- [ ] Mode 4 자동 전환 조건 구현 (착지 감지 알고리즘)
- [ ] `DataPacket.len` 필드를 실제 데이터 길이로 채우기
- [ ] 지상국 소프트웨어와 `STX`, `SEQ`, `CRC` 등 패킷 구조 최종 협의

### 기계 / 하드웨어

- [ ] MPU-6050 센서 축 방향 확인 (로켓 추력 방향 = Z+)
- [ ] I2C/UART 레벨 시프터 배선 검증 (UNO R4 5V ↔ 센서 3.3V)
- [ ] UBEC 5A 전원 계통 배선 검증

---

## 라이브러리 설치 목록

Arduino IDE `도구 → 라이브러리 관리`에서 설치:

| 라이브러리 | 버전 | 용도 |
|---|---|---|
| `Adafruit MPU6050` | 2.x | IMU (가속도, 자이로) |
| `Adafruit BMP3XX Library` | 2.x | 기압 고도계 |
| `Adafruit Unified Sensor` | 1.x | Adafruit 센서 공통 의존성 |
| `SdFat` | 2.x | 고속 SD 카드 이진 로깅 |
| `Servo` | 내장 | SG90 서보 모터 |
| `Wire` | 내장 | I2C 통신 |

> **보드 패키지**: `도구 → 보드 → 보드 매니저`에서 **Arduino UNO R4 Boards** 설치 필요
