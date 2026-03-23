# 프로젝트 설계 문서 v1.2

**작성일: 2026년 3월 22일 | 업데이트: 2026년 3월 23일기반: CATNIP v9.5 → 초음파 자율주행 버전으로 재설계**

> **v1.1 → v1.2 변경 사항**
>
> - `CAN_CMD_TIMEOUT_MS` → `SENSOR_MSG_TIMEOUT_MS` 이름 변경 및 주석 정리
> - `front_min` 정의 명시 (6-3절)
> - `TRIGGER_INTERVAL_MS` 초기값 및 타이밍 계산 근거 추가 (5-4절)
> - watchdog reset bit2 세트 근거 명시 (`RCC->CSR` IWDGRSTF 비트 확인)
>
> **v1.0 → v1.1 변경 사항**
>
> - `0x101 VEHICLE_STATE` 메시지 추가 (주행 ECU → RPi5)
> - CAN ID 느슨한 네임스페이스 규칙 추가
> - `VEHICLE_STATE.fault_flags` 비트 정의 (v1.0 Fail-safe 기준 확정)
> - `0x021 SENSOR_STATUS` 선택 항목으로 추가
> - `SENSOR_STATUS` 없이 진행 시 fault 정보는 `VEHICLE_STATE.fault_flags`로 통합

---

## 1. 프로젝트 개요

### 한 줄 요약

> **[ STM32F103 (센서 ECU) + STM32L4 (주행 ECU) + Raspberry Pi 5 (미션 ECU) ]**
FreeRTOS 기반 분산 ECU 구조에서 초음파 센서 4개로 장애물을 감지하고,
CAN 통신으로 실시간 제어하는 자율주행 RC카 시스템
>

### CATNIP v9.5 대비 변경 사항

| 구분 | CATNIP v9.5 | CATNIP v2 (이번 프로젝트) |
| --- | --- | --- |
| 자율주행 방식 | 카메라 + 라인트레이싱 + ArUco | 초음파 4개 장애물 회피 |
| 화물함 ECU | STM32F103 (PIN/서보/LCD) | **제거** |
| 서버/클라이언트 | RPi3 (MQTT 브로커) + RPi4 (Qt UI) | **제거** |
| MQTT | 사용 | **제거** |
| 센서 ECU | 없음 | **STM32F103 신규 투입** |
| RTOS | 슈퍼루프 | **FreeRTOS 도입** |
| 미션 ECU | RPi1 (카메라/라인트레이싱) | RPi5 (모니터링/상태 관리) |
| 통신 명세 | CAN ID 테이블 문서 | **DBC 파일 (CANdb++ Editor)** |

### 핵심 기술 선택 근거

| 기술 | 선택 이유 |
| --- | --- |
| FreeRTOS | 센서 취득 / 제어 / 통신을 태스크로 분리 → 블로킹 없는 실시간 처리 |
| CAN 통신 (250 kbps) | 분산 ECU 간 단일 버스 연결, 기존 CATNIP 인프라 재활용 |
| HC-SR04 × 4 | 앞쪽 시야각 전체 커버, 앞/대각선 방향 장애물 동시 감지 |
| 초음파 순차 트리거 | 크로스토크(센서 간 간섭) 방지 |
| 타이머 입력 캡처 | 블로킹 _delay_us 제거 → 다른 태스크와 병행 처리 가능 |
| STM32F103 센서 전담 | 센서 취득 부하를 주행 ECU에서 분리 |
| SocketCAN + C++ | RPi5에서 Linux 사용자 공간으로 CAN 버스 모니터링 |
| DBC 파일 | 통신 명세를 코드와 분리하여 문서화, Vector 툴체인 경험 |

---

## 2. 전체 시스템 구조

### 2-1. 노드 구성

| 구분 | 보드 | 역할 |
| --- | --- | --- |
| **센서 ECU** | STM32F103 (MangoM32) | HC-SR04 × 4 측정 → CAN 송신 |
| **주행 ECU** | STM32L4 (B-L475E-IOT01A) | CAN 수신 → 장애물 판단 → PID + 모터 제어 |
| **미션 ECU** | Raspberry Pi 5 | SocketCAN 모니터링, 모드 전환, 로그 |

### 2-2. 통신 구조

```
┌──────────────────────────────────────────────────────┐
│                    RC카 (온보드)                       │
│                                                      │
│  STM32F103 (센서 ECU)                                │
│  HC-SR04 × 4 ──→ 타이머 캡처 ──→ CAN 0x020 TX       │
│                         │                           │
│                    CAN 버스 (250 kbps)               │
│                         │                           │
│  STM32L4 (주행 ECU)                                  │
│  CAN 0x020 RX ──→ 장애물 판단 ──→ PID ──→ 모터       │
│  CAN 0x100 TX (속도 피드백)                           │
│  CAN 0x101 TX (차량 상태)                            │
│  CAN 0x200 TX (Heartbeat)                            │
│                         │                           │
│  Raspberry Pi 5 (미션 ECU)                           │
│  MCP2515 (SPI) ──→ SocketCAN (can0) ──→ 모니터링     │
└──────────────────────────────────────────────────────┘
```

### 2-3. 각 노드 역할 분담 원칙

- **센서 ECU (STM32F103)**: "측정하고 보낸다." 거리값 취득과 CAN 송신만 담당
- **주행 ECU (STM32L4)**: "판단하고 제어한다." 장애물 회피 결정과 모터 제어 담당
- **미션 ECU (RPi5)**: "모니터링하고 개입한다." 실시간 제어에 관여하지 않고 상태 관찰

> **정책 A (확정):** RPi5는 선택적 상위 노드. 자율주행 판단의 핵심은 주행 ECU에 있으며, RPi5 미수신이 fault 조건이 아님.
>

---

## 3. 초음파 센서 배치

### 3-1. 물리적 배치

앞쪽 4개 배치. 정면 시야각을 넓게 커버하는 구조.

```
        [차량 전방]

   [대각 좌]  [정면 L]  [정면 R]  [대각 우]
      ↙          ↓         ↓          ↘
```

| 센서 번호 | 방향 | 역할 |
| --- | --- | --- |
| Sensor 0 | 대각 좌 (약 45°) | 좌측 전방 장애물 조기 감지 |
| Sensor 1 | 정면 좌 | 직진 경로 장애물 |
| Sensor 2 | 정면 우 | 직진 경로 장애물 |
| Sensor 3 | 대각 우 (약 45°) | 우측 전방 장애물 조기 감지 |

### 3-2. 크로스토크 방지 전략

- 4개 센서를 **절대 동시에 트리거하지 않는다**
- 순차 트리거: Sensor 0 → 1 → 2 → 3 → 0 반복
- 센서 간 최소 간격: 측정 완료 후 다음 트리거 (타임아웃 포함 최대 30ms/센서)
- 이상치 제거: 측정값 -1 (타임아웃) 또는 2cm 미만 → 직전 유효값 유지

---

## 4. CAN 프로토콜

### 4-1. 통신 속도

| 항목 | 값 |
| --- | --- |
| 통신 속도 | 250 kbps |
| 센서 데이터 주기 | 50ms (4개 순차 측정 완료 후 송신) |

### 4-2. CAN ID 네임스페이스 규칙

현재 ID 체계를 기준으로 느슨한 구역을 부여한다. 재배치 없이 규칙만 정의.

| 범위 | 용도 |
| --- | --- |
| `0x010 ~ 0x01F` | Mission ECU → Drive ECU 명령 |
| `0x020 ~ 0x02F` | Sensor ECU 측정 / 상태 |
| `0x100 ~ 0x10F` | Drive ECU 주행 상태 / 피드백 |
| `0x200 ~ 0x20F` | Heartbeat / Diagnostic |

### 4-3. CAN ID 테이블

| CAN ID | 이름 | 송신 | 수신 | 주기 | 비고 |
| --- | --- | --- | --- | --- | --- |
| **0x010** | DRIVE_CMD | RPi5 | 주행 ECU | 이벤트 | 주행 명령 (모드 전환용) |
| **0x011** | E_STOP | RPi5 | 주행 ECU | 즉시 | 비상 정지 |
| **0x020** | SENSOR_RAW | 센서 ECU | 주행 ECU | 50ms | HC-SR04 거리값 |
| **0x021** | SENSOR_STATUS | 센서 ECU | 주행 ECU, RPi5 | 100ms | 센서 fault/alive 정보 **(선택)** |
| **0x100** | SPEED_FB | 주행 ECU | RPi5 | 50ms | 엔코더 RPM 피드백 |
| **0x101** | VEHICLE_STATE | 주행 ECU | RPi5 | 50ms | 상태머신/모드/fault 상태 |
| **0x200** | HEARTBEAT | 주행 ECU | RPi5 | 100ms | 생존 확인 0xAA |

### 4-4. CAN 페이로드 상세

**0x020 — SENSOR_RAW (센서 ECU → 주행 ECU, DLC=4)**

| Byte | 신호명 | 단위 | 범위 | 비고 |
| --- | --- | --- | --- | --- |
| [0] | diag_left_cm | cm | 0~255 | 대각 좌, 0xFF = 타임아웃 |
| [1] | front_left_cm | cm | 0~255 | 정면 좌, 0xFF = 타임아웃 |
| [2] | front_right_cm | cm | 0~255 | 정면 우, 0xFF = 타임아웃 |
| [3] | diag_right_cm | cm | 0~255 | 대각 우, 0xFF = 타임아웃 |

**0x021 — SENSOR_STATUS (센서 ECU → 주행 ECU, RPi5, DLC=4) ※ 선택 구현**

| Byte | 내용 | 비고 |
| --- | --- | --- |
| [0] | sensor fault bitmask | bit0=diag_left, bit1=front_left, bit2=front_right, bit3=diag_right timeout |
| [1] | alive counter | 송신마다 +1 (wrap around) |
| [2~3] | reserved | — |

**0x010 — DRIVE_CMD (RPi5 → 주행 ECU, DLC=1)**

| Byte | 내용 | 값 |
| --- | --- | --- |
| [0] | 주행 모드 | 0x00=정지, 0x01=자율주행 시작, 0x02=수동 모드 |

**0x100 — SPEED_FB (주행 ECU → RPi5, DLC=4)**

| Byte | 내용 | 타입 |
| --- | --- | --- |
| [0~1] | 좌 RPM × 10 | int16 Big-Endian |
| [2~3] | 우 RPM × 10 | int16 Big-Endian |

**0x101 — VEHICLE_STATE (주행 ECU → RPi5, DLC=3)**

| Byte | 내용 | 값 |
| --- | --- | --- |
| [0] | state | 0x00=IDLE, 0x01=DRIVE, 0x02=STOP, 0x03=TURN_LEFT, 0x04=TURN_RIGHT, 0x05=AVOID_LEFT, 0x06=AVOID_RIGHT, 0x07=ESTOP |
| [1] | mode | 0x00=STOP, 0x01=AUTO, 0x02=MANUAL |
| [2] | fault_flags | bit0=sensor timeout, bit1=reserved, bit2=watchdog reset, bit3~7=reserved |

> **fault_flags 정의 근거**
>
> - `bit0`: SENSOR_RAW (0x020) 3초 미수신 → v1.0 Fail-safe 근거
> - `bit1`: reserved (cmd timeout 정책 미확정 — 정책 A 유지 시 미사용, 추후 결정)
> - `bit2`: IWDG 300ms 만료 후 MCU 리셋 감지 → v1.0 Fail-safe 근거

**0x200 — HEARTBEAT (주행 ECU → RPi5, DLC=1)**

| Byte | 내용 |
| --- | --- |
| [0] | 0xAA 고정 |

### 4-5. DBC 파일 (catnip_v2.dbc)

CANdb++ Editor로 작성. 런타임 파싱에는 사용하지 않고 **통신 명세 문서 + 포트폴리오 용도**로 활용.

```
VERSION ""

NS_ :

BS_:

BU_: SENSOR_ECU DRIVE_ECU MISSION_ECU

BO_ 32 SENSOR_RAW: 4 SENSOR_ECU
 SG_ diag_left_cm   : 0|8@1+  (1,0) [0|255] "cm" DRIVE_ECU
 SG_ front_left_cm  : 8|8@1+  (1,0) [0|255] "cm" DRIVE_ECU
 SG_ front_right_cm : 16|8@1+ (1,0) [0|255] "cm" DRIVE_ECU
 SG_ diag_right_cm  : 24|8@1+ (1,0) [0|255] "cm" DRIVE_ECU

BO_ 16 DRIVE_CMD: 1 MISSION_ECU
 SG_ drive_mode : 0|8@1+ (1,0) [0|2] "" DRIVE_ECU

BO_ 256 SPEED_FB: 4 DRIVE_ECU
 SG_ rpm_left  : 0|16@1+ (0.1,0) [-3276.8|3276.7] "rpm" MISSION_ECU
 SG_ rpm_right : 16|16@1+ (0.1,0) [-3276.8|3276.7] "rpm" MISSION_ECU

BO_ 257 VEHICLE_STATE: 3 DRIVE_ECU
 SG_ state      : 0|8@1+  (1,0) [0|7]   "" MISSION_ECU
 SG_ mode       : 8|8@1+  (1,0) [0|2]   "" MISSION_ECU
 SG_ fault_flags: 16|8@1+ (1,0) [0|255] "" MISSION_ECU

BO_ 512 HEARTBEAT: 1 DRIVE_ECU
 SG_ alive : 0|8@1+ (1,0) [0|255] "" MISSION_ECU
```

---

## 5. 센서 ECU 상세 설계 (STM32F103)

### 5-1. 핀맵

| 기능 | 핀 | 설정 | 연결 |
| --- | --- | --- | --- |
| HC-SR04 #0 TRIG | PA0 | GPIO_Output | 대각 좌 Trig |
| HC-SR04 #0 ECHO | PA1 | TIM2_CH2 Input Capture | 대각 좌 Echo |
| HC-SR04 #1 TRIG | PA2 | GPIO_Output | 정면 좌 Trig |
| HC-SR04 #1 ECHO | PA3 | TIM2_CH4 Input Capture | 정면 좌 Echo |
| HC-SR04 #2 TRIG | PB0 | GPIO_Output | 정면 우 Trig |
| HC-SR04 #2 ECHO | PB1 | TIM3_CH4 Input Capture | 정면 우 Echo |
| HC-SR04 #3 TRIG | PB6 | GPIO_Output | 대각 우 Trig |
| HC-SR04 #3 ECHO | PB7 | TIM4_CH2 Input Capture | 대각 우 Echo |
| CAN1_RX | PB8 | CAN1_RX | SN65HVD230 RXD (AFIO Remap2) |
| CAN1_TX | PB9 | CAN1_TX | SN65HVD230 TXD (AFIO Remap2) |
| USART1_TX | PA9 | USART1_TX | 디버그 115200 |

### 5-2. CubeMX 설정

```
CAN1:   PB8(RX)/PB9(TX), Prescaler=6, BS1=13TQ, BS2=2TQ → 250kbps
        AFIO Remap2: __HAL_AFIO_REMAP_CAN1_2()
TIM2:   CH2/CH4 Input Capture, Prescaler=71 → 1MHz (1μs 해상도)
TIM3:   CH4 Input Capture, 동일
TIM4:   CH2 Input Capture, 동일
TIM5:   Prescaler=71, Period=9999 → 10ms 순차 트리거 인터럽트
USART1: PA9(TX)/PA10(RX), 115200/8N1
Clock:  HSI 8MHz → PLL×9 → SYSCLK 72MHz
```

### 5-3. FreeRTOS 태스크 구성

```
┌─────────────────────────────────────────┐
│           센서 ECU 태스크 구조           │
│                                         │
│  TIM5 인터럽트 (10ms)                   │
│    └→ trigger_flag 세트                 │
│                                         │
│  Task: ultrasonic_trigger (우선순위 3)  │
│    trigger_flag 감지                    │
│    → 현재 센서 TRIG 10μs 펄스           │
│    → 다음 센서로 인덱스 전진             │
│                                         │
│  ISR: TIMx Input Capture                │
│    Rising edge  → capture_start 저장    │
│    Falling edge → 펄스폭 계산           │
│    → distance_queue에 enqueue           │
│                                         │
│  Task: distance_process (우선순위 2)    │
│    distance_queue 수신                  │
│    → 타임아웃/이상치 필터링              │
│    → g_distance[4] 업데이트             │
│                                         │
│  Task: can_tx (우선순위 1)              │
│    50ms 주기                            │
│    → g_distance[4] 읽기                 │
│    → 0x020 CAN 프레임 송신              │
└─────────────────────────────────────────┘
```

### 5-4. 핵심 파라미터

```c
#define SENSOR_COUNT          4
#define TRIGGER_INTERVAL_MS   10     // 초기값. 센서 간 간섭 및 실제 응답시간에 따라 15~20ms까지 조정 가능
#define MEASURE_TIMEOUT_US    30000  // 측정 타임아웃 (약 5m 해당)
#define DISTANCE_MAX_CM       200    // 유효 거리 상한
#define DISTANCE_MIN_CM       2      // 유효 거리 하한 (이하 이상치)
#define CAN_TX_PERIOD_MS      50     // CAN 송신 주기
```

> **타이밍 계산 근거:** 정상 측정 시 4개 × 10ms = 40ms로 50ms CAN TX 주기 내 완료 가능.
단, MEASURE_TIMEOUT_US = 30ms짜리 타임아웃이 발생하면 해당 센서 사이클이 30ms를 점유하여
한 라운드가 50ms를 초과할 수 있음. 실 운용 환경에서 TRIGGER_INTERVAL_MS 튜닝 필요.
>

### 5-5. 이상치 처리 규칙

```c
// 타임아웃 → 0xFF (255) 송신 (주행 ECU가 "감지 없음"으로 해석)
// 2cm 미만  → 직전 유효값 유지
// 200cm 초과 → 200cm로 클리핑
```

---

## 6. 주행 ECU 상세 설계 (STM32L4)

### 6-1. 핀맵 (CATNIP v9.5에서 유지)

| 기능 | 핀 | 설정 | 연결 |
| --- | --- | --- | --- |
| CAN1_RX | PB8 | CAN1_RX | SN65HVD230 RXD |
| CAN1_TX | PB9 | CAN1_TX | SN65HVD230 TXD |
| TIM5_CH1 | PA0 | Encoder | 왼쪽 엔코더 A |
| TIM5_CH2 | PA1 | Encoder | 왼쪽 엔코더 B |
| TIM3_CH1 | PA6 | Encoder | 오른쪽 엔코더 A |
| TIM3_CH2 | PA7 | Encoder | 오른쪽 엔코더 B |
| TIM2_CH1 | PA15 | PWM | L298N ENA (왼쪽) |
| TIM2_CH3 | PA2 | PWM | L298N ENB (오른쪽) |
| PC2~PC5 | GPIO_Output | L298N IN1~IN4 |  |

### 6-2. FreeRTOS 태스크 구성

```
┌─────────────────────────────────────────┐
│          주행 ECU 태스크 구조            │
│                                         │
│  ISR: CAN RX (0x020 수신)               │
│    → sensor_queue에 enqueue             │
│                                         │
│  Task: obstacle_decision (우선순위 3)   │
│    sensor_queue 수신 (50ms)             │
│    → 장애물 판단                        │
│    → 회피 방향 결정                     │
│    → drive_state 업데이트               │
│                                         │
│  Task: pid_control (우선순위 2)         │
│    TIM6 인터럽트 동기화 (10ms)          │
│    → Encoder_Update() → RPM 계산       │
│    → PID_Compute() → PWM 출력          │
│    → drive_state에 따라 모터 방향 설정  │
│                                         │
│  Task: can_tx (우선순위 1)              │
│    50ms: 0x100 속도 피드백 송신         │
│    50ms: 0x101 차량 상태 송신           │
│    100ms: 0x200 Heartbeat 송신          │
│                                         │
│  Task: watchdog_feed (우선순위 0)       │
│    IWDG 주기적 리셋                     │
└─────────────────────────────────────────┘
```

### 6-3. 장애물 회피 상태머신

```
STATE_IDLE
└→ 0x010 수신 (drive_mode=0x01) → STATE_DRIVE

STATE_DRIVE (정상 주행)
├→ 전방 장애물 감지 (front_min < STOP_DIST)
│   → STATE_STOP
├→ 대각 좌 장애물 (diag_left < WARN_DIST)
│   → STATE_AVOID_RIGHT
└→ 대각 우 장애물 (diag_right < WARN_DIST)
    → STATE_AVOID_LEFT

STATE_STOP (정지)
└→ 전방 장애물 해제 → 좌우 거리 비교
    → 여유 있는 쪽으로 STATE_TURN_LEFT or STATE_TURN_RIGHT

STATE_TURN_LEFT / STATE_TURN_RIGHT
└→ 회전 완료 (시간 기반 or 전방 거리 확보) → STATE_DRIVE

STATE_AVOID_LEFT / STATE_AVOID_RIGHT (조향 보정)
└→ 장애물 해제 → STATE_DRIVE

STATE_ESTOP (비상 정지)
└→ 0x010 (drive_mode=0x01) 재수신 → STATE_DRIVE
```

> **VEHICLE_STATE.state ↔ 상태머신 1:1 매핑**
>
>
> ```
> STATE_IDLE       → 0x00
> STATE_DRIVE      → 0x01
> STATE_STOP       → 0x02
> STATE_TURN_LEFT  → 0x03
> STATE_TURN_RIGHT → 0x04
> STATE_AVOID_LEFT → 0x05
> STATE_AVOID_RIGHT→ 0x06
> STATE_ESTOP      → 0x07
> ```
>

### 6-4. 장애물 거리 임계값

전방 장애물 판단은 `front_left_cm`, `front_right_cm` 중 최솟값 기준으로 수행한다.

```c
// front_min = min(front_left_cm, front_right_cm)
```

```c
#define STOP_DIST_CM    30    // 전방 정지 거리
#define WARN_DIST_CM    50    // 대각 경고 거리 (조향 보정 시작)
#define CLEAR_DIST_CM   60    // 장애물 해제 판단 거리
```

### 6-5. PID 파라미터 (CATNIP v9.5에서 유지)

```c
#define PWM_MAX             999
#define PWM_OFFSET          300.0f
#define PULSE_PER_REV       5280
#define PID_INTERVAL_MS     10
#define KP                  0.5f
#define KI                  0.05f
#define KD                  0.0f
#define INTEGRAL_LIMIT      150.0f
#define TARGET_RPM_DEFAULT      100.0f
#define SENSOR_MSG_TIMEOUT_MS   3000    // SENSOR_RAW (0x020) 3초 미수신 시 자동 정지 → STATE_ESTOP, fault_flags bit0 세트
```

### 6-6. Fail-safe

| 상황 | 감지 | 대응 |
| --- | --- | --- |
| 센서 ECU 응답 없음 | 0x020 3초 미수신 | 안전 정지, STATE_ESTOP, fault_flags bit0 세트 |
| RPi5 다운 | 0x010 미수신 | 자율주행 유지 (정책 A — 선택적 상위 노드) |
| MCU 무한루프 | IWDG 300ms 만료 | MCU 리셋 → 모터 정지, 재기동 후 `RCC->CSR`의 `IWDGRSTF` 비트 확인하여 fault_flags bit2 세트 (부팅 직후 1회 읽고 클리어) |
| 모든 센서 타임아웃 | front = 0xFF × 2 | 안전 정지, STATE_ESTOP |

---

## 7. 미션 ECU 상세 설계 (Raspberry Pi 5)

### 7-1. 하드웨어 연결 (CATNIP v9.5에서 유지)

```
RPi5
└── MCP2515 (SPI) → CAN 버스
     ├── 오실레이터: 8MHz
     ├── 전원: 5V
     ├── 트랜시버: TJA1050 내장
     └── J1 점퍼 제거 (중간 노드)
```

### 7-2. SocketCAN 설정

```bash
# /boot/firmware/config.txt
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25

# 인터페이스 활성화
sudo ip link set can0 up type can bitrate 250000
sudo ip link set can0 txqueuelen 1000

# 모니터링
candump can0
```

### 7-3. C++ 모니터링 툴 구조

```cpp
// can_monitor.cpp
// SocketCAN 기반 CAN 프레임 직접 디코딩

struct SensorData {
    uint8_t diag_left_cm;
    uint8_t front_left_cm;
    uint8_t front_right_cm;
    uint8_t diag_right_cm;
};

struct VehicleState {
    uint8_t state;       // 0x00~0x07, 상태머신과 1:1 대응
    uint8_t mode;        // 0x00=STOP, 0x01=AUTO, 0x02=MANUAL
    uint8_t fault_flags; // bit0=sensor timeout, bit2=watchdog reset
};

// CAN ID별 직접 파싱 (DBC 파서 미사용)
if (frame.can_id == 0x020) {
    SensorData s;
    s.diag_left_cm   = frame.data[0];
    s.front_left_cm  = frame.data[1];
    s.front_right_cm = frame.data[2];
    s.diag_right_cm  = frame.data[3];
    log_sensor(s);
}

if (frame.can_id == 0x101) {
    VehicleState vs;
    vs.state       = frame.data[0];
    vs.mode        = frame.data[1];
    vs.fault_flags = frame.data[2];
    log_vehicle_state(vs);
}
```

### 7-4. RPi5 역할

```
핵심
- CAN 버스 실시간 모니터링
- VEHICLE_STATE 기반 차량 상태 해석 (0x101)
- 주행 모드 전환 명령 (0x010)
- 비상 정지 명령 (0x011)

선택 (시간 되면)
- 센서/속도/상태 데이터 로그 파일 저장
- 파라미터 튜닝 인터페이스
- 터미널 기반 상태 시각화
```

---

## 8. CAN 버스 배선

CATNIP v9.5 배선 구조 유지.

```
[STM32F103 센서 ECU]      [RPi5 + MCP2515]      [STM32L4 주행 ECU]
    (버스 끝단)              (중간 노드)              (버스 끝단)
    SN65HVD230               TJA1050 내장             SN65HVD230
    종단저항 있음             J1 점퍼 제거              종단저항 있음
  220Ω||220Ω=110Ω           내장 120Ω 비활성화       220Ω||220Ω=110Ω
        │                        │                        │
        └──────── CANH ──────────┴──────── CANH ──────────┘
        └──────── CANL ──────────┴──────── CANL ──────────┘
        └──────── GND  ──────────┴──────── GND  ──────────┘
```

**SN65HVD230 연결 공통:**

```
STM32 CAN_TX → SN65HVD230 TXD
SN65HVD230 RXD → STM32 CAN_RX
SN65HVD230 VCC = 3.3V  ← GND 혼동 주의
SN65HVD230 GND = GND
```

---

## 9. 구현 우선순위 및 일정

### 9-1. 핵심 (반드시 구현)

- [ ]  FreeRTOS 태스크 분리 (센서 ECU / 주행 ECU)
- [ ]  초음파 순차 트리거 + 타이머 입력 캡처
- [ ]  타임아웃 + 기본 이상치 제거 (0xFF, 2cm 미만)
- [ ]  CAN 원시값 전송 (0x020)
- [ ]  주행 ECU 장애물 회피 상태머신
- [ ]  PID 속도 제어 (CATNIP 코드 재활용)
- [ ]  **VEHICLE_STATE 송신 (0x101)**

### 9-2. 선택 (시간 되면 구현)

- [ ]  Median / Moving average 필터링
- [ ]  드라이버 레이어 분리 (bsp_can.c, bsp_ultrasonic.c)
- [ ]  RPi5 SocketCAN 모니터링 C++ 툴
- [ ]  로그 저장 및 파라미터 튜닝 인터페이스
- [ ]  DBC 파일 완성 (catnip_v2.dbc)
- [ ]  **SENSOR_STATUS 송신 (0x021)**

### 9-3. 검증 순서

```
1단계: 센서 ECU 단독 검증
  - 초음파 1개 정상 측정
  - 4개 순차 측정
  - 타임아웃 처리
  - CAN 0x020 송신 확인 (candump)

2단계: 주행 ECU 단독 검증
  - CAN 수신 파싱
  - 상태머신 전이
  - PID 속도 유지
  - 정지/회피 동작
  - CAN 0x101 VEHICLE_STATE 송신 확인

3단계: 2노드 통합
  - 센서 ECU → 주행 ECU 실제 장애물 반응

4단계: 3노드 통합
  - RPi5 모니터링 연결
  - VEHICLE_STATE 수신 및 상태 해석
  - 수동 모드 전환 명령
```

---

## 10. 핵심 키워드

```
- 분산 ECU 아키텍처 (3노드 CAN 버스)
- FreeRTOS 기반 태스크 분리
- 타이머 입력 캡처 기반 비블로킹 센서 처리
- 다중 초음파 센서 크로스토크 방지 (순차 트리거)
- PID 기반 속도 제어
- Linux 상위 노드 (RPi5) + MCU 하위 제어 노드 역할 분리
- SocketCAN 기반 CAN 모니터링
- DBC 파일 기반 통신 명세 관리 (Vector CANdb++ Editor)
- 상태머신 외부 노출 (VEHICLE_STATE CAN 메시지)
- Fail-safe 설계 (sensor timeout, watchdog, fault_flags)
```
---

## 11. 기술 스택

| 영역 | 기술 |
| --- | --- |
| 센서 ECU 펌웨어 | C + STM32 HAL + FreeRTOS |
| 주행 ECU 펌웨어 | C + STM32 HAL + FreeRTOS |
| 미션 ECU | C++ + SocketCAN (Linux) |
| CAN 컨트롤러 | bxCAN 내장 (STM32F103, STM32L4) / MCP2515 (RPi5) |
| CAN 트랜시버 | SN65HVD230 (STM32) / TJA1050 내장 (MCP2515) |
| 통신 명세 | DBC 파일 (Vector CANdb++ Editor) |
| 빌드 | STM32CubeIDE / g++ |

---

*문서 버전: v1.2 | 최종 업데이트: 2026-03-23 | 기반 프로젝트: CATNIP v9.5*
