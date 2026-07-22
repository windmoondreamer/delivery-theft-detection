# 택배 도난 감지 시스템 (Delivery Theft Detection)

문 앞에 방치된 택배의 **도난을 실시간으로 감지**하고, 도난 순간을 **자동 촬영**해 **카카오톡으로 즉시 알리는** 임베디드 시스템입니다.
마이크로프로세서 응용 및 실험 Term Project (12조 · 김민섭 · 장재원 · 정광현).

---

## 1. 문제 정의

비대면 수령이 일상화되면서 택배는 수취인이 없는 현관 앞에 장시간 방치됩니다. 누구나 손쉽게 가져갈 수 있어 문 앞 택배 도난이 늘고 있지만, **피해는 뒤늦게 발견되고 범인 특정도 어렵습니다.**

이 시스템은 도난을 **"뒤늦게 아는 일"에서 "즉시 대응 + 증거 확보"로** 바꾸는 것을 목표로 합니다.

- 무설치: 고가의 무인 택배함 없이 **문 앞에 두기만** 하면 됨
- 즉시성: 도난 발생 즉시 스마트폰 알림
- 증거: 도난 순간 사진을 자동 저장 → 분실/보상 신고 시 객관적 증거

---

## 2. 시스템 아키텍처

역할을 두 MCU로 분리했습니다. **Mega는 센서 판독과 판단(상태머신), ESP32는 촬영과 통신**을 전담합니다.

```
 [로드셀+HX711]  [VL53L0X ToF]
        │  무게        │  거리
        └──────┬───────┘
               ▼
     ┌───────────────────────┐        UART(115200)         ┌──────────────────────┐
     │   Arduino Mega 2560    │  ── "ALERT_ARRIVE" ────────▶│   ESP32 DevKitC V4    │
     │   (순수 AVR C)         │  ── "ALERT_THEFT"  ────────▶│  (Arduino framework)  │
     │   센서 판독·상태머신    │                             │  카메라·WiFi·통신      │
     └───────────────────────┘                             └──────────┬───────────┘
                                                                       │ 촬영(SPI)
                                                             [Arducam Mega 3MP]
                                                                       │ JPEG
                                                                       ▼ HTTPS POST
                                                          [Cloudflare Worker] → [R2 저장]
                                                                       │ 공개 URL
                                                                       ▼
                                                             [카카오톡 알림(사진·텍스트)]
```

**핵심 설계 원칙:** 큰 JPEG를 UART로 넘기지 않습니다. Mega는 **짧은 명령 한 줄만** 보내고, 사진 촬영·전송은 ESP32가 내부에서 독립적으로 처리합니다. 덕분에 UART 버퍼·타이밍이 안정적입니다.

---

## 3. 하드웨어

| 부품 | 역할 | 인터페이스 |
|---|---|---|
| Arduino Mega 2560 | 센서 판독 · 상태머신 판단 | — |
| ESP32 DevKitC V4 | 카메라 제어 · WiFi / HTTPS 통신 | — |
| HX711 + 로드셀 | 무게 측정 (도착/도난 판단) | 비트뱅잉 (DT/SCK) |
| VL53L0X | ToF 거리 측정 | I²C (TWI) |
| Arducam Mega 3MP | 도난 순간 촬영 | SPI (ESP32) |

### 배선 (핀맵)

**Arduino Mega 2560**
- VL53L0X : `SDA=D20 · SCL=D21` (I²C)
- HX711 : `DT=D2 · SCK=D3`
- → ESP32 : `TX1(D18) → GPIO16`, `RX1(D19) ← GPIO17`, GND 공통

**ESP32 DevKitC V4**
- Arducam Mega (SPI 3.3V) : `SCK=18 · MISO=19 · MOSI=23 · CS=5`
- 전원 : USB-C

---

## 4. 소프트웨어 구조

### 4-1. Arduino Mega — 순수 AVR C (라이브러리 미사용)

Arduino 라이브러리에 의존하지 않고 **ATmega2560 레지스터를 직접 제어**해 드라이버를 구현했습니다. 동작 원리를 정확히 이해하고 타이밍을 직접 통제하기 위함입니다.

| 파일 | 내용 |
|---|---|
| `main.c` | 5단계 상태머신 · 센서 판단 로직 · UART 명령 송신 |
| `hx711.c/.h` | HX711 24bit 비트뱅잉 (gain 128, 채널 A). 클럭 반주기 지연으로 상위 비트 오독 방지, **중앙값 필터(read_median)로 글리치 제거**, tare |
| `i2c.c/.h` | 폴링 방식 TWI 마스터 (100kHz, `TWBR=72`) — VL53L0X 통신용 |
| `vl53l0x.c/.h` | ToF 거리 센서 드라이버 (single-shot range, 타임아웃 처리) |
| `uart.c/.h` | UART0 = 디버그 로그, UART1 = ESP32 명령 채널 |
| `millis.c/.h` | 타이머 기반 밀리초 카운터 (상태 전이 타이밍용) |
| `config.h` | 임계값·핀 정의 (`WEIGHT_PRESENT_RAW`, `WEIGHT_THEFT_DELTA`, `DISTANCE_PRESENT_MAX`, `DISTANCE_THEFT_MIN`, `BASELINE_MS`, `COOLDOWN_MS` 등) |

**HX711 안정화 예시** — 너무 빠른 클럭은 상위 비트 오독(값이 수만으로 튐)을 유발하므로 반주기 지연(`HX711_CLK_US`)을 두고, 여러 샘플 중 중앙값을 취해 튀는 값 1개를 버립니다.

### 4-2. ESP32 — 카메라 · 통신

- Mega로부터 UART 명령 수신
  - `ALERT_ARRIVE` → 텍스트 카카오톡 (도착 알림)
  - `ALERT_THEFT` → Arducam으로 **직접 촬영** → JPEG 전송
- 촬영 JPEG를 `WiFiClientSecure` + `HTTPClient`로 **Cloudflare Worker에 HTTPS POST**
- Worker가 **R2에 사진 저장 → 공개 URL 발급 → 카카오 메시지에 첨부**
- 카카오 `refresh_token`으로 `access_token` 자동 갱신

---

## 5. 상태머신 (State Machine)

5개 상태를 순환하며 도착과 도난을 구분합니다.

```
  IDLE ──(무게↑ + 거리↓, 3회 확인)──▶ ARRIVED ──(5초 안정화 후 baseline 기록)──▶ MONITORING
   ▲                                    │                                          │
   │                              (사라지면 false trigger)                   (무게↓ + 거리↑, 3회 확인)
   │                                    ▼                                          ▼
  IDLE ◀──(쿨다운 30초)── COOLDOWN ◀──────────────── ALERT (ESP32 촬영·도난 알림)
```

| 상태 | 역할 | 전이 조건 |
|---|---|---|
| `ST_IDLE` | 대기 | 무게 > 임계값 AND 거리 < 임계값 → **3회 연속 확인 시** ARRIVED |
| `ST_ARRIVED` | 도착 감지·안정화 | `BASELINE_MS` 경과 → baseline 기록 + `ALERT_ARRIVE` 송신 → MONITORING (중간에 사라지면 IDLE 복귀) |
| `ST_MONITORING` | 감시 | baseline 대비 무게 급감 AND 거리 증가 → 3회 확인 시 ALERT |
| `ST_ALERT` | 도난 알림 | `ALERT_THEFT` 송신 → COOLDOWN |
| `ST_COOLDOWN` | 재감지 억제 | `COOLDOWN_MS` 경과 → IDLE |

### 판단 알고리즘 — 오탐 억제 설계

- **무게 AND 거리 동시 조건**: 둘 중 하나만 변하는 건 무시. 도착 = 무게↑ + 거리↓, 도난 = 무게↓ + 거리↑ 가 **함께** 나타날 때만 인정
- **디바운스(`CONFIRM_N = 3`)**: 조건을 연속 3회 만족해야 상태 전이. 카운터를 증감하되 글리치로 즉시 리셋하지 않음
- **baseline 기준**: 도착 시점의 무게·거리를 기준값으로 저장하고 이후 상대 비교
- **거리 측정 실패 처리(`distInvalid`)**: 9999/0 값은 무시하고 카운터 유지 → 순간 오측정에 흔들리지 않음
- **쿨다운**: 알림 후 30초간 재감지 억제로 중복 알림 방지

---

## 6. 트러블슈팅 (설계 변경 이력)

| 구분 | 초기 | 최종 | 이유 |
|---|---|---|---|
| WiFi 모듈 | ESP-01 (RAM 8KB) | **ESP32 DevKitC V4 (RAM 520KB)** | 사진 전송에 메모리가 부족 → RAM이 큰 ESP32로 교체 |
| 카메라 제어 주체 | Mega 제어 → UART로 JPEG 전달 | **ESP32가 직접 촬영·전송** | Mega 메모리 부족 + UART 버퍼 오버플로로 **JPEG 손상** → 역할 이전 (가장 큰 변경) |
| 통신 프로토콜 | JPEG 바이트(5KB+)를 UART 전송 | **짧은 명령만 전송** | 대용량 전송을 없애 통신을 단순·안정화 |
| HX711 판독 | 빠른 클럭 | **반주기 지연 + 중앙값 필터** | 상위 비트 오독(값 튐) 제거 |

---

## 7. 빌드 · 실행

- **Mega**: Microchip Studio + STK500 ISP로 굽기 (ISP 클럭 62.5kHz). 순수 AVR C 프로젝트
- **ESP32**: Arduino IDE 1.8.19 (`Arducam_Mega`, `WiFi.h`, `HTTPClient`, `WiFiClientSecure`). `esp32_final.ino`에서 WiFi SSID/비밀번호와 Worker URL 설정 후 업로드

---

## 8. 담당 · 팀

- **장재원 — 상태머신 설계 및 전체 펌웨어 구현 (소프트웨어 전담)**
  - Mega 측 순수 AVR C 전체 (상태머신, HX711/I²C/VL53L0X/UART 드라이버, 판단 로직)
  - ESP32 측 촬영·통신 펌웨어 및 클라우드 연동
- 팀(3인): 김민섭 · 장재원 · 정광현

## 9. 한계 · 개선 방향

- **정상 수령과 도난 구분**: 수취인이 직접 가져가도 무게가 급감 → 앱 버튼·RFID로 "정상 수령" 입력 시 알림 보류
- **개인정보**: 도난 의심 순간만 촬영, 저장 기간 제한·본인 열람
- **전원·네트워크 의존**: 배터리·LTE 모듈 추가로 실외 독립 설치 확장
- **임계값 민감도**: 캔틸레버 틀 제작 후 캘리브레이션 + 적응형 임계값
