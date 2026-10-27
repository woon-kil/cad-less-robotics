# P-007E-R1 — Timing-Controlled Servo Command Experiment

P-007B 와 **단 하나의 변수만** 다르게 하여 같은 명령을 다시 보냈다.
바꾼 것은 마지막 baseline read 완료 ~ servo write 시작 사이의 **explicit quiet interval (>= 50 ms)** 뿐이다.

| 고정 | 값 |
| --- | --- |
| joint | Arm2 / ID2 |
| baseline | 150° |
| target | 152° (`+2°`) — `+5°` 금지 |
| duration | 2000 ms |
| API | `Arm_serial_servo_write` |
| command budget | **1회** (retry · 자동 원위치 없음) |

## Phase A — harness 준비 (서보 쓰기 0)

`acceptance.json` · `dryrun.json` · `guard_negative_test.json`.
판정 **`PHASE-A PASS — HARNESS READY / NO MOTION COMMAND EXECUTED`**.

- 쓰기는 `P007E_R1_ALLOW_SERVO_WRITE=1` opt-in 없이는 도달 불가. 기본값 **OFF**
- write 직전 hard guard — `quiet_interval_ms >= 50.0` 미달이면 `HARNESS_TIMING_ABORT`
- negative test: 0 / 10 / 49.9 ms → ABORT, 50.0 / 60 ms → 통과
- `t_last_read_end` 는 **마지막 I2C 트랜잭션 완료 시각**. quiet window 안에서 버스 접근 0

## Phase B — 승인된 단일 명령 집행

`phase_b_acceptance.json` 이 판정 본문이다. 사전 고정 규칙은 `phase_b_decision_rules.json`
(🔴 **명령 전에** 작성했다).

| 항목 | 값 |
| --- | --- |
| 명령 | `Arm_serial_servo_write(2, 152, 2000)` — 1회, `commanded_at 2026-10-27T17:49:06+0800` (= **KST 18:49:06**) |
| quiet interval | **60.099 ms** (>= 50.0) |
| 예외 · `I2C error` | 0 · 0 |
| 명령 전 readback (3라운드) | `[86, 150, 0, 18, 93, 178]` · None 0 |
| 명령 후 readback (5라운드) | **`[86, 152, 0, 18, 93, 178]`** · None 0 · I2C error 0 |
| 물리 관찰 | 🔴 **UNOBSERVED / OPERATOR_ABSENT** |

🔴 **시계**: Jetson 은 `+0800`, 이 저장소의 git 커밋 시각은 KST(`+0900`) 다. KST 환산 타임라인 —
사전 readback 18:47:10 · 판정 규칙 고정(mtime) 18:47:50 · **명령 18:49:06** · 사후 readback 18:51:39 ·
커밋 `be1491e` 19:02:46.

판정:

```
P-007E-R1 PHASE-B RESULT — A/C NOT DISCRIMINATED
  READBACK CHANGE (사전 기준 충족)
  MOTION: realtime UNOBSERVED · post-hoc photo WITHDRAWN
```

🔴 `TIMING HYPOTHESIS STRONGLY SUPPORTED` · `NOT SUPPORTED` · `ROOT CAUSE VERIFIED` 는 **쓰지 않는다**.

## readback 152 의 산술 (추측 0)

`evidence/P-007B/command_api_analysis.md` 의 라이브러리 식을 그대로 재계산했다.

```
cmd 152        → target raw 1242 → 정확 안착 시 readback 153
observed 152   ⇒ raw 1243..1254   (target 보다 1~12 count = 0.08°~0.98° 모자람)
pre      150   ⇒ raw 1267..1278
raw 이동폭      13..35 counts = 1.06°~2.86° (library scale)
```

readback 152 는 보낸 target raw 의 **정확한 echo 가 아니다** — echo 라면 153 이어야 한다.
target 근처에 안착한 측정 위치와 모양이 같아서 **A 를 배제할 수 없다**. 다만 컨트롤러 내부
보간 결과 보고와 이 데이터만으로 갈리지 않는다. 🔴 **n=1 이다** — 성질로 일반화하지 않는다.

## 프로토콜 이탈 (검수 보고용)

1. 운영자가 명령 전 4조건을 "전부 YES"로 응답했으나 실제로 **자리에 없었다** — ④ 미충족
2. 관찰 순서 역전 — 실시간 관찰 불성립이어서 readback 을 먼저 고정했다
3. 운영자 비맹검 — readback 결과와 A/C 매핑을 들은 뒤 사후 비교
4. 사후 사진 비교는 사전 등록되지 않았고, 운영자가 "5mm 판별 불가"로 **철회**했다
5. `phase_b_decision_rules.json` 은 명령 전에 **작성**됐으나(mtime 18:47:50 KST) 명령 전에
   **커밋되지는 않았다**(커밋 19:02:46 KST). 사전 고정 근거는 git 이 아니라 mtime 이다

## 현재 로봇 상태 (후속 세션 필수 확인)

🔴 readback 은 이제 `[86, 152, 0, 18, 93, 178]` 이다. **baseline 150 을 전제한 스크립트는 전부
stale** 이다 (이 harness 의 `EXPECTED_BASELINE` 포함). 동일 조건 재실행은 추가 모션 명령 없이
불가능하다 — baseline guard 가 `ID2 == 150` 을 요구한다.

RGB 는 `P-007D-R1 induced BLUE` 유지. 이 게이트에서 RGB write 0.

## 선행 증거 불변

`logs/phase_b_manifest_before.json` ↔ `logs/phase_b_manifest_after.json` — 9게이트 134파일
변경 0 · 누락 0 · 추가 0. Phase A 산출물도 수정하지 않았다.
