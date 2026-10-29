# P-007F-07 Phase A — §11 Two-Host Synchronization Design

## 문제

```text
p620    -- Gemini 335L 연속 캡처(rclpy, host monotonic clock A)
Jetson  -- Arm2 controller write(Arm_Lib, host monotonic clock B)
```

clock A와 clock B는 서로 다른 머신의 서로 다른 monotonic clock이다 — 두 숫자를 직접 빼서
밀리초 단위로 비교하지 않는다(§11 명시 요구).

## 설계 결정 — orchestrator를 p620에서 실행한다

이전 gate(P-007F-05)는 **Mac**에서 orchestrator를 실행하고 ssh로 p620(카메라)과
Jetson(controller) 양쪽에 각각 명령을 보냈다 — 3-host 체인(Mac/p620/Jetson)이었다.

P-007F-07은 continuous capture와 command dispatch 타이밍의 상대적 선후관계가 핵심이므로,
**연속 캡처 루프(clock A)와 Jetson에 ssh로 write를 보내는 호출을 같은 Python 프로세스,
같은 p620 머신에서 실행**하도록 설계한다:

```text
p620 위에서 실행되는 단일 프로세스:
  1. record_continuous_window() 루프 시작 (frame마다 host_monotonic_ns 기록, clock A)
  2. PRE 구간 경과 후:
       ssh_dispatch_sent_ns = time.monotonic_ns()      <- clock A
       subprocess.run(["ssh", jetson, "... --write ..."])  <- 동기 호출, 완료까지 블록
       ssh_dispatch_returned_ns = time.monotonic_ns()  <- clock A
  3. 루프 계속, POST 구간까지 계속 캡처
```

`ssh_dispatch_sent_ns`/`ssh_dispatch_returned_ns`는 **둘 다 clock A**(p620) 값이므로,
frame의 `host_monotonic_ns`(역시 clock A)와 **직접 비교 가능**하다 — Jetson의 clock B는
비교에 전혀 쓰지 않는다. Jetson이 반환하는 `command_call_enter_timestamp_ns`/
`command_call_exit_timestamp_ns`(clock B)는 **참고/corroborating 용으로만 보존**하고,
"이 ns에 물리적으로 발생했다"는 주장에는 쓰지 않는다.

## Uncertainty envelope

```text
COMMAND_INTERVAL = [ssh_dispatch_sent_ns, ssh_dispatch_returned_ns]   (clock A, 둘 다 p620)
```

- `frame.host_monotonic_ns < ssh_dispatch_sent_ns` → **BEFORE command**
- `frame.host_monotonic_ns > ssh_dispatch_returned_ns` → **AFTER command**
- 그 사이 → **DURING command interval(uncertain)** — 이 구간 안의 어느 순간에 실제
  `Arm_serial_servo_write()` 호출이 일어났는지는 SSH round-trip 지연(네트워크+프로세스
  오버헤드) 때문에 정확히 알 수 없다

P-007F-05 실측 참고치: `Arm_serial_servo_write()` 호출 자체의 소요시간은 0.000691초(<1ms,
`command_record.json`)로 극히 짧다 — COMMAND_INTERVAL의 폭은 실제 write 동작 시간이
아니라 **거의 전부 SSH round-trip 오버헤드**다. 목표(§11)는 ms 단위 정밀도가 아니라
"before/during/after" 구분의 신뢰성이므로, COMMAND_INTERVAL 폭 자체를 evidence에 그대로
기록해 불확실성을 숨기지 않는다.

## 목표 미달 시 fallback

추가 물리 actuator(예: LED 트리거)는 쓰지 않는다(§11 명시 금지). SSH round-trip이 과도하게
길면(연속 캡처의 프레임 간격보다 크면) COMMAND_INTERVAL이 여러 프레임에 걸치는 것으로
그대로 기록하고, 그 구간에 속한 frame들은 "DURING, 선후 미확정"으로 분류 — 억지로 더
정밀하게 보이도록 만들지 않는다.

## Phase A에서 검증한 것 / 하지 않은 것

- 검증함: 설계와 uncertainty envelope 계산 로직(코드/문서)
- 검증 안 함: 실제 p620-Jetson SSH round-trip 실측(하드웨어 미사용 원칙, Phase A는 완전
  offline) — 실측은 Phase B 직전 "quick invariant check" 단계 또는 별도 비파괴 latency
  측정에서 수행
