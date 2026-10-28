# P-007F-04 — §17/§19/§20 Future Execution Preconditions

이 파일은 **이 gate에서 지금 검증하는 것이 아니다** — 향후 P-007F-05 (별도로
명시적으로 열린 경우에만) 실행 직전 반드시 통과해야 하는 사전조건을 지금
고정해 둔다.

## §17 — Duration

```text
command_duration_ms = 2000
```

R3/R4/R5 선례(동일 2000ms)를 그대로 따른다. 빠른 motion 보다 관측/안전성을
우선한다는 원칙 — 이미 2000ms 로 다수 gate 에서 검증된 선례가 있으므로 변경하지
않는다(변경 시 근거 명시가 요구되나, 변경하지 않으므로 해당 없음).

## §18 — No automatic return

```text
ONE COMMAND MAXIMUM
NO AUTO RETURN
```

P-007F-05 는 command 전송 후 결과가 무엇이든(controller readback 변경 여부,
Gemini motion 관측 여부와 무관하게) **자동으로 원래 값(154)으로 복귀하지
않는다**. 실행 후 STOP한다 — R3/R4/R5 의 기존 `reject_automatic_return()` 설계를
그대로 재사용한다(수정하지 않음).

## §19 — PRE visual quality gate (abort if any fails)

P-007F-05 실행 직전, 다음 중 **하나라도** 실패하면 write 를 호출하지 않고
`ABORT_BEFORE_MOTION`으로 종료한다. candidate 가 이미 승인되어 있어도 적용된다:

```text
- Gemini identity mismatch (serial CP263630009D / firmware 1.4.60 불일치)
- Baseline B changed (whole-frame/ROI diff 비정상적으로 크거나 phase-correlation
  shift 가 P-007F-03 threshold 를 초과)
- insufficient valid frames (P-007F-03 quality_guard.json window-level guard:
  valid frame < 2/3)
- insufficient matches/inliers (match_count < 4, inlier_count < 4, 또는
  inlier_ratio < 0.5)
- static-reference guard failure (STATIC_REFERENCE_ROI 자체가 threshold를
  넘는 변화를 보임 — 카메라/배경 자체가 흔들렸다는 신호)
- ROI invalid (ARM2_ROI/ROBOT_ROI/STATIC_REFERENCE_ROI 중 하나라도 keypoint
  부족/crop 실패)
- lighting/occlusion problem (사람이 workspace 에 들어왔거나 노출이 붕괴됨)
```

이 7개 조건은 P-007F-03 `quality_guard.json`(§12/§13) 을 그대로 재인용한다 —
새로 완화하거나 추가하지 않는다.

## §20 — Controller PRE-state gate (abort if mismatch)

```text
expected_controller_reported_pre_id2 = 154
```

P-007F-05 실행 직전 controller-reported ID2 readback(write 전, read-only)이
정확히 154 가 아니면:

```text
ABORT_BEFORE_MOTION / CONTROLLER_STATE_MISMATCH
```

으로 종료한다. **새 상태에 맞춰 target 을 자동 재계산하지 않는다** — 154 가
아니면 이 preregistration 전체(selected_candidate.json 의 `command_target=151`)
가 무효가 되고, 완전히 새로운 P-007F-04 재실행(새 candidate 비교)이 필요하다.

## Combined gate ordering (실행 직전)

```text
1. Operator presence confirmation (§17 원래 15단계 순서, P-007F-03 과 동일)
2. §20 controller PRE-state gate
3. §19 PRE visual quality gate(7개 조건)
4. exact approval phrase 일치 확인(approval_phrase.txt)
5. ledger ARMED 기록(durable, fsync+재읽기 검증)
6. write 호출(Arm_serial_servo_write(2, 151, 2000)) — 이 지점이 유일한 write
7. POST 관측(controller readback + Gemini visual window)
8. result matrix 분류(§24, quality guard 재확인 — 실패 시 EXPERIMENT_INVALID)
9. STOP — 자동 복귀 없음, 다음 gate 자동 실행 없음
```
