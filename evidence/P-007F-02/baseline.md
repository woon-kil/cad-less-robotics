# P-007F-02 — Authoritative Baseline

🔴 **DESIGN / PREREGISTRATION ONLY.** 이 문서를 포함한 P-007F-02 전체 작업에서
로봇 상태는 전혀 바뀌지 않았다 — servo write, torque write, reset, calibration,
action playback, PWM write, 수동 이동, R5 재실행, 카메라/USB/I2C 재설정, 관련
process kill/restart 모두 0회.

## P-007F-01 결과(그대로 사용, 재해석하지 않음)

```text
READBACK_SEMANTICS      = UNRESOLVED
TORQUE_STATE            = UNKNOWN
SERVO_COMMUNICATION     = SUPPORTED
PHYSICAL JOINT POSITION = UNKNOWN
```

출처: `evidence/P-007F-01/diagnosis.md`, `readback_semantics.md`,
`readonly_probe.json`. 이 gate는 이 결과를 수정하지 않는다.

## Attempt #4 (P-007E-R2-R5, `evidence/P-007E-R2-R5/run_20261028T211746/r5_final_result.json`)

```text
controller PRE  = [152, 152, 152]
controller POST = [154, 154, 154, 154, 154]

real hardware write = TRUE
approval = {granted: True}

machine-recorded operator response (operator_observation_fn, 실시간 질문-응답) = YES

POST visual quality guard = FAIL
  (DaBai POST window: n_frames=60, n_valid=28(46.7%), 80% 기준 미달)

result_code = X (EXPERIMENT_INVALID)
controller_changed / visual_changed = None (invalid_reasons 때문에 계산되지 않음)
```

**later operator recollection/correction = NO MOTION** — 이 정정은 run 종료 뒤
운영자가 재고하여 검수 세션에 전달한 것으로, `r5_final_result.json` 자동 기록에는
반영되어 있지 않다.

## 두 provenance 를 병렬 유지(덮어쓰지 않음)

| provenance | 출처 | 값 | 비고 |
|---|---|---|---|
| machine record | `r5_final_result.json.operator_observed_physical_motion` | `YES` | 실시간 질문-응답, 수정 안 함 |
| operator correction | 검수 세션 보고(이번 작업의뢰서 §1) | `NO MOTION` | 사후 재고, 별도 보존 |

이 gate는 두 값 중 어느 것도 "맞다"고 판정하지 않는다. 다음 섹션의 결론은 **둘 중
하나를 선택하지 않고도** 성립하도록 설계한다.

## 결론: PHYSICAL MOTION 취급

```text
PHYSICAL MOTION = UNRESOLVED
```

`154°`는 **controller-reported state** 일 뿐이다. 이 문서를 포함한 P-007F-02의
어떤 섹션도 `154°`를 physical Arm2 shaft angle 로 표현하지 않는다.

## 이 gate가 전제하는 것

- controller communication 은 살아 있다(SERVO_COMMUNICATION=SUPPORTED, P-007F-01).
- physical actuator 상태(torque ON/OFF, 실제 shaft 각도)는 **모두 UNKNOWN** 이다.
- 이 불확실성을 해소하려면 **position command 가 아닌** 별도의 물리적 witness 가
  필요하다 — 이것이 P-007F-02 의 설계 대상이다.
