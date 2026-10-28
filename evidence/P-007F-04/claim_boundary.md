# P-007F-04 — §24 Future Result Matrix + §25 Claim Boundary

## §24 — Future execution result matrix

P-007F-03 에서 고정한 matrix 를 수정 없이 그대로 재사용한다:

| Controller-reported change | Gemini external motion | Result |
|---|---|---|
| YES | YES | `PHYSICAL_MOTION_EVIDENCE_SUPPORTED` |
| YES | NO | `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION` |
| NO | YES | `EXTERNAL_MOTION_WITHOUT_CONTROLLER_CHANGE` |
| NO | NO | `NO_OBSERVABLE_RESPONSE` |

Quality guard(§19) 실패 시:

```text
EXPERIMENT_INVALID
```

이 경우 위 matrix 를 적용하지 않는다 — guard 실패는 "motion 없음"으로 해석되지
않는다(측정 자체가 무효라는 뜻).

## §25 — Claim boundary

향후 P-007F-05 실행에서 가장 좋은 결과(`PHYSICAL_MOTION_EVIDENCE_SUPPORTED`)가
나오더라도, 최대 claim 은 다음 한 문장이다:

> One authorized Arm2 command was accompanied by independently observed
> physical robot motion from the fixed Gemini external witness.

### 여전히 금지되는 표현

```text
controller readback = physical angle
physical Arm2 angle = commanded angle
2° command = physical 2° motion     (이번 gate의 candidate 는 3° 지만 원칙은 동일)
absolute positioning accuracy verified
servo encoder accuracy verified
```

### 이 gate(P-007F-04) 자체에 대한 claim boundary

이 gate는 로봇을 전혀 움직이지 않았다(`ROBOT_COMMAND_COUNT=0`). 따라서 이
gate 의 산출물(joint-limit/FK/collision/environment/visual-detectability 분석)
에 대해서도 다음을 명확히 한다:

- `candidate_set.json`/`fk_sweep.json`/`displacement_envelope.json` 의 모든
  수치는 `MODEL_ASSUMPTION_ONLY` 다 — controller-reported ID2=154 가 physical
  Arm2 shaft angle 과 대응한다고 가정했을 때의 예측이며, 실측이 아니다
  (`P-007F-01 READBACK_SEMANTICS_UNRESOLVED` 가 여전히 유효)
- `collision_analysis.md` 의 판정은 `MODEL_LEVEL_COLLISION_CHECK_PASS` 이지
  `PHYSICAL_COLLISION_FREE_PROVEN` 이 아니다
- `visual_detectability.md` 의 추정은 camera extrinsic 캘리브레이션 없는
  근사치이며, "Gemini가 이 motion을 반드시 검출할 것이다"라는 확정적 주장이
  아니다 — 상대적 비교(어느 candidate 가 더 유리한가)에만 쓴다
- `selected_candidate.json` 은 `PREREGISTERED_COMMAND_CANDIDATE` 일 뿐이며,
  **실행 승인이 아니다** — 실제 command 는 이 결과를 별도 검수자가 검토하고
  P-007F-05 가 명시적으로 열린 경우에만 허용된다
