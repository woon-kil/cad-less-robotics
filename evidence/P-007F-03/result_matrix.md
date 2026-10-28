# P-007F-03 — Result Matrix (사전등록, §19)

향후 execution에서 다음 matrix를 그대로 사용한다. 수치는 바뀌지 않는다
(execution 결과를 보고 사후 조정 금지).

| Channel A — controller-reported change | Channel B — Gemini external visual change | Result |
|---|---|---|
| YES | YES | `PHYSICAL_MOTION_EVIDENCE_SUPPORTED` |
| YES | NO  | `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION` |
| NO  | YES | `EXTERNAL_MOTION_WITHOUT_CONTROLLER_CHANGE` |
| NO  | NO  | `NO_OBSERVABLE_RESPONSE` |

## Channel B 판정 기준 (이번 gate에서 고정)

```text
VISUAL_MOTION_DETECTED  ⟺  robot-relative translation_px > 1.7826
                          또는 |robot-relative rotation_deg| > 1.0778
```

(`thresholds.json`, robot-relative = ARM2_ROI 변화 − STATIC_REFERENCE_ROI
변화, ORB+RANSAC 기반)

## Quality guard 우선 적용

PRE/POST 어느 한쪽이라도 `quality_guard.json`의 window-level guard 또는
static_reference_guard를 통과하지 못하면 **위 matrix를 적용하지 않는다**.
대신:

```text
EXPERIMENT_INVALID
```

로 판정한다(§19 명시).

## 향후 execution에서 Channel A 정의

`CONTROLLER_REPORTED_SERVO_STATE`(§4) — P-007F-01에서 확정한 대로 PRE/POST
controller readback(`Arm_serial_servo_read` 등, read-only API)의 변화
유무. 이 값은 physical joint angle이 아니다(`claim_boundary.md`).
