# Attempt history

Attempt 1–6 evidence는 폐기하지 않는다. 파일 내용과 PASS/FAIL은 그대로다.

| Attempt | 분류 | 이 revision에서의 역할 |
|---|---|---|
| 1 | `FIXED_POSE_REPEATABILITY` | 고정 자세 반복성. 이후 diversity의 기준 포즈. |
| 2 | `INITIAL_DIVERSITY_VALIDATION` | 여섯 라벨을 처음 측정. tilt는 조건을 넘었고 left/right/near/far는 그 시도에서 넘지 못했다. |
| 3, 4 | `EXTENDED_POSITION_DIVERSITY` | 같은 규칙을 유지한 위치 재측정. Attempt 3 left/right는 20 mm 규칙을 넘었다. near/far의 optical Z는 ±40 mm에 못 미쳤다. |
| 5 | `PHYSICAL_WORKSPACE_CONFLICT` | 같은 높이의 면을 쓰라는 조건에서 far가 `FAR_PHYSICAL_WORKSPACE_LIMIT`로 멈췄다. 그 프레임은 검출되지 않아 포즈가 없다. |
| 6 | `WORKPLANE_GEOMETRY_DIAGNOSIS` | 기존 포즈만으로 작업면과 optical Z 변화율을 계산한 진단. 새 측정이 아니다. |

Attempt 6의 실현 가능성 분류 `INSUFFICIENT_EVIDENCE`와 카메라 relocation 판단 `UNDETERMINED`는 그 진단 파일에 그대로 둔다. R1은 그 숫자를 요구사항 충돌의 근거로 인용하고, 진단 결과를 PASS로 바꾸지 않는다.
