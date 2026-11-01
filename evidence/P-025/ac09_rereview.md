# P-025 AC-09 re-review

```text
BASELINE = 6225967
INPUT = evidence/P-025/visual_render/repair/
BEFORE = evidence/P-025/visual_render/VR-*.png

AC09_REPAIR_IMPLEMENTED = YES
AC09_FIRST_VERDICT = FAIL
AC09_REVIEW_AFTER_REPAIR = PASS
AC09_FINAL = PASS

AC10_CURRENT_VERDICT = BLOCKED
P025_PASS = NOT_EVALUATED
```

2026-11-01. 입력은 repair 캡처뿐이다. 수정 전 `VR-*.png`는 before evidence로 남긴다. AC-10은 이 판정에서 바꾸지 않는다.

| 요구 | 1440 `VR-04` | 1280 `VR-04c` | 390 `VR-04b` |
| --- | --- | --- | --- |
| `ROBOTTRAJECTORY_CREATED`와 `NO`가 독립적으로 식별된다 | PASS | PASS | PASS. 라벨 다음 줄에 `NO` |
| `FAIL_CARTESIAN_API` 전체 | PASS | PASS | PASS |
| `0.606061` 전체 | PASS | PASS | PASS |
| `20/33` 전체 | PASS | PASS | PASS |

Scene 5는 `P-022: CLOSED_FAIL`, full 17, `START_IK_FAILED` 5·7·12의 `N/A`, partial run 20, `FAIL_CARTESIAN_API`를 화면만으로 보여 준다. 390에서만 깨지지 않았다.

최초 캡처 판정 `AC09_FINAL = FAIL`(`2738e8f`)은 지우지 않는다. 이 재검수가 그 뒤를 `PASS`로 갱신한다.
