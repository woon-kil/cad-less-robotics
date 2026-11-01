# P-025 visual render review

```text
P025_VISUAL_RENDER_REVIEW = IN_PROGRESS

VISUAL_FINDING_01 =
ROBOTTRAJECTORY_CREATED value "NO" overlaps its label
on desktop and boundary render.

CLASSIFICATION =
FUNCTIONAL_READABILITY_DEFECT

VISUAL_FINDING_02 =
390px Scene 5 clips/truncates final_gate and partial-run evidence.

CLASSIFICATION =
FUNCTIONAL_READABILITY_DEFECT

AC09_IMPACT =
SUPPORTED

AC09_FINAL =
NOT_YET_EVALUATED

P025_AC01_AC10 =
PENDING_REVIEWER

P025_PASS =
NOT_EVALUATED
```

검토자 기록이다. 2026-11-01. `origin/main`의 `evidence/P-025/visual_render/`를 본 관찰이다. 전체 AC-01–AC-10 판정은 아직 하지 않았다. `AC09_IMPACT = SUPPORTED`는 AC-09가 실패로 확정됐다는 뜻이 아니다.

기준은 줄바꿈 유무가 아니다. evidence의 의미가 화면만으로 복원되는가다.

| 현상 | 판정 | 이유 |
| --- | --- | --- |
| Desktop `ROBOTTRAJECTORY_CREATED` + `NO` 겹침 | 수정 필요 | 라벨과 값의 의미적 분리가 깨짐 |
| 390px `final_gate = FAIL_CARTESI...` | 수정 필요 | Gate 결과 자체가 잘림 |
| 390px partial `0.606061 · 2...` | 수정 필요 | 핵심 실패 증거 `20/33`을 화면만으로 확인하기 어려움 |
| `trajectory_written` 설명 중간 줄바꿈 | 조건부 | 문장이 전부 읽히면 허용, 내용이 clipping되면 수정 |
| 일반적인 자연스러운 문장 wrapping | 허용 | 의미가 보존되면 문제 없음 |
| Overview 실패 상자 | 현재 관찰상 양호 | `CLOSED_FAIL`, `NOT ACHIEVED` 전달 |
| 「주장하지 않음」 목록 | 현재 관찰상 양호 | claim boundary 유지 |

Scene 5의 `NO`, `FAIL_CARTESIAN_API`, `0.606061 · 20/33`은 장식 metadata가 아니다. P-022를 성공처럼 보이게 하지 않는 증거다.

이 기록은 viewer, `visualization_model.json`, adapter, P-022 evidence를 수정하지 않았다. 수정이 필요하면 P-025 시각 검수 안에서 viewer presentation만 고친다. 그 작업의뢰서는 AC-01–AC-10을 항목별로 본 뒤에 만든다.
