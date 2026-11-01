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
FAIL

AC10 =
BLOCKED

P025_AC01_AC08 =
PASS

P025_AC01_AC10 =
REVIEWER_EVALUATED

P025_FIRST_REVIEWER_EVALUATION =
COMPLETE

P025_REPAIR_SCOPE =
DEFINED

P025_REPAIR_IMPLEMENTATION =
NOT_STARTED

P025_PASS =
NOT_EVALUATED
```

검토자 기록이다. 2026-11-01. `origin/main`의 `evidence/P-025/visual_render/`를 본 관찰이다. 아래 최초 판정이 AC-09를 FAIL로 확정한다. `AC09_IMPACT = SUPPORTED`는 그 확정 전의 관찰이다.

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

이 기록은 viewer, `visualization_model.json`, adapter, P-022 evidence를 수정하지 않았다. 수정이 필요하면 P-025 시각 검수 안에서 viewer presentation만 고친다. 최초 판정은 아래에 있다. viewer 수정 범위와 AC-10 unblock은 그 다음 결정이다. 이 기록은 그 결정을 하지 않는다.


## 최초 reviewer evaluation

기준선 `d8cca7d`. 입력은 `evidence/P-025/visual_render/` 캡처뿐이다. 2026-11-01 검토자가 동의했다.

```text
AC01 = PASS
AC02 = PASS
AC03 = PASS
AC04 = PASS
AC05 = PASS
AC06 = PASS
AC07 = PASS
AC08 = PASS
AC09 = FAIL
AC10 = BLOCKED

AC09_IMPACT = SUPPORTED
AC09_FINAL = FAIL
P025_PASS = NOT_EVALUATED
```

AC-04는 원본 값이 기준선과 같으므로 PASS다. 390px에서 값이 잘리는 것은 presentation이다. AC-09는 그 presentation 때문에 실패 경계의 `NO`, `FAIL_CARTESIAN_API`, `20/33`을 화면만으로 복원하지 못하므로 FAIL이다. AC-10은 캡처가 upstream 파일 불변을 보여 주지 않으므로 BLOCKED다. V-01–V-10으로 AC-10을 넘기는 것은 이 입력 계약 밖이다.

`P025_PASS`는 정하지 않는다. presentation을 고친 뒤 AC-09를 다시 볼 수 있다. visualization model, adapter, upstream evidence는 이 판정에서 수정하지 않았다.
