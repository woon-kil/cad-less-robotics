# P-025 repair scope

```text
BASELINE_OBSERVATION = d8cca7d
BASELINE_VERDICT = 2738e8f
P025_REPAIR_SCOPE = DEFINED
P025_REPAIR_IMPLEMENTATION = IMPLEMENTED

AC09_FIRST_VERDICT = FAIL
AC09_REVIEW_AFTER_REPAIR = PASS
AC09_FINAL = PASS
AC09_REPAIR = VIEWER_PRESENTATION_ONLY

AC10_CAPTURE_INPUT = BLOCKED
AC10_UNBLOCK = V10_MANIFEST_MATCH
AC10_RECHECK_PATHS = 81
AC10_RECHECK_MISMATCH = 0

P025_PASS = NOT_EVALUATED
UPSTREAM_EVIDENCE = READ_ONLY
```

2026-11-01. 이 문서는 수정 범위다. viewer, visualization model, adapter, P-011–P-022는 이 결정에서 고치지 않았다.

## AC-09

실패는 값의 오류가 아니다. `NO`, `FAIL_CARTESIAN_API`, `20/33`이 화면에서 라벨과 분리되어 끝까지 읽혀야 한다.

허용 파일은 `evidence/P-025/viewer/index.html`의 presentation뿐이다. CSS로 부족하면 `dt`/`dd`를 나누는 markup만 허용한다.

- `dl`의 고정 `220px` 첫 열이 값과 겹치지 않게 한다. `ROBOTTRAJECTORY_CREATED`와 `NO`는 서로 다른 상자다. 라벨을 줄이거나 `NO`를 다른 단어로 바꾸지 않는다.
- 폭 390에서 `final_gate`는 `FAIL_CARTESIAN_API` 전체가 보인다. partial 값은 `0.606061`과 `20/33`이 보인다. 잘리는 `overflow`와 `nowrap`은 쓰지 않는다. 의미가 남는 줄바꿈은 허용한다.
- `trajectory_written` 문장이 폭 390에서 잘리면 같은 방식으로 끝까지 보이게 한다.
- run_id 0–20, fraction, scope 문장, 「주장하지 않음」 목록, 수치, 색이 구분하는 full·fail·partial 의미는 바꾸지 않는다.

금지: `visualization_model.json`, `viewer/model.js`, `scripts/p025_evidence_adapter.py`, P-011–P-022, datum, threshold, 재계산.

수정 뒤 같은 VR 세트를 다시 찍고 AC-09만 재검수한다. AC-01–AC-08과 AC-10 캡처 판정은 이 수정으로 덮어쓰지 않는다.

## AC-10

캡처 입력의 `BLOCKED`는 유지한다. 캡처는 파일 불변을 보여 주지 않는다.

unblock 경로는 기존 V-10이다. `scripts/p025_validate_view.py`의 `upstream_manifest()`는 `evidence/P-011`–`P-022` 파일과 roadmap의 `PRIMARY STORY:`부터 `### P-026 Portfolio / Demo Packaging` 직전까지를 해시한다. 2026-11-01 재계산은 `visualization_model.json`의 `provenance.upstream_sha256` 81개와 같고, 불일치는 0이다.

이 재계산은 upstream을 고치지 않는다. 새 실험이 아니다. AC-10 전체 PASS 기록은 이 범위 문서와 별도다. V-10 표를 고쳐서 캡처 판정을 지우지 않는다.

## 아직 하지 않는 것

구현, 재렌더, `P025_PASS`, model·adapter·upstream 수정은 하지 않는다.
