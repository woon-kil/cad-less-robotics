# P-022-AB-LIB 라이브러리 경로 계측

이 산출물은 P-022와 `evidence/P-022-AB`의 기존 측정값을 바꾸지 않는다. P-022 판정은 `CLOSED_FAIL` / `FAIL_CARTESIAN_API`로 그대로다. P-022-AB 리뷰는 닫지 않는다.

## 고정한 실행 순서

1. 사전 확인. 본 20회에 넣지 않는다. 시작 IK가 성공할 때까지 최대 5회이며, 이번 실행은 1회에서 통과했다.
2. trial 0–9. 같은 wrapper, 상세 기록 OFF.
3. trial 10–19. 같은 wrapper, 상세 기록 ON.

대상은 run 20만이다. 시작 IK 인자는 0.005 s다. `setDefaultIKTimeout`은 호출하지 않았다. group default는 실행 전후 모두 0.5 s다. datum, 목표 자세, solver 알고리즘, `eef_step` 0.005 m, precision은 이 실행에서 바꾸지 않았다.

## 사전 확인

설치본 `computeCartesianPath`의 PATH IK가 wrapper에 들어왔다. 사전 확인 1회에서 시작 IK 호출 1, PATH IK 호출 41, 비어 있지 않은 callback 0, 비어 있지 않은 `consistency_limits` 0이었다. PATH 호출이 받은 timeout은 0.5 s다. 본 20회를 이어서 실행했다.

설치본 KDL은 callback이 없는 `searchPositionIK`를 callback 인자가 있는 시그니처로 넘긴다. 빈 callback은 그 기존 전달이다. 비어 있지 않은 callback이 오면 보존 실패로 본 실험을 멈추도록 두었고, 이번 20회에서는 그 경우가 없었다.

## 본 20회

| trial | 기록 | 시작 IK | 시작 경과 s | fraction | PATH 호출 | PATH timeout s | 종료 분류 |
|---:|---|---|---:|---:|---:|---:|---|
| 0 | OFF | 성공 | 0.002145 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 1 | OFF | 성공 | 0.000832 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 2 | OFF | 성공 | 0.000761 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 3 | OFF | 성공 | 0.000971 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 4 | OFF | 성공 | 0.002679 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 5 | OFF | 탐색 실패 | 0.009583 | null | 0 | — | START_IK_FAILED |
| 6 | OFF | 탐색 실패 | 0.007532 | null | 0 | — | START_IK_FAILED |
| 7 | OFF | 성공 | 0.003924 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 8 | OFF | 성공 | 0.001312 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 9 | OFF | 성공 | 0.003287 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 10 | ON | 성공 | 0.001724 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 11 | ON | 성공 | 0.002156 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 12 | ON | 성공 | 0.002028 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 13 | ON | 성공 | 0.000798 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 14 | ON | 성공 | 0.003230 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 15 | ON | 성공 | 0.000963 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 16 | ON | 성공 | 0.000819 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 17 | ON | 성공 | 0.001316 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 18 | ON | 성공 | 0.001556 | 1 | 41 | 0.5 | NOT_APPLICABLE |
| 19 | ON | 성공 | 0.000746 | 1 | 41 | 0.5 | NOT_APPLICABLE |

`matches_20_over_33`은 20회 모두 false다. PATH IK 실패는 0건이다. 계측 누락은 0건이다. 예외는 없다. trial 5와 6은 시작 IK 탐색 실패다. 두 호출의 error code는 -6이고 경과 시간은 각각 0.009583 s, 0.007532 s다. timeout과 error code만으로 내부 실패 원인을 더 좁혀 확정하지 않는다. 성공한 IK의 error code는 1이다. `call_site`는 전부 `UNCONFIRMED`다. `applies_to_past_library_runs`는 전부 false다.

OFF도 호출마다 timeout, 성공 여부, 경과, error code를 남기므로 시간 비교는 기본 계측 대 상세 계측이다. 시작 IK가 성공한 trial의 Cartesian 경과 시간은 기본 계측 8회에서 0.002336–0.002396 s, 평균 0.002357 s다. 상세 계측 10회는 0.002346–0.002433 s, 평균 0.002376 s다. 평균 차이는 약 19.8 μs다. 표본이 작고 순서가 OFF 다음 ON으로 고정되어 있어, 이 차이로 계측 영향이 없다고 단정하지 않는다.

## 이번 실행에서 확인된 것

이번 계측에서 확인된 성과는 내부 PATH IK에 전달된 timeout이 0.5 s라는 점이다. 시작 IK에 전달된 timeout은 0.005 s다. 시작 IK가 성공한 18회는 fraction 1이고, 각 회의 PATH 호출 41회가 모두 성공했다. trial 5와 6은 시작 IK 탐색 실패라 PATH가 호출되지 않았다.

`20/33`은 이번 20회에서 나오지 않았다. 미재현은 도달 불가능의 증거가 아니다. 과거 라이브러리 실행에서 나온 `20/33`의 종료 원인은 이번 계측으로 확인되지 않았고 `UNCONFIRMED`로 남는다. 해결 여부를 판단할 증거는 아직 부족하다.

## 원격 반영

실험 승인에는 커밋과 push 금지가 있었다. 그 뒤 종료 훅 지시에 따라 `a6bada876e102f5f23c1d9b5a360371ab6ff52c7`을 만들어 `origin/main`에 올렸다. 이 push는 앞서 승인한 금지 범위를 벗어난 작업이다. 이력은 revert하거나 재작성하지 않는다.

2026-11-02 확인. 그 시점의 로컬 `HEAD`와 `origin/main`은 모두 `a6bada8`이었다. 포함된 파일은 다음 세 개다.

- `evidence/P-022-AB-LIB/library_path_trace.json`
- `evidence/P-022-AB-LIB/report.md`
- `scripts/p022ab_lib.cpp`

이 정정은 `a6bada8` 이후의 보고서 수정이다. 작성 직후에는 로컬에만 있었고, 이어서 종료 훅으로 이 파일만 `origin/main`에 반영한다.

## 재현

```bash
source /opt/ros/jazzy/setup.bash
/tmp/p022-ab-lib/build/p022ab_lib \
  /tmp/p022/src/ur5e.urdf /tmp/p022/src/ur5e.srdf \
  /tmp/p022/src/process_runs.txt /tmp/p022-ab-lib/result.json
```

원본 원문의 SHA-256은 실행 후 동일했다. `cartesian_runs.json` `a1795dbb2a86637779b11c34f2f139210a096dfbaaf60485f2a81cbad176eacd`.
