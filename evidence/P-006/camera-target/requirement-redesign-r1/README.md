# P-006C-R1

상태: **READY_FOR_REVIEW**

이 디렉터리는 요구사항 재설계 문서다. 새 촬영, 카메라 이동, 로봇 이동, threshold 변경은 없다.

기존 P-006C는 `BLOCKED`로 보존한다. Attempt 1–6의 PASS/FAIL을 새 기준으로 소급하지 않는다. `NO RETROACTIVE PASS`.

기존 near/far 규칙은 camera optical Z ±40 mm였다. Attempt 6은 그 변화에 작업면 위 약 140.7 mm가 필요하다고 예측했다. far 방향은 카메라와 로봇이 놓인 설치 영역과 맞닿는다. 이것은 센서 정확도 실패가 아니라 `VALIDATION_GEOMETRY_CONFLICTS_WITH_PHYSICAL_WORKCELL`이다.

새 원칙은 camera 좌표의 임의 이동량이 아니라, 로봇이 실제로 쓸 작업 영역을 검증 범위로 삼는 것이다. 그 영역의 미터 값은 이 단계에서 정하지 않는다. 측정은 다음 게이트 `P-006C-R2 — Task-Workspace ROI Measurement`의 일이다. R2 구현은 시작하지 않는다.
