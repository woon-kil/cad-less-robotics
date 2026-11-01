# P-025 repair render

이 디렉터리의 PNG는 AC-09 viewer presentation 수정 뒤의 화면이다. 수정 전 캡처는 상위 `evidence/P-025/visual_render/VR-*.png`에 그대로 있다. 그 실패 판정의 기준선은 `2738e8f`다.

복구 범위는 `5b623fe`다. 바뀐 파일은 `evidence/P-025/viewer/index.html`의 표시와, 새 경로로 찍기 위한 `scripts/p025_capture_render.mjs` 출력 인자다. visualization model, adapter, P-011–P-022, 라벨, 수치는 바꾸지 않았다.

이 캡처는 AC-09 재검수 입력이다. `AC09_FINAL`을 PASS로 바꾸지 않는다. `P025_PASS`는 `NOT_EVALUATED`다. AC-10은 여기서 판정하지 않는다.

회귀로 볼 문자열은 `ROBOTTRAJECTORY_CREATED`와 `NO`의 분리, `FAIL_CARTESIAN_API`, `0.606061`, `20/33`이다. 데스크톱 1440, 노트북 1280, 폭 390 모두 `VR-04`, `VR-04c`, `VR-04b`에 있다.
