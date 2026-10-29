# P-007F-07 Phase A — §8 Continuous Gemini Recording Design

`scripts/p007f_07_continuous_capture.py`(신규, rclpy 포함 — camera wiring 계층이라
`p007f_07_execution.py`의 self_ast_check 대상이 아님, p007e_r2r5_hardware_wiring.py /
p007f_05_hardware_wiring.py와 동일한 설계 원칙).

## 시간축

```text
T-3s(최소) ──────── T0(command dispatch) ──────── T+3s(최소, settle)
```

기본값 `PRE_SECONDS_MIN=3.0`, `POST_SECONDS_MIN=3.0`(§8 최소 요구 그대로). 실제 Phase B
preregistration에서 필요시 늘릴 수 있다(줄이지 않음).

## 저장 방식

- `scripts/p007f_03_capture_static.py`의 `Grab`/`image_array`/`write_png` 로직과 동일한
  방식(코드는 재사용하지 않고 1:1 동일 알고리즘으로 재작성 — 원본이 CLI 진입점 중심이라
  함수 단위 재사용이 어색했음, PNG 인코딩은 바이트 단위로 검증 가능).
- RGB+Depth 동시 기록(§8), RGB가 primary witness, Depth는 corroborating only(P-007F-03
  원칙 유지).
- `TARGET_FPS_HINT=10`으로 다운샘플(Gemini native 30fps 전부 저장은 저장 안정성 위험 —
  §8 "카메라 FPS/저장 안정성을 고려해 결정"). 실제 수치는 Phase B 직전 실측 후 조정 가능.
- 각 frame은 삭제되지 않는다(실패/누락이 있어도 전부 보존, P-007F-03 §12 전통 유지).

## Command trigger hook

`record_continuous_window(..., trigger_command_fn=...)` — PRE 구간이 끝나면 정확히 1회
`trigger_command_fn()`을 호출하도록 설계(§8/§10). Phase B에서 이 콜백이 실제
`scripts/p007f_07_jetson_controller.py.do_write()`를 ssh로 호출하는 wrapper가 된다.
**Phase A에서는 이 콜백이 실제로 연결되지 않는다** — 코드 경로만 준비.

## 미실행 사유 공개

이 스크립트는 rclpy 의존성 때문에 이 세션(Mac, `.venv-p007f03`)에서 직접 실행/테스트할
수 없다(rclpy는 p620의 ROS2 Humble 환경에만 있음) — `python3 -c "import ast; ast.parse(...)"`
로 문법만 검증했다(PASS). 실제 라이브 카메라 대상 실행은 Phase A 범위 밖(§1, 완전
offline)이며, 다음 단계(§27 marker static qualification 또는 Phase B 직전)에서 처음
실행된다.
