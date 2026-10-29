# P-007F-07 Phase A — Marker + Continuous External Motion Witness Preparation

🔴 **완전 OFFLINE.** `robot_command_count=0`, `servo_write_count=0`(AST 검증,
`hardware_free_check.txt`/`one_write_static_check.txt`). P-007F-03/04/05/06 evidence
전부 불변(git diff 확인됨). **실제 motion은 이번 gate에서 전혀 실행되지 않았다.**

## 최종 상태

```text
P-007F-07 PHASE A = PASS / CLOSED

READY_FOR_MARKER_SETUP = YES
READY_AND_ARMED         = NO
MOTION_AUTHORIZED       = NO

ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT   = 0
```

## 핵심 산출물

1. **Marker**: `cv2.aruco DICT_4X4_50`, 권장 크기 50mm(실측 ARM2_ROI depth 677.6mm +
   color fx=610.5px 기준 pinhole 계산 → 45px footprint), fallback 40mm. 부착 위치는
   PROPOSED(사진 기반, 사람 확인 필요) — `marker_spec.md`/`marker_mount_instruction.md`.
2. **Secondary marker**: `RECOMMENDED`(downstream-localized motion 구분용).
3. **Continuous capture 설계**: `scripts/p007f_07_continuous_capture.py`(rclpy 포함,
   코드 완성·문법 검증, 실제 카메라 대상 미실행).
4. **Marker detector**: `scripts/p007f_07_marker_detector.py`, 합성 이미지로 27~45px
   전 구간 검출 검증.
5. **Trajectory classifier**: `scripts/p007f_07_trajectory_classifier.py`, M1~M6/MX
   전체 taxonomy 구현. **10/10 synthetic test PASS**(Case A~E + critical PRE=POST
   regression + abort 조건 4종 + threshold formula 2종).
6. **🔴 자체발견·수정**: oscillation(부호반전) 이 magnitude-only 판정으로는 안 잡혀
   Case C 최초 실패 → `_count_sign_reversals()` 추가 + 분류 우선순위 재배치로 수정
   (`trajectory_classifier.md`).
7. **One-write architecture**: `scripts/p007f_07_jetson_controller.py.do_write()`가
   유일한 `Arm_serial_servo_write` 호출 지점(AST로 call-site 수·위치 증명),
   `scripts/p007f_07_execution.py`는 그 모듈조차 import하지 않음(AST 증명).
8. **Ledger**: `.cache/p007f07_ledger/`(미생성), 6단계 상태머신, restart 차단을
   COMMAND_SENT/POST_CAPTURE_COMPLETE/COMPLETE 전부에 적용(P-007F-05에서 발견된
   "POST_COMPLETE는 차단 안 됨" 버그를 처음부터 재발 방지).
9. **승인→명령 fast path**: `quick_invariant_checks()` 5개 injected callable만 호출,
   build/test/camera-init/ssh-config/file-transfer 없음(구조로 보장).
10. **Two-host sync 설계**: orchestrator를 p620에서 실행해 continuous-capture
    timestamp와 SSH dispatch timestamp가 **같은 clock domain**이 되도록 설계
    (`two_host_sync_design.md`) — Jetson 쪽 clock은 참고용으로만 보존.

## 파일

```text
README.md
design.md
marker_spec.md
marker_mount_instruction.md
marker_setup/marker_candidate_overlay.png
continuous_capture_design.md
timestamp_design.md
two_host_sync_design.md
marker_detector_design.md
trajectory_classifier.md
background_reference_design.md
synthetic_test_results.json
transient_return_regression.json
one_write_static_check.txt
hardware_free_check.txt
ledger_design.md
phase_a_acceptance.json
```

## 다음 단계 (이번 gate에서 열지 않음)

```text
P-007F-07A — MARKER STATIC QUALIFICATION
```

사용자가 할 물리 작업은 marker 출력 + 부착뿐. 그 상태에서 로봇을 움직이지 않고
Gemini로 marker visibility/static noise를 검증한 뒤에만 실제 threshold를 확정한다.
