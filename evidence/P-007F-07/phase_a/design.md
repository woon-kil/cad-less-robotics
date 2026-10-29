# P-007F-07 Phase A — Design Overview

## 목적

P-007F-05(PRE/POST 정지 비교, `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`)와 P-007F-06
(offline forensic, `F2 = POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED`)이
남긴 미해결 질문 — "Arm2가 실제로 rigid하게 움직였는가, 아니면 diffuse한 효과인가"를,
marker 기반 continuous recording으로 Phase B에서 답할 수 있도록 **준비**한다. 이번
Phase A는 실행이 아니라 harness/알고리즘 준비와 검증이다.

## 구성요소

```text
scripts/p007f_07_marker_detector.py       ArUco 검출(§4/§12)
scripts/p007f_07_trajectory_classifier.py 궤적 추출 + M1~M6/MX 분류(§13~§18)
scripts/p007f_07_synthetic_tests.py       Case A~E + critical regression(§13/§17/§18)
scripts/p007f_07_continuous_capture.py    연속 RGB+Depth 캡처 설계(§8~§10, rclpy 포함)
scripts/p007f_07_execution.py             ledger/quick-invariant/approval 게이트 골격(§20~§24)
scripts/p007f_07_jetson_controller.py     Phase B의 유일한 write 지점(§23, Phase A 미호출)
scripts/p007f_07_hardware_free_check.py   AST 검증(§28)
```

## 설계 원칙 재확인

- **MAX_SERVO_WRITES=1, AUTO_RETRY=False, AUTO_RETURN=False**를 구조로 보장
  (`p007f_07_jetson_controller.py.do_write()`가 유일한 call site, AST로 증명).
- **승인→명령 사이에 무거운 준비 없음**(§21/§22) — `quick_invariant_checks()`는 5개
  injected callable 호출만 한다, build/test/camera-init/SSH-config/file-transfer 없음.
- **Phase B의 실제 target/duration은 이번 gate에서 확정하지 않는다**(§19) — 152를
  historical reference로만 언급, 자동 결정 로직 없음.

## P-007F-03/04/05/06 evidence 불변

이번 세션에서 `evidence/P-007F-03/`, `evidence/P-007F-04/`, `evidence/P-007F-05/`,
`evidence/P-007F-06/`를 전혀 수정하지 않았다(git diff로 확인, 아래 acceptance 참조).
