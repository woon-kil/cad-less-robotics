# P-007F-05 — Hardware-free preparation (§34)

이 디렉터리는 **§34까지만** 수행한 결과다. `ROBOT_COMMAND_COUNT = 0`.
P-007F-05 실행(실제 Arm2 servo write)은 아직 열리지 않았다.

## 수행 내역

1. **§2 consistency check** — P-007F-04 `joint_limit_analysis.json` 의
   `directional_asymmetry_finding` 서술문이 152° candidate 의 POST-state
   margin(0.4887rad/28.0°)을 "현재(154°)" 값으로 잘못 인용한 documentation
   typo임을 `candidate_raw.json`(실제 계산)과 대조해 확인 —
   `p007f04_consistency_check.json`, verdict=`P007F04_MARGIN_REPORT_TYPO_CONFIRMED`,
   계산 자체와 candidate 선택 로직은 오염되지 않음, `decision=PROCEED`.
2. **harness 구현**(§1) — `scripts/p007f_05_execution.py`(순수 오케스트레이션,
   하드웨어 import 0), `scripts/p007f_05_analysis.py`(Channel B, P-007F-03
   알고리즘/threshold 그대로 SHA-pin 재사용), `scripts/p007f_05_jetson_controller.py`
   (Jetson 전용, 유일한 `Arm_serial_servo_write` 호출 지점),
   `scripts/p007f_05_hardware_wiring.py`(Mac↔p620/Jetson 2-host 조율,
   self_ast_check 비대상), `scripts/p007f_05_run.py`(CLI 진입점).
3. **AST/static 검사**(§5) — `scripts/p007f_05_hardware_free_check.py`:
   execution.py 하드웨어 import 0 확인 + jetson_controller.py 의
   `Arm_serial_servo_write` 호출이 정확히 1곳·인자(2,151,2000) 리터럴·루프/
   except/finally 밖에 있음을 AST 노드 구조로 증명. true-positive 8/8
   (합성 위반 샘플이 실제로 탐지됨을 먼저 증명).
4. **negative tests**(§6) — NT-01~NT-15 전부 mock, 15/15 PASS,
   `hardware_write_count=0` 전체 확인(`negative_tests_raw.json`).
5. **ledger simulation**(§5 대응) — 신규 `.cache/p007f05_ledger/` 는 이번
   단계에서 생성하지 않음(확인됨), negative tests 가 임시 디렉터리로 전체
   상태 머신을 모의(`ledger_design_verification.md`).
6. **camera-only PRE dry-run**(§6 대응) — p620 Gemini 335L driver 재시작
   (serial/firmware 재확인) → 5프레임 synced capture → Baseline B 확인
   (P-007F-03 reference 대비 shift 0.226px, threshold 1.0px 이내) →
   window/session-level quality guard PASS(5/5 valid) →
   driver 즉시 정지(`pre_dry_run_quality.json`, `pre_dry_run_frames/`).
7. **controller PRE read-only recheck** — Jetson 에서
   `p007f_05_jetson_controller.py --read 5`, ID2=[154,154,154,154,154] 완전
   안정, Arm_Lib 무결성(disk/egg SHA) PASS, `servo_writes=0`
   (`controller_pre_precheck.json`).

## 발견·수정한 이슈(이번 세션 자체 발견)

- **nohup/& 로 ssh 세션 내 장수 프로세스(`ros2 launch`) backgrounding 시
  ssh 세션이 30s 타임아웃까지 블록** — `ssh -f` 로 교체해 해결
  (`scripts/p007f_05_hardware_wiring.py.start_gemini_driver()` docstring에 기록).
- **Python `subprocess.run(capture_output=True)` 의 PIPE 가 `ssh -f` 의
  daemonized 잔존 프로세스에 의해 EOF 를 못 받아 블록** — PIPE 를 DEVNULL 로
  교체해 해결(같은 함수).
- **컨테이너 내부 `/tmp` ≠ host `/tmp`**(기존 메모리 교훈 재발) —
  `capture_window_on_p620()` 출력 경로를 bind-mount 된 workspace 내부
  (`.cache/p007f05_capture_tmp/`)로 바꿔 plain `scp` 로 가져올 수 있게 수정.

## 파일

```text
README.md                        이 파일
p007f04_consistency_check.json   §2
harness_identity.json            §1 — 신규 스크립트 SHA256 전부
hardware_free_check_raw.json     §5 — AST 검사 전체 출력(overall_pass=true)
negative_tests_raw.json          §6 — NT-01~NT-15 전체 출력(15/15)
analysis_selftest_raw.json       Channel B 분석 모듈 자체 selftest(4/4)
ledger_design_verification.md    §5 대응 — ledger 격리/시뮬레이션 설명
baseline_b_precheck.json         (pre_dry_run_quality.json 에 통합되어 있음)
controller_pre_precheck.json     controller PRE read-only 5라운드
pre_dry_run_quality.json         §6 대응 — 실측 PRE dry-run 결과
pre_dry_run_frames/              실측 5프레임 원본(RGB+depth) 보존
harness_ready_report.json        §35 형식 최종 요약
```

## 최종 상태

```text
READY_FOR_EXPLICIT_APPROVAL = YES
ROBOT_COMMAND_COUNT = 0
```

🔴 **실제 Arm2 motion 은 이 단계에서 전혀 실행되지 않았다.** 이 결과를
ChatGPT 가 검수한 뒤, 사용자가 직접 정확한 승인 문구
(`EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`)를 입력해야만 §36 실행 단계로
넘어갈 수 있다.
