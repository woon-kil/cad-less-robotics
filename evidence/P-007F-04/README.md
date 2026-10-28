# P-007F-04 — Single-Motion Execution Safety Review & Preregistration

🔴 이 gate는 **실제 robot command를 실행하지 않는다**(`ROBOT_COMMAND_COUNT=0`).
P-007F-03에서 승인된 Gemini Baseline B external motion witness protocol을
전제로, 다음 단 한 번의 Arm2 motion execution **후보**를 계산하고 안전성을
사전 검수한 gate다. 실행 승인이 아니다.

## 수행 내역

1. **현재 controller 상태 재확인**(§4) — `scripts/p007f_01_readonly_probe.py`
   를 수정 없이 재사용, Jetson 에서 5-round read-only probe 를 다시 실행:
   ID1-6 = `[86, 154, 0, 18, 93, 178]`, 완전 안정(variance=0), P-007F-01
   (2026-10-28) 값과 정확히 동일. `servo_writes=0`.
2. **Baseline B 확인**(§5) — p620 Gemini 335L driver 재시작(firmware 1.4.60
   재확인), 신규 5프레임 synced RGB+depth capture 후 driver 즉시 정지. P-007F-03
   baseline 대비 phase-correlation shift 0.256px(서브픽셀) — `BASELINE_B_UNCHANGED`.
3. **Candidate 생성**(§6~§11) — 현재(154) 기준 ±1/±2/±3° 양방향 6개 candidate 를
   `scripts/p007f_04_candidates.py`(신규, P-007E-R2-R4 FK 모듈을 SHA-pin import)
   로 계산: joint-limit margin, FK sweep(TCP/DaBai), 보수적 displacement
   envelope, self-collision spot-check(SRDF 1차 + stride=40 FK spot-check 2차
   — R4 선례값과 float 오차 이내로 재현 확인한 뒤 신규 candidate 에 적용).
4. **Environment review**(§12) — 신규 캡처 RGB 를 육안 확인, 예측 envelope
   (≤10.89mm)가 관찰된 clearance(수 cm 이상)보다 현저히 작음을 확인.
5. **Gemini visual detectability**(§13) — camera extrinsic 캘리브레이션이
   없어 엄밀한 projection 은 불가능 → pinhole 근사(depth 3종 × in-plane
   discount 3종 = 9개 가정 조합)로 범위 추정. |delta|=3° 만이 8/9 조합에서
   threshold(1.7826px) 를 안정적으로 넘김.
6. **Candidate 비교·선택**(§14~§16) — lower(-3°, target=151) 선택: visual
   detectability margin + Arm2 joint-limit margin 관리(현재 더 가까운 upper
   limit 로부터 멀어지는 방향) 두 근거.
7. **향후 execution 사전조건 고정**(§17~§23) — duration=2000ms(선례 유지),
   PRE visual/controller abort 조건, exact approval phrase, 신규 ledger 설계
   (`.cache/p007f05_ledger/`, R5 ledger 와 완전 분리).
8. **Claim boundary + result matrix**(§24~§25) — P-007F-03 matrix 재사용,
   claim 상한 명시.
9. **하드웨어 미참조 검증**(§25 AST) — `scripts/p007f_04_hardware_free_check.py`
   (신규) 로 `p007f_04_candidates.py` 가 Arm_* API 를 전혀 참조하지 않음을
   AST 로 증명(true-positive 2종 포함 7/7 PASS).
10. **Negative tests**(§29) — 결정론성, candidate 단일성, joint-limit 위반
    탐지(synthetic), sphere≥exact cross-check, FK module sha-pin 무결성 —
    6/6 PASS.

## 파일

```text
README.md                        이 파일
current_controller_state.json    §4 — 신규 5-round read-only probe 결과
baseline_b_check.json            §5 — Baseline B freeze 확인
baseline_b_check_raw.json        §5 — raw diff/phase-correlation 계산값
baseline_check_frames/           §5 — 신규 5프레임 RGB+depth (원본 보존)
candidate_set.json               §6 — 6개 candidate 정의
candidate_raw.json               §7~§11 — 전체 candidate 원본 계산 결과
joint_limit_analysis.json        §8 — joint-limit margin (6개 candidate)
fk_sweep.json                    §9 — TCP/DaBai FK PRE/POST
displacement_envelope.json       §10 — D_model + margin = D_conservative
collision_analysis.md            §11 — SRDF + FK spot-check, 재현성 검증
environment_review.md            §12 — 육안 clearance 확인
visual_detectability.md          §13 — 근사 추정(9개 가정 조합)
visual_detectability_raw.json    §13 — raw 27행 추정치
candidate_comparison.md          §14~§15 — 비교표 + 선택 근거
selected_candidate.json          §16 — 선택된 candidate 상세
execution_preconditions.md       §17,§19,§20 — duration/abort 조건
approval_phrase.txt              §21 — 향후 승인 문구(정확히 일치만 인정)
ledger_design.md                 §22~§23 — 신규 ledger 설계, R5 분리 확인
claim_boundary.md                §24~§25 — result matrix + claim 상한
negative_tests.json              §29 — 6개 negative test 결과
hardware_free_check.txt          §25 — AST 하드웨어 미참조 검증 로그(7/7)
preregistration.json             종합 요약 + 최종 판정
```

## 최종 판정

```text
P-007F-04 = PASS / CLOSED
```

`PASS 기준`(§26) 13개 항목 전부 충족(`preregistration.json.pass_criteria_checklist`).
`ROBOT_COMMAND_COUNT = 0`.

🔴 **P-007F-05 는 열지 않는다.** `selected_candidate.json` 은
`PREREGISTERED_COMMAND_CANDIDATE` 일 뿐 실행 승인이 아니다 — 실제 command 는
이 결과를 별도 검수자가 검토하고 P-007F-05 가 명시적으로 열린 경우에만
허용된다.
