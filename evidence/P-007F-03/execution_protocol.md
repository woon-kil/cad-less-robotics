# P-007F-03 — Future Execution Protocol (사전등록, 이번 gate에서 실행하지 않음)

§17 순서를 그대로 고정한다. 실제 로봇 motion command는 **이번 gate에서
전혀 전송되지 않았다**(§2, robot command count = 0).

```text
1.  Operator presence confirmation
2.  Workspace safety checklist
3.  Baseline B identity/placement check
       — current RGB overview 1장 vs baseline_b_identity.json 비교,
         global phase-correlation shift 확인(static_reference_analysis.md
         방법 재사용). 벗어나면 ABORT_BASELINE_B_CHANGED.
4.  PRE Gemini capture
       — WINDOW>=3 프레임, 동일 sync 기준(color/depth stamp delta<=0.05s)
5.  PRE Channel A controller read
       — Arm_serial_servo_read(2) 등 read-only API만 사용(P-007F-01
         AST whitelist 재사용). controller read != physical angle(§4).
6.  PRE window validation
       — quality_guard.json 의 window-level guard + static_reference_guard
         모두 통과해야 함. 실패 시 ABORT_BEFORE_MOTION(§17 명시, PRE 단계
         실패는 이 이름으로 abort).
7.  Exact approval phrase
       — §31(R5에서 확립된 승인 게이트 관례)과 동일하게, 실제 Arm2 motion
         command 직전 운영자에게 정확한 문구로 예고하고 명시 승인을 받는다.
8.  ONE authorized Arm2 command
       — magnitude/target은 이번 gate에서 확정하지 않는다(§18). 별도
         safety review에서 결정.
9.  settle
       — 서보 정착 시간 확보(R5 window 구조 재사용 가능, 수치는 별도 review).
10. POST Gemini capture — PRE와 동일 WINDOW 크기
11. POST Channel A controller reads
12. POST window validation — 6번과 동일 guard. 실패 시 전체 결과는
        EXPERIMENT_INVALID(§19), result matrix 적용 안 함.
13. Offline Channel B analysis
        — visual_algorithm.md 의 PRIMARY(ORB+RANSAC)로 ARM2_ROI/
          STATIC_REFERENCE_ROI robot-relative translation_px·rotation_deg
          계산, thresholds.json 의 T_translation_px/T_rotation_deg 적용.
14. Result classification — result_matrix.md 적용
15. STOP
```

## Restart/retry protection (§22, 사전등록 — 아직 구현되지 않음, 향후 execution harness가 반드시 가져야 할 설계)

- write-ahead durable state: `ARMED` → `COMMAND_SENT` → `POST_COMPLETE`
  3단계를 R5(`scripts/p007e_r2r5_execution.py`)와 동일한 패턴(ledger
  디렉터리에 JSON 기록, 재시작해도 자동 재전송 금지)으로 재사용한다.
- **기존 R5 ledger(`.cache/p007e_r2r5_ledger/`)를 삭제·수정하지 않는다** —
  별도 ledger 디렉터리(`.cache/p007f_03_ledger/` 등)를 신설해야 한다.
- 🔴 비차단 공개(R5에서 이미 1회 공개한 이슈, 아직 미해결): R5 provenance
  self-contamination 수정(commit `d02611e`)의 allowlist가
  `evidence/P-007E-R2-R5/run_` prefix만 포함하고 `.cache/p007e_r2r5_ledger/`
  는 포함하지 않는다 — R5를 다시 돌리면 같은 self-contamination이 재발할
  수 있다. 이번 gate는 R5를 재실행하지 않으므로(§0 금지) 영향 없지만,
  향후 P-007F-03 execution 설계 시 **새 ledger 경로를 쓰더라도 동일한
  self-contamination 패턴(harness가 만든 evidence가 harness 자신의
  dirty-tree 검사를 막는 구조)이 재현되지 않는지 별도로 점검**해야 한다.

## 이번 gate에서 확정하지 않은 것 (§18)

다음 Arm2 target angle(152/156/기타)을 이번 gate에서 선택하지 않았다.
Command magnitude/target은 별도 safety review 대상으로 남긴다.
