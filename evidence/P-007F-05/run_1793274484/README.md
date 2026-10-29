# P-007F-05 — run_1793274484 (§36 REAL EXECUTION)

🔴 이 디렉터리는 **실제 Arm2 servo write 1회가 집행된** 유일한 run이다.
`motion_commands_sent = 1`. 사용자(운영자)가 정확한 승인 문구
`EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`를 직접 입력해 승인한 뒤 실행됨.

## 결과 요약

```text
Command:            Arm2 servo write(id=2, target=151, duration=2000ms) — 정확히 1회
Controller PRE:      154, 154, 154, 154, 154 (5/5 일치)
Controller POST:     152, 152, 152, 152, 152 (5/5 일치, 하지만 commanded target(151)과도 다름
                     — raw/readback 버킷 불확실성, P-007F-01 READBACK_SEMANTICS_UNRESOLVED 재확인)
Channel A:           controller_change = YES (154 -> 152)
Channel B (Gemini):  robot_relative translation_px=0.2297 (< T=1.7826)
                     robot_relative rotation_deg=0.3976 (< T=1.0778)
                     -> motion_yes_no = NO
Quality guard:       PASS (5/5 valid frames PRE/POST, static-reference guard PASS)
Result matrix cell:  CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION
Operator observation (SECONDARY): YES — 운영자는 육안으로 물리적 움직임을 봤다고 보고
```

## 🔴 세 갈래 불일치 공개 (수치를 맞추려고 조정하지 않음)

1. **Commanded target(151) ≠ 실제 readback(152)** — 2°만큼 명령·결과가 어긋남
2. **Gemini external witness = NO** — threshold 미달(여유 있게 미달, 1.78px 대비 0.23px)
3. **Operator visual observation = YES** — 이 값은 §22에 따라 SECONDARY이며 Channel A/B
   판정을 덮어쓰지 않는다

이 세 신호가 서로 다른 방향을 가리킨다. `result_matrix_cell`은 사전등록된 규칙(Channel A
YES + Channel B NO → `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`)을 기계적으로 적용한
결과이며, 최종 판정은 검수자(사용자/ChatGPT)가 이 세 신호를 모두 보고 내린다.

## 하드웨어 실행 전 발견·수정한 버그 2건(이 run을 시작하기 전)

1. `p007f04_consistency_check.json`의 `decision` 필드가 정확한 문자열 "PROCEED"가
   아니라 설명 문장이어서 체커가 오탐 abort(안전하게, write 0건) — 수정 후 재시도
   (`run_1793274190/`에 그 안전한 abort 기록 보존)
2. `build_real_callables()`가 Gemini driver를 실제로 시작시키지 않고 과거 로그 파일만
   읽어 identity_valid를 거짓으로 True 반환 → 캡처가 빈 이미지로 실패(write 이전,
   안전하게 중단, 두 번째 시도에서 발견 — 이 시도는 evidence 디렉터리가 비어 있어
   정리 후 재삭제함, git에 흔적 없음). `ensure_driver_fresh()` 추가로 수정.

## 파일

```text
README.md                        이 파일
execution_manifest.json          §32 — git head/commit/experiment_id
executable_identity.json         §32 — harness SHA256
operator_precheck.json           §8/§9 — 운영자 현장확인 + 9개 안전항목(AskUserQuestion)
controller_pre.json              §10 — PRE read 5라운드
gemini_pre_manifest.json         §11 — PRE capture summary
pre_frames/                      §11 — PRE 5프레임 원본(RGB+depth)
pre_quality.json                 §12 — PRE window guard
approval_record.json             §15 — 승인 문구 일치 기록
approval_screen_shown.txt        §15 — 실제 표시된 승인 화면
workspace_attestation.json       §9 — 안전 attestation 원본
ledger_snapshot_final.json       §16/§31 — 최종 ledger 상태(POST_COMPLETE)
command_record.json              §17 — 실제 write 호출 기록(servo_id/target/duration/elapsed)
gemini_post_manifest.json        §20 — POST capture summary
post_frames/                     §20 — POST 5프레임 원본(RGB+depth)
controller_post.json             §21 — POST read 5라운드 + Channel A
operator_observation.json        §22 — 운영자 관찰(SECONDARY, immutable)
post_quality.json                §23 — POST quality guard
static_reference_result.json     §24 — static-reference guard
visual_motion_result.json        §25/§26 — Channel B 원시 수치
depth_corroboration.json         §26 — depth는 corroborating only, 이번 run 별도분석 안 함
result_classification.json       §27 — result matrix cell
execution_result.json            전체 원본 result dict(불변, 수정 없음)
baseline_check_frame/            §7 — Baseline B 확인용 1프레임
```

## Claim boundary (§28/§29)

이 결과로도 다음은 주장하지 않는다:

```text
Physical Arm2 angle = 151° 또는 152°
Physical Arm2 moved exactly -3°
Controller readback = shaft encoder angle
Servo positioning accuracy verified
Absolute robot positioning verified
```

최대 claim(Channel A=YES, Channel B=NO인 이번 결과에 적용 가능한 범위):

> An authorized Arm2 command was sent and accompanied by a controller-reported
> state change (154→152); the fixed Gemini external witness did not detect
> robot-relative visual motion above its preregistered threshold in this window.

## §30 — No automatic return, no retry

이 write 이후 추가 command는 전혀 전송되지 않았다. `151→154` 복귀 없음. 재시도 없음.
