# P-007F-04 — §22/§23 Durable Execution State (Ledger Design)

## 신규 ledger 경로

```text
.cache/p007f05_ledger/
```

`.gitignore` 에 이미 `.cache/` 가 등록되어 있다(기존 R4/R5 ledger 도 동일 패턴
— 확인: `.cache/p007e_r2r4_ledger/`, `.cache/p007e_r2r5_ledger/` 모두 gitignored).
이번 gate 에서는 이 디렉터리를 **생성하지 않는다** — 설계만 고정한다
(실제 ledger 생성·write 는 P-007F-05 실행 시점).

## 상태 머신

```text
PREPARED -> PRE_VALIDATED -> ARMED -> COMMAND_SENT -> POST_COMPLETE
```

- `PREPARED`: candidate 확정(이 gate의 산출물, `selected_candidate.json`) +
  approval phrase 일치 확인 직후
- `PRE_VALIDATED`: §19/§20 gate 전부 통과 직후(write 호출 전)
- `ARMED`: write 함수 호출 **직전**(R4/R5 와 동일 설계 — write_wrapper 의 첫
  줄에서 durable fsync 기록, 재읽기로 검증)
- `COMMAND_SENT`: write 함수 호출 **직후**(성공/예외 무관하게 기록 — R3 의
  기존 "I2C 보드에 도달했는지 모르면 0 으로 적지 않는다"는 fail-closed 관례를
  유지)
- `POST_COMPLETE`: POST controller readback + Gemini visual window 관측 완료

## Restart/retry protection

```text
COMMAND_SENT 또는 불확실한 sent state 가 존재하면 자동 retry 금지
SSH disconnect 후에도 자동 재전송 금지
```

구현 설계(R4/R5 패턴 재사용, 수정 없이 동일 원칙):

- `experiment_id = sha256(PREREG_SHA + servo_id + target + duration)[:32]` —
  결정적 키. 이 gate의 `selected_candidate.json`(또는 최종 preregistration.json)
  의 sha256 을 PREREG_SHA 로 쓴다
- P-007F-05 실행 시작 시(승인 화면 도달 **전**) 동일 experiment_id 로 ledger 를
  조회한다 — `ARMED`/`COMMAND_SENT`/`POST_COMPLETE` 중 **무엇이든** 이미
  존재하면 즉시 `ABORTED(MANUAL REVIEW REQUIRED)`로 종료하고 R3/승인화면까지
  진행하지 않는다
- durable 기록은 R4 의 `arm_ledger()`(fsync 쓰기) 를 그대로 재사용하고,
  R5 의 `arm_ledger_durable()`(쓴 뒤 **다시 열어 읽어서** 상태를 재확인)
  패턴도 그대로 재사용한다 — 재확인 실패 시(파일시스템 오류 등) 실제 write
  함수를 호출하지 않는다

## §23 — R5 ledger 와의 완전한 분리

```text
기존: .cache/p007e_r2r5_ledger/   (수정/삭제 금지, 이번 gate에서 참조조차 하지
                                    않았다 — p007f_04_candidates.py 는 ledger
                                    디렉터리를 전혀 import/open 하지 않는다)
신규: .cache/p007f05_ledger/      (P-007F-05 전용, 완전히 별도 네임스페이스)
```

R5 provenance self-contamination 문제(요약에 기록된 🔴 미해결 이슈,
`execution_preconditions.md`/P-007F-03 `execution_protocol.md` 에서도 재공개
했던 것과 동일 이슈)는 이번 gate에서도 억지로 수정하지 않는다 — 단지 신규
ledger 가 그 문제와 완전히 격리된 별도 경로를 쓴다는 것만 보장한다.

## 🔴 미해결 공개 사항(재확인)

`.cache/p007e_r2r5_ledger/` 가 현재 provenance allowlist 밖에 있다는 이슈
(P-007F-03 에서 처음 공개)는 여전히 미수정 상태다. 이 gate는 그 문제를 고치지
않고, 단지 신규 P-007F-05 ledger 를 그 문제와 **완전히 분리된 새 경로**에
두기로 설계만 확정한다.
