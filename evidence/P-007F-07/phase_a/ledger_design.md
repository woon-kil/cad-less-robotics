# P-007F-07 Phase A — §24 Durable Ledger Design

## 경로

```text
.cache/p007f07_ledger/
```

`.gitignore`에 이미 `.cache/p007e_r2r4_ledger/`·`.cache/p007e_r2r5_ledger/`·
`.cache/p007f05_ledger/`가 등록되어 있다(P-007F-05에서 "이미 gitignore 되어 있다"는
잘못된 과거 가정을 바로잡아 명시적으로 추가한 패턴, 이번에도 동일하게 추가).
이번 gate에서 이 디렉터리를 생성하지 않는다(negative test는 전부 `tempfile`
임시 디렉터리 사용 — 확인됨, 아래).

## 상태 머신

```text
PREPARED -> READY_AND_ARMED -> APPROVAL_RECEIVED -> COMMAND_SENT
         -> POST_CAPTURE_COMPLETE -> COMPLETE
```

R4/R5/P-007F-05의 `arm_ledger()`(상태 하나 "ARMED"만 하드코딩)를 그대로 재사용하지
않는다 — 이번 상태머신은 6단계라 범용 `write_ledger_state(exp_id, state, ...)`를
새로 작성했다(`scripts/p007f_07_execution.py`). fsync 쓰기 + 재읽기 검증(R4/R5와 동일
`_atomic_write_json` 패턴) 원칙은 그대로 유지.

## Restart/retry 보호 (§24)

```text
COMMAND_SENT, POST_CAPTURE_COMPLETE, COMPLETE  <- 이 중 하나라도 이미 있으면
                                                    check_ledger_allows_restart() 가
                                                    allowed=False 반환
```

R4/R5/P-007F-05는 `ARMED`/`SENT` 두 상태만 차단 대상으로 봤다(P-007F-05에서 자체발견한
버그 — POST_COMPLETE 이후 재실행이 막히지 않던 문제, commit 참조). 이번에는 **처음부터**
"COMMAND_SENT 이후의 모든 상태"를 명시적으로 차단 목록에 포함해 그 버그 패턴을
재발시키지 않는다.

## negative test 결과

`scripts/p007f_07_execution.py.negative_tests()` NT-01~NT-10, 전부 `tempfile`
임시 디렉터리 사용(실제 `.cache/p007f07_ledger/` 미생성 확인) — 10/10 PASS. 특히:

- NT-03: COMMAND_SENT 직후 재시작 차단 확인
- NT-04: POST_CAPTURE_COMPLETE/COMPLETE까지 가도 여전히 차단(P-007F-05 버그 재발 방지 확인)
