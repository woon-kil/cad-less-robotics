# P-007F-05 — §5 Ledger simulation (hardware-free)

## 신규 ledger 격리

```text
.cache/p007f05_ledger/
```

이번 §34 준비 단계에서 이 디렉터리는 **실제로 생성되지 않았다** — 확인:

```text
$ find /Users/woonkil/projects/cad-less-robotics/.cache/p007f05_ledger -maxdepth 1
bfs: error: ... No such file or directory.
```

`.cache/p007e_r2r5_ledger/`(R5 전용) 는 참조·수정하지 않았다 — `scripts/p007f_05_execution.py`
는 이 경로를 전혀 import/open 하지 않는다(소스에 문자열로도 등장하지 않음, grep 로 확인).

## 시뮬레이션 방법

`scripts/p007f_05_execution.py.negative_tests()`(NT-01~NT-15, 15/15 PASS —
`negative_tests_raw.json`)가 매 테스트마다 `tempfile.mkdtemp()`로 만든
**격리된 임시 ledger 디렉터리**를 사용해 전체 상태 머신
(`PREPARED -> PRE_VALIDATED -> ARMED -> COMMAND_SENT`, 이번 준비 단계에서는
POST_COMPLETE 까지도 모의 콜백으로 재현)을 모의 실행하고, 테스트 종료 시
`shutil.rmtree()`로 정리한다 — 실제 `.cache/p007f05_ledger/` 는 한 번도 건드리지
않는다.

핵심으로 검증된 것(§6/§16/§17/§22):

- **NT-07/NT-11** — 동일 experiment_id 로 두 번째 `execute_p007f05()` 호출 시
  두 번째 write 가 전혀 발생하지 않는다(restart protection, ledger 파일이
  ARMED/SENT/POST_COMPLETE 어떤 상태로든 이미 존재하면 차단).
- **NT-08** — ARMED 상태에서 "크래시" 를 모의(ledger 파일만 ARMED 로 남기고
  프로세스가 죽은 상황)해도 재시작 시 자동 재시도하지 않는다.
- `arm_ledger_durable()` 가 fsync 쓰기 후 **다시 열어 읽어서** `status=="ARMED"`
  를 확인하기 전에는 실제 write 함수가 호출되지 않는다(R4/R5 선례 패턴 그대로
  재사용, `scripts/p007e_r2r4_preflight.py` 의 `arm_ledger`/`_ledger_path` 를
  SHA-pin import 로 재사용).

## 결론

```text
LEDGER_SIMULATION = PASS (15/15 negative tests, 0 real .cache/p007f05_ledger/ writes)
```
