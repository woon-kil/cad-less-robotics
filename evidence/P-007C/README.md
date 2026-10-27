# P-007C — 서보 명령 경로·enable 상태 진단

P-007B에서 왜 정상으로 보이는 단일 서보 명령이 움직임도 readback 변화도 만들지 않았는지 조사했다. **모션 명령 0, torque 변경 0, reset 0.** 하드웨어 조회는 소스로 register 비공유를 확인한 2종만 했다.

판정은 **DIAGNOSIS PASS — D (COMMUNICATION PATH HEALTHY, MOTION CAUSE UNRESOLVED)** 다.

## 조회 결과 — 통신은 전부 정상

```
Arm_get_hardversion()  reg 0x01  value='0.20'          0.002s   예외 없음
Arm_ping_servo(1)      reg 0x38  value=218 (0xda)  NORMAL      0.004s
Arm_ping_servo(2)      reg 0x38  value=218 (0xda)  NORMAL      0.004s   ← P-007B 대상
Arm_ping_servo(3..6)   reg 0x38  value=218 (0xda)  NORMAL      각 0.004s

조회 전 ID1~6 5라운드  [86,150,0,18,93,178]  error 0
조회 후 ID1~6 5라운드  [86,150,0,18,93,178]  error 0   ← state 변화 없음
```

벤더 주석(236행)이 `0xda`를 정상으로 정의한다. 보드는 `0x01` 왕복에도 응답한다. 즉 `0x30+id` 읽기 경로와 **독립적인 두 경로**에서 통신이 살아 있고, 컨트롤러는 **서보 6개 전부**와 통신된다.

## 가설별 결과

| 가설 | 결과 | 근거 |
| --- | --- | --- |
| H-C1 통신 | **ELIMINATED** | 보드 `0x01` 응답 + 서보 6/6 ping `0xda` |
| H-C2 torque·enable | **CANNOT_DECIDE** | `0x1A`는 write-only. 소스 전체에 읽기 경로가 없다. §26은 진단 A에 read evidence를 요구한다 |
| H-C3 필수 초기화 누락 | **NOT_SUPPORTED** | 기동 체인에 `set_torque`·`Button_Mode`·보드 enable 호출 0건. 벤더 데모는 `Arm_Device()` 직후 바로 write 한다 |
| H-C4 single-servo 경로 제약 | **NOT_SUPPORTED** | `0x10+id`는 벤더 표준 경로다. `bus_servo_control`도 같은 register를 쓰고 데모들이 `Arm_serial_servo_write`를 직접 호출한다 |
| H-C5 명령 전파 | **OPEN** | 상태 조회는 서보까지 도달하는데 위치 명령은 효과가 없다 |
| H-C6 기타 | **OPEN** | 읽을 수 없는 게이팅 상태가 둘 있다 (torque `0x1A`, 학습모드 `0x03`) |

## 가장 구체적인 발견 — 트랜잭션 유형

| 트랜잭션 | 사용처 | 이 세션 결과 |
| --- | --- | --- |
| `write_byte_data` 1B + `read_word_data` | `Arm_serial_servo_read` (`0x30+id`) | ✅ 1,290+회 성공 |
| `write_byte_data` 1B + `read_byte_data` | `Arm_ping_servo` (`0x38`), `Arm_get_hardversion` (`0x01`) | ✅ 7회 성공 |
| `write_i2c_block_data` **4B** | `Arm_serial_servo_write` (`0x12`) | ❓ 1회, 오류 없음, **효과 없음** |
| `write_i2c_block_data` 12B | `write6` 계열 (`0x1D`) | 호출 안 함 |

검증된 것은 바이트/워드 트랜잭션뿐이고, **block write가 컨트롤러에 의미 있게 수락되는지는 한 번도 확인된 바 없다.**

한계: SMBus 쓰기는 커널이 ACK를 받으면 예외를 내지 않는다. "예외 없음"은 보드가 바이트를 ACK했다는 뜻까지이고, 펌웨어가 payload를 수락했다는 뜻은 아니다. 소스로는 더 좁힐 수 없다.

## 읽을 수 없는 게이팅 상태 둘

| 상태 | register | 접근 |
| --- | --- | --- |
| torque on/off | `0x1A` | **write-only.** 읽기 경로 없음 (`NO_TORQUE_READ_PATH`) |
| K1 버튼 / 학습 모드 | `0x03` | **write-only.** 읽기 경로 없음 |

위치 명령이 무시될 수 있는 두 유력 상태가 모두 이 라이브러리로 확인 불가다. 이것이 진단을 D에서 더 좁히지 못한 직접적인 이유다.

운영자 증언(관찰이며 증거 승격 안 함): torque를 끈 이력 **없다/기억 없다**, 팔 외관은 **단단하게 유지 중으로 보인다**(육안, 만지지 않음).

## 🔴 다음 게이트로 이월하는 위험

원인이 torque·enable이면 **P-007B가 보낸 목표(ID2, raw 1242)가 컨트롤러에 래치돼 있을 수 있다.** 나중에 torque를 켜는 순간 ID2가 그 목표로, 또는 저장된 다른 목표로 움직일 수 있다. 파이썬 정적 분석으로는 어느 쪽인지 알 수 없다.

→ 다음 게이트는 **`Arm_serial_set_torque(1)`을 모션 가능 행위로 취급**하고 전체 모션 게이트 절차(운영자 직시·전원 차단 접근·작업대 clearance·홀드 포인트·단일 명령)를 적용해야 한다.

## 🔴 이 게이트에서 발생한 위반과 복구

조회 직전 preflight에 `scripts/p007b_preflight.py`를 출력 경로 지정 없이 실행해 **`evidence/P-007B`의 `preflight.json`·`control_ownership.json`·`logs/preflight_raw.txt` 3개를 덮어썼다.** 이 플랜의 1단계가 "도구 분리"로 막으라고 적어둔 사고를 그대로 냈다.

- 발견: `logs/evidence_immutability.txt`의 작업 전후 sha256 비교
- 복구: `git checkout -- evidence/P-007B`. 50개 파일 전체 sha256이 P-007C 착수 전 기준선과 동일함을 확인
- 재발 방지: `scripts/p007b_preflight.py`가 `P007_OUT` 환경변수로 출력 경로를 받는다
- **P-007B의 판정과 내용은 소급 변경되지 않았다.** `acceptance.json`의 P007C-29는 `FAIL_THEN_RESTORED`로 기록했다

## 산출물

`acceptance.json`(P007C-01~34) · `environment.json` · `preflight.json` · `control_ownership.json` · `source_integrity.json` · `api_inventory.json`(28함수) · `register_map.json`(23 register) · `single_servo_path.md` · `multi_servo_path.md` · `torque_analysis.md` · `ping_analysis.md` · `hardware_version_analysis.md` · `startup_initialization_analysis.md` · `controller_state_semantics.md` · `query_safety_analysis.json` · `safe_query_plan.json` · `pre_query_reads.csv` · `post_query_reads.csv` · `query_results.json` · `operator_observation.json` · `logs/`

도구: `scripts/p007c_safe_query.py` (allowlist 3함수만 호출).

## STOP

torque를 켜지 않는다. +2°를 재시도하지 않는다. `write6`를 시험하지 않는다. APP·joystick을 재기동하지 않는다. 다음 실험은 권고만 한다.
