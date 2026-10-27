# 정지 전략 — P-007A-R1

이 문서는 P-007B 전에 쓸 수 있는 정지 경로를 두 계층으로만 정의한다. 이번 게이트에서 실제 정지 시험은 하지 않았다.

## Normal Stop

추가 command를 즉시 중단한다.

- motion authority 프로세스에 더 이상 명령을 보내지 않는다.
- 필요하면 그 프로세스를 SIGKILL로 끝낸다. SIGINT는 쓰지 않는다. KeyboardInterrupt가 cleanup 코드를 돌려 `write6`를 낼 수 있다.
- **한계**: 이미 컨트롤러로 전송된 이동 명령은 Normal Stop으로 취소되지 않는다. 프로세스를 죽여도 서보는 마지막으로 받은 목표까지 간다. 진행 중인 이동에는 부족하다 (`../P-007A/stop_path_analysis.md`).

## Emergency Stop

운영자가 DOFBOT 전원을 물리적으로 차단한다.

- R1 시점에 운영자가 전원 스위치·전원선 접근을 YES로 확인했다 (`operator_confirmations.json`, R1-20). 실제 차단 시험은 하지 않았다.
- 이것이 검증 가능한 유일한 최종 수단이다. 소프트웨어 경로는 Emergency Stop이 아니다.

## Emergency Stop이라고 부르지 않는 것

| 후보 | 왜 아닌가 |
| --- | --- |
| `Arm_serial_set_torque(0)` | `0x1A`에 `0x00`을 쓰는 미시험 동작이고, 토크가 풀리면 팔이 중력으로 내려갈 수 있다 |
| `Arm_reset` (`0x05` ← `0x01`) | 소스에 동작 정의가 없고 시험하지 않았다 |
| 위치 명령으로 덮어쓰기 | 그 자체가 motion이다 |
| 프로세스 종료만 | 위 Normal Stop의 한계가 그대로 남는다 |

## 전원 재투입 경고

전원을 다시 넣거나 재부팅·재로그인하면 벤더 경로가 팔을 움직일 수 있다.

- `joystick.py`가 시작할 때 `move_init_pose()` → `Arm_serial_servo_write6_array([90,164,18,0,90,30])`을 호출한다 (`../P-007A/startup_inventory.json`).
- APP 문서는 초기화가 끝나면 팔이 initial posture로 회전한다고 적는다. 호스트 autostart를 격리해도 컨트롤러·벤더 전원투입 동작까지 막았다고 볼 수 없다.
- 따라서 전원 재투입은 작업대 clearance를 다시 확인한 뒤에만 한다.

## ONE MOTION AUTHORITY 정책 (P-007B 적용)

P-007B에서 서보에 쓰기를 할 수 있는 프로세스는 **그 게이트에서 따로 승인하는 단 하나의 script/process**뿐이다.

동시에 다음은 모두 금지한다.

- APP (`YahboomArm.pyc`, Flask 6500)
- `joystick.py`
- `arm_driver.py`
- Jupyter 안의 하드웨어 코드
- MoveIt execution
- 벤더 demo

운영 규칙:

1. 승인된 writer를 띄우기 전에 `process_after.json` 기준으로 uncontrolled writer가 0인지 다시 확인한다.
2. 승인된 writer는 자기 pid를 기록하고, 종료할 때 cmdline·starttime으로 신원을 확인한 뒤 종료한다.
3. 두 번째 writer가 관측되면 motion을 시작하지 않는다. 이미 시작했으면 Normal Stop → 필요 시 Emergency Stop.
4. 운영자는 motion 구간 전체에서 팔을 보고 있고, 전원 차단에 손이 닿는 위치에 있다.
