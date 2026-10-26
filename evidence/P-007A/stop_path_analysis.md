# 멈추는 방법

이번 게이트에서 토크, reset, 위치 덮어쓰기를 시험하지 않았다. 지연 시간은 모두 unknown이다.

| 방법 | 설치 근거 | 부작용 | 안전 | 시험 |
|---|---|---|---|---|
| 명령을 더 보내지 않음 | 읽기 루프는 위치를 쓰지 않는다 | 이미 올라간 이동 명령이 있으면 멈추지 않을 수 있다 | 진행 중인 이동에는 부족 | not tested |
| 토크 끄기 | `Arm_serial_set_torque(0)`가 `0x1A`에 `0x00`을 쓴다 | 팔이 중력으로 내려갈 수 있다 | 시험 전이라 safe라고 하지 않는다 | not tested |
| 위치 명령으로 덮어쓰기 | `write`와 `write6` | 그 자체가 motion이다 | unsafe as a stop test | not tested |
| 컨트롤러 reset | `Arm_reset`이 `0x05`에 `0x01`을 쓴다 | 동작은 소스에 없다 | unknown | not tested |
| 전원 분리 | APP 문서는 전원 스위치가 있다고 적는다 | 전원 차단 | 물리 fallback | not tested. 이 세션은 스위치에 손이 닿는지 확인하지 않았다 |

운영자 확인이 없는 전원 접근은 `NOT_CONFIRMED`다. 전원을 끄는 시험은 하지 않았다.

P-007B 전에 시험된 정지 경로는 없다. 후보만 소스에서 구분했다.
