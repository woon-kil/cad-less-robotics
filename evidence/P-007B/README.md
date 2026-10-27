# P-007B — Arm2 최소 제어 모션 식별

이 프로젝트 최초의 의도된 로봇 명령을 보냈다. `Arm_serial_servo_write(2, 152, 2000)` **1회**.

판정은 **NO_MOTION_RESPONSE**다. 명령은 오류 없이 나갔지만 컨트롤러 보고 각도와 물리 링크 모두 변하지 않았다.

| 항목 | 결과 |
| --- | --- |
| preflight | READY, 13/13 PASS. 재부팅·재로그인 없음 |
| uncontrolled servo writer | 0 (명령 직전 재확인에서도 0) |
| baseline (신규 5라운드) | `[86, 150, 0, 18, 93, 178]`, error 0 |
| 명령 | `Arm_serial_servo_write(2, 152, 2000)` — 1회, 예외 없음, `I2C error` 출력 없음 |
| 전송 raw pos | 1242 (현재 readback 150이 뜻하는 raw는 1267~1278) |
| 명령 후 readback | `[86, 150, 0, 18, 93, 178]` — **ID2 불변**, 전 채널 불변, error 0 |
| 예상 readback | 152 또는 153 (정확히 안착 시 153) |
| 운영자 관찰 | A 전혀 움직이지 않았다 · C 다른 관절 없음 · D 이상 없음 |
| 비상 전원 차단 | 사용 안 함 |
| H0 물리 방향 | **NOT_VALIDATED** — 움직임이 없으면 방향 증거가 없다 |
| P4-CONVENTION | 승격하지 않음. H0는 P3 그대로 |
| 자동 복귀·명령 확대 | 0 |
| 이 게이트의 로봇 이동 | 0 |

## CONTRADICTED가 아니다

방향이 반대로 나온 것이 아니라 **아무 일도 일어나지 않았다.** H2를 승인하지 않고 H0도 반박하지 않는다. 방향에 대한 증거가 생기지 않았다.

## 명령 전에 잡아 둔 것 — readback +1 편향

`Arm_serial_servo_write`는 ID2·3·4에 `angle = 180 - angle` 반전을 걸고(19행), `Arm_serial_servo_read`는 같은 반전을 되돌린다(212~213행). 상쇄되지만 양쪽이 `int()` 버림을 하므로 +1 편향이 생긴다.

```
cmd 150 → raw 1266 → read 151
cmd 152 → raw 1242 → read 153      ← 이번 명령
readback 150 ⇒ 실제 raw 1267..1278
1 library degree = raw 12.22 counts
```

그래서 예상 readback을 `152 또는 153`으로 **미리 고정**했다. 이 분석이 없었다면 153을 command semantics 불일치로 읽어 FAIL 판정을 낼 수 있었다. 이번에는 150이 나왔으므로 이 편향과 무관한 결과다. 상세는 `command_api_analysis.md`.

## 측정된 사실만

- I2C 레벨에서 파이썬 예외도, 라이브러리의 `I2C error` 출력도 없었다. 호출은 0.0007초에 반환했다(non-blocking).
- 2000ms + 1.5s 대기 후 읽었다. 5라운드 × 6 = 30 샘플, error 0.
- 컨트롤러 보고 각도는 명령 전후 동일하다.
- 운영자가 무동작을 확인했다. 진동·소음·충돌도 없었다.

**원인 분석은 하지 않았다.** 스펙 §25에 따라 별도 게이트다. 명령을 더 크게 하지 않았고, 토크·reset·calibration을 건드리지 않았고, 복귀 명령도 보내지 않았다.

## 안전 쪽은 전부 지켜졌다

- 승인은 운영자가 홀드 포인트 이후 **직접 타이핑**한 문구로만 받았다. ChatGPT 스펙 §14에 같은 문구가 있어 붙여넣기는 승인으로 보지 않도록 미리 규칙을 박았다.
- 쓰기 직전 가드: Arm_Lib sha256 재확인 → `Arm_Device.__init__`/`__del__` AST 쓰기 호출 검사 → ID2 readback == 150 확인 → `assert id == 2` → `target == base + 2` 검증. `id = 0`은 6축 일괄 경로라 리터럴 `2`만 넘겼다.
- 금지 API 호출 0: `write6`, `write6_array`, `write_any`, `offset_switch`, `set_torque`, `Arm_reset`, `move_init_pose`, gripper.
- 운영자 확인 3종(작업대·비상전원·직시 위치)은 R1 답을 재사용하지 않고 새로 받았다. 모두 YES.
- P-007A-R1 증거 28개는 해시 불변을 작업 전후로 확인했다.

## 산출물

`acceptance.json`(P007B-01~38) · `environment.json` · `preflight.json` · `control_ownership.json` · `source_integrity.json` · `command_api_analysis.md` · `baseline_reads.csv` · `candidate_before.json` · `candidate_target.json` · `expected_motion.json` · `hold_point.json` · `command_log.json` · `post_reads.csv` · `joint_delta.json` · `operator_observation.json` · `urdf_joints.json` · `logs/`

도구: `scripts/p007b_{fk,preflight,write_once}.py`. FK 도구는 `evidence/P-007A/candidate_fk.json`의 좌표를 **오차 0**으로 재현해 검증한 뒤에만 예측에 썼다.

## STOP

원위치 복귀하지 않는다. 명령을 키우지 않는다. Arm3·Arm4·MoveIt·카메라 보정으로 가지 않는다. 다음 게이트는 제안만 하고 실행하지 않는다.
