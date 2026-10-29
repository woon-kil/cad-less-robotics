# P-007 — Hardware Motion Qualification Final Closure

```text
P-007 = CLOSED / BLOCKED

Controller command path = VERIFIED
Controller-reported state change = VERIFIED

Physical Arm2 angle = UNKNOWN
Physical Arm2 motion = UNRESOLVED

Independent Gemini witness:
STATIC TRACKING = VERIFIED
MOTION OBSERVABILITY = UNRESOLVED

Root blocking condition:
T_Gemini<-RobotBase = NOT VERIFIED

P-007_ADDITIONAL_DIAGNOSTICS = STOPPED
P-006D = NOT OPENED
```

`BLOCKED`는 구현이 실패했다는 뜻이 아니다. 지금 있는 evidence만으로는 controller가 보고한 상태와 실제 관절 운동을 독립적으로 연결할 수 없다는 뜻이다.

이번 문서는 기존 evidence를 고치지 않는다. 새 실험, 촬영, 보정, 로봇 명령은 없다.

## 질문과 결론

| Question | Result |
|---|---|
| Command issued successfully? | VERIFIED |
| Controller state changed? | VERIFIED |
| Readback equals physical angle? | UNRESOLVED |
| Independent static visual tracking works? | VERIFIED |
| Small ID2 motion observable with current measurement model? | UNRESOLVED |
| Physical Arm2 motion established? | NO — UNRESOLVED |
| Additional P-007 diagnostics? | STOPPED |

## A. Command path

Arm_Lib 명령 경로, I2C/MCU 통신, single-write, controller-reported 상태 변화는 기존 게이트에서 확인됐다.

```text
CONTROLLER_COMMAND_PATH = VERIFIED
CONTROLLER_REPORTED_POSITION != VERIFIED_PHYSICAL_JOINT_ANGLE
```

P-007F-01은 `READBACK_SEMANTICS = UNRESOLVED`, `PHYSICAL JOINT POSITION = UNKNOWN`으로 남아 있다.

## B. P-007F-05

한 번의 승인된 Arm2 명령. target 151. controller-reported PRE 154. POST 152. 사전등록된 Gemini 검출은 motion threshold를 넘지 않았다. 실행 시각 운영자 관찰은 YES이고 secondary다.

historical classification은 그대로다.

```text
CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION
```

근거: `evidence/P-007F-05/run_1793274484/`. 이 분류를 바꾸지 않는다.

## C. P-007F-06

```text
F2 = POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED
```

로봇이 차지한 영역에 통계적으로 의미 있는 영상 변화가 있었다. Arm2의 rigid motion으로는 특정하지 못했다. P-007F-05 결과는 그대로다.

## D. External witness

P-007F-07 계열에서 Gemini Baseline B를 고정 외부 카메라로 두었고, M2가 ID2 downstream임을 확인했다. ArUco 정지 qualification은 실패했다. 장착면의 영상 기하를 분석했지만, 경사판과 marker redesign은 설치하지 않았다. 그 미설치는 limitation이다.

## E. P-007F-08 Phase A

M2 Lucas-Kanade 특징점 13/13, STATIC_REFERENCE 18/18, 228프레임, relative 최대 0.155 px, `T_motion = 1.0 px`, holdout false positive 0.

```text
STATIC_TRACKING = VERIFIED
```

정지 추적이 된다는 뜻이다. 작은 물리 회전이 그 임계값을 넘는다는 검증은 아니다. Phase B는 실행하지 않았다.

## F. P-007F-08A

```text
OBSERVABILITY_UNRESOLVED
```

FK는 `RobotBase` frame이다. 카메라 측정은 Gemini camera frame이다. `T_Gemini<-RobotBase`가 없다. ID2 ±1°, ±2°, ±3°의 신뢰할 수 있는 픽셀 변위는 계산하지 않았다. 임의의 extrinsic은 만들지 않았다.

## 결론

The DOFBOT command path and controller-reported state transitions were verified. However, the semantics of the reported servo position as an independently verified physical joint angle remain unresolved. A fixed external Gemini witness was successfully qualified for static continuous tracking, but the observability of the intended small Arm2 motion could not be established because a verified RobotBase-to-Gemini extrinsic transform was unavailable. Therefore physical Arm2 motion remains UNRESOLVED.

쓰지 않는 말:

- physical motion verified
- physical angle verified
- controller readback equals encoder angle
- Gemini proved no physical motion

`NOT OBSERVED`는 `PROVED NO MOTION`이 아니다. Gemini가 임계값을 넘지 못한 것은 움직임이 없었다고 증명한 것이 아니다.

## 서로 다른 기록

삭제하지 않는다. 어느 쪽을 맞게 고르지 않는다.

| 기록 | 값 | 위치 |
|---|---|---|
| P-007E-R2-R5 Attempt #4 실시간 응답 | YES | `evidence/P-007E-R2-R5/run_20261028T211746/r5_final_result.json` |
| 그 실행 뒤 운영자 재고 | NO MOTION | `evidence/P-007F-02/baseline.md`가 사후 재고로 병기 |
| P-007F-05 실행 시각 관찰 | YES, secondary | `evidence/P-007F-05/run_1793274484/operator_observation.json` |
| P-007F-05 Gemini primary | threshold 미달, `motion_yes_no=NO` | 같은 run의 result classification |
| P-007F-06 | F2 | `evidence/P-007F-06/` |
| P-007F-08 | static tracking verified | `evidence/P-007F-08/phase_a/` |
| P-007F-08A | observability unresolved | `evidence/P-007F-08A/verdict.json` |

## Stop Rule

이 프로젝트의 핵심은 `3D Scan → Geometry → Path → MoveIt → Robot`이다.

P-007 계측을 더 하려면 Robot↔Gemini extrinsic calibration, 측정 체계 qualification, marker/mount 재설치, 반복 물리 검증이 필요하다. 그것은 지금 portfolio 목표에 비해 범위가 너무 크다.

```text
P-007_ADDITIONAL_DIAGNOSTICS = STOPPED
```

## 다음

이 closure는 P-006D를 PASS로 만들지 않고, P-006D를 열지도 않는다. 다음 구조 검토에서 고를 후보는 여기 적기만 한다. 이번 작업은 고르지 않는다.

```text
A. P-006D를 현재 physical limitation을 반영하여 재정의
B. Physical execution을 optional/deferred로 두고 CAD-less core pipeline을 계속 진행
C. 다른 verified robot platform에서 execution layer를 나중에 교체
```

```text
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```
