# P-007F-02A — Required Measurement Performance

🔴 이 단계의 목적은 **actuator torque-state discrimination** 이다 — 아주 작은
position command(예: 1-2°)를 검증하는 것이 **아니다**. 분해능 요구사항은 이
목적에 맞춰 도출한다.

## 판별 대상의 물리적 규모 (왜 "서보 스펙"이 아니라 이 규모로 추정하는가)

이 저장소에는 실제 bus servo 의 stall torque/해상도 데이터시트가 없다
(`P-007F-01 Task A/E`에서 확인된 한계와 동일 — 공식 spec 문서가 없다).
대신 다음 **기존 evidence 수치**로 규모를 추정한다:

- Arm2 하류 총 질량 ≈0.211kg(`torque_experiment_design.md`) — weight-bearing
  pitch joint.
- Arm2 joint limit margin(현재 lib=152° 기준) — upper 쪽 약 28.0°
  (`evidence/P-007E-R2-R4/joint_limits.json.pre.Arm2_Joint.margin_to_upper_rad`
  = 0.4886958637635157 rad = 27.996°), lower 쪽 약 152.1°
  (margin_to_lower_rad = 2.6529041362364842 rad).

torque가 OFF이고 현재 pose에서 중력 모멘트가 0이 아니라면, 저항 없는
weight-bearing joint는 통상 **여러 도(度) 단위**로 눈에 보이게 처지는 것이
물리적으로 자연스럽다(28°에 달하는 가용 joint range 자체가 "작은 처짐"을
의도한 설계가 아님을 시사한다) — 반면 torque가 ON이고 bounded force 가
작다면 변위는 **0에 가까워야** 한다. 즉 두 가설이 만드는 신호의 크기 차이는
구조적으로 크다(수 도 vs 거의 0) — 이것이 분해능 요구를 낮게(=쉽게 달성
가능하게) 잡을 수 있는 근거다.

## 요구사항(설계 목표로 사전고정, 실험 후 변경 금지)

```text
measurement_resolution_deg     <= 1.0   (판독 가능한 최소 눈금/보간 단위)
repeatability_requirement_deg  <= 1.0   (동일 정지 상태를 반복 판독했을 때 일치해야 하는 범위)
minimum_detectable_change_deg  >= 2.0   (이 값 이상의 변화는 "변화 있음"으로, 이 값보다
                                          확실히 작은 차이는 "변화 없음"으로 판정할 수 있어야 함)
```

🔴 이 숫자들은 서보 데이터시트에서 역산한 값이 **아니다** — 위 "판별 대상의
물리적 규모"에서 설명한 정성적 규모 차이(수 도 vs 거의 0)에 맞춰 **여유
있게** 설계 목표로 잡은 값이다. 실험을 먼저 하고 나서 이 값에 맞춰 threshold
를 고치지 않는다 — 반대로, 실제 도구(protractor 등)가 이 목표를 못
만족한다면 그 도구를 PRIMARY로 쓰지 않는다(`candidate_comparison.md`).

## 근거 문서화 — 왜 "더 정밀하게"가 아니라 이 수준으로 충분한가

- 더 높은 분해능(예: 0.1°)을 요구하면 protractor 같은 비카메라 방법으로는
  달성하기 어려워 방법 선택의 폭이 좁아진다. 이번 목적(ON vs OFF 처럼
  **질적으로 다른 두 상태**를 구별)에는 과도한 정밀도다.
- 더 낮은 분해능(예: 5°)을 요구하면 "거의 안 움직였다"와 "약간 움직였다"를
  구별하지 못해 B(ACTUATOR_TORQUE_RESPONSE_NOT_ESTABLISHED)와
  D(NO_OBSERVABLE_TORQUE_RESPONSE)를 혼동할 위험이 커진다.

## 검증되지 않은 전제(명시)

- 위 "수 도 단위로 처진다"는 예측은 **물리적 추론**이며, 실제로 torque OFF
  상태에서 그렇게 처지는지 자체는 **아직 관측되지 않았다**(Q2 실험은
  `torque_experiment_design.md`에서 REJECT됨). 이 문서의 요구사항은 그 추론이
  맞다는 가정 위에 설계된 **목표치**이지, 실측으로 확정된 값이 아니다.
