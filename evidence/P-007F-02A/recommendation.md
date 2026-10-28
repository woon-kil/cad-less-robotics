# P-007F-02A — Final Recommendation

🔴 **DESIGN / QUALIFICATION ONLY.** 이 문서 작성 시점까지 로봇 상태 변경 0,
외력 적용 0, 카메라/장치 재배치 0.

## 1~4. Inventory / Loading / Protractor — 요약 참조

전체 근거는 `instrument_inventory.md`(§3) · `smartphone_loading_analysis.md`
(§4) · `protractor_design.md`(§5) 참조. 요지: 저장소에 신규 물리 계측 도구의
보유 evidence가 없다 — 추정하지 않고 `OPERATOR_CONFIRMATION_REQUIRED`/
`NOT VERIFIED`로 정직하게 남겼다. 스마트폰은 추가 mechanical loading 의
안전성을 수치로 확인할 수 없어(m_phone/r_phone 미지) primary에서 배제.
Protractor는 재료 확보가 필요하지만 추가 하중이 설계상 거의 0이고 독립성·
분해능 목표를 만족해 primary로 선택.

## 5~8. Measurement requirements / Independence / Ranking — 요약 참조

`measurement_requirements.md`: resolution≤1°/repeatability≤1°/minimum
detectable change≥2°(물리적 규모 추론에 근거, 서보 spec 역산 아님, 명시).
`independence_analysis.md`: A/B/C/D 전부 이론상 INDEPENDENT_PHYSICAL_WITNESS,
DaBai는 NOT_INDEPENDENT(이 목적). `candidate_comparison.md`: 최종 순위
PRIMARY=C(protractor), SECONDARY=A(smartphone, 조건부).

## 9. 다음 bounded-force probe의 최대 허용 변위(사전 검토, 실행 없음)

🔴 아직 실제 외력을 가하지 않았다 — 이 섹션은 **향후** Q1 probe(torque 불변,
`torque_experiment_design.md`)가 승인될 경우 쓸 상한을 사전 검토한다.

### 가용 근거

- **URDF joint limit** — Arm2(lib=152° 기준) upper margin ≈27.996°, lower
  margin ≈152.08°(`evidence/P-007E-R2-R4/joint_limits.json`, 이미 SHA 검증된
  R4 산출물 재인용). 이것은 **하드 기구학적 상한**이다 — 이보다 크게 움직이면
  물리적으로 joint limit 또는 그에 준하는 기계적 걸림에 부딫힌다.
- **현재 physical pose = UNKNOWN**(P-007F-01) — 위 margin은 controller-reported
  lib=152° 를 기준으로 계산된 것이라, 실제 physical angle이 다르면 실제
  여유도 다를 수 있다. 이 상한을 "확정된 안전 여유"로 오인하지 않는다.
- **cable/camera configuration** — DaBai 케이블의 실제 여유 길이, 그리퍼
  주변 간섭 등은 R4의 152→154(+2°) 국소 분석(`swept_motion.json`)에서만
  정량화되어 있고, 이번처럼 더 큰 범위의 임의 변위에 대한 전수 clearance
  분석은 이 저장소에 없다.

### 결론

```text
maximum_allowed_physical_displacement_deg = REQUIRES_OPERATOR_SETUP_MEASUREMENT
```

**근거**: URDF joint limit(~28°)은 "넘으면 안 되는 절대 상한"으로는 쓸 수
있지만, bounded-force probe 의 **실제 안전 상한**(운영자가 손으로 미는 힘의
크기·그 힘에 대한 서보 반응·cable 여유·그리퍼 주변 간섭을 모두 반영한 값)을
숫자로 산출하려면 서보 stiffness 추정치(없음, P-007F-01) 또는 operator 의
실시간 중단 판단이 필요하다. 근거 없는 숫자(예: "5° 이하면 안전")를 임의
선정하지 않는다. P-007F-03 설계 시 운영자가 다음을 직접 확인해야 한다:
(1) 실제 적용할 외력의 크기, (2) 그 힘을 가하는 동안 즉시 멈출 수 있는
"보이는 변화가 생기면 바로 멈춘다"는 절차 — 이 절차가 사실상의 안전상한
역할을 한다. URDF joint limit(~28°)은 이 절차가 실패했을 때의 **최후
하드 상한**으로만 취급한다.

## 최종 판정

```text
PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP
```

**근거**: 방법론(Candidate C, 독립 protractor) 은 선정됐고
independence·resolution·mechanical loading 요구를 설계상 만족하도록
구체화됐다. 그러나 (a) 실제 재료(프린터 출력물·경질 pointer·robot base와
독립된 고정대) 보유가 저장소 evidence로 확인되지 않고, (b) §9의 bounded-force
probe 상한도 운영자의 실시간 설정·확인이 필요하다. 따라서
`PHYSICAL_WITNESS_QUALIFIED`(완전 확정)는 아직 이르고,
`NO_QUALIFIED_PHYSICAL_WITNESS`(방법 자체가 없음)도 아니다 — 방법은 있고
순위도 정해졌으나 **운영자 측 물리적 설정이 남아 있다.**

운영자가 다음 1~2개를 확인해주면 `PHYSICAL_WITNESS_QUALIFIED`로 격상할 수
있는 조건을 명시한다:

1. protractor 재료(출력 가능한 프린터, 가벼운 경질 pointer, robot base와
   독립된 고정 받침) 확보 여부.
2. (선택) 스마트폰을 secondary 로 쓰고자 한다면 실제 기기의 질량과 장착
   후보 위치(=`r_phone`)를 알려주면 `smartphone_loading_analysis.md`의
   수식에 대입해 비로소 수치적 안전성 판단이 가능해진다.
