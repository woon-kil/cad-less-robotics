# P-007F-02A — Printed Protractor Method Qualification

🔴 설계 문서다. 실제로 출력·부착·관찰하지 않았다. 재료 보유는
`instrument_inventory.md` 기준 `OPERATOR_CONFIRMATION_REQUIRED`.

## 목표 재확인

**절대 각도(absolute joint angle) 보정이 목표가 아니다.** 측정 대상은:

```text
Δθ_physical = θ_physical(POST) - θ_physical(PRE)
```

즉 동일 기준에서의 PRE→POST **상대** 변화뿐이다. 이것은 controller-reported
`152→154`(상대 +2°, 그러나 physical 값은 UNKNOWN)와 **독립적으로** 같은
사건의 물리적 크기를 추정하는 것이 목적이다.

## 필수 설계(지시 §5 그대로 반영)

1. **protractor는 robot base와 독립된 stationary reference에 고정** — 로봇
   base_link나 작업대가 아니라, 로봇과 기계적으로 분리된 별도 받침(삼각대·
   클램프·고정된 가구 등)에 고정한다. base_link 자체가 흔들리면(safety_analysis
   abort 조건 5) 이 전제가 깨진다 — 그 경우는 abort 대상이다.
2. **pointer/marker만 움직이는 link에 위치** — protractor 원판은 고정, 그
   원판의 각도 스케일을 가리키는 가벼운(질량 최소화) 경질 pointer를 Arm2
   하류의 움직이는 링크에 부착한다. pointer 질량은 `smartphone_loading_analysis.md`
   와 같은 이유로 **0에 가깝게** 설계한다(가벼운 철사/이쑤시개 수준 — 실제
   재료는 운영자 확보 대상).
3. **PRE/POST 동일 관찰 위치** — 관찰자(눈 또는 카메라)의 위치·각도를
   PRE/POST 사이에 바꾸지 않는다. 바뀌면 parallax 오차가 Δθ_physical 측정값에
   그대로 섞인다.
4. **가능한 경우 사진으로 기록** — 필수는 아니다(순수 육안 판독도 유효하다,
   단 재현 가능하게 기록). 사진을 쓸 경우 그 카메라는 protractor와 같은
   stationary reference에 고정해야 한다 — DaBai(eye-in-hand)는 이 역할에
   쓸 수 없다(움직이는 구조에 있으므로 "동일 관찰 위치" 전제가 깨진다).
5. **parallax 최소화** — pointer 끝과 protractor 스케일 사이 거리를 최소화하고,
   관찰 시선을 protractor 평면에 수직으로 유지한다.
6. **판독 단위를 사전 고정** — 예: 1° 눈금 + 눈금 사이 보간은 0.5° 단위까지만
   인정(그 이상 세분화된 보간은 주관적 판단으로 취급). 숫자는
   `measurement_requirements.md`에서 분해능 요구사항과 함께 확정.

## Operator 주관 판단 최소화

- "움직였다/안 움직였다"는 이진 판단을 **허용하지 않는다** — 반드시 숫자
  각도값(판독 단위에 맞춘)을 PRE/POST 양쪽에서 기록한다.
- 가능하면 **blind 판독**(측정자가 PRE/POST 중 어느 것을 보고 있는지, 또는
  torque가 어느 상태인지 모르는 채로 각도만 읽는 절차)을 도입해 확인-편향을
  줄인다 — 구체적 절차는 P-007F-03에서 설계(이 gate는 "도입을 권장한다"만
  사전등록).
- 동일 조건에서 **반복 판독**(같은 정지 상태를 두 번 이상 읽어 일치하는지)으로
  판독 자체의 잡음을 측정한다 — `measurement_requirements.md`의
  repeatability_requirement_deg 참조.

## 기계적 하중

pointer를 제외하면 **추가 하중 없음**(protractor 원판 자체는 로봇에 붙지
않는다). pointer 질량을 최소화하면 전체 추가 하중은 스마트폰 방법보다
구조적으로 작다 — 이것이 `candidate_comparison.md`에서 mechanical loading
항목의 우위로 반영된다.

## 예상 angular resolution

육안 판독 기준, 1° 눈금 protractor + 신중한 parallax 관리 시 일반적으로
0.5~1° 수준의 판독 정밀도를 기대할 수 있다는 것은 광학/측정학의 일반 원칙
(눈금 간격과 보간 한계)이며, 이 저장소의 실측값이 아니다 — 실제 성능은
운영자가 재료를 확보한 뒤 반복판독 pilot(이 gate 범위 밖)으로 확인해야
한다. 이 gate는 "설계상 1° 내외 분해능을 목표로 한다"까지만 사전등록한다.

## 결론

재료(프린터·pointer·고정대) 보유는 `OPERATOR_CONFIRMATION_REQUIRED`이지만,
**방법론 자체는** independence·mechanical loading·parallax 관리 가능성
면에서 다른 후보보다 우위에 있다고 평가한다(`candidate_comparison.md` 참조).
