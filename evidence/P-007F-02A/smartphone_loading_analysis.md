# P-007F-02A — Smartphone Mechanical Loading Analysis

🔴 스마트폰을 실제로 장착하지 않았다. 이 문서는 **수식과 그 수식에 필요한 미지
변수**만 제시한다 — 가정값을 사실처럼 쓰지 않는다.

## 기존 하류 질량 기준선(재사용, 새로 측정하지 않음)

`evidence/P-007F-02/torque_experiment_design.md` 에서 URDF `<mass>` 전부
합산해 이미 계산한 값:

```text
m_downstream_existing ≈ 0.211 kg   (Arm3+Arm4+DaBai+Arm5+그리퍼 조립체)
```

## 스마트폰 추가 모멘트 — 변수만 정의

```text
m_phone   = UNKNOWN   (운영자가 실제로 쓸 기기의 질량 — 저장소에 없음)
r_phone   = UNKNOWN   (Arm2 회전축에서 장착 중심까지의 거리 — 장착 방법에 따라 달라짐, 저장소에 없음)
g         = 9.81 m/s^2 (물리 상수, 유일하게 가정 가능한 값)

tau_phone = m_phone * g * r_phone   [upper-bound 추정, 실제 모멘트는 장착 각도에 따라 이보다 작을 수 있음]
```

`m_phone`과 `r_phone`을 가정하지 않으므로 **`tau_phone` 의 수치 결과를
만들지 않는다.** "일반적인 스마트폰은 대략 N그램" 같은 공개 평균값도 이
저장소의 실측이 아니므로 사용하지 않는다.

## 기존 하류 질량과 비교할 수 있는 형태(수치 없이)

```text
tau_phone_relative_to_existing = tau_phone / (m_downstream_existing * g * r_existing_avg)
```

이 비율조차 `r_existing_avg`(기존 하류 질량의 유효 모멘트 암, 링크별로 다름)를
알아야 계산되며, 이 값도 현재 physical pose UNKNOWN(P-007F-01/02 baseline)과
결합돼 있어 **현재로서는 계산할 수 없다.**

## 질적 판단(수치 없이 내릴 수 있는 결론)

- 스마트폰은 기존 하류 질량(0.211kg)에 **0이 아닌 추가 모멘트**를 반드시
  더한다 — 부호는 항상 같다(추가 하중이 Arm2의 중력부하를 줄이는 방향으로
  작용할 보장이 없다). 이것은 `m_phone`/`r_phone` 값과 무관하게 성립하는
  유일한 정성적 사실이다.
- Arm2가 현재 UNKNOWN 한 physical pose에 있고(P-007F-01), torque 상태도
  UNKNOWN(P-007F-01)이므로, 스마트폰 장착이 "이미 불확실한 weight-bearing
  joint에 알려지지 않은 크기의 추가 부하를 얹는" 행위라는 점은 수치와
  무관하게 성립하는 안전성 우려다.
- 장착 방식(Arm2에 직접 vs Arm2보다 하류의 다른 링크)에 따라 `r_phone`이
  달라지고, 하류로 갈수록 모멘트가 커진다 — 장착 위치 선택 자체가 위험을
  증폭할 수 있다.

## 결론

```text
SAFETY OF DIRECT SMARTPHONE MOUNTING = CANNOT BE CONFIRMED
(m_phone, r_phone 미지 → 수치적 안전 여유를 계산할 수 없음)
```

이 분석만으로 스마트폰 직접 장착을 **추천하지 않는다** — "위험하다고
확인됨"이 아니라 "안전하다고 확인할 수 없음"이 결론이다. 이 결론은
`candidate_comparison.md`의 순위 산정에 그대로 반영된다(스마트폰을 primary
witness로 선택하지 않는 근거 중 하나).
