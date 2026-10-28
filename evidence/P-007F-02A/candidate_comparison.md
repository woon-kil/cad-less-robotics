# P-007F-02A — Candidate Ranking

기준: independence · measurement resolution · repeatability · mechanical
loading · safety · setup complexity · currently available · evidence
capture quality.

| 기준 | A. Smartphone | B. Physical level | C. Protractor | D. Fixed camera |
|---|---|---|---|---|
| independence | INDEPENDENT | INDEPENDENT | INDEPENDENT | INDEPENDENT(이론상) |
| measurement resolution | 기종 의존(앱마다 다름), 보통 1° 미만 가능(스펙 미확인) | 기구 의존, 미확인 | 설계목표 ≤1°(`protractor_design.md`) | 사용 불가라 평가 불가 |
| repeatability | 앱/센서 드리프트 가능성, 미확인 | 미확인 | 반복판독 pilot으로 확인 필요(미실행) | 평가 불가 |
| mechanical loading | 🔴 **미확인·확인 불가**(`smartphone_loading_analysis.md` — m_phone/r_phone 미지, 안전 여유 계산 불가) | 기구 질량 미확인이나 통상 소형 | **거의 0**(pointer 질량 최소화 설계) | 해당 없음(비접촉) |
| safety | 추가 하중의 안전성 **확인 불가** | 미확인 | mechanical loading 최소 → 상대적으로 낮은 위험 | 해당 없음(사용 불가) |
| setup complexity | 장착 방법·고정 신뢰성 설계 필요 | 미확인 | protractor 고정대+pointer 제작 필요, 그러나 로봇과의 상호작용은 최소 | 사용 불가 |
| currently available | `OPERATOR_CONFIRMATION_REQUIRED` | `NOT VERIFIED` | 재료는 `OPERATOR_CONFIRMATION_REQUIRED`, 방법론은 평가 가능 | 신규 가용 후보 없음 |
| evidence capture quality | 앱에 따라 로그/그래프 가능(미확인) | 미확인 | 숫자 판독 + 선택적 사진, 재현 가능한 기록 설계(`protractor_design.md`) | 해당 없음 |

## 순위 산정 근거

1. **D(고정 외부 카메라)** — 현재 가용 후보가 전혀 없다(Gemini=unusable,
   Mac=미해결 blocking unknown, 그 외 없음). **순위에서 제외.**
2. **B(physical inclinometer)** — 보유 여부조차 저장소에서 확인되지 않아
   평가할 구체적 기구가 없다. **순위에서 제외(정보 부족).**
3. **A(smartphone)** — 독립성·이론적 분해능은 양호하나, **mechanical
   loading 안전성을 확인할 수 없다는 것이 유일하지만 결정적인 결격 사유**다.
   현재 UNKNOWN 상태인 physical pose + torque state 위에 수치를 알 수 없는
   추가 부하를 얹는 것은 `torque_experiment_design.md`의 §7 위험분석(gravity
   drop 등)을 **악화시킬 수 있는 방향으로만** 작용한다 — 이 분석은 숫자 없이도
   성립한다. primary로 선택하지 않는다.
4. **C(protractor)** — 추가 하중을 설계상 거의 0으로 만들 수 있고(고정
   원판은 로봇에 붙지 않고, pointer만 가볍게 설계), 독립성도 확보되며,
   설계 목표 분해능(≤1°)이 `measurement_requirements.md`의 요구를 만족한다.
   재료 확보는 운영자 확인이 필요하지만, 확보 난이도(종이 출력+경질 pointer+
   독립 받침)는 스마트폰 안전성 미확인 문제보다 훨씬 다루기 쉬운 종류의
   미해결 사항이다.

## 최종 순위

```text
PRIMARY_WITNESS   = C. Printed protractor + fixed external observation
SECONDARY_WITNESS = A. Smartphone inclinometer
                    (조건부 — mechanical loading 이 실측으로 확인되기 전까지는
                     보조 신호로만 기록하고 단독 결론의 근거로 쓰지 않는다)
```

B, D 는 현재 가용성 정보 부족/부재로 순위에 포함하지 않는다. DaBai 는
`independence_analysis.md`에 따라 witness 후보 자체가 아니다(이미
P-007F-02 에서 secondary/corroborating 로만 결정됨).
