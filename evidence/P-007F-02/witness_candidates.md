# P-007F-02 — Torque/Actuator-State Witness Candidates

🔴 operator의 "느낌상 단단하다/부드럽다"만으로 PASS 시키지 않는다 — 아래 모든
candidate 는 **정량 측정치**를 요구한다.

## Candidate A — Controlled External Displacement Witness (비카메라)

**방법**: Arm2 하류 구조(예: Arm3~Arm4 사이의 forearm 세그먼트)에 손으로 작고
일정한 외력을 가하며, 그 변위를 카메라가 아닌 물리적 기준(예: 관절 축에 맞춰 붙인
각도 스케일 + 운영자가 들고 있는 폰/태블릿 inclinometer(수평계) 앱, 또는 간단한
protractor + 고정 포인터)으로 읽는다.

| 검토 항목 | 내용 |
|---|---|
| torque ON/OFF discrimination 가능성 | **가능, 조건부**. torque ON 이면 같은 bounded force 에 대해 변위가 거의 0이어야 하고, torque OFF 면 그 force 가 gravity moment 를 넘는 순간 유의미한 변위가 생겨야 한다 — 두 상태를 같은 force 프로토콜로 비교하면 판별력이 생긴다. 단, **torque OFF 상태 확보 자체**가 Q2 의 위험(아래 `torque_experiment_design.md` 참조)을 수반한다 |
| 필요한 힘 제한 | 사전 고정 값(예: 500g f 이하 상당의 손끝 압력, 1-2초 유지) — 기어/서보 정격을 넘지 않는 작은 힘. 정확한 상한은 실행 gate에서 서보 데이터시트 대조 후 확정(이 gate 에서는 "작고 일정"만 사전등록, 숫자는 P-007F-03 책임) |
| joint/gear 손상 위험 | 낮음(힘이 작고 bounded) — 단, torque OFF 상태에서 **과도한 힘**을 실수로 가하면 기어박스에 역방향 하중이 걸릴 수 있어 상한을 반드시 지정해야 함 |
| 재현성 | 높음 — 같은 지점·같은 방향·같은 유지시간으로 반복 가능. operator 변동성(손힘 일관성)이 주 오차원 |
| 측정 방법 | inclinometer 앱(폰/태블릿, 독립 host — Jetson/로봇과 무관) 또는 protractor+포인터, 전/후 각도 차이를 숫자로 기록 |
| operator bias | 존재 — 같은 운영자가 "토크 상태를 안다"는 선입견으로 힘의 크기를 무의식적으로 조절할 위험. **blind 측정**(측정자가 현재 torque 상태를 모르고 측정만 하는 절차)을 설계에 포함하는 것을 권장(P-007F-03 에서 구체화) |

**판정**: 현재 보유 장비로 바로 가능. 카메라 의존 없음. **권장 후보의 핵심
구성요소**로 선택.

---

## Candidate B — Visual Displacement Witness (카메라)

**방법**: 고정 외부 기준 또는 적절한 camera witness로 같은 외력 조건에서
joint/link displacement를 pixel/marker 변위로 정량화.

| 검토 항목 | 내용 |
|---|---|
| eye-in-hand DaBai 적합성 | `camera_witness_analysis.md` 참조 — Arm2 motion 감도는 높지만 **독립 witness 자격 없음**(카메라 자신이 그 motion 에 실려 있음). torque discrimination 목적에서 "독립적 확인"을 주장할 수 없다 |
| Gemini 335L 적합성 | `camera_witness_analysis.md` 참조 — **NOT CURRENTLY USABLE**(팔이 화각 밖, 3건 독립 evidence로 확인). 이 gate에서 재배치 안 함 |
| pixel/marker displacement 측정 가능성 | DaBai 로는 가능(기존 estimator 재사용 가능 — ORB+RANSAC, frozen). Gemini 로는 현재 불가(대상이 화각 밖) |
| 카메라 자체 이동과 robot movement 구별 가능성 | DaBai: **불가능**(둘이 같은 물리 이동). Gemini: 원칙적으로 가능(고정, 로봇과 분리)했겠지만 현재 화각 밖이라 무의미 |

**판정**: **현재 장비로 독립 visual witness 불가능.** DaBai 는 Candidate A 의
보조 신호로만 병행 기록(아래 "권장 조합" 참조). Gemini 는 사용하지 않음.

---

## Candidate C — 기타 비침습적 witness (신규 구매 전제 없음)

후보: 간단한 **물리 inclinometer/수평계**(폰 앱 또는 독립 아날로그 기구)를
Arm2 하류 세그먼트에 임시로(접착·벨크로 등 비파괴적 방법으로) 부착해 각도를
직접 읽는다. 이것은 Candidate A의 측정 장치와 동일한 발상이지만, "외력을 가하는
행위"와 "각도를 읽는 장치"를 분리해 기록하는 것이 목적이다.

- 기존 재고 평가(`evidence/P-007E-R2-R1/decision.json` primary_candidate)에서
  "MacBook Pro 내장 카메라 — 외부 독립 관찰자"가 R6(robot/controller 독립) 를
  완전히 만족하는 후보로 이미 식별되어 있었으나, 물리 배치·macOS 카메라 권한·
  FOV 검증이 전부 미해결(`blocking_unknowns`)로 남아 있다. 이번 gate 에서도 그
  미해결 상태를 바꾸지 않는다 — 노트북을 옮기거나 카메라를 여는 새 작업을 하지
  않는다.
- 따라서 Candidate C 의 실질적 후보는 "카메라가 아닌 독립 측정 기구"(inclinometer
  앱이 설치된 폰/태블릿, 또는 순수 아날로그 protractor)로 좁힌다 — 이것이
  Candidate A 의 측정 장치와 수렴한다.

**판정**: Candidate A 와 사실상 같은 장치로 수렴. 별도 후보로 유지하되, 실행
설계는 Candidate A 문서에 통합.

---

## 권장 조합 (recommended witness)

**Candidate A(bounded external force + 독립 inclinometer/protractor 측정)를
primary witness로, DaBai ego-motion 기록을 secondary/corroborating 신호로
동시 기록**한다. Gemini 는 사용하지 않는다(NOT CURRENTLY USABLE). 두 signal 이
일치하면 corroborating 증거가 되고, 불일치하면 그 자체가 보고할 가치가 있는
발견이다 — 어느 쪽도 사전에 "맞다"고 가정하지 않는다.

## 기각된 witness와 이유

- **Gemini 335L(그대로 사용)** — 팔이 화각 밖(3건 독립 evidence). 재배치는 이
  gate 범위 밖(§6, 작업판 baseline 보호).
- **DaBai를 단독/독립 witness로 승격** — 명시적으로 금지(이번 작업의뢰서 서두,
  Block 3 기존 정의). eye-in-hand 구조상 "독립 확인"을 구성할 수 없음.
- **신규 센서 구매** — 전제하지 않음(지시).
- **operator 정성 판단("단단하다/부드럽다")만으로 PASS** — 지시로 금지, 정량
  측정 요구.
