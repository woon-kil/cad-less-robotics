# P-011 Complex Surface CAD Test Coupon

상태: `OPEN`. 이 문서는 형상 계약이다. STEP 생성, qualification, 출력, 촬영은 하지 않았다.

저장소는 `cad-less-robotics`를 유지한다. 이름을 바꾸지 않고, 디렉터리를 재구성하지 않는다. 기존 scan pipeline을 CAD로 바꾸지 않는다. CAD reference branch는 그 옆에 추가한다.

## 질문

이 Coupon은 이후 CAD geometry 시험체로 사용할 수 있는가.

곡률 추출은 이 질문이 아니다. 그것은 P-012다.

## 역할

이 자산은 footwear ground truth가 아니다. `REFERENCE_GEOMETRY` 후보이며 `GROUND_TRUTH = NONE`이다.

이후 공통 시험체 후보다. 이 Gate는 CAD 자산까지만 본다.

```text
Parametric Definition → BREP → STEP → Qualification
```

하지 않는 일: surface sampling engine, normal processing, curvature processing, ProcessPath, STL 출력, Gemini capture, registration, MoveIt.

## 크기

단위는 mm다.

| 축 | 목표 | PASS 범위 |
| --- | --- | --- |
| X | 180 | 150–200 |
| Y | 100 | 80–120 |
| Z | 35–50 | 25–60 |

작으면 depth noise 대비 형상 변화가 작고, 크면 출력과 촬영이 불편하다. 이 범위는 제조 공차가 아니다.

## 표면

단순 반구가 아니다. X와 Y 모두에서 높이가 변한다.

포함할 영역:

- low-curvature region
- convex region
- concave region
- saddle 또는 transition region

이 네 영역이 있어야 이후 path orientation이 곡률에 따라 달라질 수 있다. 그 경로 생성은 이 Gate 밖이다.

## Face

의미 있는 surface patch로 구성한다. 목표 수는 10–20이다. `FACE_COUNT >= 10`만 맞추려고 면을 쪼개지 않는다.

필요한 성질:

- different topology
- different curvature
- continuous usable process surface

예시 역할은 approach/flat reference, low curvature, convex, convex transition, saddle, concave transition, concave, exit transition, side, base다. 생성 시 이름과 개수는 이 역할을 만족하면 되고, 번호 목록을 억지로 맞추지 않는다.

## 비대칭 landmark

좌우 또는 180° 대칭으로 두지 않는다. registration landmark를 한쪽에 둔다.

```text
Registration landmark ≠ Process surface
```

Landmark는 process path 영역에 넣지 않는다. 산업 fixture의 datum 역할이며, registration을 통과시키려고 넣은 편법이 아니다. 이 Gate에서 registration은 실행하지 않는다.

## 재질

CAD에는 재질을 넣지 않는다. 이후 출력 계약만 예약한다.

```text
opaque
matte
non-reflective
```

검은색 고반사와 투명 재료는 그 출력 계약에서 피한다. 출력은 지금 하지 않는다.

## PASS 조건

`PASS_COMPLEX_SURFACE_CAD_TEST_COUPON`은 아래가 모두 기록될 때다. 지금은 조건만 고정했고 측정은 없다.

```text
PROVENANCE                PASS
REPRODUCIBLE_GENERATION   PASS
STEP_READ                 PASS
UNIT                      mm
SOLID_COUNT               >= 1
CLOSED_SHELL              PASS
CAD_VALIDITY              PASS
FACE_COUNT                >= 10
BOUNDING_BOX              X 150–200, Y 80–120, Z 25–60 mm
FINITE_GEOMETRY           PASS
CONVEX_REGION             PRESENT
CONCAVE_REGION            PRESENT
TRANSITION_REGION         PRESENT
ASYMMETRIC_FEATURE        PRESENT
SURFACE_QUERY             PASS
NORMAL_QUERY              PASS
REPRODUCIBILITY           PASS
```

`SURFACE_QUERY`와 `NORMAL_QUERY`는 별도 geometry engine이 아니다. P-010과 같이 OpenCascade에서 자산이 query 되는지만 본다.

`CAD_VALIDITY`는 `Shape.isValid()`만으로 통과시키지 않는다. P-010에서 BOP 검사가 중단된 STEP은 유효성 PASS가 아니었다.

곡률 query는 P-011 PASS에 넣지 않는다.

## 이후

P-012가 surface, normal, curvature engine이다. P-013이 CAD 6D ProcessPath다. P-014가 출력과 Gemini capture다. P-015가 CAD↔Scan registration이다. 그 Gate들은 열지 않았다.
