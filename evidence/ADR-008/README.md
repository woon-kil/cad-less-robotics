# ADR-008 — Process Boundary Responsibility

상태: `ACCEPTED`  
날짜: 2026-10-31

핵심 파이프라인이 Gemini 335L 한 장에서 공정 경계를 자동으로 찾는 일을 필수로 지지 않는다. 공정 정의와 측정 표면 위의 6D ProcessPath 생성은 서로 다른 책임이다.

이 분리는 perception이 성공했다는 증거가 아니다. R1에서 자동 공정 경계를 현재 acquisition으로 입증하지 못했고, 산업 전반에서 그 방식이 불가능하다고 말하지 않는다.

## 1. 질문

ShoePath가 단일 Gemini 335L 촬영에서 공정 경계를 자동으로 찾는 일을 핵심 파이프라인의 필수 책임으로 계속 요구할 것인가.

아니면 공정 정의와 측정 표면 경로 생성을 다른 하위 시스템의 책임으로 나눌 것인가.

## 2. 배경

처음 개념은 다음이었다.

```text
Physical Shoe
→ Gemini 335L
→ Automatic Shoe Geometry
→ Automatic Process Boundary
→ Robot-independent 6D ProcessPath
→ MoveIt RobotTrajectory
```

R1은 자동 기하·공정 경계 구간이 증거로 성립하는지를 보았다.

| Gate | 결과 |
| --- | --- |
| R1J | appearance 분리 실패. `NOT_ROBUSTLY_SEPARABLE` |
| R1K | 일반 기하 위상 복원 실패. `FAIL_GEOMETRY_FEASIBILITY` |
| R1L | 공정에 쓸 안전 표면 실패. `FAIL_PROCESS_SURFACE_FEASIBILITY` |
| R1M | 추가 시선 가치 실패. `FAIL_ADDITIONAL_VIEW_VALUE` |
| R1N | 신발 단면 특징 실패. `FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY` |

R1N은 이 조사의 마지막 새 perception 방법으로 미리 정해 두었다. R1 perception 조사는 `CLOSED`다.

## 3. 로컬 증거

수치는 각 Gate의 기록을 그대로 옮긴다. 기준은 결과를 본 뒤에 낮추지 않았다.

### R1J

접촉 색 balanced accuracy 0.823. `NOT_ROBUSTLY_SEPARABLE`. 겉모습만으로는 분리가 충분히 안정하지 않다.

### R1K

둘레 덮임 0.714. 가장 큰 빈 구간 0.125 m. 위상은 안정하지 않다. 위치 기하는 쓸 수 있고, 법선 유효 비율은 0.981이다. 국소 측정 표면은 쓸모가 있고, 전체 위상 복원은 충분히 안정하지 않다.

### R1L

최소 공정 길이 0.100 m. 명목 10 mm clearance의 길이는 0.073 m. 5 mm는 0.120 m, 15 mm는 0.045 m. 방향은 성립하고, 명목 계약의 위치 연속은 성립하지 않는다. 표면 법선은 여전히 쓸모가 있고, 공정 안전 영역은 경계 clearance에 민감하다.

### R1M

B의 옆면은 A의 1.58배이고, 사전 기준은 2배다. 평균 방향 차이는 6.0°이고, 사전 기준은 15°다. `FAIL_ADDITIONAL_VIEW_VALUE`. 손으로 돌린 추가 시선은 정합·융합 하위 시스템을 정당화하지 않는다.

### R1N

유효 단면 비율 0.688 (기준 ≥ 0.80). 단일 특징 비율 0.125 (기준 ≥ 0.80). 연속 결손 6 (기준 ≤ 2). 세로 덮임 0.123. 특징 간격 0.121 m (허용 0.030 m). `PARAMETER_SENSITIVITY = STABLE`. 실패는 미리 정한 파라미터 변화에서도 유지된다. 현재 Gemini 촬영은 시험한 절단 단면 방법으로 신발 공정 특징을 충분히 안정하게 찾지 못한다.

## 4. 외부 사례

신발 자동화는 공정 기하의 출처가 하나로 고정되어 있지 않다. 아래는 검토자가 제시한 공개 사례이며, 이 ADR은 그 본문을 재조사하지 않았다.

| 패턴 | 흐름 | 출처 |
| --- | --- | --- |
| A. Vision에서 궤적까지 | 3D scan → 점군 → sole/공정 기하 → glue trajectory → 로봇 좌표 | [IFR, Wipro glue on shoe soles](https://ifr.org/case-studies/automation-of-glue-application-on-shoe-soles) |
| B. 측정한 공정선 | 3D 측정 → gauge line → smoothing → target point → position/orientation | [DBpia NODE11616288](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11616288) |
| C. 모델에서 target | 3D 모델/outsole → bonding line 또는 target → trajectory → 실행 | [KCI ART002771990](https://www.kci.go.kr/kciportal/ci/sereArticleSearch/ciSereArtiView.kci?sereArticleSearchBean.artiId=ART002771990) |

2025년 생산 사례는 upper와 outsole 기하로 bonding line을 정한 뒤, 그 선의 surface normal로 분사 방향을 정한다. [MDPI Applied Sciences 15(11):6365](https://www.mdpi.com/2076-3417/15/11/6365). 제조 의도와 표면에서 얻은 방향은 다른 정보다.

따라서 공정 정의와 trajectory 생성은 하나의 perception 알고리즘일 필요가 없다. Wipro 사례가 있으므로, 점군에서 궤적까지 가는 방식을 불가능하다고 말하지 않는다. 현재 ShoePath acquisition과 R1 증거에서는 그 자동 경계를 입증하지 못했다.

## 5. 결정

`PROCESS DEFINITION`과 `MEASURED-SURFACE 6D PROCESSPATH GENERATION`을 분리한다.

```text
ProcessSpecification
        +
MeasuredSurface
        ↓
Surface Association
        ↓
Position Sampling
        ↓
Surface Normal Association
        ↓
Tangent / Orientation Construction
        ↓
Path Smoothing / Resampling
        ↓
Robot-independent 6D ProcessPath
        ↓
Robot Adapter
        ↓
MoveIt Collision-Aware RobotTrajectory
```

## 6. 주장하지 않는 것

다음은 풀렸다고 말하지 않는다. 상태는 `UNSOLVED / OUTSIDE CURRENT CORE`다.

- `AUTOMATIC_SHOE_SEGMENTATION`
- `AUTOMATIC_PROCESS_BOUNDARY`
- `FULL_SHOE_TOPOLOGY`
- `TRUE_BONDING_LINE_RECOVERY`
- `MULTIVIEW_RECONSTRUCTION`

R1 실패 기록은 그대로 둔다.

## 7. ProcessSpecification

ProcessSpecification은 제조 공정이 쓰려는 영역이나 곡선을 말한다.

최종 로봇 XYZ, 최종 표면 법선, 로봇 관절값을 넣지 않는다. 측정 기하를 건너뛰지 않는다.

허용하는 의미 입력의 예는 정규화한 세로 구간, 측면 지정, surface strip, 시작·끝 anchor, 공정 offset, 점 간격, tool standoff, 공정 방향이다.

```text
ProcessSpecification {
    type: SURFACE_STRIP
    longitudinal_start: 0.20
    longitudinal_end:   0.80
    side: LATERAL_POSITIVE
    process_offset_m: 0.010
    sample_spacing_m: 0.005
    tool_standoff_m: 0.020
}
```

위 예는 설명이다. P-009 스키마는 이 ADR이 고정하지 않는다.

## 8. MeasuredSurface

MeasuredSurface는 관측한 기하를 제공한다. XYZ, frame, 국소 이웃, 유효한 곳의 표면 법선, 있을 때의 geometry confidence가 필요하다.

핵심 ProcessPath 인터페이스는 RGB를 요구하지 않는다. 생성기는 사람이 넣은 최종 좌표가 아니라 측정 기하에서 3D 위치와 방향을 얻는다.

## 9. CAD-less의 뜻

이 프로젝트에서 CAD-less는 핵심 ProcessPath 생성에 CAD 모델, 기준 mesh, 손으로 가르친 로봇 궤적이 필수가 아니라는 뜻이다.

공정 사양이 없다는 뜻도, 제조 의도를 자동으로 찾는다는 뜻도, 모든 신발을 의미적으로 이해한다는 뜻도 아니다.

프로젝트 설명은 “CAD-less measured-surface 6D ProcessPath generation”을 쓴다.

## 10. 로봇 독립

핵심 생성기의 출력은 DOFBOT, myCobot, 특정 SDK, MoveIt 관절 표현, 로봇별 관절 한계에 묶이지 않는다.

```text
ProcessPathPoint {
    position
    normal
    tangent
    orientation
    confidence
}
```

값은 선언한 기하·작업 frame에 둔다. 로봇 적용은 그 다음이다.

## 11. Perception 실험을 더 하지 않는 이유

R1O, R1P처럼 perception 알고리즘을 더 쌓는 선택은 거부한다. 서로 다른 가설을 증거와 함께 다섯 번 보았다. 임계값이나 알고리즘을 더 돌리는 일은 로봇 소프트웨어라는 중심 목표에서 벗어난다. R1N Stop Rule도 이 길을 닫아 두었다.

## 12. R1L 기준을 낮추지 않는 이유

0.100 m를 0.073 m로 바꾸거나, clearance 0.010 m를 0.005 m로 바꾸는 선택은 거부한다. 둘 다 결과를 본 뒤의 변경이다. 기존 기준은 유지한다.

## 13. 다시점 융합을 지금 넣지 않는 이유

R1M은 보완 시선의 가치를 보이지 못했다. 정합과 융합은 그 근거 없이 복잡도를 더한다. 나중 연구로 다시 볼 수 있고, 현재 핵심에는 넣지 않는다.

## 14. CAD·밑창 기준을 필수로 하지 않는 이유

외부 사례에서 outsole·기준 기하는 유효한 산업 해법이다. 그것을 핵심 입력으로 의무화하면 이 프로젝트의 CAD-less 성격이 약해진다. `REFERENCE_GEOMETRY = OPTIONAL_FUTURE_ADAPTER`이며 `CORE_REQUIREMENT`가 아니다.

## 15. 결과

얻는 것: 증거 무결성, CAD-less 핵심, 로봇 독립, 이미 보인 XYZ·법선 능력의 사용, 제조 의도와 기하 실행의 분리, ProcessPath 생성의 독립 시험, MoveIt 앞의 경계.

감수하는 것: 신발에서 경로까지 완전 자동이라고 말하지 않는다. ProcessSpecification이 필요하다. 위쪽의 공정 영역 생성은 미해결이다. 산업 적용에는 recipe, 측정, 기준 기하, vision 하위 시스템, 또는 다른 공정 정의 출처가 필요하다.

## 16. 포트폴리오에 적을 말

Gemini 335L 점군이 신발 표면 로봇 가공에 필요한 공정 경계를 자동으로 줄 수 있는지를 먼저 시험했다. 겉모습 분리, 전체 기하, 안전 표면, 추가 시선, 단면 특징을 미리 정한 기준으로 보았다. 증거는 안정한 자동 공정 경계를 지지하지 않았고, 결과를 본 뒤에 기준을 낮추지 않았다. 그래서 제조 공정 의도와 측정 표면 경로 생성을 분리했다. 핵심은 명시적 ProcessSpecification을 받고, 측정한 3D 표면에서 로봇에 묶이지 않은 6D 경로의 위치와 방향을 만든다.

## 17. 다음 Gate

이 승인으로 P-009를 열지 않는다. P-009를 열기 전에 다음을 고정한다.

1. ProcessSpecification 입력 계약
2. MeasuredSurface 입력 계약
3. ProcessPath 출력 계약
4. frame 규약
5. 법선 방향 규약
6. 접선 구성 규칙
7. quaternion·자세 규약
8. 간격·리샘플 기준
9. smoothing 한계
10. geometry confidence 거절 규칙
11. 경로 연속 기준
12. 합격 지표

사람이 최종 XYZ를 넣어 기하 처리를 건너뛰는 경로는 그 계약에서 막는다. 열두 항목이 고정되기 전에는 P-009를 `OPEN`으로 두지 않는다.

## 18. 상태

```text
ADR-008 = ACCEPTED
R1_PERCEPTION_INVESTIGATION = CLOSED
P-008-R1N = FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY / CLOSED
P-009 = NOT_OPENED
P-010 = NOT_OPENED
P-011 = NOT_OPENED
HISTORICAL_EVIDENCE_MODIFIED = NO
```
