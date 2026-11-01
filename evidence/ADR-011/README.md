# ADR-011 — CAD와 Footwear 자산 분리

상태: `ACCEPTED`  
날짜: 2026-11-01  
Gate: P-010A `PASS_ARCHITECTURE_DECOUPLING`

요청된 파일명 `ADR-010`은 쓰지 않는다. `evidence/ADR-010`은 2026-10-31에 닫힌 camera↔workboard 회복 결정이다. 이 문서는 그 결정을 바꾸지 않는다.

이 Gate는 구현하지 않는다. Test Coupon을 만들지 않고, P-011을 열지 않는다.

## Context

초기 가정은 하나의 공개 화형 STEP/BREP가 CAD 검증, footwear 관련성, CAD↔Scan registration을 함께 담당하는 것이었다.

```text
licensed public Shoe Last STEP/BREP
        ↓
CAD Geometry
        ↓
Scan Registration
        ↓
ProcessPath
```

## Problem

P-010과 P-010R 이후, 그 세 책임을 하나의 외부 자산에 둘 수 없다.

- 공개 화형 STEP 후보는 있었으나 라이선스가 포트폴리오 재배포를 허용하지 않거나 적격 검사를 통과하지 못했다.
- 라이선스가 명시된 footwear 자산은 STL, OBJ, GLB, Fusion `.f3d`가 중심이다.
- qualification에 쓸 licensed shoe-last BREP는 확보되지 않았다.

## Evidence

| Gate | 결과 | 이 결정에 쓰는 사실 |
| --- | --- | --- |
| P-010 | `FAIL_CAD_ASSET_QUALIFICATION` / `CLOSED` | `freecad-shoe-last-wb` `59d1db3b`는 STEP을 만들었다. 단위와 크기는 mm 화형 범위다. face는 3개다. `Shape.isValid()`는 true이나 BOP 검사는 `OperationAborted`다. 라이선스는 `NOT_DECLARED`다. 수리하지 않았다. |
| P-010R | `CANDIDATE_FOUND` / `CLOSED` | primary asset은 `NONE`이다. GPL-3.0 Open Shoe Designer와 MIT New World Grade는 mesh다. CC0 last-shoe는 자전거 신발이지 화형 BREP가 아니다. `shoes.stp`는 라이선스가 없어 거절했다. Zenodo GLB는 CC BY 4.0 스캔 mesh다. |

근거 파일: `evidence/P-010/acceptance.json`, `evidence/P-010R/candidate_matrix.md`.

## Decision

`DECOUPLE_CAD_AND_FOOTWEAR_ASSETS`

CAD geometry 기술 검증과 footwear domain 검증은 하나의 공개 shoe-last STEP/BREP에 의존하지 않는다.

### A. CAD Engine Branch

STEP/BREP, OpenCascade, surface evaluation, normal, curvature, 6D ProcessPath를 검증한다. 입력은 프로젝트가 직접 만들고 provenance가 분명한 Complex Surface Test Coupon이다. 이 자산은 footwear ground truth가 아니다.

### B. Footwear Branch

실제 footwear geometry에서 RGB-D, point cloud, surface, normal, ProcessPath를 검증한다. 입력은 Gemini 335L 측정 또는 라이선스가 명확한 footwear mesh다. CAD engine 검증 책임은 없다.

### C. CAD ↔ Scan Branch

같은 물체의 registration만 검증한다. 경로는 Test Coupon, STL export, 3D print, Gemini 335L scan이다. 비교 대상은 original CAD와 measured point cloud다. Registration은 correspondence다. CAD와 scan 어느 쪽도 ground truth로 선언하지 않는다.

## Alternatives Considered

1. 라이선스가 있는 화형 STEP을 더 찾을 때까지 CAD engine을 멈춘다. P-010R에서 primary가 나오지 않았고, 신발 CAD 확보가 프로젝트 성공 조건은 아니다.
2. P-010 STEP을 수리하거나 face를 나눠 통과시킨다. 거절한 증거를 고치는 일이라 하지 않는다.
3. Mesh를 STEP으로 바꿔 CAD처럼 쓴다. BREP 적격의 대체로 쓰지 않는다.
4. `riccca/shoes_last_data`의 STEP을 라이선스 추정으로 채택한다. 저장소 라이선스가 없고 상업 이용은 협의다.

선택한 대안은 책임을 나누는 것이다.

## Common Contract

세 geometry source는 같은 중립 계약으로 모인다. 이 Gate에서 schema와 코드는 만들지 않는다.

```text
CAD/BREP ─────────┐
                  │
Footwear Mesh ────┼──> Geometry Contract
                  │
RGB-D Scan ───────┘
                         ↓
                   6D ProcessPath
                         ↓
                    Robot Adapter
                         ↓
                    ROS2 / MoveIt2
```

ProcessPath는 로봇에 독립이다. 최소 개념만 적는다.

```text
ProcessWaypoint
{
    position
    orientation
    surface_point
    surface_normal
    process_offset
    source
    confidence
}
```

`source`는 `CAD`, `SCAN`, `MESH`다.

## Ground Truth Policy

`CAD is ground truth`와 `Scan is ground truth`는 쓰지 않는다. 별도 측정 검증이 없으면 `GROUND_TRUTH = NONE`이다.

- CAD는 `REFERENCE_GEOMETRY`
- Scan은 `MEASURED_GEOMETRY`

현재 `CAD_PRIMARY_ASSET = NONE`, `CAD_GROUND_TRUTH = NONE`, `SCAN_GROUND_TRUTH = NONE`이다.

## Registration Policy

`T_target_source`는 좌표 correspondence다. Registration 성공은 두 geometry가 같다는 뜻이 아니다. CAD↔Scan 차이에는 다음이 섞일 수 있다.

- manufacturing deviation
- printing deviation
- RGB-D measurement error
- calibration error
- registration error
- actual geometry difference

차이는 제거할 오차로 자동 판정하지 않는다.

## Adaptive Path Policy

Scan이 있어도 CAD path를 scan surface에 무조건 투영하지 않는다. 이후 correction은 association, distance, normal consistency, confidence가 모두 만족될 때만 한다. 하나라도 아니면 `KEEP_REFERENCE_PATH` 또는 `REJECT_WAYPOINT`다. 이 조건의 수치는 여기서 정하지 않는다.

## Responsibility Boundary

| 책임 | 증명하는 것 |
| --- | --- |
| CAD Engine | CAD/BREP processing |
| Scan Engine | measured geometry processing |
| Footwear Demo | domain relevance |
| Registration | CAD/Scan correspondence |
| Process Path | geometry에서 로봇 독립 task representation |
| Robot Adapter | task representation에서 로봇 동작 |

책임을 서로 대신하게 하지 않는다.

## Industrial Use Case

대표 사례 이름은 `LASTED_UPPER_ROUGHING`이다. 현재 프로젝트가 실제 roughing 공정을 완성했다고 주장하지 않는다.

검증 범위는 Geometry, Surface, 6D ProcessPath, IK, collision, simulated RobotTrajectory다.

범위 밖은 physical robot execution, force control, spindle control, tool wear compensation, PLC, safety PLC, production cell validation이다.

## Architecture

```text
             Geometry Sources
                    │
       ┌────────────┼────────────┐
       │            │            │
    CAD/BREP    Footwear     RGB-D Scan
       │          Mesh            │
       ▼            ▼            ▼
 OpenCascade      Mesh        Point Cloud
       │         Geometry      Geometry
       └────────────┼────────────┘
                    ↓
             Geometry Contract
                    ↓
              6D ProcessPath
                    ↓
               Robot Adapter
                    ↓
              ROS2 / MoveIt2
                    ↓
              RobotTrajectory
```

## P-011 Entry Contract

이 문서가 있어도 P-011은 시작하지 않는다. 목적만 예약한다.

`P-011 = COMPLEX_SURFACE_CAD_TEST_COUPON`

나중에 검증할 항목은 reproducible CAD generation, valid STEP/BREP, 의미 있는 여러 face, compound curvature, surface query, normal query, curvature query, deterministic export다.

P-011에서 ProcessPath 생성, Gemini scan, registration, MoveIt은 하지 않는다. 치수와 face topology는 이 문서에서 정하지 않는다.

## Consequences

- CAD engine의 다음 입력은 공개 화형 STEP이 아니라, 아직 만들지 않은 Test Coupon이다.
- Footwear 주장은 Gemini 측정 또는 라이선스가 명확한 mesh가 담당한다.
- 이후 CAD↔Scan 실험 대상은 신발 CAD가 아니라 같은 Test Coupon의 출력물이다.
- P-010 거절 증거는 유지한다.

## Risks

- Coupon이 신발 표면과 다르므로, CAD engine 통과가 footwear 공정 통과로 읽힐 수 있다. 책임 경계가 그 읽기를 막는다.
- 공개 화형 BREP가 나중에 생겨도 이 결정을 자동으로 되돌리지 않는다. 별도 결정이 필요하다.
- Registration 정책의 원인 목록은 구분 대상이지, 현재 데이터에서 어느 원인이 지배적인지 정한 것이 아니다.

## Scope

P-010A의 산출은 이 결정과 P-010A 기록뿐이다.

## Non-Goals

- P-010 STEP 수리
- P-010 qualification 재실행
- P-011 Test Coupon 설계와 구현
- schema, 노드, launch
- Gemini 촬영, 출력, registration
- MoveIt `execute`와 로봇 동작
- roughing 공정 완료 주장
