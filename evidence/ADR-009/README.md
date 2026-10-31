# ADR-009 — Workboard와 Workpiece Instance

상태: `ACCEPTED / CLOSED`  
날짜: 2026-10-31

카메라에 보이는 장면과, 이번 공정 대상인 하나의 작업물을 다른 책임으로 둔다. 구현, fiducial 설치, registration 실행, P-009-R1은 이 결정에 포함되지 않는다.

```text
camera_color_optical_frame
        │  registration
        ▼
    workboard
        ├── PlacementEnvelope
        └── WorkpieceInstance
                ▼
          MeasuredSurface
                + ProcessSpecification
                ▼
           ProcessDomain
                ▼
         P-009 ProcessPath
```

## 1. Workboard frame

논리 이름은 `workboard`다. 물리 크기는 X 801 mm, Y 400 mm다. 오른손 좌표계를 쓴다.

원점은 작업판에서 다시 찾을 수 있는 기준점이다. +X는 801 mm 길이 방향, +Y는 400 mm 폭 방향, +Z는 작업판 표면에서 위쪽 free-space다.

원점의 실제 위치와 +X/+Y의 물리 방향은 개념까지만 고정한다. fiducial로 원점을 실현하는 일은 다음 Gate다. `workboard`는 robot base가 아니다.

## 2. Frame 분리

`camera_color_optical_frame`, `workboard`, robot base는 서로 다른 프레임이다.

`camera↔workboard`는 vision registration이다. `workboard↔robot`은 robot adapter와 calibration이다.

P-009는 `workboard`까지만 쓸 수 있다. robot base, IK, reachability, collision으로 process domain을 줄이거나 정의하지 않는다.

## 3. Fiducial

ArUco fiducial은 camera observation에서 `workboard`를 찾고 `camera↔workboard` rigid transform을 추정·검증하는 물리 기준이다.

신발 segmentation, 신발 경계, 공정 경계, robot calibration, reachability 정의가 아니다.

W-002A는 `PASS_FIDUCIAL_OBSERVABILITY / CLOSED`다. 6개 이산 위치에서 검출, 전체 가시, 중심 depth가 확인되었다. 그 직사각형 전체의 연속 가시성으로 확대하지 않는다.

## 4. PlacementEnvelope

작업자가 작업물을 놓도록 허용하는 `workboard` 위의 물리 후보 영역이다.

`CameraObservableRegion`, `RobotReachableRegion`, `WorkpieceInstance`, `ProcessDomain`과 다르다.

작업판은 X 0..801 mm, Y 0..400 mm다. 카메라 마운트가 좌측 일부를, 로봇이 우측 일부를 점유한다. 예전의 대략적인 X 100..500 mm 구간은 ProcessSurfaceScope로 얼리지 않는다. 최종 수치는 별도 physical-layout evidence가 필요하다.

## 5. WorkpieceInstance

이번 ProcessPath의 대상으로 지정된 하나의 물리 작업물이다.

이 정의는 automatic shoe segmentation, 전체 신발 경계, 전체 위상, 실제 접착선 검출의 성공을 뜻하지 않는다. 그 claim은 계속 금지한다.

지정 방식은 고르지 않았다. 비교만 둔다.

| 방식 | 내용 |
| --- | --- |
| A | operator-defined ROI |
| B | fiducial-assisted placement zone |
| C | geometry-assisted connected component |
| D | 위 방식의 제한적 조합 |

`WORKPIECE_INSTANCE_MECHANISM_SELECTED = NO`

## 6. CameraObservableRegion

현재 고정 Gemini 335L에서 RGB와 depth로 실제로 보이는 `workboard` 영역이다. W-002A는 이산 증거만 제공한다.

`CameraObservableRegion`과 `PlacementEnvelope`는 같지 않다. 둘의 교집합이 나중에 측정 가능한 배치 조건이 될 수 있다.

## 7. MeasuredSurface

`MeasuredSurface`는 `WorkpieceInstance`에 속한다. 장면 전체의 above-plane geometry를 측정 표면으로 쓰지 않는다.

P-009의 `support_plane_pca`가 올라온 장면 전체를 process domain으로 쓴 문제를 이 경계에서 막는다. 이미 기록된 판정 `FAIL_SURFACE_ASSOCIATION`은 바꾸지 않는다.

```text
SceneCloud
  → workboard registration
  → WorkpieceInstance scope
  → MeasuredSurface
```

측정 표면에는 `frame_id = workboard`, 촬영 identity, `workpiece_instance_id`, 표면 점, 유효한 법선, confidence와 validity가 있어야 한다. 이 필드 목록은 계약이다. 이번 ADR은 그 데이터를 만들지 않는다.

## 8. ProcessDomain

`ProcessSpecification`이 요구하는 작업 영역과, `WorkpieceInstance`에 속한 `MeasuredSurface`의 유효 영역의 교집합이다.

작업판의 무관한 면, 카메라 마운트, 로봇 형상, 배경, 다른 물체는 넣지 않는다. `RobotReachableRegion`으로 ProcessDomain을 정의하지 않는다.

## 9. P-009와 P-010

P-009는 ProcessSpecification과 작업물로 한정된 MeasuredSurface에서 로봇에 독립인 6D ProcessPath를 만든다.

P-010은 그 경로와 로봇 모델, 로봇 프레임에서 RobotTrajectory의 성립을 본다. IK, reachability, collision, joint limit, trajectory feasibility는 P-010이다. P-009로 되돌리지 않는다.

## 10. 결정 질문

| 질문 | 답 |
| --- | --- |
| Q1. `workboard`는 로봇과 독립인가 | YES |
| Q2. 카메라 관측 영역과 로봇 도달 영역은 같은가 | NO |
| Q3. PlacementEnvelope와 ProcessDomain은 같은가 | NO |
| Q4. WorkpieceInstance 없이 above-plane 장면 전체를 P-009에 넘길 수 있는가 | NO |
| Q5. WorkpieceInstance가 automatic segmentation 성공을 뜻하는가 | NO |
| Q6. P-009가 reachability로 경로를 만드는가 | NO |
| Q7. P-010이 로봇 feasibility를 판단하는가 | YES |

## 11. 기존 결정과의 관계

ADR-008은 공정 정의와 측정 표면 6D ProcessPath를 나누고, 자동 경계·위상·접착선을 `UNSOLVED`로 둔다. ADR-009는 그 미해결을 유지한 채, 측정 표면의 범위를 하나의 WorkpieceInstance로 제한한다. 충돌은 없다.

동결된 P-009 계약은 ProcessPath와 MeasuredSurface의 `frame_id`가 같고, robot base·tool·MoveIt 프레임으로 바꾸지 않는다. 개발 당시 프레임은 `camera_color_optical_frame`이다. ADR-009는 그 JSON을 고치지 않는다. `workboard`는 registration 이후의 측정 표면 프레임으로 추가되는 이름이고, 이미 끝난 P-009 평가의 프레임을 소급하지 않는다. 동결 파일과의 충돌은 없다.

검토자는 ROS의 오른손 좌표, REP-103 계열, tf2의 rigid transform과 static transform을 근거로 제시했다. 첨부 URL은 [index.ros.org myahrs_driver](https://index.ros.org/p/myahrs_driver/)이다. 이 ADR은 그 페이지를 다시 열지 않았고, 그 페이지를 REP-103 원문으로 쓰지 않는다.

## 12. 이번 합격이 아닌 것

fiducial layout 동결, 마커 설치, workboard registration 완료, WorkpieceInstance 방식 선택, P-009-R1, P-010은 열리지 않았다.

## 아직 정하지 않은 것

- 원점의 물리 위치와 +X/+Y의 실현
- fiducial ID와 좌표의 대응
- PlacementEnvelope의 수치
- CameraObservableRegion의 경계
- WorkpieceInstance를 A/B/C/D 중 무엇으로 지정할지
