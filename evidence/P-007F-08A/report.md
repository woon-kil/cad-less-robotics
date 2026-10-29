# P-007F-08A — Motion Observability Qualification

판정: `OBSERVABILITY_UNRESOLVED`

```text
EXTRINSIC_NOT_VERIFIED
READY_FOR_PHASE_B_REVIEW = NO
PHASE_B_EXECUTED = NO
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```

질문은 하나다. 지금 Gemini Baseline B와 M2 타깃에서, ID2의 작은 회전이 이미 고정된 `T_motion = 1.0 px`보다 큰 영상 변위로 보일 것임을 기존 evidence만으로 미리 보일 수 있는가.

보이지 않는다. 카메라는 움직이지 않았고, 로봇도 움직이지 않았다. 임계값은 바꾸지 않았다. 그림은 만들지 않았다.

## 1. Gemini intrinsics source

`evidence/P-006/camera-target/color-camera-info.yaml`

fx 610.5073852539062, fy 610.349853515625, cx 641.7918090820312, cy 364.4433288574219. 1280×720. `camera_color_optical_frame`.

## 2. Robot/FK source

`scripts/p007e_r2r4_fk.py`. 축과 링크 포즈는 로봇 base frame이다.

## 3. ID2 joint axis/parent/child

`Arm2_Joint`. parent `Arm1_Link`. child `Arm2_Link`. revolute. axis_local `(1, 0, 0)`.

근거: `evidence/P-007F-07/m2_kinematic_verification/membership_report.json`.

## 4. M2 downstream evidence

`M2_DOWNSTREAM_OF_ID2 = YES`. 브래킷이 Arm2_Link 끝인지 Arm3_Link 시작인지는 같은 문서가 확정하지 않았다. 둘 다 downstream이다.

## 5. Current M2 image position

attempt-02 검출 17프레임 중심 중앙값 (776.02, 126.48) px. P-007F-08은 그 카드가 들어 있는 ROI를 역사적 프레임과 0.163 px 차이로 다시 보았다. 새 중심 좌표를 만들지는 않았다.

## 6. Current M2 depth availability

마커 원점의 색 카메라 거리 중앙값 0.681 m는 P-007F-07C solvePnP에 있다. 이것은 마커의 위치다. ID2 축의 위치가 아니다.

RGB-D를 그 카드에 픽셀 단위로 대응시킨 측정은 없다. P-007F-06은 depth 정합을 `DEPTH_SPATIAL_LOCALIZATION_LIMITED`로 두었다. P-007F-04의 678 mm는 다른 ROI의 근사다.

## 7. Robot↔Gemini extrinsic status

`EXTRINSIC_NOT_VERIFIED`

`T_Gemini<-RobotBase`는 evidence에 없다. P-006은 BLOCKED이고 `T_base_camera`는 없다. depth와 color 사이의 23.80 mm는 카메라 내부 간격이다. Baseline B는 배치가 그대로라는 뜻이다. solvePnP는 마커와 카메라의 관계다. 어느 것도 로봇 base로 바꾸지 못한다.

추정해서 채우지 않았다.

## 8. 사용한 established method

pinhole, 보정된 intrinsics, 기존 FK, rigid-body transform, image Jacobian을 요건으로 대조했다.

투영과 Jacobian은 카메라 프레임의 3D 변위가 있어야 한다. FK는 base frame 변위를 준다. 둘을 잇는 rigid transform이 없다. finite difference도 같은 입력이 필요하다. 그래서 ±1°, ±2°, ±3° 투영은 실행하지 않았다.

## 9. 계산 가능한 자유도

로봇 base 안에서는 기존 FK가 URDF 점의 이동을 계산할 수 있다. 그 결과는 이미 P-007F-04에 있고, 이번 판정의 픽셀 값이 아니다.

카메라 화면의 `(u, v)`로 넘어가는 자유도는 계산하지 못한다. 카메라의 위치와 방향, 축에서 마커까지의 거리, 브래킷이 어느 링크인지가 비어 있다.

## 10–12. ±1°, ±2°, ±3° predicted displacement

여섯 후보 모두 `NOT_COMPUTED`. `observability_results.csv`.

임의의 카메라 자세를 넣어 숫자를 만들지 않았다.

## 13. `T_motion = 1.0 px` 비교

비교할 예측 픽셀이 없다. `T_motion`은 1.0 px 그대로다.

하한을 모든 미지 카메라 방향에 대해 잡으면 0 px다. 회전이 마커를 시선 방향으로만 움직이면 그 점의 1차 영상 변위는 0이다. 지금 evidence는 그 방향을 배제하지 못한다. 0 px 하한은 1.0 px를 넘지 못하므로 `QUALIFIED`가 될 수 없고, 이 배치의 예측도 아니므로 `INSUFFICIENT`도 아니다.

P-007F-04의 0.355–2.464 px(1°) 범위는 임의 계수를 쓴 과거 근사다. 1.0 px 양쪽을 모두 포함하고, M2가 아니다. 채택하지 않았다.

## 14. 주요 uncertainty

- 카메라와 로봇 base 사이의 회전과 이동
- 축에서 마커까지의 미터 거리
- 브래킷의 링크
- controller readback 152는 물리 각도가 아니다

이 중 카메라 방향만 바뀌어도 예측이 1.0 px 아래와 위로 갈 수 있다.

## 15. 최종 verdict

`OBSERVABILITY_UNRESOLVED`

추가 calibration, 마커, 카메라 실험은 시작하지 않는다.

## 16. READY_FOR_PHASE_B_REVIEW

`NO`

## 17. Robot command count

0

## 18. Servo write count

0

## 19. Evidence path

`evidence/P-007F-08A/`

`report.md`, `source_evidence.md`, `geometry_assumptions.json`, `observability_results.csv`, `verdict.json`. `projection_sensitivity.png`는 없다.

## 20. Commit SHA / git status

푸시 뒤 완료 보고에 적는다.
