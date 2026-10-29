# P-007F-07C — M2 Marker Mount Geometry Redesign

판정: `DESIGN_READY`

이 판정은 장착 설계를 볼 수 있다는 뜻이다. 마커 검출 PASS가 아니다. 설치와 로봇 동작은 승인하지 않는다.

```text
30.8° image distortion != 30.8° physical mounting correction
MARKER_MOUNT_INSTALLATION_AUTHORIZED = NO
READY_FOR_MOTION_PREREGISTRATION = NO
MOTION_AUTHORIZED = NO
```

최종 설계 결정: `MOUNT_REDIRECTION_RECOMMENDED`

설치 여부를 사진으로 보려면 `figures/proposed_mount.png`를 본다. 초록은 지금 붙어 있는 마커, 주황은 제안 판이 같은 Gemini 화면에 보이는 모양이다. 경첩은 DaBai 쪽(화면 왼쪽)이고, 들림은 모니터 쪽(화면 오른쪽)이다.

## 1. Source evidence used

새 촬영은 하지 않았다. 로봇, Gemini, 마커는 움직이지 않았다.

- P-007F-07 marker setup capture, marker mount close-up (`marker_surface_review.md`): M2는 DaBai 바로 옆의 크림색 브래킷.
- P-007F-07A: `M2_DOWNSTREAM_OF_ID2 = YES`.
- P-007F-07B-R1 attempt-02: 검출 17프레임의 `marker_detection.csv`. 배경 사진은 `raw/representative_detected/000128-color.png`.
- 그려진 사각형은 reprojection이 가장 작은 프레임 12의 꼭짓점이다. attempt-02에서 검출 중심 흔들림은 1 px 미만이라 같은 카드 위에 겹친다.

## 2. Gemini RGB intrinsics source

`evidence/P-006/camera-target/color-camera-info.yaml`

- 1280×720, `camera_color_optical_frame`, `rational_polynomial`
- fx 610.5073852539062, fy 610.349853515625, cx 641.7918090820312, cy 364.4433288574219
- 같은 K가 P-007F-03, P-007F-05 프레임에도 있다.
- 초점 거리를 가정하지 않았다. `METRIC_PLANE_ORIENTATION`은 미해결로 두지 않았다.

## 3. Current marker mean four-corner geometry

검출 17프레임. 꼭짓점 순서는 TL, TR, BR, BL. 단위는 px. 중앙값이다.

| 꼭짓점 | x | y |
| --- | ---: | ---: |
| 0 TL | 740.74 | 127.74 |
| 1 TR | 774.21 | 105.27 |
| 2 BR | 811.00 | 123.00 |
| 3 BL | 777.20 | 149.79 |
| 중심 | 776.02 | 126.48 |

꼭짓점 각 중앙값: 64.9°, 120.8°, 64.1°, 110.4°.

## 4. Current projected side lengths

네 변 평균의 평균 41.65 px, 중앙값 41.69 px.

변별 평균: 40.60, 40.21, 43.28, 42.50 px.

대각선 평균: 70.57 px 와 44.03 px. 짧은 대각이 긴 대각의 약 0.62배다.

맞은편 변 비 최댓값 중앙값 1.076. 투영 면적 중앙값 1570 px².

## 5. Current perspective/skew characterization

직각에서 가장 먼 꼭짓점 각의 벗어남 중앙값 30.83°. 이것이 영상 왜곡이다.

```text
30.8° image distortion != 30.8° physical mounting correction
```

이 30.8°를 쐐기 각도로 쓰지 않았다. 변 길이 비(aspect 중앙값 1.088)보다 대각선 비와 꼭짓점 각이 기울기를 보여 준다.

## 6. Estimated marker normal/orientation

방법: 한 변 50 mm 정사각형, OpenCV `SOLVEPNP_IPPE_SQUARE`. 물체점 순서는 TL, TR, BR, BL이고, +Z는 인쇄면 바깥이다. 카메라에 가까운 해(t_z > 0) 중 reprojection이 작은 쪽을 골랐다.

법선 중앙값(카메라 좌표): `[-0.141, -0.535, -0.834]`

마커 원점에서 카메라로 향하는 시선과 법선 사이 각(incidence) 중앙값 53.37°. 0°이면 인쇄면이 그 시선을 정면으로 본다. 광축에 수직이라는 뜻이 아니고, 30.8°와도 다른 양이다.

법선을 카메라 -Z에 대해 나눈 값(설계 쐐기각이 아님): yaw 중앙값 -9.54°, pitch 중앙값 -32.67°.

거리 중앙값 0.681 m.

## 7. Orientation estimate stability

17프레임.

| 양 | 평균 | 표준편차 | 중앙값 |
| --- | ---: | ---: | ---: |
| incidence (°) | 53.64 | 1.84 | 53.37 |
| yaw (°) | -9.93 | 1.00 | -9.54 |
| pitch (°) | -33.02 | 1.92 | -32.67 |
| 거리 (m) | 0.680 | 0.005 | 0.681 |

한 프레임의 최댓값이 incidence 60.7°, 영상 각 벗어남 46.8°까지 올라간다. 쐐기각은 그 프레임이 아니라 중앙값으로 정했다.

## 8. Reprojection error

평균 0.93 px, 중앙값 0.76 px, 최대 2.36 px. 대표 프레임은 index 12.

## 9. Proposed correction direction

마커 원점을 중심으로, 인쇄면 법선을 시선 쪽으로 돌린다. 카메라에 가까워지는 꼭짓점은 index 2(BR, 화면 오른쪽, 모니터 쪽)이다. 경첩은 그 반대편, 화면 왼쪽, DaBai 쪽이다.

후보:

| 후보 | 보정 | 남은 incidence | 모서리 이동 | 변 평균 | 면적 | 직각 벗어남 | 대각선 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A 현재 | 0° | 52.4° | 0 mm | 41.5 px | 1558 | 26.5° | 69.9 / 44.6 |
| B 절반 | 26.7° | 25.7° | 16.3 mm | 48.0 px | 2299 | 4.4° | 70.2 / 65.6 |
| C 시선 정면 | 53.4° | 1.0° | 31.6 mm | 50.6 px | 2551 | 3.1° | 70.3 / 72.8 |

A의 각·면적은 대표 프레임 재투영이다. 검출 사각형의 직각 벗어남 중앙값은 30.8°이다.

고른 안은 B다. C는 모서리 이동이 20 mm를 넘는다.

## 10. Proposed correction angle(s) + evidence level

쐐기각 26.7°. evidence level `ESTIMATED`.

근거는 17프레임 incidence 중앙값의 절반이다. 자로 잰 물리 각이 아니다. 영상 각 30.8°도 아니다.

모서리 이동 16.3 mm도 `ESTIMATED`.

## 11. Proposed plate dimensions

- 최소 폭·높이 56 × 56 mm. 검정 사각형 50 mm에 quiet zone 가이드 3 mm를 양쪽에 더한 값이다. 50 mm는 사용자가 실측한 값이고, 3 mm는 출력 시트의 가이드다.
- 설치 권장도 56 × 56 mm. 크기 evidence level `APPROXIMATE`.
- 70 × 70 mm는 테이프를 흰 여백 밖에 두기 위한 여유안이다. DaBai가 카드 왼쪽에 붙어 있으므로 이번 권장에서는 빼 두었다.

## 12. Proposed attachment location/method

지금 M2 카드가 붙어 있는 크림색 브래킷 면. 같은 자리에 얇은 쐐기를 두고, 그 위에 마커를 둔다.

경첩(얇은 쪽)은 DaBai 쪽. 두꺼운 쪽은 모니터 쪽이며, 그 모서리가 카메라 쪽으로 약 16 mm 나온다.

고정은 기존 카드 면에 붙였다 뗄 수 있는 테이프 또는 클립. 서보에 나사를 박지 않고, 축에 접착하지 않는다. 이번 Gate에서 만들거나 붙이지 않는다.

## 13. ID2 downstream witness validity

P-007F-07A를 유지한다. M2는 ID2 downstream이다. 판이 M2에 강체로 고정되면 마커도 ID2 downstream의 외부 시각 witness로 쓸 수 있다.

다음은 여전히 주장하지 않는다.

```text
marker angle == exact ID2 shaft angle
marker displacement == exact joint rotation
controller readback == physical angle
```

용도는 물리 동작이 있었는지 없었는지를 밖에서 보는 것이다.

## 14. Mechanical interference review

`POTENTIAL_INTERFERENCE`

- 들림 방향 자체는 DaBai 반대편(화면 오른쪽, 모니터·Gemini 쪽)이다.
- DaBai는 카드 바로 왼쪽에 있다. 지금 카드보다 넓은 판은 그쪽으로 겹칠 수 있다. 그래서 권장 크기를 56 mm로 두었다.
- 케이블은 브래킷 아래쪽에 있고, 이번 정지 화면에서 코드 위를 지나지 않는다.
- 그리퍼와 아래쪽 링크는 카드 면의 16 mm 들림과 바로 겹쳐 보이지 않는다.
- ID2 ±3° 구간은 움직이지 않고 확인하지 않았다.
- CAD로 workcell collision-free를 증명하지 않았다.

## 15. Predicted projected-geometry improvement

후보 B, 대표 pose 기준. 검출률은 예측하지 않는다.

- 변 평균 41.5 px → 48.0 px
- 면적 1558 → 2299 px² (약 1.48배)
- 짧은 대각 44.6 px → 65.6 px. 긴 대각은 70 px 근처에서 거의 그대로다.
- 재투영 사각형의 직각 벗어남 26.5° → 4.4°
- 맞은편 변 비 1.028 → 1.015
- 가장 긴 변 / 가장 짧은 변은 1.028 → 1.071로, 이 비는 줄지 않는다. 개선으로 읽는 양은 대각선과 꼭짓점 각이다.

95% 검출은 설치 뒤 static qualification에서 다시 잰다.

## 16. Final design decision

`MOUNT_REDIRECTION_RECOMMENDED`

- 방향: 법선을 시선 쪽으로. 화면에서는 왼쪽 경첩, 오른쪽 들림.
- 각: 26.7°, `ESTIMATED`
- 판: 56 × 56 mm, `APPROXIMATE`
- 고정: M2 카드 면, 탈착 테이프 또는 클립
- 기대: 투영이 더 크고, 대각선이 더 비슷해진다. 검출률은 모른다.
- 주의: DaBai 쪽 돌출, ±3° 미확인, 충돌 없음은 증명되지 않음

수용:

```text
AC01 existing M2 geometry characterized
AC02 30.8° image distortion not misused as physical angle
AC03 camera intrinsics provenance established
AC04 marker plane orientation estimated
AC05 proposed correction supported by geometry
AC06 plate remains rigidly attached to ID2 downstream structure
AC07 POTENTIAL_INTERFERENCE explicitly identified
AC08 proposed mount visual produced
AC09 no robot/camera/marker physical change
AC10 no commands/writes
```

## 17. Evidence path / Commit SHA / git status

`evidence/P-007F-07/marker_mount_geometry_redesign/`

- `report.md`
- `geometry_analysis.json`
- `mount_design.md`
- `pose_estimates.csv`
- `figures/current_mount.png`
- `figures/proposed_mount.png`
- `figures/marker_projective_geometry.png`

커밋 해시는 푸시 후 이 파일 밖의 완료 보고에 적는다. 기존 attempt-02 evidence는 수정하지 않았다.

## 18. Robot command / Servo write / Torque command counts

```text
robot_command_count = 0
servo_write_count = 0
torque_command_count = 0
```

```text
MARKER_MOUNT_INSTALLATION_AUTHORIZED = NO
READY_FOR_MOTION_PREREGISTRATION = NO
MOTION_AUTHORIZED = NO
```
