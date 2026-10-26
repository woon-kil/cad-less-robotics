# Workcell constraint

분류: `VALIDATION_GEOMETRY_CONFLICTS_WITH_PHYSICAL_WORKCELL`

센서 정확도 실패로 분류하지 않는다. Attempt 1–5에서 검출된 프레임의 재투영, 깊이 평면, 간격은 잠긴 센서 threshold 안에 있었다. 막힌 항목은 optical Z ±40 mm를 현재 설치에서 만드는 일이었다.

## Attempt 6에서 가져온 예측

작업면 법선 `[-0.053887, 0.284731, 0.957092]`. 광축과의 각 16.845°.

작업면 위 화면 아래 100 mm는 optical Z 약 −28.4 mm, 화면 위 100 mm는 약 +28.4 mm다. ±40 mm에는 그 방향으로 약 140.7 mm가 필요하다. 이 거리는 예측이고 measured PASS가 아니다. 정밀 값은 `evidence/P-006/camera-target/attempt-06-diagnosis/`에 있다.

## 설치와의 충돌

far를 만들려면 체크보드를 화면 위, 카메라 몸체 쪽으로 약 140.7 mm 옮겨야 한다. 그 방향에는 Gemini 받침과 DOFBOT이 놓이는 설치 영역이 있다. Attempt 5는 그 방향에서 `FAR_PHYSICAL_WORKSPACE_LIMIT`를 기록했다. 해당 프레임에는 checkerboard 포즈가 없다.

유효 포즈가 실제로 간 화면 위 이동은 약 20.8 mm이고, 그때 ΔZ는 +10.7 mm다. 받침까지 남은 작업면 길이는 측정하지 않았다.

참조 영상은 촬영 프레임이다. 작업면 측량 사진이 아니다.

- `evidence/P-006/camera-target/attempt-05/frames/far-t01-00-color.png`
- `evidence/P-006/camera-target/attempt-05/frames/near-t01-00-color.png`

용지가 이미지 안에 있고, far 프레임에서는 용지와 받침 사이에 면이 보인다는 정성 기록만 유지한다. 픽셀 간격을 밀리미터로 바꾸지 않는다.

near의 최선 ΔZ −38.2 mm는 화면 아래 150 mm와 화면 오른쪽 94 mm가 함께 들어간 결과다. 그 사실도 센서를 불량으로 만들지 않는다. ±40 mm 규칙이 이 작업면의 설치 기하와 맞지 않는다는 기록이다.
