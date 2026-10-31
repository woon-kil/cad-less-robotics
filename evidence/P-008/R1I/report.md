# P-008-R1I — 지지면 단순화 뒤 접촉 관측

Dataset v2와 R1H evidence는 바꾸지 않았다. segmentation은 없다. 로봇은 움직이지 않았다.

신발은 검은 매트 위가 아니라 나무 지지면 위에 있다. 떼어 낸 원판은 전경에 보인다. 카메라는 Gemini 335L, serial `CP263630009D`, firmware 1.4.60, OrbbecSDK_ROS2 2.10.6이다. frame은 `camera_color_optical_frame`이다. 점군 필드는 `x y z rgb`이고 유한 점은 756376개다.

RGB↔depth는 Dataset v2와 같은 내부 검사를 통과했다. depth↔cloud p95는 6.2e-8 m, color 채널 중앙 차이는 0이다. 물리 등록 정확도는 `NOT_CLAIMED`다.

지지 평면은 카메라 쪽이 양수다.

```text
n = (0.004095, -0.687865, -0.725827)
d = 0.527413
inlier = 148529 / 276452
residual median = 0.000723 m
residual p95 = 0.002427 m
```

접촉 띠는 점 수를 보기 전에 고정했다. 지지면은 `|h| ≤ 0.004 m`, 신발 쪽은 `0.004 < h ≤ 0.020 m`, 수평 15 mm 이내다. R1H의 5 mm 띠는 쓰지 않았다. 진단 창은 픽셀 u 420–660, v 340–540이다.

신발 쪽 1178점, 가장 큰 10 mm 성분 1088점, 높이 중앙값 0.011 m. 지지면 쪽 5223점, 가장 큰 성분 3967점, 높이 중앙값 약 0. 확대 그림에서 주황 점은 창 가장자리가 아니라 밑창 둘레의 나무면이다.

```text
P-008-R1I = PASS_CONTACT_OBSERVABILITY
R1H_BLOCKER_REMOVED = YES
```

다음 Gate는 열지 않는다.
