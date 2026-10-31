# W1 정지 검출

Gemini 335L, serial `CP263630009D`. color frame은 `camera_color_optical_frame`이다. 깊이 이미지도 같은 frame_id로 발행됐다. 이것은 SW D2C 설정이다. native depth 프레임의 포즈로 쓰지 않았다.

Arm1 라이브러리 각도는 촬영 전후 모두 86°이다. 여섯 축은 `[86, 152, 0, 18, 93, 178]`이다. 서보 쓰기는 0회다.

8장의 color에서 OpenCV `findChessboardCorners` (6×9, 9×6)는 16번 모두 실패했다. 화면 오른쪽 탁자에 체커보드가 보이지만 Arm1에 고정된 표적이 아니다. 팔 위에는 같이 돌 witness가 없다.

첫 프레임은 이후 프레임보다 어둡다. 픽셀 평균 차이는 노출 변화이고, 로봇 명령은 없었다.

```text
W1 = BLOCKED_WITNESS_DETECTION
```

`+5°`는 보내지 않았다.
