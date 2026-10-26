# P-006 Gemini 335L ↔ DOFBOT base extrinsic

상태: **BLOCKED**. `T_base_camera`는 없다. P-007은 열지 않았다.

P-005의 `TEST_ONLY_SYNTHETIC_PLACEMENT`는 그대로 둔다. 이번 후보 이름은 `MEASURED_EXTRINSIC_CALIBRATION`이고, 측정값이 생기기 전에는 값이 없다.

## 이번에 읽은 것

p620 USB에 Orbbec Gemini 335L (`2bc5:0804`)이 있었다. 펌웨어 1.4.60, USB3.2, 협상 속도 5000M, 포트 2-1, 시리얼은 적지 않았다. 색 영상과 점군 스트림은 열지 않았다.

같은 호스트에 DOFBOT 시리얼 장치는 없었다. base 설치와 카메라 설치는 보지 못했다. 타깃의 칸 수와 치수는 측정하지 않았고, 가정한 숫자도 없다.

## 하지 않은 것

sample 0. fit 0. validation 0. solve 0. static TF 0. identity TF 0. synthetic placement 사용 0. P-004 점의 base 변환 0. 로봇 trajectory 0. controller send 0.

acceptance threshold는 고정하지 않았다. 보드와 base 기준이 없는 상태에서 숫자를 정하면 측정이 아니라 가정이 된다.

## 재개

실제 보드 치수와, target pose를 `base_link`에서 아는 측정 절차가 생긴 뒤에만 sample 수와 threshold를 고정한다. 그 전에 촬영하거나 TF를 만들지 않는다.
