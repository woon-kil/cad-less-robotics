# W-002F-R3-D2 — 촬영된 마커의 상 형성

저장된 R3 컬러 프레임 한 장만 보았다. 새 촬영, 검출 파라미터 변경, solvePnP는 없다.

정본 생성 PNG 해시는 `dd37b453f71ccb86be32d506fad85bb1673c4240b3004889da6aa056f35d29d6`다. `a67dc728…`는 pHYs 청크를 넣기 전의 생성물이며 덮어쓰지 않았다.

## 측정

거부된 후보 14개의 네 변 평균은 11.6–35.5 px다. 중앙값은 23.5 px, 전체 평균은 23.9 px다. 테두리까지 7칸으로 나누면 평균 변 길이에서 칸당 약 3.4 px다. 이 숫자를 합격 기준으로 쓰지 않았다.

perspective warp는 진단용이다. 원본 RGB는 바꾸지 않았다. 칸 밝기는 그 정규화 영상에서 읽었다. 임계 127로 만든 비트는 OpenCV `identify()`가 아니다. 수락된 ID가 없어 생성 PNG의 특정 마커와 대응하지 않았다. 가장 가까운 사전 ID와의 Hamming은 대응 오류가 아니다.

11개 후보의 payload 비트는 전부 0이다. 그 5×5 칸 평균의 최댓값과 최솟값 차이는 대부분 3–13 단계다. 흑백 칸이 갈라지지 않는다. 흑백이 둘 다 있는 후보는 변 길이 약 12 px인 두 개뿐이라, 전체 백색 칸 평균은 큰 후보를 대표하지 않는다.

## 상관

가까운 사전 코드까지의 Hamming은 14개 중 12개가 7이다. 크기, 대비, 선명도, 화면 위치와의 관계는 이 값으로는 식별되지 않는다. 인과로 읽지 않는다.

## 판정

상 수준에서는 성긴 샘플링과 칸 대비 붕괴가 함께 측정된다. 인쇄, 광학 흐림, 샘플링 중 무엇이 그 붕괴를 만들었는지는 갈라지지 않는다.

```text
CLASSIFICATION = MULTIPLE_IMAGE_FORMATION_FACTORS_SUPPORTED
ROOT_CAUSE = UNRESOLVED
PRINT_CAUSE = NOT_ESTABLISHED
IMAGING_CAUSE = NOT_ESTABLISHED
SAMPLING_CAUSE = NOT_ESTABLISHED
REPRINT_REQUIRED = NO
CAMERA_CHANGE_REQUIRED = NO
REGISTRATION_EXECUTED = NO
NEXT_GATE_OPENED = NO
```
