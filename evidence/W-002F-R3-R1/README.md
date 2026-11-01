# W-002F-R3-R1 — 마커 샘플링 복구 비교

400×300 reference board만 카메라 쪽으로 옮긴 뒤, 컬러 10장을 읽었다. 카메라, 로봇, 작업판은 움직이지 않았다. 보드는 이 기록 뒤에 원래 자리로 되돌리지 않았다.

ChArUco 사전은 `DICT_5X5_100`이다. 화면에 다시 보인 작업판 ArUco는 `DICT_4X4_50`이며, 검출과 등록에 쓰지 않았다. 그 종이는 ChArUco 패턴을 가리지 않는다. ChArUco 시트는 화면 안에 다 들어온다.

검출기는 R3과 같다. `CORNER_REFINE_NONE`만 지정하고 다른 파라미터는 기본값이다. solvePnP와 registration은 없다.

## 비교

기준 변 길이 중앙값은 23.486 px다. 이번 10장에서 같은 방식으로 계산한 프레임별 중앙값의 중앙값은 25.322 px다. 프레임 0에서 정사각형에 가까운 가장 큰 후보는 38.8 px이고, 기준 최대는 35.5 px다. 변 길이 최대 50.7 px는 가로세로 비가 0.59라 ChArUco 마커 사각형이 아니다.

10장 모두 수락된 `DICT_5X5_100` ID는 0개다. ChArUco 코너는 0개다. 거부 후보는 프레임마다 17–21개다.

가까워진 배치가 마커 ID 복구로 이어지지는 않았다. 화소 크기 증가도 작아서, 샘플링만으로 충분한지는 이번 이동으로 확인되지 않았다. 인쇄와 광학은 여전히 확정하지 않는다.

```text
MARKER_DECODING_RECOVERED = NO
SAMPLING_OR_DISTANCE_EFFECT_SUPPORTED = NO
SAMPLING_INCREASE_ALONE_SUFFICIENT = NOT_DEMONSTRATED
ROOT_CAUSE = UNRESOLVED
REPRINT_REQUIRED = NO
REGISTRATION_EXECUTED = NO
NEXT_GATE_OPENED = NO
```
