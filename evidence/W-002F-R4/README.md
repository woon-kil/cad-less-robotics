# W-002F-R4 — Gemini에서 볼 reference 형상 선택

물리 보드를 만들지 않았다. 카메라도 다시 찍지 않았다. 후보 A/B/C는 OpenCV 4.5.4 `CharucoBoard_create`와 `DICT_5X5_100`으로 만들었고, 칸 수·마커 길이·마커 개수·코너 개수는 그 board 객체에서 읽었다.

세 후보의 생성 PNG는 보드 ID와 ChArUco 코너를 모두 검출했다. A는 마커 17개·코너 24개, B는 12개·15개, C는 10개·12개다.

화소 추정은 기준 프레임의 15 mm 마커 중앙값 23.486 px만 쓴다. 1 mm당 1.566 px다. 카메라 내부 파라미터도 검출 합격선도 아니다. 그 비율로 A는 23.486 px, B는 39.143 px, C는 46.972 px다.

그림의 사각형은 그 비율로 그린 설계 도형이다. 관측된 보드 사변형에 투영한 것이 아니다. `SIMULATED_NOT_OBSERVED = YES`. 210×150 mm 패턴이 R3 화면에 다 들어왔다는 기록과 비교하면, 세 후보의 패턴 크기는 그 근처다.

400×300 backing은 조건으로 쓰지 않았다. 제안 backing은 패턴 사방 10 mm이며 최종 치수가 아니다. C는 220×180 mm다.

C를 미리 고르지 않았다. 비교 뒤에 C를 골랐다. 샘플링 기회가 가장 크고, 패턴 면적은 기존 210×150 mm와 비슷하다. 코너는 24개에서 12개로 줄어든다. 12개는 이 5×4 보드의 전체 코너다.

```text
SELECTED = C
5 x 4
square = 40 mm
marker = 30 mm
pattern = 200 x 160 mm
markers = 10
corners = 12
ESTIMATED_MARKER_EDGE_P50_PX = 46.972
PROPOSED_BACKING = 220 x 180 mm
SIMULATED_NOT_OBSERVED = YES
PRINT_EXECUTED = NO
NEXT_GATE_OPENED = NO
```
