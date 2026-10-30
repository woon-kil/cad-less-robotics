# P-008-R1H — Shoe/Support Feature Separability

Dataset v2의 `capture_01`만 사용했다. capture_02–05는 열지 않았다. production segmentation은 없다.

판정 규칙과 진단 사각형은 점수 계산 전에 고정했다. 사각형 좌표는 그대로다. 점 수와 bounds는 측정 뒤에 같은 파일에 적었다.

## 입력

- dataset: P-008B Dataset v2
- capture: capture_01
- cloud SHA256: `2891c0e16ff602d6e7dfac867e7bfd610b4f6b8683b1d23d7c06baae6b847a1c`
- fields: `x y z rgb`
- frame: `camera_color_optical_frame`
- prism: PCL 1.12.1, `scripts/p008r1e_prism.cpp` 파라미터 유지. viewpoint 계수로 높이를 읽었다.

## 진단 영역

둘 다 `DIAGNOSTIC_ONLY_MANUAL_REFERENCE`다. production ROI가 아니다.

- shoe_interior: 픽셀 (485,365)–(550,430). 갑피와 칼라 안쪽. 3598점. 지지면 높이 중앙값 0.109 m, p05 0.050 m.
- support_interior: 픽셀 (650,430)–(740,510). 검은 매트 안쪽, 신발과 떨어짐. 7187점. 높이 중앙값 0.020 m, p95 0.022 m.

매트는 RANSAC 평면 위 약 20 mm에 있다. 평면 자체(5 mm 이하)가 아니다.

## 내부 영역 separability

결정 숫자는 spatial-block balanced accuracy다. HIGH ≥ 0.90, MEDIUM ≥ 0.75, 유효 fold 3개 미만은 LOW. random split은 선택에 쓰지 않았다.

| set | block accuracy | grade | 대표 JS (log2) |
| --- | --- | --- | --- |
| RGB | 0.618 | LOW | R 0.511, G 0.517, B 0.420 |
| HSV | 0.492 | LOW | H 0.412, S 0.432, V 0.482 |
| Lab | 0.628 | LOW | L* 0.536, a* 0.005, b* 0.331 |
| Height | 0.995 | HIGH | 1.0 |
| HSV+Height | 0.956 | HIGH | |
| Lab+Height | 0.975 | HIGH | |
| RGB+Height | 0.985 | HIGH | |

a*는 거의 겹친다. 색만으로는 갑피·칼라와 검은 매트가 spatial block에서 0.90에 못 미친다. 높이는 내부 영역에서 구간이 갈린다. 신발 p05 0.050 m, 매트 p95 0.022 m.

## 접촉

고정한 띠는 신발 내부 점으로부터 평면 위 거리다.

- 신발 쪽: 높이 0.005–0.025 m, 수평 ≤ 0.015 m. 390점. 높이 중앙값 0.022 m.
- 매트 쪽: 높이 ≤ 0.005 m, 수평 0.015–0.035 m. 0점.

매트 표면이 약 0.020 m라서 5 mm 이하 띠에 점이 없다. 가까운 평면 점은 수평으로 0.16 m보다 멀다. 한쪽이 100점 미만이므로 `CONTACT_REGION_SEPARABILITY = NOT_IDENTIFIABLE`.

이 기준을 본 뒤 바꾸지 않았다. 내부 높이가 갈리더라도 접촉을 확인하지 못했으므로 Case A도 Case B도 아니다.

## 선택

```text
FEATURE_SEPARABILITY = NOT_ROBUSTLY_SEPARABLE
SELECTED_FEATURE_FAMILY = NONE
SELECTED_SEGMENTATION_STRATEGY = SEGMENTATION_STOP_RULE
STOP_RULE_TRIGGERED = YES
```

가장 큰 내부 점수는 Height 0.995다. 규칙이 받아들이는 feature set은 없다.

추가 CNN, threshold 반복, capture_02–05 확인은 하지 않는다. R1I는 열지 않는다.

## 시험

`scripts/test_p008r1h_features.py` 5 passed. RGB→HSV, RGB→Lab, hue wrap, 평면 높이, 선택 규칙.
