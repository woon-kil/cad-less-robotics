# P-008-R1J — 매트 제거 뒤 feature 분리

입력은 R1I 개발 캡처뿐이다. SHA256은 `68b7b0a08164aeda291349be46dfbb3a6eacd66cd801d04d8b1a9a1025482377`이다. R1H·R1I 파일은 바꾸지 않았다.

점수는 R1H와 같다. spatial-block balanced accuracy, HIGH ≥ 0.90, MEDIUM ≥ 0.75, 유효 fold 3개 미만은 LOW. R1H 픽셀 사각형은 신발이 옮겨져서 쓸 수 없다. 같은 역할의 진단 사각형을 점수 전에 지정했다. 신발 내부는 u 598–632, v 405–428이다. 나무면 내부는 u 280–430, v 530–600이고, 접촉 15 mm 안에 들어간 점은 0이다. 숫자 비교는 같은 지표이지 같은 픽셀의 짝 비교가 아니다.

평면은 R1I와 다시 맞았다. 계수 차이는 1e-6보다 작다. 카메라 쪽이 양수다.

## 내부

신발 566점, 나무 10630점. 라벨은 픽셀 상자다. 높이와 독립이다.

| set | block accuracy | grade |
| --- | ---: | --- |
| RGB | 0.998 | HIGH |
| HSV | 0.999 | HIGH |
| Lab | 1.000 | HIGH |
| Height | 0.955 | HIGH |

접촉에서 먼 갑피·칼라와 노란 나무면은 색으로 갈린다. R1H 내부 색상은 LOW였다.

## 접촉

R1I 띠를 그대로 썼다. 신발 1178점, 지지면 5223점. 이 라벨은 높이로 만들어졌다.

색만의 접촉 점수는 독립이다. Lab 0.823, RGB 0.821, HSV 0.818. 모두 MEDIUM. Lab의 한 fold는 0.673이다. 포화도 중앙값은 신발 0.50, 나무 0.80이라 hue를 버리지 않았다. 그래도 HIGH가 아니다.

높이만의 접촉 점수 0.978은 HIGH다. 라벨이 `|h| ≤ 0.004`와 `0.004 < h ≤ 0.020`이라 이 숫자는 `CIRCULAR`이다. 색+높이 0.97도 높이 라벨을 다시 넣은 것이라 `PARTIAL`이다. 이 점수로 분리를 채택하지 않는다.

Lab 오분류는 532점이고, 가장 큰 10 mm 성분은 343점이다. 기준 100점을 넘는다. 그림의 빨간 점은 밑창과 나무면이 만나는 둘레다.

```text
FEATURE_SEPARABILITY = NOT_ROBUSTLY_SEPARABLE
SELECTED_FEATURE_FAMILY = NONE
HEIGHT_LABEL_LEAKAGE = YES
DOWNSTREAM_GEOMETRY_RISK = YES
```

다음 Gate는 열지 않는다.
