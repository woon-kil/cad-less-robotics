# P-008-R1H 방법

분류: `OFFICIAL_IMPLEMENTATION_VERIFIED` (prism), `LITERATURE_VERIFIED` (색 공간과 JS)

공간 제한은 P-008-R1E와 같은 PCL 1.12.1 `ExtractPolygonalPrismData`다. 파라미터는 `scripts/p008r1e_prism.cpp`의 voxel 0.002 m, SOR meanK 30, stddev 1, RANSAC 0.004 m, 높이 0–0.18 m, viewpoint (0,0,0)이다. 색이 있는 원본 점을 이 평면과 hull 안에 다시 넣었다. voxel 중심에는 RGB가 없다.

지지면 높이는 prism이 viewpoint 쪽으로 뒤집은 계수 `viewpoint_a..d`의 부호 있는 거리다. RANSAC 계수 자체의 부호를 높이로 쓰지 않는다.

RGB→Lab은 IEC 61966-2-1 sRGB 선형화 뒤 CIE L*a*b*, D65 (Xn 0.95047, Yn 1, Zn 1.08883)다. HSV의 H는 0..1이고, 분류기에는 sin(2πH), cos(2πH), S, V를 넣는다.

Separability의 결정 숫자는 spatial-block logistic regression의 balanced accuracy다. block은 각 reference region의 영상 사분면이고, 한 fold는 그 사분면을 양쪽 클래스에서 함께 뺀다. L2는 1, 반복 탐색은 없다. random point split은 저장만 하고 선택에 쓰지 않는다.

등급은 측정 전에 고정했다. HIGH는 유효 fold 3개 이상이고 평균 0.90 이상. MEDIUM은 0.75 이상. 그 아래는 LOW.

정량 보조값은 채널별 Jensen-Shannon divergence (log2)와 Bhattacharyya distance다. bin 경계도 고정이다. RGB는 0–256을 32칸, H·S·V는 0–1을 32칸, L*는 0–100, a*·b*는 -128–127, height는 -0.02–0.18 m.

접촉 띠는 신발 내부 점의 평면 위 거리다. 신발 쪽은 높이 0.005–0.025 m, 수평 0.015 m 이내. 매트 쪽은 높이 0.005 m 이하, 수평 0.015–0.035 m. 상한 0.025 m는 R1F bridge 0.021 m를 넘기 위해 미리 둔 값이다. 한쪽이 100점 미만이면 `NOT_IDENTIFIABLE`이고 Case C다.

선택 규칙은 `scripts/p008r1h_features.py`의 `select_method`다. 색만 양쪽 HIGH이면 `COLOR_ASSISTED_ROI`. 아니면 height 또는 color+height가 양쪽 HIGH일 때 `COLOR_GEOMETRY_ASSISTED_ROI`. 아니면 `SEGMENTATION_STOP_RULE`. 점수가 0.02 안이면 feature가 적은 쪽, 같으면 Lab 쪽을 고른다.

이 분류기는 진단용이다. segmentation accuracy로 부르지 않는다.
