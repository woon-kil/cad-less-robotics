# P-011 보고

```text
P-011 = PASS_COMPLEX_SURFACE_CAD_TEST_COUPON

SOURCE = SELF_GENERATED
GENERATOR = FreeCAD Part / OpenCascade
VERSION = FreeCAD 1.1.4 / OCC 7.8.1

STEP_READ = PASS
UNIT = mm

SOLID_COUNT = 1
SHELL_COUNT = 1
FACE_COUNT = 20
EDGE_COUNT = 40

BOUNDING_BOX_MM =
X = 180.0
Y = 100.0
Z = 41.252357465702

CAD_VALIDITY = PASS
CLOSED_SHELL = PASS

LOW_CURVATURE = PRESENT
CONVEX = PRESENT
CONCAVE = PRESENT
SADDLE_TRANSITION = PRESENT

LANDMARK = PRESENT
LANDMARK_OUTSIDE_PROCESS_SURFACE = PASS

SURFACE_QUERY = PASS
NORMAL_QUERY = PASS
NORMAL_FAILURE_COUNT = 0

REPRODUCIBILITY = PASS

FINAL_GATE = PASS_COMPLEX_SURFACE_CAD_TEST_COUPON
```

P-012는 열지 않았다. 곡률 질의, ProcessPath, 출력, 촬영, registration은 하지 않았다.

## 면의 책임

20개 면은 네 영역 solid를 fuse한 결과다. 각 영역이 윗면, 옆면, 바닥을 가지고, 영역 사이의 맞닿은 벽은 fuse에서 사라진다. +X landmark 원통이 옆면과 끝 뚜껑을 더한다.

| face | 역할 | 책임 |
| --- | --- | --- |
| 0 | low_curvature | 낮은 곡률의 공정 표면 |
| 3 | convex | 중심이 주변보다 높은 공정 표면 |
| 7 | saddle | X는 골, Y는 등인 공정 표면 |
| 11 | concave | 중심부가 주변보다 낮은 공정 표면 |
| 5, 9, 13, 17 | base | 영역별 평면 바닥 |
| 1, 2, 4, 6, 8, 10, 12, 14, 15, 16 | side | 영역을 닫는 옆면 |
| 18, 19 | landmark | 공정 표면 밖 +X 원통과 끝 뚜껑 |

## 형상 확인

다시 읽은 STEP에서 공정 면의 높이를 재었다. 비교 거리는 18 mm다.

안장 면의 중심 `(103, 50, 27.22)` mm는 서쪽 33.27 mm, 동쪽 30.65 mm보다 낮다. 같은 점의 남쪽과 북쪽은 25.40 mm로 중심보다 낮다. X는 골이고 Y는 등이다.

볼록 면의 중심은 39.46 mm이고, 사방 18 mm는 30.74–36.0 mm다. 오목의 가장 낮은 점은 `(146.0, 38.05, 22.87)` mm다. 낮은 곡률 면의 높이 범위는 약 0.8 mm다.

landmark의 최대 z는 15.0 mm이고, 공정 표면의 최소 z는 21.57 mm다. landmark는 +X로 168 mm를 넘어 180 mm까지 나간다. 공정 표면은 그 안으로 들어가지 않는다.

## 자산

`assets/cad/complex_surface_coupon.step`

SHA-256 `2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa`

GROUND_TRUTH = NONE. ROLE = REFERENCE_GEOMETRY.
