# P-012 보고

```text
P-012 = PASS_CAD_GEOMETRY_ENGINE

INPUT_SHA256 = 2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa
INPUT_INTEGRITY = PASS

FACE_COUNT = 20
SURFACE_TYPES = BSpline 14, Plane 5, Cylinder 1

TOTAL_SAMPLE_COUNT = 1620
VALID_SAMPLE_COUNT = 1585
INVALID_SAMPLE_COUNT = 35

OUTSIDE_TRIM_COUNT = 35
DEGENERATE_DIFFERENTIAL_COUNT = 0
NORMAL_FAILURE_COUNT = 0
CURVATURE_UNDEFINED_COUNT = 0
NONFINITE_COUNT = 0

NORMAL_LENGTH_MIN = 0.9999999999460213
NORMAL_LENGTH_MAX = 1.0000000000512939

K_MIN = -0.0011310794
K_MAX = 0.0017847765

NEGATIVE_K_SAMPLE_COUNT = 144
POSITIVE_K_SAMPLE_COUNT = 61
NEAR_ZERO_K_SAMPLE_COUNT = 1380

LOW_CURVATURE_REGION_DETECTED = PASS
POSITIVE_K_REGION_PRESENT = PASS
NEGATIVE_K_REGION_PRESENT = PASS

SADDLE_POST_VALIDATION = PASS

FACE_ORIENTATION_HANDLING = PASS
REPRODUCIBILITY = PASS

FINAL_GATE = PASS_CAD_GEOMETRY_ENGINE
```

P-011 STEP은 수정하지 않았다. P-013은 열지 않았다.

## 세 가지 확인

`BRepLProp_SLProps`의 법선은 이 빌드에서 `Su × Sv`와 같다. `TopoDS_Face`가 `REVERSED`인 14개 면은 법선을 뒤집고 principal curvature 부호를 바꿨다. 바닥 면의 보정 후 법선은 `-Z`다. Gaussian curvature는 곱이라 부호가 유지된다.

UV bounds는 trim의 bounding rectangle이다. 그 안의 점을 `BRepClass_FaceClassifier`로 검사했다. 35개는 `OUTSIDE_TRIM`이고 geometry failure로 세지 않았다.

분류 코드에는 P-011 좌표가 없다. `K < -1e-5`인 샘플은 144개다. 그중 `|k1|`, `|k2|`가 모두 `5e-3` 미만인 8개는 정의대로 `LOW_CURVATURE`다. 나머지 136개는 `SADDLE`이다. 가장 작은 K는 `-0.0011310794`이고 위치는 `(103.0, 50.0, 27.219)` mm다. 엔진이 끝난 뒤에 P-011 oracle과 비교했고, 거리는 0.001 mm다.

## 단위와 부호

위치는 mm다. `k1`, `k2`, `H`는 1/mm이고 `K`는 1/mm²다. 법선이 고체 바깥을 향할 때, 볼록한 타원점은 principal curvature가 음수이고 `K`는 양수다. `CONVEX`와 `CONCAVE`는 그 부호만으로 정하지 않았다. oracle의 볼록·오목 위치는 사후에 둘 다 `ELLIPTIC`이었다.
