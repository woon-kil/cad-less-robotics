# URDF와 실물

대조 사진은 `evidence/P-006/base-reference/attempt-02b/dofbot-arm1-annotated.png`다. 픽셀로 길이이나 각을 재지 않았다. 사진에 이미 그려진 축 글자는 이번 판정의 근거가 아니다.

보이는 구조는 은색 원판, 그 아래 검은 하우징, 알루미늄 사각판, 검은 제어 상자, 나사, 위로 접힌 팔이다.

| 실물 | 판정 | 이유 |
|---|---|---|
| 은색 원판 | POSSIBLE_MATCH | URDF에서 Arm1 mesh는 베이스 위의 작은 원판이다. 같은 부품이라고 측정하지 않았다. |
| 수직 회전축 | POSSIBLE_MATCH | URDF 축은 `base_link` +Z다. 사진의 원판 축이 그 관절인지는 길이로 확인하지 않았다. |
| 사각판 | POSSIBLE_MATCH | mesh의 낮은 판은 -Y로만 길다. 사진의 판이 그 형상인지는 축을 맞춘 관찰이 아니다. |
| 제어 상자 | NOT_IDENTIFIABLE | `base_link` 안에 별도 제어기 링크가 없다. |
| 나사 | NOT_IDENTIFIABLE | 구멍 중심을 mesh에서 꺼내지 않았다. |
| 팔이 향하는 쪽 | NOT_IDENTIFIABLE | 사진의 관절 각이 URDF 0이라는 기록이 없다. |

사각판의 긴 변을 `base_link` +X로 두지 않는다. mesh에서 긴 쪽은 Y이고, X 범위는 -0.092500 .. 0.092453 m로 거의 대칭이다.

```text
PLATE_EDGE_TO_BASE_X = UNVERIFIED
```

| URDF datum | URDF definition | Physical candidate | Confidence | Status |
|---|---|---|---|---|
| base origin X/Y | Arm1 origin x=0, y=0. 축이 원점을 지난다 | J1 회전 중심 | LOW | UNRESOLVED |
| base origin Z | 링크 원점. mesh 바닥은 -0.015 m, 판은 z=0을 가로지른다 | 없음 | LOW | UNRESOLVED |
| +X | q1=0에서 Arm2 축. 팔 길이 방향이 아니다 | 없음 | LOW | UNRESOLVED |
| +Y | +Z와 +X의 오른손 축. mesh 긴 판은 -Y | 없음 | LOW | UNRESOLVED |
| +Z | Arm1 axis `0 0 1`. 관절은 z=0.0895 m, mesh 위쪽 | J1 축, 판에서 팔 쪽 | MEDIUM | RESOLVED |
| J1 axis | `0 0 1`, XY 오프셋 0 | 사진의 원판 축 | LOW | UNVERIFIED |

+Z만 방향이 맞다. URDF 축이 +Z이고, 관절 원점은 mesh 위쪽이며, 사진에서는 팔이 판 위에 있다. 이 방향은 z=0의 위치가 아니다.

J1 중심은 모델에서 XY 원점과 같은 축이다. 실물 중심을 그 원점에 맞춘 측정은 없다. fixture로 쓰지 않는다.

```text
CHECKERBOARD_FIXTURE_DATUM_CANDIDATES = NONE
```
