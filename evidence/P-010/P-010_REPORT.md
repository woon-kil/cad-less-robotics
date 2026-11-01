# P-010 CAD reference asset

질문: 이 공개 CAD 자산이 CAD branch의 입력으로 쓰기에 충분히 유효하고 재현 가능한가.

이 자산은 CAD branch input이다. 측정 표면의 ground truth가 아니고, scan surface를 대체하지 않으며, CAD 좌표를 ProcessPath로 강제하지 않는다. `GROUND_TRUTH = NOT_ASSIGNED`.

## 결과

```text
P-010
SOURCE              https://github.com/peterzask/freecad-shoe-last-wb
COMMIT              59d1db3b48b460634f1504ddab7d1a1f33be05d7
LICENSE             NOT_DECLARED
CAD_FORMAT          STEP, SI millimetre
STEP_READ           PASS
SOLID / SHELL / FACE / EDGE
                    1 / 1 / 3 / 7
BOUNDING_BOX_MM     X 299.973  Y 98.465  Z 118.996
CAD_VALIDITY        INVALID
SURFACE_QUERY       FAIL
NORMAL_QUERY        FAIL
NORMAL_FAILURE_COUNT 0
REPRODUCIBILITY     PASS
FINAL_GATE          FAIL_CAD_ASSET_QUALIFICATION
```

정점 수는 4다. STEP 단위는 `SI_UNIT(.MILLI.,.METRE.)` 다. 좌표에 NaN/Inf는 없다. 경계 상자 크기는 성인 화형 크기 범위 안에 있어 `DIMENSION_PLAUSIBLE = PASS` 다. 이 범위는 제조 공차가 아니다.

`Shape.isValid()` 는 true이고 shell은 하나이며 closed다. `Shape.check(True)` 의 BOP 검사는 `BOPAlgo OperationAborted` 로 끝났다. 자기교차가 없다고 확인되지 않았으므로 `CAD_VALIDITY = INVALID` 다.

면은 3개이고 모두 `Part::GeomBSplineSurface` 다. 각 면에서 파라미터 9점을 평가했고, 그 27점의 좌표는 유한하다. 같은 점의 법선도 유한하고 길이가 1이다. 실패 개수는 0이다. 다만 고정한 검사는 서로 다른 면 10개 이상이다. 3개뿐이므로 `SURFACE_QUERY` 와 `NORMAL_QUERY` 는 FAIL이다. 면을 쪼개 개수를 맞추지 않았다.

같은 STEP을 두 번 읽었다. topology 개수, 경계 상자, 표본 좌표, 유효성 상태가 같았다. `REPRODUCIBLE = PASS`.

라이선스 파일은 없고 GitHub license도 null이다. 상태를 `NOT_DECLARED` 로 기록했다. `LICENSE_RECORDED = PASS` 는 그 기록이 있다는 뜻이다. 재배포 권리는 `UNKNOWN` 이다.

## 항목

| 항목 | 결과 |
| --- | --- |
| STEP_READ | PASS |
| CAD_VALIDITY | FAIL |
| FACE_COUNT > 0 | PASS (3) |
| SURFACE_QUERY | FAIL |
| NORMAL_QUERY | FAIL |
| DIMENSION_PLAUSIBLE | PASS |
| REPRODUCIBLE | PASS |
| LICENSE_RECORDED | PASS (`NOT_DECLARED`) |

하나라도 실패하면 게이트는 `FAIL_CAD_ASSET_QUALIFICATION` 이다. P-011은 열지 않았다.

## 그림

- `cad_overview.png` — 전체 화형, XYZ 축, 경계 상자
- `cad_faces.png` — F0 본체, F1, F2. 본체를 반투명하게 두어 나머지 면 라벨이 보이게 했다
- `cad_normals.png` — 각 면 중앙 표본과 법선
