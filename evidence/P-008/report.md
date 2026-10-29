# P-008 — Shoe Geometry Pipeline

```text
P-008 = BLOCKED_REAL_SHOE_INPUT_REQUIRED
```

질문은 하나다. 실제 Gemini 335L point cloud에서 신발을 작업대와 분리하고, 순서가 있는 closed boundary를 만들 수 있는가.

그 점군이 저장소와 p620 작업 복사본에 없다. 파이프라인은 실행하지 않았다. 새 촬영도 하지 않았다.

## 찾은 것

| 후보 | 내용 | 사용 |
|---|---|---|
| `evidence/P-004/artifacts/snapshot.pcd` (p620만, gitignore) | Gemini `/camera/depth/points`, 254957점, SHA256 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60` | 아니오. 신발로 기록된 입력이 아니다. P-004를 P-008 근거로 올리지 않는다 |
| `evidence/P-007E-R2/logs/stage1_fov_color.png` | Gemini 컬러 1280×720. 체커보드 위 신발이 보인다. SHA256 `5b9c725cce92266a4de260c35104d0d97542dab3ebac35c93d4da86b712dd594` | 아니오. point cloud가 아니다. 같은 순간의 depth PNG는 evidence에 없다 |

다른 `.pcd` / `.ply`는 없다. 이후 depth PNG는 마커 qualification 장면이다.

## 재사용할 코드

실행하지 않았다. 다음 capture 뒤에 중복 구현하지 않기 위한 위치만 적는다.

`src/vision_path_generator/src/path_extractor.cpp`에 VoxelGrid(leaf-key fallback 포함), StatisticalOutlierRemoval, RANSAC 평면 제거가 있다. P-008은 그 필터 결과를 PASS로 쓰지 않는다. 신발 분리와 boundary는 그 다음 단계다.

## 측정하지 않은 항목

전처리, 평면, cluster, PCA, 투영, concave hull, topology, support distance, spacing, repeatability, 그림 8종은 입력이 없어 만들지 않았다. synthetic cloud로 PASS를 만들지 않았다.

## 다음

별도 capture에서 정지 운동화의 Gemini 335L point cloud를 확보한 뒤에 이 파이프라인을 실행한다. P-009는 열지 않는다.

```text
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```
