# P-007E-R2 — Independent Physical Observation System Qualification

목적: **다음 단 한 번의 motion command 때 controller readback 과 independent physical
measurement 를 동시에 확보할 방법을 설계·실증한다.** 🔴 이 게이트의 motion command 는 0 이다.

## Stage 1 — Transport / recorder qualification

**`TARGET_NOT_DETECTED` 로 조기 종료했다.**

| 단계 | 결과 |
| --- | --- |
| 카메라 컨테이너 | 🔴 `/dev/bus/usb/002/013` 으로 만들어져 있었고 카메라는 `002/014` 였다 → 장치 미노출 |
| 조치 | 프로젝트 전용 `scripts/run_humble_camera.sh` 로 재생성 (p620·Mac 사본 sha256 동일) |
| 드라이버 | `ros2 launch orbbec_camera gemini_330_series.launch.py` → color/depth 8토픽 정상 |
| 1 unit sanity | color 1280x720 `fx=610.51` · depth 848x480 `fx=410.41` · 프레임 1.94 MB · 인코딩 0.103 s |
| sham 부하 추산 | 600프레임 = **1166 MB / 61.8 s** (p620 여유 667 GB) |
| **FOV 1프레임** | 🔴 **DOFBOT 구조물이 프레임에 없다** — 테이블·체커보드·신발·케이블만 보인다 |

판정:

```
P-007E-R2 STAGE-1 — TARGET_NOT_DETECTED / STOP
```

검수 지시대로 **우회 구현을 즉석에서 추가하지 않았다.** 카메라 재배치도 하지 않았다.

## 왜 이것이 놀랍지 않은가

P-006C-R2 는 **작업면(workplane)** 을 관측했고 `ROBOT_SIDE` 위치는 52캡처 중 **4프레임**뿐이었다.
이 카메라는 원래 테이블을 보도록 놓였고 **팔을 보도록 놓인 적이 없다.** 이번 관측은 그 기록과
모순되지 않는다. R2 플랜이 "가장 큰 단일 리스크"로 적어 둔 항목이 첫 프레임에서 바로 확인됐다.

## 추적 대상 정의 (바뀌지 않는다)

`evidence/P-007E/fk_clearance.json` 실측 — ID2 를 돌렸을 때:

| 링크 | mm/° |
| --- | ---: |
| `Arm1_Link` · `Arm2_Link` | **0.00** |
| `Arm3_Link` | 1.45 |
| `Arm4_Link` | 2.04 |
| `Arm5_Link` | 1.91 |
| TCP | 2.73 |

**`Arm2_Link` 는 움직이지 않는다.** 관측 대상은 **ID2 원위 링크(Arm3~그리퍼)** 다.

## 로봇·컨트롤러 불변

```
servo writes            0
motion-capable commands 0
torque writes           0
RGB writes              0
raw I2C writes          0
robot motion            0
robot access            0      ← Jetson 에 접속조차 하지 않았다
```

controller-reported state 는 `[86, 152, 0, 18, 93, 178]` 그대로다.

## 되돌린 것

- 드라이버 정지 (프로세스 0 확인)
- p620 `.p007e_r2_runtime/` 삭제 (sanity 프레임 2장)
- 🔴 카메라 컨테이너 재생성은 **되돌리지 않았다** — 올바른 USB 노드를 가리키도록 고친 것이고,
  되돌리면 다시 장치가 보이지 않는다. p620 저장소 사본(`8c070d0` + 로컬 수정)에 git 조작은 하지 않았다

## 다음 — 검수 결정 사항

Stage 2 로 가려면 **ID2 원위 링크가 보이는 관측 포즈**가 필요하다. 선택지는 운영자 물리 작업이다.

1. 카메라 재배치 — 🔴 P-006C 의 **고정 카메라 전제**를 깬다(52캡처의 기준). 재배치 후 작업면
   관측 자산의 재사용 가능성이 달라진다
2. 두 번째 관측 장치 추가 — 기존 포즈를 보존한다
3. `OBSERVATION QUALIFICATION BLOCKED / CLOSED` 로 종료

🔴 R2 는 스스로 이 결정을 하지 않는다. 그리고 어느 경우에도 servo motion gate 를 열지 않는다.
