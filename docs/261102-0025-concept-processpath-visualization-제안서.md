# Concept / ProcessPath Visualization — Implementation Proposal

작성: 2026-11-02 00:25. **조사·제안만.** 구현·렌더·포트폴리오 수정·commit/push 없음.

## 판정 요약

```text
VISUALIZATION_PURPOSE = CLEAR
SOURCE_ASSETS = IDENTIFIED
PROCESSPATH_SOURCE = FROZEN_EVIDENCE
ROBOT_MODEL_SOURCE = IDENTIFIED (runtime package; not vendored in repo)
SIMULATOR_OR_VIEWER = SELECTED → Offline frozen-data renderer (Python) + optional web poster
NEW_CALCULATION_REQUIRED = NO
P022_EVIDENCE_MODIFIED = NO
P025_REOPEN_REQUIRED = NO
CLAIM_BOUNDARY = PRESERVED
IMPLEMENTATION_READY = YES
```

권장안은 Isaac Sim / RViz “성공 실행”이 아니다. frozen P-013 ProcessPath를 주인공으로 하는 **15~30초 concept clip**이다.

---

## 1. 사용 가능한 frozen asset

| Asset | 경로 | 용도 (Storyboard) |
| --- | --- | --- |
| Complex Surface Coupon STEP | `assets/cad/complex_surface_coupon.step` (SHA는 P-011/P-012와 일치) | Scene A geometry |
| P-011 overview PNG | `evidence/P-011/coupon_overview.png` 등 | Scene A poster / fallback still |
| P-011 mesh summary | `evidence/P-011/mesh_faces.json` | Scene A/B surface mesh (재계산 없이 표시) |
| P-012 surface / normals PNG·JSON | `evidence/P-012/cad_*.png`, `cad_surface_samples.json` | Scene B context (선택) |
| **P-013 6D ProcessPath** | `evidence/P-013/cad_process_path.json` **741 waypoints** | Scene B/C 정본 |
| P-013 기존 정적 렌더 | `cad_process_path.png`, `cad_tool_frames.png`, `cad_process_anchor.png` | 참고·poster 후보 (영상 대체가 아님) |
| P-021 contract | `evidence/P-021/*` | Adapter 의미 문구만 |
| P-022 planning input | `evidence/P-022/robot_planning_input.json` | Scene D: frozen `translation_m`, identity rotation, `poses[]` in `base_link` (m) |
| P-022 environment | `evidence/P-022/environment.json` | UR5e model provenance: `ur_description` 3.5.1 on P620 |
| P-025 display model | `evidence/P-025/visualization_model.json` | **읽기만** (수정 금지). scene3에 741 wp 이미 복제됨 |
| P-025 portfolio viewer | `pf…/cad-less-robotics/` (koraiagent.com copy) | Evidence 레이어 유지 |

**사용하지 않음 (의도적으로):** `assets/cad/shoe_last.*` — footwear/대표 형상으로 오해됨. Coupon만 사용.

---

## 2. P-013 ProcessPath 재사용 방법

정본 파일: `evidence/P-013/cad_process_path.json`

- `waypoint_count = 741`
- `frame_id = CAD_MODEL_FRAME`, 단위 mm
- 각 waypoint: `surface_anchor`, `command_position`, `surface_normal`, `orientation_quaternion` (x,y,z,w), `process_offset_mm = 20.0`
- 계약: `command_position = surface_anchor + standoff * surface_normal` (재유도하지 않고 저장된 두 점을 그림)

영상 파이프라인:

1. JSON을 **read-only** 로드.
2. polyline = `command_position` 시계열 (또는 anchor와 command를 구분 색).
3. TCP frame = 일부 index만 (예: 0, 50, 100… 또는 pass 경계) — 741개 전부 표시하지 않음.
4. Scene C zoom = 저장된 필드만 라벨링. standoff 화살표 = `command_position - surface_anchor` (저장된 두 점의 표현; 새 법선 피팅 금지).

대안 소스: `robot_planning_input.json`의 `poses[]`는 동일 path를 `base_link`·m로 옮긴 frozen 결과. Scene D에서만 사용. Scene B/C는 CAD_MODEL_FRAME mm 정본(P-013)을 우선.

`visualization_model.json`의 scene3 waypoints도 동일 좌표를 담지만, **presentation 입력은 P-013 JSON을 SSOT로 고정**하고 P-025 모델은 수정하지 않는다.

---

## 3. UR5e 모델 재사용 방법

| 사실 | 값 |
| --- | --- |
| repo 내 vendored UR5e URDF | **없음** |
| P-022에 기록된 출처 | `/opt/ros/jazzy/share/ur_description/...` (`ros-jazzy-ur-description` 3.5.1), `ur_type=ur5e` |
| `.cache` | DOFBOT URDF만 있음 — **사용 금지** (다른 로봇) |

권장:

- Scene D에서 UR5e **articulated motion / FK 재생은 필수가 아님**.
- 1순위: coupon + ProcessPath를 frozen `translation_m = [0.3244, -0.0500, 0.1113]` (identity R)로 `base_link`에 배치한 **planning context 다이어그램** + 반투명 robot silhouette(선택).
- 2순위(선택): 렌더 호스트(P620)에서 `ur_description`을 **런타임 read-only**로 로드해 정지 자세 1프레임. URDF를 evidence에 복사·커밋하지 않음.
- 금지: 새 datum, 새 IK, joint trajectory를 RobotTrajectory로 저장/표시.

`robot_trajectory: false`가 planning input에 명시되어 있다. 영상 카피도 이에 맞춘다.

---

## 4. RViz / custom viewer / Isaac 비교

| 후보 | Evidence fidelity | ProcessPath 이해도 | 구현 복잡도 | 성공 암시 위험 | 판정 |
| --- | --- | --- | --- | --- | --- |
| **A. Offline Python renderer** (matplotlib/plotly/pyvista 등) + MP4/WebM | 높음 (JSON 직독) | 높음 (Scene A–E 완전 제어) | 낮음~중 | 낮음 (라벨 고정 가능) | **1순위** |
| **B. Portfolio Three.js / canvas 확장** (기존 P-025 viewer 옆) | 높음 | 높음 (인터랙티브) | 중 | 낮음 | 2순위 (영상 대신 또는 poster+interactive) |
| **C. RViz / MoveIt on P620** | 중 (패키지 모델) | 중 (TCP path는 추가 작업) | 높음 | **높음** (실행처럼 보임) | 비권장 primary |
| **D. Isaac Sim** | 낮음~중 (시각 우선) | 중 | 매우 높음 | **매우 높음** | **비채택** |

선택 기준 적용: fidelity > 이해도 > 복잡도 > 화려함 → **A**.

---

## 5. 권장안 (최소 구현)

### 목적 한 문장

CAD/3D에서 만든 robot-independent 6D ProcessPath가 표면 작업 경로(위치+자세)이며, Adapter를 거쳐 planning context로 들어가지만 **검증된 RobotTrajectory가 아님**을 15~30초에 보여 준다.

### 파이프라인

```text
READ evidence/P-013/cad_process_path.json
READ evidence/P-011/{coupon_overview.png | mesh_faces.json}
READ evidence/P-022/robot_planning_input.json  (translation_m only)
→ render script (신규, presentation only)
→ concept-processpath.webm + .mp4 + poster.png
→ (승인 후) pf public/cad-less-robotics/ 에 배치
```

### Storyboard 매핑

| Scene | 초 | 소스 | 화면 라벨 |
| --- | --- | --- | --- |
| A | 0–4 | Coupon PNG/STEP mesh | `REFERENCE_GEOMETRY` · Complex Surface CAD Coupon |
| B | 4–10 | P-013 JSON | `741-waypoint` · `Robot-independent 6D ProcessPath` |
| C | 10–15 | 동일 JSON 확대 | surface_anchor → normal → command_position → TCP |
| D | 15–22 | + frozen translation_m | ProcessPath → Robot Adapter → UR5e Planning Context · **Concept** |
| E | 마지막 ~5 | 고정 텍스트 | P-022 `CLOSED_FAIL` · RobotTrajectory `NOT ACHIEVED` |

전 구간 워터마크/하단 고정:

```text
ProcessPath 개념 시각화
검증된 RobotTrajectory가 아닙니다.
```

### 새 계산

- **없음.** mm→m 표시 변환·frozen translation 적용은 display mapping (P-025가 이미 허용한 SOURCE_TO_DISPLAY_MAPPING과 동형).
- ProcessPath 재생성, datum 재추정, IK, threshold 변경 **STOP**.

---

## 6. 필요한 신규 파일 (승인 후)

`cad-less-robotics` (presentation, evidence 밖):

```text
scripts/render_concept_processpath_video.py   # 또는 tools/presentation/
docs/…-concept-processpath-render-절차.md     # 재현 절차
# 산출물 (git LFS 또는 pf repo로만):
presentation/concept-processpath.webm
presentation/concept-processpath.mp4
presentation/concept-processpath-poster.png
```

`koraiagent.com` (승인 후 배포):

```text
public/cad-less-robotics/concept-processpath.webm
public/cad-less-robotics/concept-processpath.mp4
public/cad-less-robotics/concept-processpath-poster.png
public/cad-less-robotics/index.html   # Hero 아래 video 블록만 추가
```

---

## 7. 수정할 기존 portfolio 파일 (승인 후)

| 파일 | 변경 |
| --- | --- |
| `koraiagent.com/public/cad-less-robotics/index.html` | 제목 아래 **Concept video** 섹션 추가. 기존 P-025 evidence viewer 본문·`model.js`·상태 줄 **유지** |
| sitemap / 홈 / projects | **불필요** (URL 동일) |

구조:

```text
[h1 / status]
[Concept / ProcessPath Visualization video]  ← NEW
[기존 Overview → Scene 1…5 evidence]       ← UNCHANGED role
[GitHub]
```

---

## 8. evidence / Gate 비변경 확인

| 항목 | 이번 제안 |
| --- | --- |
| `evidence/P-011` … `P-022` | **수정 없음** (read-only) |
| `evidence/P-025/visualization_model.json` | **수정 없음** |
| P-025 reopen | **NO** |
| P-022 `CLOSED_FAIL` / RobotTrajectory | **재해석·성공화 없음** |
| P-023 / P-024 | **NOT_OPENED 유지** |
| 기존 failure evidence | **변경 없음** |

---

## 9. 예상 영상 생성 절차 (승인 후 Do)

1. Reviewer 승인 (본 제안서).
2. Mac 또는 P620에서 render script 작성 — pytest로 “로드한 waypoint 수=741”, “translation_m 바이트 일치”, “출력 라벨에 NOT ACHIEVED 포함” 게이트.
3. 15~30초 WebM/MP4 + poster 렌더.
4. 로컬 검수: Scene E·워터마크·금지 문구 부재.
5. `koraiagent.com`에 자산 복사 + `index.html`에 muted controls video (자동재생은 선택, poster 필수).
6. `pf-sync`로 배포. evidence repo Gate 파일은 push 대상에 넣지 않음.

---

## 10. Claim-boundary 위반 가능성

| 위험 | 완화 |
| --- | --- |
| UR5e 관절 움직임 = 실행 성공으로 보임 | Scene D는 정지/schematic 우선; 움직이면 상시 “Concept / Not validated RobotTrajectory” |
| “741 poses reachable” 암시 | Scene E에 CLOSED_FAIL / NOT ACHIEVED 고정 |
| Isaac/physics | **미사용** |
| shoe_last / footwear | **미사용** |
| P-025 PASS = 로봇 성공 | 기존 페이지 문장 유지; 영상은 explanation only |
| display transform을 calibration으로 오해 | 라벨: `CONFIGURED_SIMULATION_DATUM` / SIMULATION_PLACEMENT |

금지 문구 체크리스트: RobotTrajectory generated, collision-free, trajectory-valid, physical execution, production-ready, ground truth, Successful UR5e execution — **영상·캡션·UI 모두 배제**.

---

## STOP 조건 점검 (현재)

| STOP 트리거 | 해당? |
| --- | --- |
| P-013 재생성 필요 | NO — frozen JSON 충분 |
| 새 CAD 필요 | NO — coupon STEP/PNG 충분 |
| 새 simulation datum | NO — frozen translation_m |
| P-022 evidence 수정 | NO |
| P-025 reopen | NO |
| 새 RobotTrajectory | NO — 생성하지 않음 |
| P-023/P-024 open | NO |

→ 구현 착수 **가능**, 단 **reviewer 승인 후**.

---

## Reviewer 결정 요청

1. 권장안 A(Offline frozen-data renderer → 15~30s clip) 승인 여부  
2. Scene D에서 UR5e mesh 포함 여부: (a) schematic only (b) P620 `ur_description` 정지 자세  
3. 자동재생 muted loop vs controls-only  

승인 전: 구현·렌더·공개 HTML 수정·commit/push **보류**.
