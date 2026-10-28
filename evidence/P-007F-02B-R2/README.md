# P-007F-02B-R2 — Gemini 335L Placement Re-evaluation

🔴 R1(`evidence/P-007F-02B-R1/`, commit `abb5016`)의 위치 조정을 전제로 하지
않고, 운영자가 새로 재배치한 현재 Gemini 335L 위치(`Baseline B Candidate #2`)
를 **처음부터 독립적으로** 재평가한 gate. 로봇은 전혀 움직이지 않았다.

## R1과의 관계

- `Baseline A`(역사적 P-003/P-004 위치): 수정 없음
- `Baseline B Candidate #1`(R1 평가 대상): **수정/삭제하지 않고 그대로
  보존** — historical / not selected placement candidate 로 취급
- `Baseline B Candidate #2`(이 gate 평가 대상): 아래 최종 판정 참조

## 캡처 개요

R1과 완전히 동일한 파이프라인 재사용: p620 `bytesize-robotics-camera`
컨테이너 + `scripts/run_humble_camera.sh` + `ros2 launch orbbec_camera
gemini_330_series.launch.py` + `scripts/p006c_capture_streams.py`(출력
경로만 변경). 12개 synced RGB+depth frame set 신규 취득, 캡처 후 드라이버
정지.

## 파일

```text
camera_identity.json         device/stream 확인
rgb_overview.png              대표 RGB(frame 0000)
rgb_annotated_overview.png    robot/Arm2/end-effector/workspace bbox 표시
depth_overview.png            depth 시각화
depth_stats.json              valid ratio/median/ROI별 분리 + R1 비교
pointcloud_sanity.png/.md     point cloud sanity
roi_measurements.json         bbox 측정치(manual approximation)
robot_visibility.md           세그먼트별 VISIBLE/PARTIAL/OCCLUDED
arm2_witness_analysis.md      8.1~8.5 세부평가 + ARM2_WITNESS_GOOD
workcell_coverage.md          작업영역 커버리지
robot_workspace_balance.md    BALANCED 판정
occlusion_analysis.md         self/robot→workspace/camera→table occlusion
static_baseline.md            STATIC_BASELINE_STABLE
r1_vs_r2_comparison.md        R1↔R2 정량 비교표
placement_decision.json       종합 판정
frames/                       원본 12개 frame set 전부 보존
```

## 최종 판정

```text
BASELINE_B_PLACEMENT_ACCEPTED
```

`ARM2_WITNESS_GOOD` + `BALANCED` + depth usable 세 조건이 동시에 확보되어,
현재 위치를 **Gemini Global Workcell Baseline B** 로 freeze 후보화한다.
추가 최적화를 위해 카메라를 더 옮기는 것은 권장하지 않는다(§19).

🔴 이 PASS는 camera placement qualification 일 뿐이다 — calibration·absolute
pose 측정·Arm2 motion 검증·P006D·RGB-D geometry regression 완료를 의미하지
않는다(`placement_decision.json.not_to_be_confused_with`). 잔존 한계(일부
self-overlap, 국소 배경대비 약함, top margin 여전히 타이트)는 숨기지 않고
공개했다(`placement_decision.json.disclosed_limitations`).
