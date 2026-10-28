# P-007F-02B-R1 — Gemini 335L Global Workcell Placement Qualification

🔴 사용자가 Gemini 335L 을 물리적으로 재배치한 뒤, 그 **현재 위치**(`Baseline B
Candidate`)를 실제 RGB/Depth 캡처로 검수한 gate. 로봇은 전혀 움직이지 않았다
(DOFBOT position/torque/reset/calibration/action playback command 0, DaBai
재배치 0, R5 재실행 0).

## 캡처 개요

- 호스트: p620(`bytesize-robotics-camera` 컨테이너, `scripts/run_humble_camera.sh`
  기존 패턴 재사용 — USB 노드 변경(카메라 재배치로 재연결됨) 감지해 컨테이너
  자동 재생성, 이미 확립된 reversible 절차)
- 드라이버: `ros2 launch orbbec_camera gemini_330_series.launch.py`
- 캡처 스크립트: `scripts/p006c_capture_streams.py`(기존, 수정 없이 출력 경로만
  변경해 재사용) — color+depth+camera_info+TF 를 동시에 읽어 12개 synced
  frame set 저장
- 종료: 캡처 완료 후 카메라 드라이버 프로세스 정지(컨테이너 자체는 기존
  관례대로 idle 상태로 유지)

## 파일

```text
camera_identity.json       Task 4 — device/serial/stream 확인
rgb_overview.png            대표 RGB(frame 0000)
robot_closeup.png           로봇 영역 3배 업스케일 크롭
roi_grid_overlay.png        100px 격자 오버레이(ROI 좌표 근거)
depth_overview.png          Task 6 — depth 시각화
depth_stats.json            Task 12 — valid ratio/median/min/max, ROI별 분리
pointcloud_sanity.png       Task 13 — 역투영 point cloud scatter
pointcloud_sanity.md        Task 13 — sanity 판정
roi_measurements.json       Task 10 — robot/arm2/workspace/table bbox(manual approximation)
robot_visibility.md         Task 7 — 세그먼트별 VISIBLE/PARTIAL/OCCLUDED
arm2_witness_analysis.md    Task 8 — ARM2_WITNESS_* 판정
workcell_coverage.md        Task 9 — 작업영역 커버리지
occlusion_analysis.md       Task 14 — self/robot→workspace/workspace→robot occlusion
placement_decision.json     Task 11/16/17/18 종합 — 최종 판정
frames/                     원본 12개 frame set(color/depth PNG + json metadata), 전부 보존
```

## 최종 판정

```text
BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT
```

상세 근거와 조정 방향은 `placement_decision.json` 참조. 핵심: robot/workspace
coverage·depth 품질·independence·baseline stability 는 양호, Arm2 witness
suitability 만 `MARGINAL`(현재 시야각이 너무 가파른 하향이라 pitch 평면
foreshortening + 링크 self-overlap) — 하향 기울기를 줄이는 방향의 소폭 조정을
권고한다. 거리/각도 수치는 증거 없이 임의 제시하지 않았다.

🔴 이 PASS/CONDITIONAL 판정은 camera placement qualification 일 뿐이다 —
calibration 완료·absolute pose 측정 가능·Arm2 motion 검증 완료·P006D 완료·
RGB-D geometry regression 완료를 의미하지 않는다(`placement_decision.json.
not_to_be_confused_with`).

기존 Gemini evidence(P-003/P-004/P-006 계열, `Baseline A`)는 수정·무효화하지
않았다. 이 새 위치의 extrinsic 은 기존 값을 재사용하지 않는다.
