# P-007F-02B-R2 — Robot/Workspace Balance

근거: `roi_measurements.json`.

| 영역 | 프레임 대비 면적 비율 |
|---|---|
| robot_bbox | 5.82% |
| workspace_bbox | 32.55% |
| table_bbox(로봇 footprint 포함) | 43.75% |

## 판정

```text
BALANCED
```

**근거**: 로봇이 Arm2 witness 에 필요한 수준으로 충분히 크고 뚜렷하게
보이면서(5.82%, R1의 5.43%와 비슷한 수준 — 작아지지 않음), 동시에 작업영역
(32.55%)과 전체 테이블(43.75%)이 프레임의 상당 부분을 차지해 global
workcell perception 목적에도 지장이 없다. 로봇이 지나치게 크게 잡혀
workspace 를 압도하지도 않고(`ROBOT_DOMINANT` 아님), workspace 만 넓고
로봇이 지나치게 작아 witness 로 쓰기 어려운 수준도 아니다
(`WORKSPACE_DOMINANT` 아님). 완벽한 중앙 배치는 아니지만(§10 "완벽히
중앙일 필요는 없다"), 두 역할(Role 1/Role 2)의 균형이라는 목표에는
부합한다.
