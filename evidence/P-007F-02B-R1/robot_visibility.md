# P-007F-02B-R1 — Robot Coverage (Visibility by Segment)

실측 근거: `frames/0000-color.png`(원본), `robot_closeup.png`(x,y=550-970,20-350 크롭, 3배
업스케일), `roi_grid_overlay.png`(100px 격자) — 전부 2026-10-29 실제 캡처.

| 항목 | 판정 | 근거 |
|---|---|---|
| robot base | `VISIBLE` | 흰색 원형 마운트판(모서리 볼트 4개)이 명확히 분리되어 보임 |
| Arm1/Arm2 주요 구조 | `PARTIAL` | base 바로 위의 검은 원통+관절 구조가 보이지만 Arm1(요 회전)과 Arm2(피치)의 경계가 시야각상 서로 겹쳐 명확히 분리되지 않음 |
| Arm3 | `PARTIAL` | Arm2-Arm4 사이 구조가 클러스터에 포함돼 개별 식별 어려움(현재 자세가 비교적 접힌 상태로 보여 더욱 겹침) |
| Arm4 | `PARTIAL` | 동일 사유 |
| wrist(Arm5+회전) | `VISIBLE` | 그리퍼 바로 위 관절부가 구분 가능 |
| end-effector/gripper | `VISIBLE` | 금속 그리퍼 손가락이 클로즈업에서 명확히 분해됨 |
| DaBai assembly | `UNKNOWN` | Arm4 부착 위치로 추정되는 영역이 다른 구조물과 겹쳐 있어, 이 해상도/각도에서 DaBai 를 다른 부품과 구분해 특정하지 못함 |

## 요약

`base → end-effector` 가 **프레임 안에 전부 들어오고 잘리지 않았다** — 최소
목표(§7)는 충족. 다만 중간 링크(Arm1~Arm4)의 **개별 구분은 어렵다** — 이는
단일 시야각에서 상부-하부 링크가 투영상 겹치는 구조적 문제이며,
`arm2_witness_analysis.md`에서 더 자세히 다룬다.

## 배경(참고, 상단 전자기기)

이미지 상단(y≈55-150, x≈700-830)의 검은 사각 물체는 벽쪽 선반 위의 별도
전자기기(공유기로 추정)로, 로봇 구조물이 아니다 — 로봇 bbox에서 제외했다
(`roi_measurements.json`).
