# P-007F-02B-R2 — Robot Coverage

실측 근거: `frames/0000-color.png`, `robot_closeup.png`(x,y=580-900,60-360
크롭, 3배 업스케일), `roi_grid_overlay.png`.

| 항목 | 판정 | 근거 |
|---|---|---|
| robot base | `VISIBLE` | 흰 마운트판, 볼트 명확 |
| Arm1/Arm2 | `VISIBLE` | 검은 원통(Arm1) 위 흰색 피벗 브래킷(Arm2)이 R1과 달리 명확히 구분됨 — 카메라가 더 측면에서 보고 있어 두 세그먼트의 경계가 뚜렷 |
| Arm3 | `PARTIAL` | Arm2와 손목 사이 대각선 구조 — 개별 경계가 완전히 분리되진 않으나 대각선 축 자체는 뚜렷이 보임 |
| Arm4 | `PARTIAL` | 동일 사유, DaBai 바로 아래 구조 |
| wrist(Arm5+회전) | `VISIBLE` | 그리퍼 바로 아래 관절부 식별 가능 |
| end-effector/gripper | `VISIBLE` | 손가락 메커니즘 명확히 분해됨 |
| DaBai assembly | `VISIBLE` | 🔴 R1에서는 `UNKNOWN`이었으나, 이번 각도에서는 렌즈가 보이는 검은 박스형 카메라 모듈이 명확히 식별된다(그리퍼 바로 위) |

## 요약

`base → end-effector` 프레임 내 확보, 잘림 없음. R1 대비 **중간 링크 구분이
크게 개선**됐다(Arm1/Arm2 명확 분리, DaBai 식별 가능) — 더 측면(oblique)에
가까운 시야각으로 바뀐 효과로 보인다(각도 수치는 산정하지 않음, §19).
