# P-007F-02B-R1 vs R2 — Comparison

실제 evidence만 사용했다. Candidate #2를 좋게 보이도록 임의 조정하지 않았다.

| Metric | Candidate #1 (R1, `abb5016`) | Candidate #2 (R2, 이 문서) |
|---|---|---|
| Robot coverage | base→end-effector 확보, 중간 링크(Arm1-4) self-overlap 심함, DaBai UNKNOWN | base→end-effector 확보, Arm1/Arm2 명확 분리, Arm3/Arm4 PARTIAL(개선), DaBai **VISIBLE** |
| Arm2 witness | `ARM2_WITNESS_MARGINAL` | `ARM2_WITNESS_GOOD` |
| Workspace coverage | 확보(workspace 16.9%, table 25.8%) | 확보(workspace 32.6%, table 43.8% — 더 넓음) |
| Robot/workspace balance | (R1에서는 별도 항목 없었음 — 이번 gate에서 신설) | `BALANCED` |
| Depth quality | overall 84.8% valid, robot ROI 77.4%, table ROI 96.6% | overall 83.3% valid, robot ROI 75.4%, table ROI 94.5% — 유사 수준 |
| Occlusion | self=HIGH, robot→workspace=LOW, workspace→robot=ACCEPTABLE | self=**ACCEPTABLE**(개선), robot→workspace=LOW, camera→table=ACCEPTABLE |
| Static stability | STATIC_BASELINE_STABLE(전체 diff ~2.23) | STATIC_BASELINE_STABLE(전체 diff ~2.55) — 둘 다 안정 |
| Frame margin(top) | 85px(11.8%) | 90px(12.5%) — 유사, 악화 없음 |
| External independence | INDEPENDENT_EXTERNAL_VISION_WITNESS | INDEPENDENT_EXTERNAL_VISION_WITNESS(동일) |
| Overall | `BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT` | `BASELINE_B_PLACEMENT_ACCEPTED`(아래 placement_decision.json 참조) |

## 핵심 차이

R1→R2 전환으로 가장 크게 개선된 항목은 **Arm2 witness**(MARGINAL→GOOD)와
**self-occlusion**(HIGH→ACCEPTABLE)이다 — 둘 다 같은 원인(시야각이 top-down
에서 더 oblique/측면에 가깝게 바뀜)으로 설명된다. depth 품질·workspace
coverage·static stability·independence 는 R1에서도 이미 양호했고 R2에서도
유지됐다(악화 없음). Frame margin(top)은 거의 동일 수준으로 유지됐다 —
개선도 악화도 아니다.
