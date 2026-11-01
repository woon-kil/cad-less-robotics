# P-025 V-01..V-10

```text
P025_VALIDATION = V01_V10_EXECUTED
P025_PASS = NOT_EVALUATED
UPSTREAM_EVIDENCE = READ_ONLY
```

비교 대상은 Visualization Model과 frozen evidence다. 실패하면 adapter 또는 화면을 고친다. upstream evidence는 고치지 않는다.

| Check | Result |
| --- | --- |
| V-01 | PASS | Scene 1 fields |
| V-02 | PASS | Scene 2 fields |
| V-03 | PASS | Scene 3 stored waypoints and residual string |
| V-04 | PASS | Scene 4 provenance split |
| V-05 | PASS | Scene 5 run_id 0-20, null fraction, no derived percent |
| V-06 | PASS | CLOSED_FAIL visible |
| V-07 | PASS | scoped NO_JUSTIFIABLE_ALTERNATIVE text |
| V-08 | PASS | ProcessPath and RobotTrajectory are distinct |
| V-09 | PASS | prohibited claims are not success statements in the page |
| V-10 | PASS | upstream evidence and historical P-025 body unchanged |

## AC 대응

이 표는 검사 결과의 대응이다. Gate PASS는 reviewer 판정 전이다.

| AC | 이 실행에서 확인한 것 | 판정 |
| --- | --- | --- |
| AC-01 | V-01..V-05. 정량 필드는 출처 파일의 저장 값이다 | PENDING_REVIEWER |
| AC-02 | Scene별 FINAL_GATE만 성공으로 표시한다 | PENDING_REVIEWER |
| AC-03 | V-06. overview와 Scene 5에 CLOSED_FAIL | PENDING_REVIEWER |
| AC-04 | V-05. 17, run_id 5·7·12, run_id 20, 0.606061, 20/33 | PENDING_REVIEWER |
| AC-05 | V-07. 고정 문구 | PENDING_REVIEWER |
| AC-06 | V-08. ProcessPath와 RobotTrajectory NOT ACHIEVED | PENDING_REVIEWER |
| AC-07 | CAD = REFERENCE_GEOMETRY, Scan = MEASURED_GEOMETRY, GROUND_TRUTH = NONE | PENDING_REVIEWER |
| AC-08 | Scene 5의 P-023, P-024는 NOT_OPENED | PENDING_REVIEWER |
| AC-09 | overview에 VERIFIED BOUNDARY와 NOT ACHIEVED가 있다 | PENDING_REVIEWER |
| AC-10 | V-10. P-011..P-022와 역사 본문 해시가 같다 | PENDING_REVIEWER |
