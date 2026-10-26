# P-006B-R4 — 서보 중위 저장 상태 조사

로봇을 움직이지 않고, MCU에 저장된 neutral 편차 숫자를 읽기 전용으로 얻을 수 있는지만 봤다.

판정은 **DIAGNOSIS PASS**다. 그 숫자를 읽는 API는 없다. 그래서 캘리브레이션 레지스터를 더 호출하지 않는다.

H0 `q = (library_deg - 90°) × π/180`는 벤더 소프트웨어 convention이다. H2 부호 반전은 근거가 없다. 이 기기에 저장된 편차와 정밀 관절 보정은 검증하지 않았다.

R3 BLOCKED 기록은 그대로다. R3A는 촬영 없이 취소했다. P-006은 PASS가 아니고 P-007은 열지 않았다.

| 게이트 | 판정 |
|---|---|
| P-006B resume-01 | DIAGNOSIS PASS / CLOSED |
| P-006B-R2 | DIAGNOSIS PASS / CLOSED |
| P-006B-R3 | BLOCKED / CLOSED |
| P-006B-R3A | CANCELLED / NOT REQUIRED |
| P-006B-R4 | DIAGNOSIS PASS / CLOSED |
| H0 | VENDOR-DEFINED SOFTWARE CONVENTION |
| H2 | unsupported alternative |
| 이 기기 저장 중위 | NOT VERIFIED |
| 정밀 관절 보정 | NOT VALIDATED |
| P-006 | NOT PASS |
| P-007 | NOT OPEN |
