# W-002E-R3 — Precision Fiducial Physical Metrology

상태: `PASS_PRECISION_PHYSICAL_METROLOGY / CLOSED`

이 합격은 등록 합격이 아니다. 디지털 캘리퍼스 표시 분해능은 0.01 mm이고, 영점 확인은 통과했다. 0.01 mm는 표시 분해능이며 측정 정확도 ±0.01 mm가 아니다.

| ID | 가로 mm | 세로 mm |
| --- | ---: | ---: |
| 0 | 59.98 | 59.98 |
| 1 첫 측정 | 60.60 | 60.60 |
| 1 반복 | 60.32 | 60.32 |
| 2 | 59.90 | 59.90 |
| 3 | 59.80 | 59.80 |

ID1 두 측정의 차이는 0.28 mm다. `ID1_TRUE_SIZE_MM = NOT_ESTABLISHED`. 60.32와 60.60 중 하나를 참값으로 고르지 않았다.

ID0, ID2, ID3는 약 59.8–60.0 mm다. 마커가 약 58 mm라는 가설과, 전체가 58 mm로 줄었다는 가설은 지지되지 않는다. 58 mm를 등록 geometry로 쓰지 않는다. 인쇄 형상이 재투영 실패의 원인이 아니라고 확정하지도 않는다. `ROOT_CAUSE = INCONCLUSIVE`.

실패한 W-002E가 쓴 크기는 명목 60 mm 그대로다. 승인된 `T_camera_workboard`는 없고, medoid는 진단용 변환이다. PnP는 하지 않았다. W-002E, W-002E-R1, W-002E-R2 기록은 바꾸지 않았다.
