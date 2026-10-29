# P-007F-07B-R1 — 사전확인에서 중단

책상 위 단일 카드는 없다. M2의 50 mm 카드는 DaBai 옆에 그대로 있다. 로봇, Gemini, M2 마커는 움직이지 않았다.

같은 화면 오른쪽 키보드 위에 ArUco 시트가 남아 있다. 24장 중 3번 프레임이 그 시트를 ID 0으로 읽었다. 중심 (1085.8, 371.5) px, 변 길이 15.41 px. P-007F-07B의 M2 군집 (775.8, 126.6) px에서는 395 px 떨어져 있다. 이 24장에서 M2 위치의 ID 0은 0장이다.

한 프레임 안에 ID 0이 동시에 둘인 경우는 없다. 그래도 화면에 ID 0으로 읽힌 인쇄물이 M2 말고 하나 더 있으므로 `ID0 physical instances visible = exactly 1` 이 아니다.

```text
ABORT
DUPLICATE_ID0_STILL_PRESENT
```

200장 이상 본촬영, 검출률, jitter, 배경 보정, detector 민감도 비교는 시작하지 않았다. 합격선은 바꾸지 않았다. P-007F-07B의 FAIL 기록은 그대로 둔다.

## 22개 항목

1. Gemini serial `CP263630009D`, firmware `1.4.60`. 이번 기동 로그와 일치한다.
2. Baseline B `unchanged`. shift 0.471 px (허용 1.0 px). whole-frame mean abs diff 10.62.
3. 보이는 ID 0은 1개가 아니다. M2 카드는 화면에 있고, 키보드 위 시트가 ID 0으로 1회 검출됐다. 책상 위 B의 21.8 px 카드 자리는 비어 있다.
4. 본촬영 프레임 수는 없다. 사전확인만 24 frame.
5. 사전확인 길이 2.447 s. 본촬영의 20초 조건은 시작 전에 멈췄다.
6. M2로 볼 수 있는 검출은 0 frame. ID 0 검출 1 frame은 키보드 시트다.
7. 본촬영 검출률은 계산하지 않았다.
8. 본촬영 변 길이 mean/median/min/max는 없다. 사전확인의 비-M2 ID 0 변 길이는 15.41 px다.
9. 본촬영 four-corner rate는 없다.
10. 본촬영 loss run은 없다.
11. 검출/실패 sharpness 비교는 없다.
12. 밝기/대비 비교는 없다.
13. M2 perspective/skew는 없다. M2가 이 24장에서 검출되지 않았다.
14. reflection/saturation 비교는 없다.
15. 책상 카드는 없다. M2 종이는 팔 위에 붙어 있다. 키보드 위 시트는 FOV 안에 있다. clipping 판정은 본촬영이 없어 하지 않았다.
16. detection-loss 시간 패턴은 본촬영이 없어 분류하지 않았다.
17. M2 static jitter는 검출 성공 프레임이 없어 계산하지 않았다. 다른 마커 위치로 대체하지 않았다.
18. 배경 보정 통계는 없다.
19. detector parameter 진단은 하지 않았다. production detector만 사전확인에 썼다.
20. 10% 검출의 원인 분류는 보류다. 이번 중단 이유는 그 재측정이 아니라 `DUPLICATE_ID0_STILL_PRESENT` 다.
21. `MARKER_STATIC_QUALIFICATION_R1` 은 PASS/FAIL/INCONCLUSIVE를 내지 않았다. 사전조건 `ABORT / DUPLICATE_ID0_STILL_PRESENT`.
22. Evidence `evidence/P-007F-07/marker_static_qualification_r1/`. 원본 24 frame은 `.cache/p007f07b_r1/precheck` 에 있고 SHA는 `precheck.json` 의 `color_sha256` 이다. Commit SHA는 푸시 후 본문에 적는다.

```text
Robot command count = 0
Servo write count = 0
Torque command count = 0

READY_FOR_MOTION_PREREGISTRATION = NO
MOTION_AUTHORIZED = NO
```

여기서 정지한다. 다음 Gate는 열지 않는다.
