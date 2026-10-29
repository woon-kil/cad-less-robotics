# P-007F-07B-R1 attempt 2

이전 abort(`DUPLICATE_ID0_STILL_PRESENT`) 파일은 그대로 뒀다. 이 디렉터리만 새 attempt다.

물리적으로 화면에 남긴 ArUco는 M2의 `DICT_4X4_50 / ID 0 / 50 mm` 하나다. 검출기가 놓친 프레임을 마커가 없는 것으로 세지 않았다.

판정: **`MARKER_STATIC_QUALIFICATION_R1 = FAIL`**. 변 길이 평균은 35 px를 넘는다. 검출률이 6.6%다. 다음 Gate는 열지 않는다.

## 22개 항목

1. Gemini serial `CP263630009D`, firmware `1.4.60`. 런치 인자는 바꾸지 않았다.
2. Baseline B `unchanged`. 본촬영 첫 프레임 shift **0.463 px** (허용 1.0 px).
3. 물리 ID 0은 **1개**. 사전확인 화면에서 책상 카드와 키보드 시트는 없고, DaBai 옆 M2 카드만 있다. 체커보드는 ArUco가 아니다. 검출기와 분리하면 사전확인 1/24, 본촬영 17/258이다. 한 프레임에 ID 0이 둘인 경우는 0이다.
4. 본촬영 **258** frame.
5. 길이 **25.942 s**, 평균 **9.95 fps**, 1280×720.
6. M2 ID 0 검출 **17** frame.
7. 검출률 **0.0659** (합격선 0.95).
8. 변 길이 mean **41.65 px**, median **41.69**, min **40.11**, max **42.33**. 크기 합격선 35 px는 넘는다.
9. 네 꼭짓점 비율 **0.0659**. 검출된 프레임은 테두리 3 px 안에 꼭짓점이 없다. 비율이 낮은 이유는 미검출이다.
10. 최장 연속 미검출 **45** frame. loss run 14개.
11. 고정 ROI Laplacian variance median: 검출 1491, 미검출 1486. 둘은 같다.
12. 고정 ROI 밝기 median 98.7 / 98.7. p10은 둘 다 36, p90은 둘 다 183. 대비(p90−p10) median 147 / 147.
13. 검출 프레임의 변 비율 median **1.088**, 맞은편 변 비 median **1.076**. 꼭짓점 각이 90°에서 벗어난 양의 median **30.8°**. 종이는 비스듬히 보인다. 미검출 프레임의 같은 ROI도 같은 종이가 그대로 있다.
14. near-black 비율 약 0.019, near-white(>240) 비율 약 0.002. 검출과 미검출이 같다. 포화로 갈라지지 않는다.
15. 대표 crop에서 코드 네 귀퉁이와 흰 여백이 프레임 안에 있다. 케이블·브래킷이 코드를 가리지 않고, 종이가 접혀 있지 않다. `clip_count = 0`. `persistent_occlusion = true`는 최장 미검출 45가 프레임의 10%를 넘었다는 손실 규칙이지, 가림이 보인다는 뜻이 아니다.
16. 손실 형태는 **burst**. 짧은 검출이 간격을 두고 나타나고, 가장 긴 빈 구간은 45 frame이다.
17. 검출된 17장만 사용했다. center x std 0.30 px, peak-to-peak 1.13 px. center y std 0.39 px, peak-to-peak 1.73 px. orientation std 2.13°, peak-to-peak 9.15°. side std 0.47 px. 미검출 프레임을 다른 좌표로 메우지 않았다.
18. 배경 보정은 성립하지 않았다. 이유 `NO_VALID_BASELINE_FRAMES`. 처음 3 frame에 ID 0이 없다. 보정 변위는 만들지 않았다. 검출 프레임의 raw 변위 평균은 0.42 px, 최대 1.74 px다. 이전 B의 163 px instance-switch는 다시 나오지 않았다.
19. production detector만 합격 판정에 썼다. offline variant의 ID 0 검출률은 corner refinement off 0.0659, adaptive window wide 0.0659, narrow 0.0이다. 어느 것도 검출률을 올리지 못했다. `DETECTOR_CONFIGURATION_LIMITATION`으로 보지 않는다.
20. 분류: `INSUFFICIENT_DETECTION_RATE`, `VIEWING_GEOMETRY_LIMITATION`, `CAUSE_UNRESOLVED`. 크기가 작아서 실패한 것이 아니다. 비스듬한 각은 검출된 프레임의 측정값이고, 미검출 프레임과 영상을 가르지 못한다. 밝기·선명도·포화·가림·세 variant도 검출 여부를 가르지 못한다.
21. **`MARKER_STATIC_QUALIFICATION_R1 = FAIL`**.
22. Evidence `evidence/P-007F-07/marker_static_qualification_r1/attempt-02/`. 컬러 원본 258장은 `.cache/p007f07b_r1/attempt2_run`에만 있다. color SHA256을 다시 해시하면 `qualification.json`의 `color_sha256_of_sha256`이다. depth PNG는 p620의 같은 디렉터리에 있다. Commit SHA는 푸시 뒤 완료 보고에 적는다.

```text
Robot command count = 0
Servo write count = 0
Torque command count = 0

READY_FOR_MOTION_PREREGISTRATION = NO
MOTION_AUTHORIZED = NO
```

여기서 정지한다.
