# P-007F-07B Phase B2 이후 — 정지 마커 qualification

사용자 확인: `50 mm 실측 OK, M2 평면 부착 완료`.

촬영 동안 마커 위치는 바꾸지 않았고, 로봇은 움직이지 않았다.
`Robot commands = 0`, `Servo writes = 0`, `MOTION_AUTHORIZED = NO`.
`command_events.json` 의 events 는 비어 있다. Jetson / `Arm_Lib` 는 호출하지 않았다.

판정: **`MARKER_STATIC_QUALIFICATION = FAIL`**.
다음 Gate(P-007F-07 single-motion preregistration)는 열지 않는다.

## §B12

1. **물리 설치 확인** — 사용자가 `50 mm 실측 OK, M2 평면 부착 완료` 라고 확인했다. 출력 실측 PASS, M2 평면 부착 PASS.
2. **마커 규격** — `DICT_4X4_50` / ID 0 / 인쇄 크기 50 mm. 검출기도 같은 사전과 ID 를 쓴다.
3. **로봇 명령** — `Robot commands = 0`.
4. **서보 쓰기** — `Servo writes = 0`.
5. **모션 허가** — `MOTION_AUTHORIZED = NO`. 이번 결과에 PASS 가 나와도 움직이지 않기로 한 조건을 유지한다. 실제 판정은 FAIL 이다.
6. **마커·카메라·로봇 고정** — 촬영 중 마커를 옮기지 않았고, Gemini / LED / 로봇을 움직이지 않았다.
7. **카메라 identity** — Gemini 335L, serial `CP263630009D`, firmware `1.4.60`. 기동 로그와 기대값이 일치한다.
8. **연속 촬영** — 119 frame, 11.981 s, 평균 9.93 fps, 1280×720. 전체 시퀀스는 `.cache/p007f07b_capture_full/run2_50mm` 에만 있고, evidence 에는 stride 10 대표 13장만 둔다.
9. **Baseline B recheck** — `unchanged = true`. phase-correlation shift 0.470 px (허용 1.0 px). whole-frame mean abs diff 10.86.
10. **가시 크기 — 상단 ID 0** — 중심 약 (775.8, 126.6) px, 12/119 frame. 변 길이 평균 **41.76 px** (41.42–41.97). 프레임 안에서 로봇 쪽 카메라(DaBai) 옆 흰 사각 마커와 위치가 같다. 검출된 구간만 보면 중심 흔들림은 1 px 미만이다.
11. **가시 크기 — 하단 ID 0** — 중심 약 (815.0, 396.4) px, 47/119 frame. 변 길이 평균 **21.85 px** (21.43–22.24). 책상 위 별도 카드다. 이쪽도 검출된 구간만 보면 중심 흔들림은 1 px 미만이다.
12. **검출률** — 59/119 = **0.496**. 합격선 0.95 미달 (`DETECTION_RATE_BELOW_95_PCT`).
13. **평균 변 길이** — 두 군집을 한 ID 로 섞은 평균 **25.90 px**. 합격선 35 px 미달 (`MARKER_TOO_SMALL_IN_IMAGE`). 50 mm 부착면이 화면에서 35 px 이상으로 안정되게 보이지 않는다. 더 자주 잡힌 쪽은 21.8 px 이고, 41.8 px 쪽은 12장뿐이다.
14. **네 꼭짓점** — 검출된 프레임은 네 꼭짓점이 있다. 전체 프레임 대비 비율은 검출률과 같은 **0.496** 이라 합격선 0.95 미달.
15. **정지 jitter (합친 시계열)** — center x peak-to-peak 40.2 px, center y peak-to-peak 270.6 px, orientation peak-to-peak 13.8°. 이 폭은 한 마커가 흔들린 값이 아니다. 같은 ID 0 이 위 마커와 책상 카드 사이를 오가며 생긴 점프다.
16. **배경 보정** — `extract_trajectory` 는 `valid = true` 로 동작했다. 보정 후 변위 평균 163.3 px, peak-to-peak 91.7 px. 배경이 흔들려서가 아니라 두 마커 사이 점프가 남아 있다. 단일 마커의 정지 안정도를 보여 주지 않는다.
17. **가림 / 손실** — 미검출 60 frame, 최장 연속 손실 6. `persistent_occlusion = false` (기준은 `max(5, 0.1×119)=11`). `clip_count = 0`, `systematic_clipping = false`. 같은 프레임에 ID 0 이 둘인 경우는 119장 중 5장이고, 파이프라인은 ID 당 하나만 남긴다.
18. **판정** — `MARKER_STATIC_QUALIFICATION = FAIL`. 이유: 검출률, 평균 변 길이, 네 꼭짓점 비율. Baseline B 변경, 체계적 clipping, 지속 가림은 이유가 아니다.
19. **다음 Gate** — 열지 않음. P-007F-07 single-motion preregistration 을 시작하지 않는다.
20. **해석 한계** — 이 FAIL 은 “50 mm 종이가 자로 50 mm 가 아니다” 가 아니다. 그 실측은 사용자 확인으로 PASS 다. Gemini 정지 영상에서는 ID 0 이 두 군데 잡히고, 더 자주 잡힌 쪽은 22 px, 41.8 px 쪽은 12장뿐이다. 임계값은 촬영 후에 낮추지 않았다.
