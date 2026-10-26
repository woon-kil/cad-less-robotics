# P-005 Geometry → Motion dry-run

Status: **FAIL**

실패 분류: `IK` / `CARTESIAN_PATH`

P-004 camera-frame waypoint는 덮어쓰지 않았다. Snapshot SHA256은 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`이다. Waypoint SHA256은 `b9a254590dcc260b4556dd38c366bdd98a64efe99bd5eee4d614b39834041021`이다.

이 결과는 `Arm5_Link` planning trajectory가 아니다. Cartesian fraction이 0.95에 못 미쳐 궤적을 만들지 못했다. process TCP도 아니고, camera→robot calibration도 아니다.

## 계약

입력 673개를 모두 받았다. 거절은 0이다. 툴 +Z 단계가 45°를 넘는 인접 쌍은 4개다.

| 경계 index | 이유 | 앞 구간 끝 | 다음 구간 시작 |
| --- | --- | ---: | ---: |
| 60 | ORIENTATION_STEP | 60 | 61 |
| 311 | EXPLAINED_GEOMETRIC | 311 | 312 |
| 488 | ORIENTATION_STEP | 488 | 489 |
| 571 | ORIENTATION_STEP | 571 | 572 |

45°는 직각의 절반이다. 이 snapshot의 툴 +Z 단계 p99는 36.5°라 연속 구간의 꼬리보다 위에 있다. 311의 97.817525013552412°는 이 한계를 넘고, 접선 각 61.7485°와 함께 구간 경계로 남았다. 점을 지우거나 법선을 평균하지 않았다. 경계 사이 정책은 `SEGMENT_BOUNDARY_REQUIRES_POLICY`다.

## 배치

`TEST_ONLY_SYNTHETIC_PLACEMENT`, `calibrated: false`, scale 1, TF publish 0.

각 창의 첫 자세를 SRDF `init` + grip `open`의 `Arm5_Link` FK로 보내는 강체 변환이다. `init`은 P-002에서 충돌이 없던 named state다. 배치 잔차는 1e-16 m이다.

창 길이는 11점, 약 0.020 m다. URDF 가동 링크 원점 거리 합 0.276878 m의 10% 미만이다. 플래너 결과를 보고 창을 바꾸지 않았다.

| Case | source | arc m |
| --- | --- | ---: |
| CASE-01 | 61–71 | 0.019448071331036983 |
| CASE-02 | 312–322 | 0.019350542104909385 |
| CASE-03 | 572–582 | 0.01986808284237673 |

CASE-01은 311 경계 앞 구간의 첫 허용 창이다. CASE-02는 그 경계 다음 구간의 첫 허용 창이다. CASE-03은 나머지 구간 중 가장 긴 572–672의 첫 허용 창이다.

## 계획

MoveIt `CartesianInterpolator`를 프로세스 안에서 호출했다. `eef_step` 0.002, `jump_threshold` 1.0, `min_fraction` 0.95, 충돌 검사는 켜져 있다. `PlanningComponent::execute()`와 controller 전송은 없다.

KDL timeout은 벤더 `kinematics.yaml`의 0.005 s다. 세 case 모두 fraction `0.090909090909090912`(1/11), trajectory point 2, 계획 시간 약 0.005–0.010 s다. 시작 자세는 맞고, 두 번째 waypoint에서 IK가 끝난다. 끝점 위치 오차는 12–19 mm, 자세 오차는 16–85°다. 만든 상태의 충돌과 위치 한계 위반은 0이다. 관절 속도 상한 0.4 rad/s는 유지했고, 짧은 궤적의 측정 속도는 0이다.

3회 분류는 모두 `CARTESIAN_PATH`로 같다. `min_fraction`은 내리지 않았다.

벤더 joint acceleration limit는 꺼져 있다. hardware-safe acceleration은 검증하지 않았다.

## 테스트

gtest 17 + 7 + 8 = 32. failed 0, skipped 0. 이전 24개에 motion contract 8개를 더했다.
