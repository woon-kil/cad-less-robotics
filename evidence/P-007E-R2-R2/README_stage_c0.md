# P-007E-R2-R2 Stage C0 — Stationary Ego-Motion Qualification Preregistration

> 선행: Stage A `STATIC SAFETY AUDIT PASS / CLOSED @ 3579ab5` · Stage B `PASSIVE UVC OBSERVATION FOV_PASS / CLOSED @ 4d0c709`
> 🔴 **두 게이트의 파일을 수정하지 않았다** — `stage_c0_immutability.json` 에서 `git diff --exit-code` 양쪽 exit 0 확인.

이번 단계는 **측정하지 않는다.** Stage C1 sham 데이터를 보기 전에
**ROI·알고리즘·가드·noise metric·threshold rule·판정 어휘를 tracked evidence 로 고정**하는 것이 전부다.

## 사후 튜닝을 막는 장치

| 장치 | 내용 |
|---|---|
| ROI 동결 | `stage_c0_roi.json` — 이미지 sha256·좌표·선정/배제 근거까지 커밋 |
| 사전등록 동결 | `stage_c0_preregistration.json` — 알고리즘·모델·가드·임계 공식·판정 어휘 |
| 🔴 harness sha 대조 | `p007e_r2r2_c1_sham.py` 가 사전등록 파일의 sha256 을 **상수로 들고 있다**. 파일이 바뀌면 `PREREGISTRATION_SHA_MISMATCH` 로 멈춘다 |
| 🔴 성공 판정 1개 | `Q1` 하나뿐. C1 결과를 본 뒤 새 성공 코드를 만들지 않는다 |
| LOO false positive | trial i 의 임계를 나머지 9 trial 로 계산해 자기참조를 끊는다 |

## 고정한 것

```
ROI            캐비닛 강체 3영역 (프레임의 5.44%, ORB 449개)
배제           로봇 가능 영역 · 케이블 · 천/배낭 · 램프 · 캐비닛 위 물건 · 포화 상단 · 테이블
알고리즘        ORB(3000, 1.2, 8, fast12) + BFMatcher(Hamming, crossCheck) + estimateAffinePartial2D(RANSAC 1.5px)
변환 모델       2D partial affine 4 DOF   (homography·full affine 쓰지 않는 이유 기록)
가드            features≥150 · matches≥60 · inliers≥40 · ratio≥0.50 · residual≤1.50px · |scale−1|≤0.05
K / M / N_sham  5 (임계 배수) / 60 (window 당 프레임) / 10 (trial)
window 대표값   성분별 median, anchor = reference window 첫 프레임
임계 공식       thr = K × max(1.4826·MAD, σ_floor),  σ_floor = 0.70 px · 0.060°
판정            X > Q3 > Q4 > Q2 > Q1  (성공은 Q1 하나)
카메라 제어     🔴 변경 없음 — 자동노출 ON·자동WB ON·50 Hz 그대로 첫 baseline 측정
```

🔴 **K/M 의미 충돌을 명시 해소했다.** 작업의뢰서 §10 의 예시는 `K = window 대표 프레임 수`였으나,
선행 설계(`docs/261027-1914-*` §D4)는 `K = 5` 를 **임계 배수**로, `M = 60 프레임` 을 window 길이로
정의한다. 같은 조항이 "기존 설계 우선"을 지시하므로 **선행 정의를 채택**하고 그 사실을 기록했다.

## 단위검증 — 물리 감도가 아니다

`stage_c0_unit_validation.json` · 🔴 `ALGORITHM UNIT VALIDATION`

- 합성 변환 14종 회수: 평행이동 오차 **≤ 0.703 px**, 회전 오차 **≤ 0.060°**
- 밝기·대비만 바꾼 섭동: 겉보기 변위 **≤ 0.015 px**, 회전 **≤ 0.0007°**
- harness `--selftest` 가 독립 분석 스크립트와 **같은 수치**를 낸다 (사전등록 파라미터가 실제로 반영됨)

🔴 합성 2D 변환에는 센서 노이즈·재노출·JPEG 재양자화가 없다. **실제 DaBai 감도 증거가 아니다.**
이 숫자는 σ_floor 와 가드의 근거로만 쓴다.

## C1 harness

`scripts/p007e_r2r2_c1_sham.py` — 🔴 **기본값 NO CAPTURE**

```
기본 실행              자기검사만 하고 종료 (sham 키 자체가 생기지 않는다)
--run-sham 만          거부
ALLOW_CAPTURE=1 만     거부
둘 다 있을 때만        캡처 경로 진입
사전등록 변조 시       PREREGISTRATION_SHA_MISMATCH 로 중단   ← 실제로 시험해 확인
자기 AST 검사          Arm_Lib·smbus·rclpy·rospy import 0 · i2c 문자열 0
```

음성 시험 3종을 모두 실행해 확인했다(카메라 0).

## 하지 않은 것

카메라 기동 · 새 캡처 · UVC 제어 변경 · recorder · ROS launch · `ros2` CLI · Arm_Lib ·
servo read/write · torque · raw I2C · robot motion · Gemini/DaBai 물리 이동 · **C1 실행**
