# P-007E-R2-R1 — Independent Physical Observation Sensor Selection

성격: **DESIGN / INVENTORY / FEASIBILITY ONLY.** 카메라 캡처 0 · 거치 0 · 로봇 모션 0 · 구매 0.

## 판정

```
R2-R1 — CANDIDATE FOUND / PHYSICAL SETUP REQUIRED   (분류 B)
```

보유 장치 중 §9 의 R1~R7 을 **설치 없이** 전부 만족하는 후보는 없다. 설치가 불필요한 후보는
하나 있지만 §16 의 '독립 rigid support' 원칙과 충돌해 **자동 채택하지 않는다**.

## 🔴 이번 게이트 최대 발견 — DaBai 는 팔에 달려 있다

```
DOFBOT_Pro-V24.urdf
  <joint name="DaBai_DCW2_Joint" type="fixed">
    <parent link="Arm4_Link" />          ← 🔴
    <child  link="DaBai_DCW2_Link" />
    origin xyz = [-0.0002, 0.0689, 0.0506]  rpy = [0.00873, 0, 0]
```

Jetson 에 연결된 **Orbbec DaBai DCW2**(RGB `2bc5:0561` + Depth `2bc5:06a0`)는 **eye-in-hand** 다.
ID2 가 돌면 카메라도 함께 돈다. 로봇의 **자기 description 파일**에서 나온 사실이다.

## 시스템 inventory

| 호스트 | 카메라 | 비고 |
| --- | --- | --- |
| **p620** | Gemini 335L 하나뿐 | `/dev/video0..7` 은 전부 Gemini 의 UVC 인터페이스(P-003 usb.txt). 🔴 **두 번째 카메라 없음** |
| **Mac** | MacBook Pro 내장 1대 | 외장 USB 카메라 없음 |
| **Jetson** | **DaBai DCW2 (RGB-D)** | `/dev/video0,1` · RGB 1920x1080/1280x720/… · `dofbot_pro_color/depth/vision` 패키지 존재 |

Jetson 접속은 **read-only inventory 만**이다 — 기존 evidence 에 장치 목록이 없었기 때문이다
(P-007A-R1 은 프로세스·autostart·I2C fd 를 수집했다). **I2C·Arm_Lib·ROS 노드 기동 0.**

## 추천

### PRIMARY — MacBook Pro 내장 카메라 (외부 독립 관찰자)

- ✅ **R6 완전 독립** — 다른 호스트, 다른 채널, 로봇에 안 묶임
- ⚠️ **R2 약함** — 노트북 리드는 독립 rigid support 가 아니다
- 🔴 미해결: Mac 이 로봇과 같은 공간인지(**운영자만 아는 사실**), macOS TCC 승인 필요 여부, 해상도·FPS
- 다음 단계: 물리 배치 + rigid support → 권한 확인 → FOV qualification → sham qualification

### SECONDARY — DaBai DCW2 (Arm4 eye-in-hand)

🔴 **측정 모델이 뒤집힌다.** 이 카메라는 팔을 *관찰*하지 않고 팔에 *실려* 있다. 질문이
"대상이 움직였는가"에서 "카메라(=Arm4 원위 구조)가 움직였는가"로 바뀌지만, **같은 물리 사실**을
묻는다 — ID2 가 실제로 돌았으면 Arm4 도 돌고 카메라도 돈다.

| | |
| --- | --- |
| 강점 | 설치 작업 **0**(이미 고정·연결) · ID2 회전에 대한 화소 민감도가 외부 관찰자보다 **한 자릿수 배 큼** · sensing 채널이 I2C 와 독립 · 드라이버 패키지 존재 |
| 약점 | 🔴 §16 원칙 충돌 · 호스트를 컨트롤러와 공유 · 배경 정지 필요 · 조명/자동노출 취약 · 🔴 **벤더 launch 가 로봇 노드를 함께 띄울 위험**(P-007A 의 `move_init_pose()` 계열) → 사용 전 정적 검사 필수 |

🔴 **검수 결정 필요**: ego-motion 측정을 independent physical evidence 로 인정할 것인가.
R2-R1 은 이 결정을 스스로 하지 않는다.

### NOT RECOMMENDED

- **Gemini 335L 재배치** — 금지. P-003/P-004/P-006C/P-007E-R2 Stage1 baseline 이 현재 포즈에 묶임
- **로봇·작업대 직접 부착 신규 카메라** — §16
- **휴대폰** — inventory 로 확인되지 않았다. 보유를 가정하지 않는다

## 민감도 참고값 (feasibility 용. 명령 크기 결정 아님)

```
외부 관찰자:  대상 변위 ≈ (mm/°) × Δangle        Arm3 1.45 · Arm4 2.04 · Arm5 1.91 · TCP 2.73 mm/°
              fx=610 · 1 m 에서 1.638 mm/px  →  2° 시 TCP 5.47 mm ≈ 3.3 px
ego-motion:   영상 전역 변위 ≈ fx × tan(Δangle)  →  fx=600, 2° 시 약 21 px
```

🔴 `Arm1_Link` · `Arm2_Link` 는 **0.00 mm/°** 다. 추적 대상은 **Arm3 이후 원위 구조**다.

## marker

이번 게이트에서 부착하지 않았다. **지금 단계에서 권하지도 않는다** — 기존 배경 특징
(작업대 체커보드·ArUco 용지, P-006A/P-006C)으로 충분한지부터 측정해야 하고, 부착은
별도 운영자 승인 사안이다.

## 불변 확인

- 기존 evidence 11게이트 161파일 — **changed 0 · missing 0** (`evidence_immutability.json`)
- Gemini 물리 포즈 변경 0 · 카메라 캡처 0 · robot controller access 0
- controller-reported state `[86, 152, 0, 18, 93, 178]` 불변
- ID2==150 전제 harness 3종 `STALE / DO NOT EXECUTE` 유지

## STOP

보고 후 멈춘다. 거치·캡처·FOV 시험·sham·feature tracking·marker·servo 명령·`152→150` 복귀·
Stage 2·P-006D 어느 것도 자동 진행하지 않는다.
