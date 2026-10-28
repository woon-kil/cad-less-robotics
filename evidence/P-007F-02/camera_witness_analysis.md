# P-007F-02 — Camera Witness Analysis

🔴 이 gate에서 카메라 위치를 변경하지 않는다. 실제 USB/카메라 재설정·기동 0회.
아래는 전부 **기존 evidence 재검토**다.

## DaBai DCW2 (현재 eye-in-hand)

### 장점

- 이미 설치·배선·드라이버·캡처 파이프라인(`p007e_r2r2_acquisition.py`/
  `measurement.py`/`analysis.py`, ORB+RANSAC+quality guard, 전부 frozen/SHA-pinned)
  존재 — 추가 설치 작업 0.
- Arm2 가 실제로 움직이면 그 하위 전체 체인(Arm3·Arm4·Arm5·gripper·DaBai 자신)이
  **기구학적으로 함께 움직인다**(`evidence/P-007E-R2-R4/swept_motion.json`:
  DaBai_DCW2_Link 가 Arm2 +2° 에 대해 worst-case exact displacement 5.93mm로
  가장 크게 움직이는 링크로 이미 식별됨) — Arm2 motion 에 대한 픽셀 감도가 매우
  높다.
- `Arm_serial_servo_read`/`read_any`/`ping_servo`(I2C) 와 **완전히 독립된 채널**
  (USB/UVC) — controller readback 이 신뢰할 수 없어도 이 채널 자체는 오염되지
  않는다.

### 문제점 — 🔴 이번 목적에 치명적

- **eye-in-hand 라서 "독립 external witness" 가 아니다.** DaBai는 팔을 관찰하는
  것이 아니라 팔에 실려 있다(`evidence/P-007E-R2-R1/sensor_inventory.json`
  `mounting_fact`: `DaBai_DCW2_Joint` type=`fixed`, parent=`Arm4_Link`).
  Block 3 의 기존 공식 정의(`evidence/P-007E-R2-R2/stage_c1r1r1_architecture_note.json`
  §24, R3 §5 에서 그대로 전재)가 이미 "independent external observer... 로
  표현하지 않는다"를 명시한다.
- 토크 판별이라는 이번 목적에서는 이 결함이 더 치명적이다 — "카메라가 움직였다"는
  측정이 "Arm2 가 실제로 돌았다"와 **같은 물리 사실의 재진술**일 뿐, "Arm2 가
  돌았다는 것을 독립적으로 확인했다"는 주장을 지지하지 않는다. 즉 torque
  discrimination 의 **독립 witness 로 격상시키면 안 된다**.
- 카메라 자체(마운트)의 미세한 흔들림·DaBai 가 Arm4 에 고정되는 나사의 유격 등이
  섞여 들어가도, eye-in-hand 구조에서는 "로봇이 움직였다"와 "카메라 마운트만
  흔들렸다"를 구별할 **외부 기준이 없다**.

### 판정

**SUITABLE AS SECONDARY/CORROBORATING SIGNAL ONLY.** 단독·독립 witness 로는
부적합. 사용한다면 "이 신호는 eye-in-hand ego-motion 이다 — 외부 독립 확인이
아니다"를 매번 명시한다(기존 Block 3 공식 정의와 동일 원칙).

---

## Gemini 335L (고정 workcell 카메라)

### 역할(기존 evidence)

`evidence/P-006/camera-target/requirement-redesign-r1/sensor-role.md`: "고정된
Eye-to-Hand 전역 인식 센서... 역할은 작업면 관찰, RGB-D 획득, 점군 생성...". P-003/
P-004 이후 설치를 유지하며 이 revision에서(그리고 이후 모든 revision에서, P-007E-R2-R1
의 `not_recommended` 재확인) 카메라를 옮기지 않는다.

### 현재 FOV가 로봇을 보는지 — 기존 evidence 재확인

- `evidence/P-007E-R2-R1/sensor_inventory.json.gemini_335l.fov_fact`: "P-007E-R2
  Stage 1 — Arm3~gripper 가 화각에 없다... 카메라는 테이블을 보고 있고 팔은
  화각 위로 벗어난다."
- `evidence/P-006/base-reference/physical-arm1/physical-arm1-reference.md`:
  Gemini 가 찍은 유일한 참조 프레임(`gemini-color.png`, 1280×720)에 "Arm1부터
  gripper까지의 연결은 화면에 없다."
- `evidence/P-007E-R2-R1/decision.json.not_recommended`: "Gemini 335L 재배치"가
  명시적으로 금지됨(P-003/P-004/P-006C/P-007E-R2 Stage1 workplane baseline 이
  현재 포즈에 묶여 있음).

### 판정

**`NOT CURRENTLY USABLE`** — Arm2를 포함한 팔 전체가 현재 Gemini 화각 밖에
있다는 것이 세 건의 독립 evidence(센서 인벤토리 조사·실측 프레임·설치사 근거)로
이미 확인되어 있다. 이 gate에서 카메라 위치를 바꾸지 않으므로 이 판정을 바꾸지
않는다.

---

## 요약

| 카메라 | 역할 적합성 | 판정 |
|---|---|---|
| DaBai DCW2 (eye-in-hand) | Arm2 motion 에 대한 감도는 높으나 독립 witness 자격 없음 | SECONDARY/CORROBORATING ONLY |
| Gemini 335L (고정) | 원칙적으로 이런 목적에 더 적합한 구조(eye-to-hand)지만 현재 팔이 화각 밖 | NOT CURRENTLY USABLE |

→ **현재 장비로는 "독립적인 visual witness" 가 존재하지 않는다.** 이것이
`witness_candidates.md`에서 카메라에 의존하지 않는 Candidate A/C 를 우선하는
근거다.
