# P-007F-07A — M2 Kinematic Membership Verification

질문: Gemini close-up(`evidence/P-007F-07/marker_mount_closeup/`)에서 `M2`로
표시한 cream/white 사각 브래킷이, servo ID2(Jetson Arm_Lib "Arm2" 채널,
P-007F-04/05 전체에서 152/154/151로 명령한 그 서보)가 실제로 회전하면 공간상
반드시 움직이는 downstream 구조물인가?

**결론: `M2_DOWNSTREAM_OF_ID2 = YES`**

로봇/서보/카메라/marker 실제 조작 전부 0 — 저장된 URDF(`.cache/jetson_robot_model/DOFBOT_Pro-V24.urdf`,
P-007E-R2-R4에서 이미 Jetson에서 가져온 read-only 캐시)와 STL mesh만 읽었다.

## 1. servo ID2 ↔ URDF joint mapping

**새로 만들지 않았다.** `scripts/p007e_r2r4_fk.py`(P-007E-R2-R4, 4건의 과거 FK
결과와 오차 ≤0.005mm로 교차검증되어 실제 P-007F-05 §36 실행의 안전판정 근거로
이미 쓰인 모듈, SHA256=`a4e9d2d7891597a6bfa62c8ca87303aeab28ba0a784f23f2640ca8dc145622cb`,
수정 없이 SHA-pin import)의 `full_angles()` 함수 소스를 직접 읽어 재확인했다:

```python
def full_angles(id2_lib_deg, base=None):
    base = dict(base or BASE_JOINT_ANGLES_LIB)
    base["Arm2_Joint"] = id2_lib_deg   # <- 바로 이 줄
    return base
```

```text
servo ID2  ==  URDF joint "Arm2_Joint"
```

## 2. Joint parent link

```text
Arm2_Joint.parent = Arm1_Link
```

## 3. Joint child link

```text
Arm2_Joint.child = Arm2_Link
type = revolute, axis_local = (1, 0, 0)  -- 수평축(pitch), Arm1_Joint의 수직축(yaw, axis=0 0 1)과 다름
```

## 4. Downstream kinematic chain (독립 재계산, parent/child 그래프 순회)

FK 모듈이 이미 선언한 `RIGID_MOVING_LINKS`/`FIXED_LINKS`를 그대로 믿지 않고,
`evidence/P-007B/urdf_joints.json`(URDF에서 이미 파싱된, FK 모듈이 쓰는 것과
동일한 소스)의 parent/child 관계만으로 Arm2_Joint의 child(`Arm2_Link`)부터
BFS/DFS로 독립적으로 재유도했다:

```text
Arm2_Link, Arm3_Link, Arm4_Link, Arm5_Link, DaBai_DCW2_Link,
Gripping_point_Link, llink1, llink2, llink3, rlink1, rlink2, rlink3
```

**교차검증**: FK 모듈이 선언한 `RIGID_MOVING_LINKS = (Arm2_Link, Arm3_Link,
Arm4_Link, Arm5_Link, DaBai_DCW2_Link, Gripping_point_Link)`는 위 독립
유도 집합의 **부분집합**이다(PASS). 차집합(`llink1/2/3, rlink1/2/3`)은 모순이
아니다 — 그 link들은 topology상으로는 Arm2_Joint의 downstream이 맞지만, FK
모듈은 `grip_joint` 등 **자체 DOF를 추가로 갖는** 그리퍼 손가락 link를
"순수 rigid 전파"에서 의도적으로 제외한 것뿐이다(그 모듈의 목적=안전 포락선
계산). M2는 그리퍼 손가락이 아니므로 이 구분은 M2 판정에 영향을 주지 않는다.

```text
servo ID2
  |
  v  (scripts/p007e_r2r4_fk.py full_angles(): base["Arm2_Joint"] = id2_lib_deg)
URDF joint: Arm2_Joint  (revolute, axis=1 0 0, parent=Arm1_Link)
  |
  v
child link: Arm2_Link  (mesh bbox z-extent = 107.0mm)
  |
  v  URDF joint: Arm3_Joint (parent=Arm2_Link, child=Arm3_Link)
child link: Arm3_Link  (mesh bbox z-extent = 107.0mm, Arm2_Link와 거의 동일)
  |
  v
M2 physical structure (cream/white bracket)
```

전문: `kinematic_chain.txt`.

## 5. M2 ↔ physical/URDF link mapping

🔴 **중요한 방법론 공개**: 이 프로젝트에는 **Gemini 335L과 로봇 base 사이의
검증된 extrinsic calibration이 없다**(P-007F-06에서 이미 명시: point cloud
registration 미검증, `NOT_ATTEMPTED_DUE_TO_UNVERIFIED_REGISTRATION`). 따라서
이번 작업도 "URDF 3D 좌표를 카메라 화면에 픽셀 단위로 투영"하는 방식은 **쓸 수
없다**(그렇게 하면 만들어낸 정밀도가 된다). 대신 각 link의 **mesh bounding-box
길이 프로파일**(STL 실측, 추정 아님)을 Gemini close-up에서 육안으로 식별한
구조물의 상대적 크기·형태와 대조하는 방법을 썼다:

```text
Arm1_Link  mesh bbox size = (54.9, 54.9, 43.5) mm   -- 짧고 원형/사각 단면 -> 흰색 디스크와 일치
Arm2_Link  mesh bbox size = (47.6, 29.1, 107.0) mm  -- 길고 좁은 단면
Arm3_Link  mesh bbox size = (47.6, 29.1, 107.0) mm  -- Arm2_Link와 거의 동일(쌍둥이 링크)
Arm4_Link  mesh bbox size = (35.1, 90.3, 50.0) mm   -- y축 우세(손목부 꺾임 구조)
```

Gemini close-up(`evidence/P-007F-07/marker_mount_closeup/02_arm2_closeup.png`)에서
관측된 구조는 "짧은 흰색 디스크 1개 + 그 위로 거의 동일한 외관의 긴 검은
원통형 몸체 2개"다 — 이는 Arm1_Link(짧음) + Arm2_Link/Arm3_Link(길고 서로
거의 동일)의 길이 프로파일과 **정성적으로 일치**한다.

`M2`(cream/white 사각 브래킷, close-up box=`(745,95,55,55)`)는 이 두 개의
긴 원통형 몸체 중 **아래쪽이 끝나고 위쪽으로 전환되는 지점 부근**에 위치한다.
즉 M2는 다음 둘 중 하나다:

```text
(a) Arm2_Link 말단부의 브래킷/구조물
(b) Arm3_Link 시작부의 브래킷/구조물
```

**(a)와 (b) 중 정확히 어느 쪽인지는 이번 gate에서 확정하지 않는다**(그러려면
camera-robot extrinsic calibration이 필요하다, 이번 범위 밖). 하지만 **둘 다
Arm2_Joint의 downstream(`RIGID_MOVING_LINKS`)에 포함되므로, 이 모호성은
`M2_DOWNSTREAM_OF_ID2` 판정 자체에는 영향을 주지 않는다** — YES/NO를 가르는
경계는 Arm1_Link/Arm2_Link 사이(Arm2_Joint 자체)이지, Arm2_Link/Arm3_Link
사이가 아니다.

시각 자료: `m2_mapping.png`(좌: Gemini close-up에 정성적 link 라벨 overlay,
우: URDF kinematic chain schematic diagram — M2 위치 표시).

## 6. 판정

```text
M2_DOWNSTREAM_OF_ID2 = YES
```

## 7. 판단 근거

1. servo ID2 → `Arm2_Joint` 매핑은 **새로 추정한 것이 아니라**, 실제
   P-007F-05 §36 실행의 안전판정에 이미 쓰인 검증된 FK 모듈의 소스에서
   직접 확인했다.
2. URDF parent/child 그래프를 **독립적으로 재순회**해 Arm2_Joint의 전체
   downstream link 집합을 구했고, 이것이 FK 모듈이 선언한
   `RIGID_MOVING_LINKS`를 부분집합으로 포함함을 **교차검증**했다(단순
   재주장이 아님).
3. M2가 Arm2_Link와 Arm3_Link 중 정확히 어느 쪽에 속하는지는 미확정이지만,
   **둘 다 downstream 집합에 포함**되므로 이 잔여 불확실성은 최종 YES/NO
   판정을 흔들지 않는다.
4. mesh bounding-box 길이 프로파일(STL 실측)이 Gemini close-up에서 관측된
   물리 구조(짧은 디스크 + 거의 동일한 긴 원통 2개)와 정성적으로 일치해,
   "사진만 보고 같이 움직일 것 같다"는 추정이 아니라 **실제 CAD 치수와의
   대조**로 보강했다.
5. (corollary, §YES 추가 확인과 별개로 기록) `M1`(disc)은 `Arm1_Link`
   — Arm2_Joint의 **parent**이자 FK 모듈의 `FIXED_LINKS`에 명시적으로
   포함된 link다. 즉 **M1은 ID2 단독 회전으로는 움직이지 않는다**(Arm1_Joint
   회전에만 반응). 이는 이번 분석에서 새로 발견된 사실이며, P-007F-05/06의
   기존 사전등록 결과를 소급 변경하지 않는다 — 향후 해석에 참고할 새 단서로만
   기록한다.

### YES인 경우 추가 확인 (요청된 1문장)

```text
An actual rotation of servo ID2 (Arm2_Joint) necessarily changes the
camera-observed pose of a marker rigidly mounted on M2 (Arm2_Link or
Arm3_Link), because M2 sits strictly downstream of Arm2_Joint in the URDF
kinematic chain.
```

**과대 주장 금지 확인**: 이 문장은 "marker가 움직인다(공간상 pose 변화)"만
주장하며, 다음은 명시적으로 주장하지 않는다 — `marker motion == exact ID2
shaft angle`, `marker rotation == exact commanded angle`,
`controller readback == physical joint angle`. M2 marker의 1차 목적은
P-007F-05/06에서 이미 확립된 대로 **physical motion/no-motion의 독립 관측**이지,
정밀 각도 측정이 아니다.

## 금지 사항 확인 (실측)

```text
robot_command_count = 0
servo_write_count   = 0
torque_commands     = 0
robot physical motion = 0
Gemini relocation    = 0
marker attachment    = 0
```

기존 `evidence/P-007F-03~07/`(marker_print 포함)는 전부 불변(읽기만 함,
이번 작업은 새 디렉터리 `m2_kinematic_verification/`에만 쓴다).

## 부수 관찰 (minor, 수정하지 않음)

`scripts/p007e_r2r4_fk.py`의 `load_stl_vertices()` docstring 주석은 "STL
단위가 mm로 보인다"고 적혀 있으나, 실측(raw float 직접 확인, 예:
Arm1_Link 첫 정점 z=0.0035)으로는 **STL 원본 단위가 미터(m)**다 — 이
모듈의 실제 호출부(`mesh_radius_table_mm()`)는 결과값에 `*1000`을 곱해
mm로 변환하므로 동작 자체는 이미 올바르다(단위 버그 아님, 주석만 오해의
소지가 있음). 이번 gate의 `load_stl_vertices()` 재사용 시에도 동일하게
`*1000.0`을 명시 적용했다 — 결과를 manual STL parser로 교차검증해 일치
확인(`Arm1_Link` bbox 54.9×54.9×43.5mm 동일). **코드 자체는 수정하지
않았다**(frozen P-007E-R2-R4 모듈, 이번 gate 범위 밖).
