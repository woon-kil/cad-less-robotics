# 실물 Arm1 zero

분류: **MODEL-DEFINED / PHYSICAL ALIGNMENT REQUIRED**

## 있는 기준

SRDF named state가 모델 숫자로 `Arm1_Joint=0`을 둔다.

- `up`: Arm1부터 Arm5까지 0
- `down`: Arm1은 0, Arm2는 -1.57
- `init`: Arm1은 0, Arm2는 1.2063, Arm3는 -1.5, Arm4는 -1.3104, Arm5는 0

출처는 `evidence/P-002/srdf.txt`다. 이것은 모델의 group state다. 서보 혼의 눈금, 조립 눈금, 제조사 기계 인덱스와 연결되어 있지 않다.

URDF, 같은 모델의 SRDF, `joint_limits.yaml`에서 zero, home, mark 문구는 찾지 못했다. repository evidence에도 하드웨어 영점 표시는 없다.

## 없는 것

현재 로봇의 Arm1 yaw가 0이라는 근거는 없다. Attempt 2B 사진은 팔이 접힌 한 장면이다. 그 어깨가 가리키는 쪽을 `base_link` +X나 +Y로 쓰지 않는다.

로봇을 zero로 움직이지 않았다.

## 맞춰야 하는 것

모델의 yaw=0은 어깨 힌지 축이 `base_link` +X가 되는 Arm1 각이다. 실물에서 그 각을 재현할 기계 기준이 없으므로, 힌지 축의 현재 방향을 +X라고 부를 수 없다.

+Y는 +X와 +Z가 정해진 뒤의 오른손 축이다. +X가 실물에 없으면 +Y도 없다.

## measurement frame

작업판 위에 `A_AXIS`와 사람이 고른 수평 기준으로 별도 `measurement_frame`을 만들 수는 있다. 그 프레임을 `base_link`라고 부르지 않는다. P-006의 최종 변환은 여전히 `T_base_camera`여야 하므로, measurement frame을 써도 `base_link`로 가는 변환이 따로 필요하다. 이번 단계에서 그 변환은 만들지 않는다.
