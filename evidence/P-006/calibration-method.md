# P-006 calibration method

상태: **실행하지 않음**. `T_base_camera`는 계산하지 않았다.

## 이름

`T_base_camera`는 `camera_depth_optical_frame`의 점을 `base_link`로 보내는 변환이다.

`p_base = T_base_camera * p_camera`

`T_camera_base`는 그 역변환이다. 둘 다 이번 게이트에서 구하지 않았다.

후보 이름은 `MEASURED_EXTRINSIC_CALIBRATION`이다. P-005의 `TEST_ONLY_SYNTHETIC_PLACEMENT`는 historical evidence로 두고, 이 계산에 넣지 않는다.

## 풀려면 있어야 하는 양

Known: `base_link`에서 본 calibration target의 pose. 이번 세션에서 측정하지 않았다.

Measured: 카메라가 본 target pose. 스트림을 열지 않았고 검출도 없다.

Solved: `T_base_camera`. 없음.

Validation: fit에 쓰지 않은 sample의 위치 오차와 자세 오차. sample이 없다.

로봇 기준 pose의 출처가 없으면 식을 세우지 않는다. URDF에서 카메라 위치를 읽거나, 눈대중 숫자, identity, synthetic placement로 이 빈칸을 채우지 않는다.

Eye-to-Hand만 대상이다. Eye-in-Hand, process TCP, 표면 궤적 실행은 이 게이트 밖이다.

## 왜 멈추는가

p620에서 Gemini 335L의 이름, 펌웨어 1.4.60, USB 5000M은 읽었다. 같은 호스트에 DOFBOT 시리얼 장치가 없고, base 설치와 카메라 설치는 보지 못했다. 타깃의 칸 수와 치수도 측정하지 않았다.

이 상태에서 sample 수, 다양성, acceptance threshold를 숫자로 고정하면 측정 전에 방법을 고른 것이 아니라 없는 보드를 가정한 것이 된다. threshold는 고정하지 않았다. 사후에 완화한 threshold도 없다.

## 재개 조건

1. 실제 보드의 rows, columns, square size, marker size, dictionary, 인쇄 치수를 잰 뒤 `target.yaml`에만 적는다.
2. target pose를 `base_link`에서 아는 측정 절차를 먼저 적는다. 추정은 절차가 아니다.
3. 그 다음에 최소 sample 수와 acceptance threshold를 정하고, 그 다음 촬영한다.
4. 로봇 trajectory는 보내지 않는다.
