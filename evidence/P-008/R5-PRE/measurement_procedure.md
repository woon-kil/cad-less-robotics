# P-008-R5-PRE 측정 절차

상태: **BLOCKED**. 체커보드를 설치하지 않았고, 촬영하지 않았고, `^B T_T`를 계산하지 않았다.

## 프레임

```text
B = base_link
C = camera_color_optical_frame
T = checkerboard_target_frame
```

`camera_depth_optical_frame`은 쓰지 않는다. R4 작업 경로는 color optical frame이다.

```text
p_B = ^B T_C · p_C
^B T_C = ^B T_T · (^C T_T)^-1
```

`^C T_T`는 나중 color 영상과 capture_01 CameraInfo의 solvePnP다. 이번 단계에서는 실행하지 않는다. depth로 base 빈칸을 채우지 않는다.

## 타깃 프레임

문서로만 고정한다. 실물 보드에는 아직 표시하지 않았다.

```text
object point (i, j) = (i * 0.020 m, j * 0.020 m, 0)
i = 0..5   한 행의 내부 코너 6개, +X
j = 0..8   행 9개, +Y
+Z = +X × +Y
```

원점은 `(0, 0)`인 그 내부 코너다. P-006의 180도 규칙은 영상에서 코너 순서를 고르는 규칙이다. 그 순서를 `base_link` 원점으로 쓰지 않는다. 촬영 전에 그 코너와 +X, +Y를 보드에 지울 수 있는 표시로 남겨야 한다. 표시가 없으면 어느 모서리인지 사진으로 정하지 않는다.

인쇄 여백을 20 mm로 두지 않는다. fixture 접촉 모서리에서 표시한 내부 코너까지의 거리는, 도구가 확인된 뒤에 재야 한다. 그 숫자는 아직 없다.

## 6자유도

P-006A가 남긴 것은 `base_link` 수직축과 x=0, y=0 중심선이다. yaw와 z=0은 `UNRESOLVED`다. 같은 폴더의 실물 대응은 금속 모서리, +X, +Y, z=0이 `NOT PHYSICALLY VERIFIED`다. URDF mesh 높이를 바닥으로 쓰지 않는다.

| DoF | 결정 방법 | physical datum | measurement tool | expected uncertainty |
|---|---|---|---|---|
| X | NOT_DEFINED | 실물로 확인된 base +X 원점이 없다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |
| Y | NOT_DEFINED | 실물로 확인된 base +Y 원점이 없다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |
| Z | NOT_DEFINED | z=0은 UNRESOLVED. 작업판은 base z=0이 아니다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |
| Roll | NOT_DEFINED | 보드 평면이 붙을 검증된 기준 평면이 없다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |
| Pitch | NOT_DEFINED | 위와 같다. 육안 평행은 쓰지 않는다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |
| Yaw | NOT_DEFINED | yaw는 UNRESOLVED. 로봇 앞쪽으로 보이면 0이 아니다 | 저장소에서 확인된 도구 없음 | NOT_ESTIMATED |

TCP로 보드를 짚지 않는다. TCP는 검증되지 않았고, 로봇은 움직이지 않는다. `DICT_4X4_50` ID 0 50 mm 마커는 검출 17/258로 실패했다. 쓰지 않는다.

## 그래서 하지 않는 설치

보드를 책상에 두고 줄자로 x, y, z, yaw를 적는 절차는 만들지 않는다. 직각자나 줄자는 P-006C-R2.5에 나중에 맞추라는 문장으로만 있다. 이 자리에서 그 도구가 있는지는 확인하지 않았다. 없는 캘리퍼스나 각도계를 전제로 치수를 적지 않는다.

구속이 생기려면, 확인된 길이 도구와 함께 다음이 먼저 있어야 한다.

1. 실물에서 식별된 `base_link` +X. 케이블 방향이나 로봇 앞쪽으로 대신하지 않는다.
2. `base_link` z=0으로 쓸 면. 작업판 표면은 그 면이 아니다.
3. 보드 평면이 그 기준에 기계적으로 닿아 roll과 pitch가 고정되는 접촉.
4. 표시된 내부 코너까지 잰 여백. 20 mm로 가정하지 않는다.

이 넷이 생기기 전에는 `^B T_T`를 계산하지 않고 Gemini를 켜지 않는다.

## 사진 계획

설치가 승인된 뒤에만 찍는다. 지금은 사진이 없다.

```text
01_base_datum_overview
02_checkerboard_origin_and_axes
03_x_y_datum
04_z_datum
05_yaw_constraint
06_roll_pitch_constraint
07_final_fixed_target
```

각 사진에는 어떤 datum인지 표시한다.

## 도식

```text
base_link B
  +Z  수직축 (P-006A, 중심선 x=0 y=0)
  yaw, z=0  비어 있음
        |
        |  ^B T_T   ← 이번 단계. 아직 null
        v
checkerboard T
  원점 = 표시할 내부 코너 (0,0)
  +X = i 방향, +Y = j 방향, +Z = X×Y
        |
        |  ^C T_T   ← solvePnP. 이번 단계 NOT_RUN
        v
camera_color_optical_frame C
  R4 작업 경로와 같은 프레임
```
