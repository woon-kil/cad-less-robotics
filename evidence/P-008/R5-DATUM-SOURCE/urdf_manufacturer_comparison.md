# URDF와 제조사 파일

기준은 바꾸지 않았다.

```text
URDF = .cache/jetson_robot_model/DOFBOT_Pro-V24.urdf
SHA256 = b626a845cd67998cd5ce47f2b3d302d9cb3e5b9da764d9a1e7e1404a277ccc28
Arm1 origin = 0 0 0.0895
Arm1 axis = 0 0 1
mesh bottom z = -0.015 m
```

공식 Drive URDF sha256은 `d55d542697546d90644158c474d98974802aec055c31ba2a20d43b312c4c57f1`이다. 바이트가 다르다.

같은 값:

```text
Arm1_Joint origin 0 0 0.0895
Arm1_Joint rpy 0 0 0
Arm1_Joint axis 0 0 1
base_link visual origin 0 0 0
base_link 가 root
```

다른 값:

```text
mesh package
  공식 package://DOFBOT_Pro-V24/meshes
  계획 package://dofbot_pro_description/meshes

Arm5_Joint axis
  공식 0 -1 0
  계획 0 1 0

Gripping_point origin z 텍스트
  공식 0
  계획 0.00
```

Arm1 숫자는 공식 URDF와 CSV에 다시 나온다. STEP 형상과는 비교하지 못했다. 바닥 15 mm, 폭, 길이, 구멍 위치는 제조사 도면에서 확인하지 못했다.

```text
URDF_TO_MANUFACTURER_MODEL_MATCH = SUPPORTED
```

모양만 보고 MATCH로 두지 않았다. STEP를 열지 못했으므로 VERIFIED가 아니다.
