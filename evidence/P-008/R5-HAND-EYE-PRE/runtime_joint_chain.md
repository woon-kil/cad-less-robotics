# Runtime joint chain

로봇은 움직이지 않았다. `arm_driver`는 켜지 않았다. 그 파일의 생성자는 서보 여섯 개를 쓴다.

이번 세션에서 서보를 다시 읽지 않았다. 각도 표는 기존 읽기 기록이다.

## 사슬

```text
servo register
  → Arm_Lib.Arm_serial_servo_read
  → library degree
  → kinemarics_dofbotpro.cpp / arm_driver.py
  → q = (degree - 90) * pi/180
  → 설치된 DOFBOT_Pro-V24.urdf 의 KDL FK
```

`Arm_Lib`는 radian과 URDF 축을 만들지 않는다. id 2, 3, 4만 라이브러리 각도에서 `180 - pos`를 적용한다. id 1과 id 5에는 그 반전이 없다.

드라이버와 운동학 서비스는 다섯 관절에 같은 식을 쓴다. 식의 부호는 +1이고, 영점 이동은 -90°다. id 5만 부호를 뒤집는 항은 없다.

`arm_driver.py`의 joint 이름 순서는 `Arm1_Joint`부터 `Arm5_Joint`, 다음이 `grip_joint`다. 그 발행 줄은 주석이다. `/joint_states`를 이번 세션에 구독하지 않았다.

계획 모델과 Jetson에 설치된 파일은 sha256 `b626a845cd67998cd5ce47f2b3d302d9cb3e5b9da764d9a1e7e1404a277ccc28`로 같은 관절 축을 가진다. Drive에서 받은 공식 URDF는 다른 sha256이고, 그 파일이 런타임에 로드된다는 기록은 없다.

`grasp.py`의 joint5 보정은 잡기 후처리다. 모델 convention으로 쓰지 않는다.
