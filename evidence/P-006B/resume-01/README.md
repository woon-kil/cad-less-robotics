# P-006B resume-01

실제 DOFBOT Pro를 움직이지 않고, 하드웨어가 읽어 주는 값과 P-002 모델의 `Arm1_Joint`~`Arm5_Joint`가 어디까지 연결되는지 확인한 기록이다.

p620 USB에는 팔 시리얼 장치가 없다. 카메라 `2bc5:0804`만 있다. 서보 버스는 로봇 위의 Jetson Orin I2C `/dev/i2c-7`, 주소 `0x15`다. 읽기 함수는 `Arm_Lib` 1.0.3의 `Arm_serial_servo_read`다. ROS `/joint_states`는 만들지 않았다. `arm_driver`와 `/TargetAngle`은 각도를 쓰는 경로라 시작하지 않았다.

100라운드 동안 라이브러리가 반환한 정수 각도는 아래처럼 고정이었다.

| servo id | 보고 각도 |
| --- | --- |
| 1 | 86 |
| 2 | 150 |
| 3 | 0 |
| 4 | 18 |
| 5 | 93 |
| 6 | 178 |

이 숫자는 컨트롤러 레지스터를 환산한 각도다. 인코더임이 증명된 값은 아니고, URDF radian도 아니다. id 1~5를 `Arm1_Joint`~`Arm5_Joint`에 대응시키는 순서는 강좌의 아래→위 설명과 같으나, 움직임을 만들지 않았으므로 `UNVERIFIED`다. id 6은 그리퍼 채널이며 `Arm6_Joint`가 아니다.

`PHYSICAL_ARM1_ZERO_TO_URDF_ZERO = UNRESOLVED`

판정은 **DIAGNOSIS PASS**다. 읽을 수 있는 값의 종류는 확인됐고, calibration reference로는 부족하다.

기존 `evidence/P-006/joint-reference/`는 수정하지 않았다. P-006C-R2, R2.5, P-007도 건드리지 않았다.
