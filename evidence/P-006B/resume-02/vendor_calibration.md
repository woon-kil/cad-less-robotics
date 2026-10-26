# 벤더 캘리브레이션

노트북 `/home/jetson/dofbot_pro/dofbot_ctrl/scripts/Dofbot_Calibration.ipynb`는 실행하지 않았다.

내용만 보면 절차는 이렇다. 셀 1이 `Arm_serial_servo_write6(90, 90, 90, 90, 90, 180, 1000)`으로 팔을 움직인다. 셀 2가 `Arm_serial_set_torque(0)`으로 토크를 끈다. 셀 3과 4가 `Arm_serial_servo_write_offset_switch(id)`로 중위 편차를 저장한다. 셀 5가 토크를 다시 켠다. 셀 6의 전체 초기화 호출은 주석이다.

저장 위치는 워크스페이스 YAML이 아니다. `Arm_Lib.py` 81행과 83행은 주소 `0x15`의 레지스터 `0x1c`에 id를 쓴다. id 0은 그 편차를 지우는 명령이다. 단위는 파일에 없고, 서보 중위 편차라고만 적혀 있다. 컨트롤러 안에 남는 값으로 보이며, `Arm_serial_servo_read`는 YAML 오프셋을 더하지 않는다.

`dofbot_pro_driver/config/offset_value.yaml`은 `x_offset`, `y_offset`, `z_offset` 미터 값이다. AprilTag 파라미터도 같은 세 축이다. 관절 각 오프셋이 아니다. `calibrate_offset.py`도 이 조사에서 실행하지 않았고, 관절 영점 파일로 쓰지 않는다.

`initial_positions.yaml`은 ros2_control fake system의 URDF 초기값이며 Arm1~Arm5와 `grip_joint`가 0이다. 하드웨어 중위가 아니다.

지금 컨트롤러에 0이 아닌 중위 편차가 저장돼 있는지는 읽지 않았다. 상태 조회 함수도 레지스터에 한 바이트를 쓴 뒤 읽으므로 호출하지 않았다. 저장값의 존재는 UNKNOWN이다.
