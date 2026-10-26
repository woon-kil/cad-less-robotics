# P-007A — 읽기 전용 bring-up

Orin의 `/dev/i2c-7`, 주소 `0x15`에서 `Arm_serial_servo_read`로 ID1부터 ID6을 100라운드 읽었다. 소스 해시는 P-006B와 같았고, 600개 각도는 변하지 않았다.

판정은 **BLOCKED**다. 통신이 깨져서가 아니다. `YahboomArm.pyc`와 `joystick.py`가 같은 I2C 장치를 연 채 서보 위치 쓰기를 할 수 있다. `joystick.py`의 시작 코드는 `move_init_pose()`를 호출한다. 데스크탑 자동시작 `start_app.sh`가 그 둘을 띄운다. `arm_driver.py`는 실행하지 않았고, 떠 있지도 않다. 생성자가 `write6`을 부르므로 계속 금지다.

후보 관절은 `candidate_urdf_joint_state`다. 측정 관절이나 보정된 TCP가 아니다. H0 식은 벤더 convention이고, 정밀 보정은 아니다.

작업대와 전원 스위치는 이 세션에서 확인하지 않았다. P-007B는 열지 않는다.

| 항목 | 상태 |
|---|---|
| 읽기 | 600/600, error 0 |
| 라이브러리 각도 | 86, 150, 0, 18, 93, 178 |
| 후보 URDF | 한계 안 |
| 이번 게이트가 내린 이동 명령 | 0 |
| 동시 서보 쓰기 프로세스 | 있음 |
| P-007B | NOT OPEN |
