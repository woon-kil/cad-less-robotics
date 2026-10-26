# P-006B resume-01 소스 안전 검토

검토 시각은 읽기 실행 전이다. 대상은 Orin에 설치된 `Arm_Lib` 1.0.3 egg와 같은 내용의 `/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py`다. 패키지 `__init__.py`는 `Arm_Device`를 import할 뿐이며, import 시 객체를 만들거나 각도를 쓰지 않는다.

`Arm_Device.__init__`는 I2C 주소 `0x15`, 기본 버스 7로 `smbus.SMBus`만 연다. 생성자에 각도 쓰기, 홈, 리셋, 토크 설정, 데모 동작은 없다.

`Arm_serial_servo_read`는 레지스터 `0x30+id`에 값 `0x00`을 쓰고 같은 레지스터의 워드를 읽은 뒤 각도로 환산한다. 위치 명령은 다른 레지스터를 쓴다. `Arm_serial_servo_write`는 `0x10+id`, 6축 쓰기는 `0x1e`와 `0x1d`다. 이번 호출은 조회 레지스터만 사용했다.

공식 노트북 `04.read_servo.ipynb`의 3번 코드 셀은 `Arm_serial_servo_write(6, 150, 500)`을 호출한다. 노트북 전체는 실행하지 않았고, 그 쓰기 함수도 호출하지 않았다.

`ros2 run dofbot_pro_driver arm_driver`와 `/TargetAngle` publish는 하지 않았다. MoveIt launch도 하지 않았다. `setup_i2c_clk_rate.py`는 egg 안에 있으나 import 경로에서 호출되지 않아 실행하지 않았다.

읽기 전에 `YahboomArm.pyc`가 이미 `/dev/i2c-7`을 열고 있었다. 이 프로세스는 이번 Gate가 시작하지 않았고, 멈추지도 않았다. 조회 트랜잭션은 커널 I2C 전송 단위로 나간다. 600회 조회 동안 보고 각도는 채널마다 한 값으로 고정되어 있었고, 위치 쓰기 호출 수는 0이다.

판정: import, 생성, `Arm_serial_servo_read`는 이번 소스에서 초기 자세 명령이 아니다. 쓰기 API와 ROS 드라이버는 실행 금지로 유지했다.
