# 벤더 운동학

`kinemarics_dofbotpro.cpp`의 서비스 콜백이 컨트롤러 각도와 URDF 각도를 연결한다. FK는 다섯 관절 모두에 `(joint_degree - 90) * DE2RA`를 적용한 뒤 URDF로 KDL FK를 호출한다. `DE2RA`는 `pi/180`이다. IK 결과는 반대로 `(urdf_radian * 180/pi) + 90`이다. 호출 위치는 46행과 85~92행이다. 하드코딩된 URDF 경로는 설치된 `DOFBOT_Pro-V24.urdf`다.

`arm_driver.py` 98~110행은 같은 식을 관절 상태에 쓴다. 이름 순서는 `Arm1_Joint`부터 `Arm5_Joint`, 그 다음 `grip_joint`다. 발행 자체는 113행에서 주석 처리돼 있다. id 6만 먼저 `[30, 180]`을 `[0, 90]`으로 보간한 뒤 90을 뺀다. id 1~5에는 그 보간이 없다.

이 두 파일은 서로 같은 식이다. `q_urdf = (q_library_deg - 90) * pi/180`. sign은 이 식에서 +1이고, offset은 -90°다.

`grasp.py` 147~176행은 잡기 동작에서 joint5에 별도의 180/90 보정을 더한다. 이것은 URDF 변환이 아니라 응용 후처리다. 모델 convention으로 쓰지 않는다.

`arm_driver.py` 51행은 생성자에서 `Arm_serial_servo_write6(90, 90, 90, 0, 90, 30, 3000)`을 호출한다. 이 파일은 실행하지 않았다. 드라이버를 켜면 읽기 전에 팔이 움직인다.

DH 파라미터를 따로 적은 테이블은 없다. 기하학은 URDF를 KDL에 넣는 방식이다.
