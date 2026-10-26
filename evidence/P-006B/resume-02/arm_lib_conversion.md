# Arm_Lib 변환

파일은 `/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py`다. SHA-256은 `edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c`이고, resume-01에서 본 설치본과 같다. 패키지 버전은 `Arm_Lib` 1.0.3이다.

세 층을 구분한다.

A. 전기/컨트롤러 변환. `Arm_serial_servo_read`는 레지스터 `0x30+id`를 읽고 펄스를 정수 각도로 바꾼다. id 5는 380~3700을 0~270°로, 그 외는 900~3100을 0~180°로 바꾼다. 해당 식은 205행과 209행이다.

B. 라이브러리 각도 안의 채널 반전. 213행은 id 2, 3, 4에만 `180 - pos`를 적용한다. 쓰기 함수는 같은 id에서 펄스로 바꾸기 전에 요청 각도를 180에서 뺀다. 이 쌍은 읽고 쓰는 정수 각도를 맞추는 라이브러리 규칙이다.

C. URDF 관절 좌표. Arm_Lib는 radian도 URDF 축도 만들지 않는다. 그 변환은 `dofbot_pro_info`와 `arm_driver.py`에 있다.

`180 - pos`만으로 URDF sign을 -1로 두지 않는다. 드라이버가 사용하는 값은 이미 이 반전이 들어간 라이브러리 각도다.
