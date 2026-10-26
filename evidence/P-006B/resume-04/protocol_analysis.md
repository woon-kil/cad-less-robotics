# 중위 저장값 조회

질문 하나다. 지금 MCU에 저장된 servo neutral 편차의 숫자를, 로봇을 움직이지 않고 읽을 수 있는가.

답은 없다. `Arm_serial_servo_read`가 돌려주는 것은 현재 라이브러리 각이지, 저장 편차가 아니다.

## 설치된 함수

파일은 `/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py`다. SHA-256은 `edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c`로 resume-02와 같다. 이번 게이트에서 `Arm_Device`는 만들지 않았고 I2C도 열지 않았다.

`Arm_serial_servo_write_offset_switch`는 78행이다. 주소 `0x15`의 레지스터 `0x1c`에 id를 쓴다. id 1에서 6은 설정, id 0은 `0x00`으로 초기화다. 소스 주석은 전원을 켠 뒤 중위로 옮기고 이 함수를 보내라고 적는다. 반환값은 없다. 공식 설명도 같다. MCU가 그 서보의 현재 각을 읽어 범위 안이면 저장한다. 이 함수는 조회가 아니다.

`Arm_serial_servo_write_offset_state`는 89행이다. 레지스터 `0x1b`에 `0x01`을 쓴 다음 한 바이트를 읽는다. 주석의 반환은 0, 1, 2다. 0은 해당 ID를 찾지 못함, 1은 설정 성공, 2는 범위 초과다. 각도도 편차 크기도 없다. 직전 설정 동작의 상태다. 쓰기 한 번이 앞에 있으므로 이번 게이트에서 호출하지 않았다. 호출해도 저장 숫자를 얻지 못한다.

클래스 안의 다른 함수 이름에도 저장 편차를 읽는 항목은 없다. `Arm_serial_servo_read`와 `Arm_serial_servo_read_any`는 현재 각이다.

`/home/jetson/dofbot_pro`와 `/home/jetson/dofbot_pro_ws/src`에서 `read_offset`, `ANGLE_OFFSET` 문자열은 0건이다. `write_offset`을 호출하는 파이썬은 `Arm_Lib.py`, 그 체크포인트, `YahboomArm.py`, `Dofbot_Calibration.ipynb`와 그 체크포인트뿐이다. OpenCV 노트북과 `magic_enum.hpp`의 `0x1b`는 이미지 데이터와 상수 테이블이다.

LX-16A 계열의 deviation read 명령은 이 트리에 없다. DOFBOT Pro 경로는 I2C 주소 `0x15`의 MCU 레지스터다.

## APP이 말하는 중위

`YahboomArm.py` SHA-256은 `adc754ae275be8e0004fa4c34d95fa7432d7050d9dfaa911b2dfde64e6efb19e`다. 이번 게이트에서 실행하지 않았다. 부팅 때 이미 떠 있던 프로세스를 재시작하지도 않았다.

명령 `07`은 385행에서 `Arm_serial_servo_write6(90, 90, 90, 90, 90, 180, 1000)`이다. 주석은 서보를 중위로 보낸다는 뜻이다. 이 호출은 팔을 움직인다. 실행하지 않았다.

공식 APP 문서의 Middle은 “Robotic arm stands upright”다. 출처는 `https://www.yahboom.net/study/DOFBOT-Pro/12506/`다. 설치 코드의 중위 명령이 라이브러리 90° 다섯 축과 그리퍼 180°다. ID6은 `grip_joint`이지 `Arm6_Joint`가 아니다.

명령 `0A`는 `write_offset_switch` 뒤에 `write_offset_state`를 부르고, 소켓으로 돌려주는 것도 id와 state다. 저장 각도가 아니다.

같은 문서 2항은 초기화가 끝나면 팔이 initial posture로 회전한다고 적는다. 이번에 재부팅하지 않았다. 이후 bring-up에서는 사용자 명령, 벤더 기동, 서비스 기동, 캘리브레이션, MoveIt 명령을 서로 다른 동작으로 적어야 한다. `arm_driver` 생성자가 `write6`을 부르는 사실은 resume-02에 있다.

## 두 캘리브레이션

서보 중위는 MCU 편차다. 3D 작업 공간 보정은 다른 것이다. `https://www.yahboom.net/study/DOFBOT-Pro/12502/`는 보드 위 알려진 위치 `(0, 0.181, -0.01)`과 비전 결과를 비교해 `x_offset`, `y_offset`, `z_offset`을 yaml에 저장한다. 관절 각이 아니다. resume-02가 둘을 섞지 않은 판단을 유지한다.

## 공식 HTML

`https://www.yahboom.net/public/upload/upload-html/1768294209/2.%20Calibrate%20the%20Robotic%20Arm.html`과 `1777290469`, `1730714141` 직조회는 시간 초과이거나 404였다. 레지스터 동작은 그 HTML이 아니라 위 설치 파일로 판정했다. 검색에 나온 같은 제목 페이지의 API 설명은 설치 주석과 같다. 중위 설정의 반환은 0, 1, 2이고, 조정 결과는 팔을 세우는 것이다.

## H0와 H2

H0 식 `q = (library_deg - 90°) * π/180`는 resume-02의 FK, IK, `arm_driver.py`와 같다. 이번 조사는 그 90°가 벤더 중위 명령과 같은 숫자임을 설치 APP 코드에서 확인했다. 그래서 H0는 소프트웨어 convention으로서 벤더 정의다. 등급은 P3다. P4가 아니다.

이 기기의 MCU에 0이 아닌 편차가 들어 있는지는 여전히 모른다. 현재 라이브러리 각도 `[86, 150, 0, 18, 93]`으로 그 편차를 역산하지 않는다. 그 자세는 중위가 아니다.

H2의 전체 부호 반전은 설치 트리와 이번 공식 조회에서 근거가 없다. 대등한 후보로 두지 않는다.

정밀 관절 보정은 검증하지 않았다. P-006은 PASS가 아니고 P-007은 열지 않았다.
