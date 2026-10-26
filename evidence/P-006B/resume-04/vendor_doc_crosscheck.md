# 문서와 설치 소스

판정의 우선순위는 2026-10-26에 다시 읽은 설치 소스다. 문서는 그 의미를 대조하는 데 썼다.

## 일치

설치 `Arm_serial_servo_write_offset_switch`는 id를 `0x1c`에 쓰고 값을 돌려주지 않는다. 검색으로 본 Yahboom 캘리브레이션 페이지도 이 함수가 현재 각을 MCU가 읽어 저장하고, id 0이 전부 지운다고 적는다. 직조회가 된 페이지는 그 HTML이 아니다.

설치 `Arm_serial_servo_write_offset_state`의 주석은 0, 1, 2다. 같은 검색 발췌의 반환 설명과 같다. 0은 서보를 찾지 못함, 1은 설정 성공, 2는 범위 초과다.

`https://www.yahboom.net/study/DOFBOT-Pro/12506/`는 이번에 본문을 가져왔다. Middle은 팔을 세운 상태다. 설치 `YahboomArm.py` 385행 명령 `07`은 `write6(90, 90, 90, 90, 90, 180, 1000)`이다. 주석은 서보를 중위로 보낸다는 뜻이다. 문서의 자세 이름과 코드의 중위 명령이 같은 방향을 가리킨다. 버튼 패킷을 캡처하지는 않았다.

`https://www.yahboom.net/study/DOFBOT-Pro/12502/`의 `x_offset`, `y_offset`, `z_offset`은 작업 공간 미터 보정이다. `arm_driver.py`와 `kinemarics_dofbotpro.cpp`에는 `0x1c`와 offset 문자열이 없다. 서보 중위와 섞이지 않는다.

## 불일치와 미확인

다음 URL은 2026-10-26 직조회에서 시간 초과이거나 404였다.

- `https://www.yahboom.net/public/upload/upload-html/1768294209/2.%20Calibrate%20the%20Robotic%20Arm.html`
- `https://www.yahboom.net/public/upload/upload-html/1777290469/2.Calibrate%20the%20Robotic%20Arm.html`
- `https://www.yahboom.net/public/upload/upload-html/1730714141/2.%20Calibrate%20the%20Robotic%20Arm.html`

검색 발췌는 위 함수 설명과 맞지만, 레지스터 번호는 그 발췌에 없다. `0x1c`와 `0x1b`의 버스 순서는 설치 소스만 근거로 둔다.

다른 SKU 페이지의 노트북 경로는 `/home/yahboom/Dofbot/...` 또는 `/home/jetson/dofbot_ws/...`다. 이 기기의 파일은 `/home/jetson/dofbot_pro/dofbot_ctrl/scripts/Dofbot_Calibration.ipynb`다. 절차 내용은 같고 경로는 다르다.

문서가 “상태 조회”라고 부르는 함수는 설치 코드에서 `0x1b`에 `0x01`을 먼저 쓴다. 이름만으로 순수 읽기라고 두지 않았다. 반대로 그 write만으로 곧장 캘리브레이션 트리거라고 단정하지도 않았다. 부작용이 저장을 바꾸지 않는다는 증명이 없어 하드웨어 조회는 승인하지 않았다.

LX-16A의 deviation read 명령은 이 컨트롤러 API에 없다.
