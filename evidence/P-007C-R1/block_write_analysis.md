# block-write semantics (스펙 §12)

## 질문

> 공식 DOFBOT-Pro 에서도 single-servo position command 가 `write_i2c_block_data` 를 사용하는가? YES/NO

## 답 — `NOT_DOCUMENTED_IN_OFFICIAL`

YES 도 NO 도 아니다. 공식 자료가 transport primitive 를 **전혀 기술하지 않는다.**

전수 판독한 공식 자료 14종에서 `write_i2c_block_data`·`smbus`·register·버스 번호·slave 주소가 **0회** 등장한다. 공식 PDF 는 Python API 층(`Arm_serial_servo_write(id, angle, time)`)에서 끝나고 그 아래를 "underlying driver source code … packaged into a python library"(`17. Basic control course/README.txt`)로 캡슐화한다.

유일한 I2C 전용 코스(`official/16-6-i2c-communication.pdf`)도 Jetson GPIO 핀에 OLED(`0x3C`)를 붙이는 범용 실습이다. `0x15` 서보 컨트롤러 프로토콜을 다루지 않는다.

## 🔴 순환논증 회피

"설치본 `Arm_serial_servo_write` 가 `write_i2c_block_data` 를 쓰므로 공식 경로도 그렇다"는 **쓸 수 없다.** 설치본이 공식본이라는 것 자체가 이 게이트에서 미증명이다(`installed_library_provenance.json` = `NO_OFFICIAL_MATCH_FOUND`). 설치본을 공식의 근거로 쓰면 증명하려는 것을 전제로 쓰는 것이다.

따라서 스펙 §12 의 "YES 이면 P-007B transport 선택 자체는 official path 와 일치"라는 추론은 **발동하지 않는다.**

## H-C5 에 미치는 영향

P-007C 의 `most_specific_finding` 은 이랬다:

> 조회(`0x01`·`0x38`·`0x30+id`)는 전부 `write_byte_data` 1바이트 + read 계열이고 모두 성공했다. 실패한 유일한 것은 `write_i2c_block_data` 4바이트(`0x12`)다.

이 관찰은 **설치본 내부에서는 그대로 유효**하다. 공식 자료는 이를 **지지하지도 반박하지도 못한다.** 그래서 H-C5 는 `OPEN` 으로 남는다 — 공식 근거로 좁히는 데 실패한 유일한 가설이다.

보강 정황 하나: 공식 `05-six-servo-sata-time.pdf` 예제는 단일 서보 명령을 `time.sleep(.001)` 간격으로 연속 발행한다. 공식 경로가 단일 위치 명령에 긴 지연이나 핸드셰이크를 요구하지 않는다는 뜻이다. 다만 이것도 API 층 근거이지 transport 근거가 아니다.
