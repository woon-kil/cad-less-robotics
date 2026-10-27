# controller read semantics (스펙 §18)

자료: `official/04-read-servo-position.pdf` (Orin판 `17. Basic Control Course/4.`, blob `45ccfa7b…`, 2페이지 전문 판독)

## 공식 원문

> `Arm_serial_servo_read(id)`
> Function: **Read the current angle value of the bus servo.**
> id: … ranging from 1 to 6. The ID of the servo at the bottom is 1, and it increases upwards.
> Return value: The current angle of the corresponding ID servo. When ID=5, the angle range is 0~270, and the others are 0~180.

## 분류

| 후보 | 판정 |
| --- | --- |
| servo actual position (encoder feedback) | 공식 근거 불충분 |
| controller-reported position | **유지** |
| commanded position | 공식 근거 불충분 |
| unknown | — |

공식 문구 "the current angle value of the bus servo" 는 자연어로는 실제 각도를 시사하지만, **commanded 와 feedback 을 구분하는 문장이 없다.** 측정원(엔코더·포텐셔미터)·갱신 주기·명령 중 읽었을 때의 거동이 전부 미기술이다.

스펙 §18 은 "공식 근거가 충분하지 않으면 기존 `CONTROLLER_REPORTED_STATE` 유지"를 지시한다. → **`CONTROLLER_REPORTED_STATE` 유지. encoder feedback 으로 승격하지 않는다.**

이 판정은 P-007B 의 해석에 직접 영향을 준다. P-007B 에서 쓰기 전후 readback 이 변하지 않은 것은 "서보가 안 움직였다"의 **보강 정황**일 뿐이고, 컨트롤러가 명령 반영 전 값을 돌려준 경우와 구분되지 않는다. 물리 무동작의 1차 증거는 운영자 직접 관찰("전혀 움직이지 않았다")이다.

## 🔴 1차 세션 기록 정정 — `89` 스크린샷

중단 시점 README §5 는 이렇게 적었다:

> 같은 문서의 공식 실행 화면이 `90 90 90 90 89 178` 을 출력한다. 네 번째까지 90인데 ID5가 89다. P-007B 가 소스에서 유도한 `int()` 절단 ±1 편향과 같은 성격의 현상이 공식 스크린샷에도 나타난다.

**이 서술을 철회한다.** 전문을 읽은 결과 그 화면은 `04.read_servo.ipynb` 의 **읽기 루프 단독 출력**이고, 같은 실행에 **commanded 값이 없다.** 비교 기준이 없으므로 `89` 를 "90 을 명령했는데 89 가 읽혔다"로 해석할 수 없다. 팔이 그 시점에 어떤 자세였는지도 문서에 없다.

`int()` 절단 편향은 **설치본 소스의 정적 분석 결과로서는 그대로 유효**하다(P-007C). 공식 스크린샷은 그 교차확인이 되지 못한다. 근거 등급을 `CROSS_CONFIRMED` 에서 `NOT_EVIDENCE` 로 내린다.
