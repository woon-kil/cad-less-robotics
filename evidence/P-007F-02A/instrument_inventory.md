# P-007F-02A — Physical Measurement Instrument Inventory

🔴 **DESIGN / QUALIFICATION ONLY.** 로봇 상태 변경 0, 외력 적용 0, 카메라 위치
변경 0. 아래는 전부 **저장소/evidence 조회**로만 확인한 내용이다 — 어떤 물리
도구도 실제로 꺼내거나 장착하지 않았다.

## 방법

`grep -rli` 로 전체 `evidence/` 를 "smartphone/스마트폰/inclinometer/수평계/
protractor/각도계/digital level/caliper/ruler/tripod/삼각대"에 대해 검색했다.
매칭된 파일은 이번 작업의뢰서 자체가 참조한 `evidence/P-007F-02/*`
(이번 gate 이전에 내가 작성한 문서, §0 선례)와 `evidence/P-007E-R2-R1/decision.json`
(아래 D 항목) 뿐이었다 — **새로운 물리 계측 도구에 대한 기존 evidence는
없다.**

`evidence/P-006/hardware-inventory.txt`(p620 USB 전체 열거, lsusb 기반)와
`evidence/P-007E-R2-R1/sensor_inventory.json`(p620+Mac+Jetson 카메라 전수조사)
를 재확인했다 — 둘 다 카메라/오디오 USB 장치만 나열하고, 물리 계측 도구(각도계·
수평계·스마트폰)는 USB로 열거되는 대상이 아니므로 이 inventory들에서 존재/부재
어느 쪽도 확인되지 않는다.

## A. Smartphone inclinometer

```text
possession evidence    = NONE FOUND IN REPOSITORY
verdict                = OPERATOR_CONFIRMATION_REQUIRED
```

저장소에는 운영자가 스마트폰을 보유하는지, 보유한다면 기종(질량에 영향)·
inclinometer 앱 유무를 확인할 근거가 없다. 추정하지 않는다.

## B. Physical inclinometer / digital level

```text
possession evidence    = NONE FOUND IN REPOSITORY
verdict                = NOT VERIFIED
```

## C. Printed protractor + fixed external observation

```text
possession evidence (프린터·경질 pointer·독립 고정대) = NONE FOUND IN REPOSITORY
verdict                                             = OPERATOR_CONFIRMATION_REQUIRED
                                                        (재료 확보 필요, 방법론은 평가 가능)
```

프린터·경질(막대·철사 등) pointer·robot base와 독립된 고정 받침(삼각대·클램프
등)의 보유 여부도 저장소에서 확인되지 않는다. 다만 이 후보는 **방법론 자체를
평가하는 데** 보유 여부가 필수는 아니다 — `candidate_comparison.md`에서
독립성·분해능·하중 등을 재료 보유와 분리해 분석한다.

## D. Fixed external camera + fiducial/marker

```text
Gemini 335L  = NOT CURRENTLY USABLE (P-007F-02 camera_witness_analysis.md 재확인 — 변경 없음)
DaBai DCW2   = eye-in-hand, primary witness 금지(지시 명시)
Mac 내장 카메라 = evidence/P-007E-R2-R1/decision.json.primary_candidate 로 이미
               식별됐으나 blocking_unknowns(물리 배치가 로봇과 같은 공간인지,
               macOS TCC 카메라 권한, FOV)가 그 gate 이후 전혀 해소되지 않음
other fixed camera = 저장소 전체 카메라 전수조사(sensor_inventory.json, p620+Mac+
                     Jetson lsusb/SPCameraDataType)에 그 외 카메라 없음
verdict             = NO NEWLY-QUALIFIED FIXED EXTERNAL CAMERA
                      (Mac 카메라는 R2-R1 당시 그대로 미해결 상태 — 이 gate에서
                      재확인만 하고 새로 해소하지 않음, 카메라 재배치 0)
```

## 요약

| 후보 | 보유 evidence | 판정 |
|---|---|---|
| A. Smartphone | 없음 | `OPERATOR_CONFIRMATION_REQUIRED` |
| B. Physical inclinometer | 없음 | `NOT VERIFIED` |
| C. Protractor + fixed stand | 없음(재료) | `OPERATOR_CONFIRMATION_REQUIRED`(재료), 방법론은 평가 가능 |
| D. Fixed external camera | Gemini=unusable, Mac=미해결, 그 외 없음 | 신규 후보 없음 |
