# P-007F-06 — §23 Controller 154→152 Review

## 사실

```text
PRE      = 154
COMMAND  = 151
POST     = 152
```

Controller readback이 변했고, commanded target(151)과도 2° 다르다.

## 금지하는 해석

```text
physical shaft moved 2°
```

이 표현은 사용하지 않는다. P-007F-01의 결론:

```text
READBACK_SEMANTICS = UNRESOLVED
```

는 이번 gate에서도 그대로 유지한다 — controller-reported 값이 실제 shaft 각도와 1:1 대응한다는
근거는 여전히 없다(raw/readback 버킷 불확실성, encoder vs 명령값 해석 등 P-007F-01에서 공개한
미해결 쟁점 전부 유효).

## 이번 forensic 결과와의 독립적 취급

이 gate(P-007F-06)의 모든 영상 forensic 분석(§5~§24)은 controller 값을 **전혀 입력으로 쓰지
않았다** — ROI 좌표, within-window noise 비교, spatial localization, depth 분석 모두 PRE/POST
RGB+Depth 픽셀만 사용했다. 따라서:

- controller가 "변했다"는 사실이 영상 forensic 결과(F2)의 신뢰도를 높이거나 낮추지 않는다
- 영상 forensic 결과(F2)가 controller 값의 물리적 의미를 해석해주지도 않는다

두 채널은 P-007F-05에서 이미 사전등록된 대로 계속 독립적으로 취급된다.
