# P-007F-03 — Claim Boundary (§20, 21)

## 향후 `YES + YES`(`PHYSICAL_MOTION_EVIDENCE_SUPPORTED`)가 나오더라도 주장 가능한 것

> Authorized Arm2 command와 시간적으로 대응하는 physical robot motion이
> independent external vision(Gemini Baseline B)에서 관측되었다.

## 아직 주장하지 않는 것

- physical Arm2 angle = controller value
- absolute angular accuracy
- servo encoder accuracy
- controller readback = physical shaft angle
- commanded 2° = physical 2°(또는 임의 commanded delta = physical delta)

이 구분은 P-007F-01(`evidence/P-007F-01/readback_semantics.md` —
`READBACK_SEMANTICS_UNRESOLVED`, H1/H2/H3 중 H3 확정)과 그대로 연결된다.
Channel B(Gemini 시각 witness)는 "physical motion이 있었는가"에 대한
독립 증거이지, "physical motion의 정확한 각도"에 대한 증거가 아니다 —
ORB+RANSAC 추정치(translation_px, rotation_deg)는 **픽셀 공간의 상대
변화량**이며 캘리브레이션(camera↔robot extrinsic, focal length 기반
mm/deg 환산) 없이 물리 각도로 직접 변환하지 않는다
(`evidence/P-007F-02B-R2/placement_decision.json.not_to_be_confused_with`
에 이미 명시된 "camera↔robot calibration 완료 아님"과 동일한 경계).

## Operator observation — provenance 분리 (§21)

향후 execution에서 operator observation을 기록하더라도:

```text
OPERATOR_OBSERVATION = SECONDARY
```

machine Channel B 결과보다 우선하지 않는다. R5 Attempt #4에서 발생한
"execution-time machine record(YES) vs 이후 operator correction(NO
MOTION)" 충돌(`evidence/P-007F-02/baseline.md`에 두 provenance 병렬
보존)을 반복하지 않도록, 향후 execution harness는 execution-time answer와
later correction을 **서로 다른 provenance field**로 저장해야 한다 —
하나를 다른 하나로 덮어쓰지 않는다.
