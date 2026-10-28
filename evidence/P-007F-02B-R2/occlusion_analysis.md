# P-007F-02B-R2 — Occlusion Analysis

## robot self-occlusion

```text
ACCEPTABLE
```

R1의 `HIGH`에서 개선됐다 — Arm1/Arm2 는 이제 명확히 분리되어 보이고,
Arm3/Arm4/wrist 영역만 부분적으로 겹친다(완전한 `LOW`는 아님 — 일부
잔존 overlap 있음, `arm2_witness_analysis.md` §8.3 참조).

## robot → workspace occlusion

```text
LOW
```

로봇(5.8%)이 작업영역(32.6%)·테이블(43.8%)을 가리는 비중이 작다 —
R1과 동일한 결론.

## camera → table visibility

```text
ACCEPTABLE
```

체커보드가 포함된 작업영역 전체가 가려짐 없이 보인다. 테이블 가장자리까지
연속적으로 보여 작업면 전체를 시각적으로 추적할 수 있다.

## 종합

세 항목 모두 `LOW`~`ACCEPTABLE` 범위 — R1에서 유일하게 `HIGH`였던
self-occlusion 이 `ACCEPTABLE`로 개선된 것이 이번 위치의 핵심 개선점이다.
