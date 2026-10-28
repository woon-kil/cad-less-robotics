# P-007F-02B-R1 — Occlusion Analysis

## Robot self-occlusion

```text
HIGH
```

`arm2_witness_analysis.md`에서 이미 다룬 Arm1/Arm2/Arm3/wrist 클러스터의
강한 self-overlap이 그대로 근거다 — 현재 시야각(가파른 하향)에서 상부 링크들이
서로를 상당 부분 가린다.

## Robot→workspace occlusion

```text
LOW
```

로봇 bbox(5.4%)가 작업영역(16.9%)·테이블(25.8%) 대부분을 가리지 않는다 —
`workcell_coverage.md` 수치 재인용. 로봇이 작업영역 뒤쪽(프레임 상단)에
위치해 전면 작업공간을 가리지 않는 구도다.

## Workspace→robot relationship

```text
ACCEPTABLE
```

작업영역과 로봇의 상대 위치(로봇이 작업영역 뒤쪽/위쪽)가 한 프레임 안에서
자연스럽게 파악된다 — 둘 사이 공간적 관계를 읽는 데 지장이 없다.

## 종합

세 항목 중 유일한 `HIGH`는 **self-occlusion**이며, 이는 Arm2 witness
suitability(`MARGINAL`)의 핵심 근거와 동일하다. robot→workspace,
workspace→robot 관계는 양호해 §9(workcell coverage) 목적에는 현재 위치가
적합하다 — 조정이 필요하다면 self-occlusion 완화(시야각)에 집중하면 된다.
