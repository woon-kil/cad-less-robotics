# P-007F-02B-R1 — Arm2 Physical-Motion Witness Suitability

🔴 Arm2 를 실제로 움직이지 않았다. 정적 frame(`frames/0000-color.png`,
`robot_closeup.png`)만으로 평가한다.

## 평가 항목

### Arm2와 주변 링크의 윤곽 분리

**약함.** `robot_visibility.md`에서 이미 확인했듯, 현재 시야각에서 Arm1(요)·
Arm2(피치)·Arm3 가 투영상 강하게 겹친다(self-overlap). Arm2 **단독**의 윤곽을
다른 링크와 분리해 추출하기는 어렵다.

### Arm2 예상 회전 방향이 image plane에서 관측 가능한지

🔴 **구조적 우려.** Arm2 는 URDF 상 axis=(1,0,0)인 **pitch joint**(팔을
앞뒤/위아래로 드는 방향)다(`evidence/P-007F-02/torque_experiment_design.md`
에서 이미 확인한 사실 재인용, 새로 측정하지 않음). 현재 Gemini 시야각은
**상당히 가파른(거의 top-down에 가까운) 하향 시점**으로 보인다(정확한 각도는
수치화하지 않는다, §15 지시대로) — `pointcloud_sanity.md`의 점군 분포가 이를
뒷받침한다. pitch 회전이 일어나는 평면이 카메라 시선축에 가깝게 정렬된
시야각에서는, 그 회전이 image plane 상에서 **foreshortening**(링크 길이의
겉보기 변화)으로만 나타나기 쉽고, 뚜렷한 횡방향 변위로는 잘 나타나지 않을
수 있다. 이것은 실측(픽셀 변위량)이 아니라 기하학적 추론이다 — Arm2 를
움직여 확인하지 않았으므로 추론임을 명시한다.

### 다른 링크와 심한 overlap 여부

**있음.** 위 "윤곽 분리" 항목과 동일 — Arm1/Arm2/Arm3/wrist 가 밀집된
클러스터를 형성한다(현재 자세가 비교적 접힌 상태로 보이는 것도 한 원인일 수
있으나, 가파른 하향 시야각 자체가 구조적으로 겹침을 늘린다).

### base/background 대비

**양호.** 로봇(어두운 금속/플라스틱)과 나무 작업대(밝은 황색 톤) 사이
명도·색상 대비가 뚜렷하다 — `robot_closeup.png`에서 로봇 윤곽이 배경과
분명히 구분된다.

### tracking 가능한 texture/edge 존재 여부

**양호.** 로봇 표면에 나사머리·관절 경계·LED 등 다수의 국소 특징점이
보인다 — ORB류 feature tracker 가 붙잡을 만한 texture 는 충분해 보인다
(정성적 판단, 실제 ORB 실행은 하지 않았다).

### marker를 추가하지 않아도 feature tracking이 가능한지

**가능할 것으로 보인다**(정성적) — 표면 자체에 충분한 코너/엣지가 있다.

## 중요한 재해석 — 이 witness 가 실제로 묻는 질문

이번 gate(§0)의 목적은 "① DOFBOT physical-motion **external witness**"다.
R3~R5 의 Block 3(DaBai eye-in-hand) 선례와 달리, Gemini 가 external witness로
요구받는 것은 **Arm2 를 다른 링크와 분리해 식별하는 것**이 아니라, **Arm2 가
움직이면 그 하류 전체(Arm3~gripper)가 함께 움직이므로, 그 "덩어리"가
고정된 배경 대비 변위했는지를 감지하는 것**이다(Arm2 가 돌면 반드시 그
하류 전체가 함께 돈다 — 기구학적으로 자명). 이 재해석 하에서는 링크 간
개별 분리 실패가 치명적이지 않다 — "클러스터 전체의 변위 감지 가능성"이
핵심이고, 이는 "base/background 대비"·"texture 존재"가 양호하므로 가능성이
있다.

단, pitch 평면이 시선축에 가깝게 정렬된 문제(foreshortening)는 "클러스터
전체" 해석에도 여전히 적용된다 — 변위의 **겉보기 크기**가 작아질 수 있다는
우려는 남는다.

## 판정

```text
ARM2_WITNESS_MARGINAL
```

**근거**: 배경 대비·texture 는 양호해 "무언가 움직였다"는 대략적 감지는
가능할 것으로 보이나, (1) 현재 시야각이 pitch 평면과 시선축이 가까워
foreshortening 우려가 있고, (2) 링크 간 self-overlap 때문에 "정확히 어느
관절이 움직였는지"는 vision 만으로 구분할 수 없다. `ARM2_WITNESS_GOOD` 으로
올리려면 §15 가 금지한 "구체적 각도 수치" 없이도, **더 수평에 가까운(덜
가파른 하향) 시야각**으로 조정하면 pitch 평면이 image plane 에 더 가깝게
놓여 foreshortening 이 줄어들 것으로 기대된다 — `placement_decision.json`의
조정 권고 참조.
