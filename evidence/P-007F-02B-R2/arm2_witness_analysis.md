# P-007F-02B-R2 — Arm2 Physical-Motion Witness Suitability

🔴 Arm2 를 실제로 움직이지 않았다. 정적 frame(`frames/0000-color.png`,
`robot_closeup.png`)만으로 평가한다.

## 8.1 Link visibility

**양호.** Arm2(흰 피벗 브래킷)와 그 하류 구조(대각선으로 뻗은 upper arm,
wrist, gripper, DaBai)가 R1보다 뚜렷이 구분된다(`robot_visibility.md`).

## 8.2 Foreshortening

**R1 대비 개선.** 팔 전체가 base에서 위쪽-좌측으로 **뚜렷한 대각선**을
그리며 뻗어 있다 — R1의 거의 수직 압축(top-down에 가까운 시야)과 달리,
이번 구도는 Arm2의 pitch 회전평면이 image plane에 더 가깝게 놓인 것으로
보인다(정성적 판단, 각도 수치화하지 않음 — Arm2를 실제로 움직이지 않았으므로
추론). 팔 길이가 화면에서 상당한 픽셀 거리로 투영되어 있어(약 base~
(700,310)에서 그리퍼~(650,150)까지), 작은 회전도 횡방향 변위로 나타날
여지가 R1보다 커 보인다.

## 8.3 Self-overlap

**감소했으나 완전히 제거되지는 않음.** Arm1/Arm2 경계는 뚜렷해졌지만,
Arm3/Arm4/wrist 영역은 여전히 서로 인접해 겹친다(§7 robot_visibility.md
PARTIAL 판정). "어느 관절이 정확히 움직였는지"를 vision만으로 완전히
분리하기는 여전히 어렵다 — 다만 "클러스터 전체가 움직였는가"를 감지하는
목적에는 지장이 적다.

## 8.4 Background separation

**부분적으로 혼재.** 그리퍼/팔 하단부는 밝은 이불(흰색) 배경과 대비가
뚜렷하다. 반면 DaBai/그리퍼 상단부 우측(x≈780-900 부근)은 어두운 모니터
베젤 등 상대적으로 어두운 배경과 겹쳐 국소적으로 대비가 약하다 — 이 부분은
한계로 공개한다.

## 8.5 Trackability

**양호.** 나사머리·관절 경계·DaBai 렌즈·그리퍼 손가락 등 다수의 국소
특징점이 뚜렷이 보인다. marker 없이 edge/corner 기반 tracking이 가능할
것으로 보인다(정성적 판단, 실제 ORB 실행은 하지 않았다).

## 종합 판정

```text
ARM2_WITNESS_GOOD
```

**근거**: link visibility·foreshortening·trackability 세 항목이 R1 대비
뚜렷이 개선됐고, self-overlap·background separation 은 완전하지 않지만
부분적 한계 수준(전체를 막는 수준이 아님)이다. 다섯 항목을 종합할 때 R1의
`MARGINAL`과 분명히 구분되는 수준의 개선으로 판단해 `GOOD`으로 판정한다.
단, 잔존 한계(Arm3/Arm4 self-overlap, 우상단 배경 대비 약함)는 숨기지 않고
위에 그대로 남긴다.
