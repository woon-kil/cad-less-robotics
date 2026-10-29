# P-007F-07 Phase A — §6/§7 Marker Mounting Instruction (PROPOSED, photo-based)

🔴 **이 문서의 좌표/박스는 `evidence/P-007F-05/run_1793274484/pre_frames/.../0000-color.png`
(1280x720, 상당히 저해상도·블러가 있는 프레임)를 4배 업스케일해 육안 판독한 결과다 —
픽셀 단위로 정밀하지 않다.** 사용자가 다음 단계에서 실제 로봇 사진/실물로 확인하기로
했으므로(원문: "marker 출력/부착 위치를 실제 사진 기준으로 확인"), 여기서는 **구조적 위치
설명**(어느 링크, 어느 joint 사이, 축에서 얼마나 떨어진 곳)을 1차로 제공하고, 픽셀 박스는
`marker_candidate_overlay.png`에 참고용으로만 표시한다.

## 로봇 구조 해석 (이미지 기준)

카메라 시점에서 아래에서 위로:

```text
[검은 사각 베이스 박스]  <- 고정 마운트, 움직이지 않음
   |
[흰색 원형 디스크/호른]  <- 회전 흔적이 보이는 joint 출력부(구체적으로 Arm1/Arm2 중
                           어느 축인지 사진만으로 100% 확정 못함 -- 아래 "불확실성" 참조)
   |
[짧은 대각선 검은 원통형 링크]  <- 다음 joint(회색 브래킷 클러스터)로 이어짐
   |
[회색 지그재그 브래킷 클러스터 + DaBai 카메라 + 그리퍼]
```

`ARM2_ROI`(기존, (650,220,120,70))는 이 중 흰색 디스크~대각선 링크 하단 일부를 포함하도록
정의되어 있었다(P-007F-02B-R2 원래 주석: "Arm1 원통형 몸체 바로 위 흰색 브래킷/피벗 구조").

## PRIMARY marker 후보 (A) — 권장

```text
위치: 흰색 원형 디스크/호른 표면, 회전 중심선에서 바깥쪽으로 오프셋된 지점
이유: (1) 평평한 원형 표면이라 marker 부착에 유리
     (2) joint 축 "바로 위"가 아니라 반경 방향으로 떨어진 지점을 택하면 오히려
         rotation witness로 더 좋다 -- 회전축 정중앙에 붙이면 회전해도 marker
         자체의 translation/orientation 변화가 거의 0이 되어 witness로서
         의미가 없다. 축에서 벗어난 지점일수록 같은 각도 회전에 대해 더 큰
         apparent displacement가 생긴다
피할 것: 디스크 중심(회전축), 디스크와 베이스 박스 사이의 이음새(joint seam),
        케이블 다발(디스크 우측에 다발로 보이는 부분)
```

## ALTERNATE marker 후보 (B)

```text
위치: 흰색 디스크와 상단 회색 브래킷 클러스터 사이의 짧은 대각선 원통형 링크
이유: 두 joint 사이의 순수 rigid link 구간이라는 점에서 "joint 축에 붙이지
     않는다"는 요구에 더 명확히 부합
단점: 구간이 짧고(사진상 폭 ~25-30px 추정) 원통형이라 평평한 marker 부착이
     어려울 수 있음 -- 실물에서 평평한 facet이 있는지 확인 필요
```

## SECONDARY marker 후보 (§7)

```text
위치: DaBai 카메라 하우징과 그리퍼 사이의 손목(wrist) 상단 하우징
판정: SECOND_MARKER = RECOMMENDED
이유: primary(Arm2 부근)와 충분히 떨어져 있어 "Arm2 marker 정지 + downstream만
     움직임"(M5, downstream-localized motion) 패턴을 구별하는 데 유용. FOV 내
     위치이며 현재 ARM2_ROI/END_EFFECTOR_ROI 와 겹치지 않아 occlusion 악화
     우려가 크지 않음
피할 것: 그리퍼의 움직이는 손가락 부분(soft/moving), DaBai 하우징 자체(센서
        모듈, 부착 금지)
```

## 공통 금지 사항 (work order §6 재확인)

```text
- joint 축 자체에는 붙이지 않음 (회전 중심선 바로 위 금지, 반경방향 오프셋 필수)
- cable/soft material에는 붙이지 않음
- downstream free-moving component(그리퍼 손가락 등)에는 붙이지 않음
```

## 불확실성 공개

이미지 해상도·블러로 인해 다음은 확정할 수 없다:

- 흰색 디스크가 정확히 Arm1 축인지 Arm2 축인지(둘 다 같은 베이스 클러스터에 인접)
- 대각선 링크(후보 B)에 실제로 평평한 marker 부착면이 있는지
- 후보 A/B/SECONDARY 각각의 Gemini Baseline B 대비 실제 정면성(frontality)

→ **실제 부착은 사람이 로봇을 직접 보고 PRIMARY(A) 우선, 어려우면 ALTERNATE(B)로
결정**한다. 이 문서는 그 결정을 위한 1차 제안이다.

## 참고 이미지

```text
marker_candidate_overlay.png
```
