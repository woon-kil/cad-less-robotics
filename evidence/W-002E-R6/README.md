# W-002E-R6 — ID2 설치 평면과 간섭

상태: `PASS_PHYSICAL_PLANARITY_VIOLATION_CONFIRMED / CLOSED`

관찰만 한다. ID2를 누르거나 옮기지 않았다. PnP는 다시 풀지 않았다. ADR-010은 소급하지 않았다. 이 합격은 등록 합격이 아니고, W-002E의 원인을 확정하지 않는다.

사진이 없던 첫 기록은 `INCONCLUSIVE_PHYSICAL_INSPECTION`이었다. 이어서 도착한 원본 두 장을 그대로 보관하고 그 사진으로 다시 보았다.

## 사진에서 보인 것

측면 사진에서 시트 아래에 어두운 틈이 있다. 검은 인쇄는 그 시트 위에 있다. 시트는 휘어 보이기보다 한쪽으로 기울어 보인다. 높이는 측정하지 않았다. 사진으로 mm를 추정하지 않았다.

사선 사진에서 흡착판 원판은 흰 테두리 모서리 아래에 겹쳐 보인다. 측면에서는 종이 끝과 흡착판이 붙어 있는지가 분명하지 않다. 그래서 접촉은 `UNRESOLVED`다. 흡착판은 검은 사각형 밖에 있다.

코너 번호는 W-002E-R4와 같다. 0, 1, 2, 3이 TL, TR, BR, BL이다. 이 사진에서 그 번호를 작업판 코너에 맞추지 못했다. 네 코너의 접촉과 들뜸은 `UNRESOLVED`, `NOT_MEASURED`다.

W-002E 모델의 ID2 object point z는 모두 0 mm다. 검은 영역이 있는 시트가 나무 면에서 떠 있으므로 평면 가정은 `VIOLATED`다. 종이 끝만 휘었다는 이유로 그렇게 둔 것은 아니다. 코너별 mm는 없다.

## 원인으로 쓰지 않는 것

leave-one-out fit 평균은 그대로다. ID0 제외 1.851 px, ID1 제외 1.910 px, ID2 제외 1.629 px, ID3 제외 1.639 px다. ID3를 빼도 fit이 비슷하므로 이것만으로 ID2를 원인으로 정하지 않는다.

R5의 signed offset -2.340 mm와 이 들뜸을 같다고 두지 않는다.

`PHYSICAL_ROOT_CAUSE_CANDIDATE = ID2_NONPLANAR_INSTALLATION`  
`ROOT_CAUSE_CONFIDENCE = INCREASED`  
`ROOT_CAUSE = INCONCLUSIVE`  
`W002E_ROOT_CAUSE = NOT_YET_CONFIRMED`

## 파일

- `result.json`
- `id2_oblique_original.jpg`
- `id2_side_original.jpg`
- `id2_side_annotated.png` — 측면 원본에 틈이 보인다는 문구만 올린 복사본. 높이는 적지 않았다.
