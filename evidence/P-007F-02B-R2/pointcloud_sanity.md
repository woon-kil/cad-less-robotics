# P-007F-02B-R2 — Point Cloud Sanity

🔴 정밀 reconstruction 이 목적이 아니다. depth_info K 로 직접 역투영(numpy,
Open3D 미사용). 대표 scatter: `pointcloud_sanity.png`.

## 확인 항목

- **table plane 존재**: `depth_overview.png`(시각화)에서 넓고 매끈한
  그라디언트 평면이 중앙에 뚜렷이 보임 — 작업대 표면으로 판단
- **robot point cloud 존재**: robot ROI 에서 75.4% valid depth(median
  746mm) — 형상 데이터 존재 확인
- **workspace usable**: table ROI 94.5% valid — 사용 가능
- **severe holes 여부**: 전체 hole ratio 16.7%(R1 15.2%와 유사), 주로
  로봇 자체의 복잡한 형상·먼 배경에 집중 — "심각한" 수준은 아님
- **clipping 여부**: depth 범위 0.47~1.6m, 센서 정상 작동 범위 내로 보임,
  뚜렷한 saturation 패턴 관찰 안 됨

## 종합

R1과 동등한 수준의 point cloud sanity를 통과한다 — depth 품질이 카메라
재배치로 악화되지 않았다.
