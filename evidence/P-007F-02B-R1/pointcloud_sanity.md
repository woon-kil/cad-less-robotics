# P-007F-02B-R1 — Point Cloud Sanity

🔴 정밀 reconstruction 이 목적이 아니다. depth_info K(fx,fy,cx,cy)로 직접
역투영한 point cloud 에 대한 sanity check만 수행했다(Open3D 등 전용 라이브러리
미사용, numpy만). 대표 scatter 시각화: `pointcloud_sanity.png`.

## 확인 항목

### table plane가 합리적으로 형성되는가

`pointcloud_sanity.png`의 우측(녹색~노란색 점군, Z 증가 방향으로 비스듬히
뻗는 형태)이 작업대 표면에 대응하는 것으로 보인다 — 오블리크 시야각에서
평면이 멀어질수록 Z 가 커지는 패턴과 정성적으로 일치한다. 정밀 평면 피팅은
하지 않았다.

### robot geometry가 point cloud에 존재하는가

존재한다 — depth_stats.json 의 robot ROI(depth 좌표 13:233,364:643)에서
77.4% 유효 깊이, median 811mm 로 로봇 형상이 포인트로 표현됨을 확인했다.

### 대규모 hole/dropout이 있는가

전체 hole ratio 15.2%(depth_stats.json). `depth_overview.png`에서 보듯
이 dropout 은 주로 **프레임 우상단의 먼 배경**(벽/문 부근)에 speckle 형태로
집중돼 있고, robot/table 근접 영역은 상대적으로 조밀하다(77~97% valid).
로봇·작업영역에 대해서는 "대규모" dropout 이라 부를 수준이 아니다.

### depth clipping 문제가 있는가

depth 값 범위 0.474~1.687m — 센서 통상 작동 범위 내로 보이며, 작업영역에서
clipping(최대치 saturation)이 집중되는 패턴은 관찰되지 않았다.

## 종합

point cloud 는 "정밀하지 않지만 기본 기하 구조(로봇·작업대·배경)가 올바르게
분리되어 있다"는 수준의 sanity 를 통과한다. 이 결과는 depth 품질이
`depth_stats.json`의 수치(특히 robot/workspace ROI 의 양호한 valid ratio)와
일관된다.
