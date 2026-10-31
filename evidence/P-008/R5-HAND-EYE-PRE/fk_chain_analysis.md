# FK chain

기준 파일은 `.cache/jetson_robot_model/DOFBOT_Pro-V24.urdf`다. 수정하지 않았다.

`base_link`가 root다. 팔 체인은 `Arm1_Joint`부터 `Arm5_Joint`다. `Gripping_point_Joint`는 `Arm5_Link`의 fixed child다. `grip_joint`는 그리퍼이고 `Arm6_Joint`가 아니다.

같은 식 `q = (library_degree - 90) * pi/180`으로 다섯 관절을 넣으면 링크 자세가 계산된다. P-006B가 남긴 라이브러리 각도의 예는 id 1~5가 86, 150, 0, 18, 93이다. 이번 세션의 실시간이 아니다.

그 각도로 Arm5만 축을 `0 -1 0`으로 둔 계산과 비교하면 `Arm5_Link` 원점 위치는 같다. 자세 차이는 `2 * |q5|`다. q5가 3°이면 차이는 6°다.

이 계산은 모델 안의 차이다. 실물 자세와 맞는지 보지 않았다. TCP는 확인하지 않았다. `Gripping_point_Link`를 hand-eye 대상으로 쓰지 않는다.

```text
FK_CHAIN_VALID = UNRESOLVED
PHYSICAL_TCP_VERIFIED = NO
TCP_CALIBRATED = NO
```
