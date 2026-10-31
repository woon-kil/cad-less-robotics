# Arm5 validation

움직이기 전에 멈췄다. Arm5 명령은 0회다.

현재 라이브러리 각도는 id 1부터 86, 152, 0, 18, 93, 178이다. 세 번 읽기가 같았다. id 5의 URDF 각은 `(93 - 90) * pi/180` = 0.05236 rad다.

계획 축 `0 1 0`과 제조사 축 `0 -1 0`의 차이는 이 정지 각도에서 자세 약 6°다. 컨트롤러 읽기만으로는 어느 축이 실물과 맞는지 알 수 없다.

Jetson의 카메라는 Orbbec DaBai DCW2다. 팔에 달린 카메라이고, 관절 회전 방향을 보는 고정 시점이 아니다. Gemini는 이 게이트에서 쓰지 않았다.

```text
ARM5_CONVENTION_RESOLVED = NO
ARM5_PHYSICAL_AXIS_MATCHES = UNRESOLVED
```
