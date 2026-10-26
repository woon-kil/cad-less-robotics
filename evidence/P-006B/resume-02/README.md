# P-006B-R2

Orin에 설치된 DOFBOT Pro 코드를 읽기만 해서, 컨트롤러 각도와 P-002 URDF 관절 좌표의 관계를 찾아 비교했다. 로봇은 움직이지 않았다. resume-01의 `[86, 150, 0, 18, 93]`으로 offset을 역산하지 않았다.

설치 드라이버는 읽기 순서를 `Arm1_Joint`~`Arm5_Joint`, 그리고 `grip_joint`로 적는다. ID6은 `Arm6_Joint`가 아니다.

운동학 서비스와 `arm_driver.py`는 같은 식을 쓴다.

`q_urdf = (q_library_deg - 90) * pi/180`

scale `pi/180`은 이 식과 URDF 한계가 서로 맞아 VERIFIED다. sign +1과 offset -90°는 그 코드가 명시한 후보이며, 실제 자세와 같다는 관측이 없어 CANDIDATE다.

`PHYSICAL_ARM1_ZERO_TO_URDF_ZERO = UNRESOLVED`

판정은 **DIAGNOSIS PASS**다.

`arm_driver.py` 생성자는 시작 직후 서보 여섯 개를 쓴다. 관절 상태를 얻으려고 그 드라이버를 켜면 안 된다. 캘리브레이션 노트북도 첫 셀에서 팔을 90° 자세로 보낸다. 실행하지 않았다.
