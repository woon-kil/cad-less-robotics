# Arm1 reference

하드웨어 값을 읽지 못했다. p620에 DOFBOT 시리얼이 없고, 워크스페이스에 팔 드라이버 패키지가 없다.

raw, radian, ROS joint, sign, scale, unit은 비어 있다. 0을 측정값으로 적지 않는다. 0에 가깝다는 이유로 physical zero를 말하지 않는다.

`DOFBOT_Pro-V24.hardware.urdf`의 `offset="0"`은 그리퍼 mimic이다. `Arm1_Joint`의 sign, offset, scale이 아니다.

hardware Arm1=0이 URDF `Arm1_Joint=0`과 같은 yaw인지는 **NOT VERIFIED**다. 코드나 config에서 그 동일성을 보여주는 근거가 없다.

`A_AXIS`는 유지한다. 현재 각이 없으므로 `base_link` +X/+Y는 계산하지 않았다.
