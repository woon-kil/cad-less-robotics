# Boundary rationale

네 경계의 미터 값과 그 근거 문장은 아직 없다. 테이블 전체를 ROI로 두지 않는다. 카메라 받침, DOFBOT 설치, 제어기, 전원, 충돌로 물체를 둘 수 없는 자리는 작업 영역에서 뺀다.

각 경계에 필요한 근거는 하나다. 그 선이 테이블 끝인지, 설치물 때문에 물체를 못 두는 선인지, 그 밖에는 향후 shoe last나 checkerboard를 두지 않을 선인지.

Attempt 5의 `FAR_PHYSICAL_WORKSPACE_LIMIT`는 far optical-Z 촬영이 카메라 받침 쪽에서 멈췄다는 기록이다. 그 프레임에는 포즈가 없고, 남은 면의 길이를 mm로 잰 값이 아니다. R2 경계 숫자로 쓰지 않는다.

정성 참조만 둔다. 측정이 아니다.

- `evidence/P-006/camera-target/attempt-05/frames/far-t01-00-color.png`
- `evidence/P-006/camera-target/attempt-05/frames/near-t01-00-color.png`

카메라 받침이 화면 위쪽에 보인다. 그 간격은 픽셀로 환산하지 않는다.
