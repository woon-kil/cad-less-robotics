#!/usr/bin/env python3
"""P-007F-08 Phase B 골격. 이 파일은 실행 경로가 없다.

Phase A 검수 전에는 호출하지 않는다. 호출해도 로봇에 닿기 전에 예외로 끝난다.
서보 write, Jetson, /TargetAngle 은 import 하지 않는다.

나중에 별도 승인이 나면 그때의 preregistration 이 명령 후보를 다시 고른다.
지금 정해 둔 순서만 적는다.

1. operator presence
2. workspace quick check
3. camera invariant quick check
4. controller PRE state check
5. tracking already running
6. exact approval
7. single command, approval 이후 2초 안에
8. continuous recording 유지
9. POST 시간을 확보
10. 사용자가 해제

No return command. No retry.
"""


class PhaseBNotAuthorized(RuntimeError):
    pass


def execute_phase_b():
    raise PhaseBNotAuthorized(
        "PHASE_B_NOT_AUTHORIZED. Phase A 검수 전이며 이번 모듈은 명령을 보내지 않는다.")
