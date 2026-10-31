# W-002D — Fiducial physical installation

상태: `PASS_FIDUCIAL_INSTALLATION / CLOSED`

ID 0~3을 작업판에 테이프로 고정한 뒤의 RGB 한 장이다. 검출기는 기본 `DICT_4X4_50`이고 PnP는 하지 않았다.

네 장 모두 평평하고, 접히거나 말리지 않았다. 투명 테이프는 종이 모서리에 있고 검은 사각형과 quiet zone을 덮지 않는다. 손상은 보이지 않는다. 인쇄 위쪽 변은 작업판 긴 변과 같은 방향이고, 네 장의 방향이 같다. yaw 각도는 추정하지 않았다.

설치 좌표의 실측값은 없다. 목표 중심은 계약의 `(150,70)`, `(500,70)`, `(500,330)`, `(150,330)` mm이고, 도구와 분해능은 보고되지 않았다.

| ID | 중심 px | 전체 가시 |
| --- | --- | --- |
| 0 | (454.75, 651.25) | YES |
| 1 | (749.75, 432.25) | YES |
| 2 | (600.25, 304.75) | YES |
| 3 | (334.00, 450.50) | YES |

`PRINT_DIMENSIONS_MEASURED = NO`, `USER_ACCEPTED_NOMINAL_GEOMETRY = YES`는 유지한다. 카메라, 로봇, 작업판은 이 프레임에서 이전과 같은 자리에 있다. 프레임 정의는 바꾸지 않았다.

registration, threshold 변경, WorkpieceInstance 선택, P-009-R1, P-010은 하지 않았다.
