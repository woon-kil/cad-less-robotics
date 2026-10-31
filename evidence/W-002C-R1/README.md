# W-002C-R1 — Nominal print geometry acceptance

상태: `PASS_NOMINAL_PRINT_GEOMETRY_ACCEPTANCE / CLOSED`

W-002C에서 쓴 ID 0~3 네 장은 같은 생성·인쇄 조건의 출력물이다. 사용자는 그 물리 치수를 명목값으로 registration에 쓰는 것을 승인했다.

```text
PRINT_DIMENSIONS_MEASURED = NO
USER_ACCEPTED_NOMINAL_GEOMETRY = YES
```

검은 사각형은 60 mm × 60 mm, quiet zone은 10 mm 이상이다. 네 ID의 geometry source는 모두 `USER_ACCEPTED_NOMINAL`이다. 자로 잰 값이 아니다.

종이 두께와 테이프 두께도 측정하지 않았다.

이 승인은 registration 합격을 면제하지 않는다. 이후 registration은 W-002B에 고정된 reprojection, 작업판 평면, 10프레임 이상 반복, known distance로 판단한다. 명목 치수 때문에 그 기준을 넘으면 registration은 FAIL이고, 출력물 치수는 다시 의심 대상이다.

W-002C의 `BLOCKED_PRINT_METROLOGY / CLOSED`는 바꾸지 않았다.

```text
FIDUCIAL_LAYOUT = VALIDATED_FOR_INSTALL_WITH_NOMINAL_GEOMETRY
```

마커 설치, registration, PnP, threshold 변경, WorkpieceInstance 선택, P-009-R1, P-010은 하지 않았다.
