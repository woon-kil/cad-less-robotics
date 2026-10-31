# 재현 가능 여부

세 질문 모두 실물에서 반복해 잴 datum으로 닫히지 않았다.

## Q1. z=0

공식 URDF와 CSV는 Arm1 원점을 z=0.0895 m에 둔다. 섀시 바닥이 그 원점보다 15 mm 아래라는 문장은 없다. STEP 본문을 읽지 못했다. 맵 외곽은 높이 datum이 아니다.

```text
BASE_Z_ZERO = UNRESOLVED
Q1_BASE_Z_REPRODUCIBLE = INSUFFICIENT_EVIDENCE
```

## Q2. +X

열린 자료에 구멍, 가공면, 케이블 구멍, 제어기 장착, J1 영점을 `base_link` +X와 연결한 치수가 없다. 서보 보정의 “팔을 세운다”는 yaw datum이 아니다. 사진의 판 변은 쓰지 않았다.

```text
BASE_POSITIVE_X = UNRESOLVED
Q2_BASE_X_REPRODUCIBLE = INSUFFICIENT_EVIDENCE
```

## Q3. XY 원점

공식 모델도 Arm1 축이 x=0, y=0을 지난다. 동심 구멍 패턴이나 가공 중심의 치수는 없다. 둥근 부품을 보고 중심을 잡는 것은 측정이 아니다.

```text
BASE_XY_ORIGIN = SUPPORTED_BUT_NOT_METROLOGICALLY_DEFINED
Q3_BASE_XY_REPRODUCIBLE = INSUFFICIENT_EVIDENCE
```

## 판정

세 datum이 VERIFIED가 아니다. checkerboard fixture로 가지 않는다. URDF, mesh, 사진, 판 가장자리 추정을 더 반복하지 않는다.

```text
P-008-R5-DATUM-SOURCE = STOP_DATUM_METHOD
NEXT_ACTION = CHANGE_EXTRINSIC_CALIBRATION_METHOD
```
