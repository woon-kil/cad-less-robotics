# P-006A Attempt 2B

상태: **PASS**

이전 커밋 `f391e6cb65bfa27e662d01dc029eb4eba56dc8c1`의 FAIL은 Arm1 불일치가 아니다. 근접 사진이 프로젝트 파일로 들어오기 전이라 `INPUT_EVIDENCE_MISSING`이다. 이번 기록은 그 파일을 넣은 뒤 같은 Attempt 2B를 다시 실행한 결과다.

P-006 extrinsic은 계속 BLOCKED다. P-007은 열지 않았다.

## 사진

`dofbot-arm1-original.jpeg`

JPEG JFIF, 576×1024, 8-bit, 3 components, 144249 bytes.

sha256 `975aa74936f61117cba43ae25bda128a0a855af519963b0c14278d96a4080424`

원본은 수정하지 않았다. 표시는 `dofbot-arm1-annotated.png`에만 있다. 픽셀 크기를 mm로 바꾸지 않았다. 기존 `evidence/P-006A/gemini-color.png`는 삭제하지 않았고, 이번 식별의 주 evidence는 이 근접 사진이다.

## Arm1

검은 하부 하우징 위의 흰색 원형 플랜지가 첫 가동 회전부다. 그 중심을 플랜지에 수직으로 지나는 직선이 `A_AXIS`다. 이 직선은 URDF `base_link`의 x=0, y=0 축에 대응한다.

+X, +Y, z=0, checkerboard 원점 C는 확정하지 않았다. `T_base_target`과 `T_base_camera`는 없다.
