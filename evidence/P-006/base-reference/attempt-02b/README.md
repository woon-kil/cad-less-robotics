# P-006A Attempt 2B

상태: **FAIL**

새 DOFBOT 근접 사진이 이 요청과 작업 트리에 없었다. Arm1을 식별하지 않았고, annotation을 만들지 않았다. 기존 `evidence/P-006A/gemini-color.png`는 삭제하지 않았고, 이번 식별의 주 evidence로 쓰지 않았다. 그 프레임에는 팔 관절이 없다.

거리, checkerboard pose, `T_base_target`, `T_base_camera`는 계산하지 않았다. 로봇은 움직이지 않았다.

## 사진을 찾은 곳

2026-10-26 18:59 KST 요청 본문에는 이미지 파일이 붙어 있지 않다. 같은 시각 이후 생성된 png, jpg, jpeg, webp, heic는 워크스페이스, Cursor 첨부 경로, p620 `evidence/`에서 찾지 못했다. Downloads의 오늘 스크린샷은 웹 페이지다.

## 유지하는 URDF

ballbot `906da9f9120c056dd0c8766fe1a72e72a086061c`의 `DOFBOT_Pro-V24.urdf`.

`Arm1_Joint` origin `0 0 0.0895`, rpy `0 0 0`, axis `0 0 1`. 모델에서 회전 중심선은 `base_link`의 x=0, y=0을 지난다. 이 문장은 실물 대응이 아니다.

## 아직 확정하지 않은 것

`base_link` +X, +Y, z=0, checkerboard 원점 C, `T_base_target`, `T_base_camera`.
