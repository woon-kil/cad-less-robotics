# Concept ProcessPath Visualization — Reviewer 검수 패키지

작성: 2026-11-02 00:33. 최종 시각 수락: 2026-11-02 01:19.  
**공개 pf 배포 없음(별도 단계).** evidence/Gate 미수정.

Repo: https://github.com/woon-kil/cad-less-robotics

## 승인 반영

```text
OFFLINE_CLIP_A = IMPLEMENTED
SCENE_D = STATIC_UR5E_MODEL
PLAYBACK_TARGET = MUTED_AUTOPLAY_LOOP_WITH_CONTROLS  (pf 반영은 별도 승인 후)
P022_REOPEN = NO
P025_REOPEN = NO

FINAL_VISUAL_ACCEPTANCE = PASS
TEXT_SAFE_AREA_DEFECT = RESOLVED
COMMIT = PRESENTATION_ARTIFACT_ONLY
PF_DEPLOY = STILL_BLOCKED
```

### 최종 인코딩 계약 (수락본)

```text
concept-processpath.mp4
  size     = 2,535,838 bytes
  sha256   = 4d1f415e83fbaa15c75444b94cc33174a1b7518e2784cc75a11f6dac98f3e885
  duration = 25.0s · 1200×750 · 12fps · 300 frames
```

## 산출물 경로

| 파일 | 역할 |
| --- | --- |
| `presentation/concept_processpath/concept-processpath.mp4` | 25.0s H.264 |
| `presentation/concept_processpath/concept-processpath.webm` | 25.0s VP9 |
| `presentation/concept_processpath/concept-processpath-poster.png` | poster (~14s TCP) |
| `presentation/concept_processpath/provenance.json` | claim / source 고정 |
| `presentation/concept_processpath/review_stills/A_geometry.png` | Scene A |
| `…/B_processpath.png` | Scene B |
| `…/C_tcp_6d.png` | Scene C (하이라이트) |
| `…/D_static_ur5e.png` | Scene D 정지 UR5e |
| `…/E_boundary.png` | Scene E boundary |
| `scripts/render_concept_processpath_video.py` | 재현 스크립트 |

재렌더:

```bash
cd /Users/woonkil/projects/cad-less-robotics
.venv-presentation/bin/python scripts/render_concept_processpath_video.py
```

UR5e collision STL + URDF는 `.cache/presentation/ur5e/` (gitignore). P620 `ur_description 3.5.1`에서 가져온 presentation cache.

## Claim boundary (영상)

전 구간 배지:

> ProcessPath 개념 시각화 · 검증된 RobotTrajectory가 아닙니다

Scene E:

```text
P-022                 CLOSED_FAIL
RobotTrajectory       NOT ACHIEVED
```

`ARTICULATED_EXECUTION=NO` · RobotTrajectory playback 없음 · 새 IK/planning/transform 없음.

STATIC_Q는 **display-only** 정지 자세이며 P-022 evidence가 아니다 (`provenance.json`).

## 검수 체크

- [x] 첫 10초에 6D ProcessPath(위치+자세)가 polyline-only와 구분되는가
- [x] TCP frame이 surface를 따라 orientation이 바뀌는가 (Scene C)
- [x] Scene D에서 로봇이 움직이지 않는가
- [x] CLOSED_FAIL / NOT ACHIEVED가 마지막에 오해 없이 보이는가
- [x] evidence/P-013·P-022·P-025 파일이 수정되지 않았는가
- [x] TEXT_SAFE_AREA (title/footer vs 3D) — PIL band burn으로 해소
- [x] 최종 25s regression R1–R5 PASS

## 다음 (pf 통합 — 별도 승인)

1. `koraiagent.com/public/cad-less-robotics/`에 video + poster 배치
2. muted autoplay loop controls + `prefers-reduced-motion` 억제
3. 기존 P-025 evidence viewer는 대체하지 않음
4. pf-sync 배포

**이 커밋은 presentation artifact만. pf HTML 수정·배포는 포함하지 않음.**
