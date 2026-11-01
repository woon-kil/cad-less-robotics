# P-025 visual render capture

```text
P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW
P025_V01_V10 = PASS
P025_DATA_FIDELITY = PASS
P025_UPSTREAM_IMMUTABILITY = PASS
P025_VISUAL_RENDER_REVIEW = NOT_EXECUTED
P025_RENDER_SCREENSHOTS = CAPTURED
P025_AC01_AC10 = PENDING_REVIEWER
P025_PASS = NOT_EVALUATED
NEXT_ALLOWED_WORK = P025_VISUAL_RENDER_REVIEW
```

로컬 Chrome headless가 `evidence/P-025/viewer/index.html`을 그린 결과다. 새 robotics 계산이 아니다. 화면 파일은 이 촬영을 위해 고치지 않았다. 촬영 스크립트는 `scripts/p025_capture_render.mjs`다.

이 파일은 시각 검수 PASS가 아니다. reviewer가 화면을 보기 전의 rendering evidence다.

| 파일 | 화면 |
| --- | --- |
| `VR-01-overview-desktop-1440x900.png` | 데스크톱 첫 화면 1440×900 |
| `VR-01b-overview-laptop-1280x800.png` | 노트북 첫 화면 1280×800 |
| `VR-01c-overview-mobile-390x844.png` | 모바일 첫 화면 390×844 |
| `VR-02-scene1-3.png` | Scene 1–3 |
| `VR-03-scene4.png` | Scene 4 |
| `VR-04-scene5.png` | Scene 5 전체 |
| `VR-04b-scene5-mobile.png` | Scene 5, 폭 390 |
| `VR-05-boundary-and-prohibited.png` | scope 문구와 주장하지 않음 |
| `VR-05b-boundary-mobile.png` | 주장하지 않음, 폭 390 |

run_id 5, 7, 12는 `fraction = N/A`이고 `trajectory_written` 키는 없다. run 20은 `0.606061`, `20/33`이다. 18개 행의 `false`와 세 행의 `absent`와 `ROBOTTRAJECTORY_CREATED = NO`는 서로 다른 표시다.
