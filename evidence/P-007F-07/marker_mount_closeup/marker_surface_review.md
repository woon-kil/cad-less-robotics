# P-007F-07 — Arm2 Marker Mount Close-up Review

소스: `evidence/P-007F-07/marker_setup_capture/raw_frames/marker_setup_capture_1793278747/0004-color.png`
(+ 동일 프레임 `0004-depth.png`, `0004.json` — 재촬영 없음). 로봇/서보/카메라 전부 0 동작,
marker 실제 부착 0.

## 방법 (정밀도 공개)

- M1/M2/M3 bounding box는 **자동 검출이 아니라, 1280x720 원본에 10px 간격 좌표 그리드를
  입힌 6~8배 확대 crop을 사람이(이번에는 Claude가) 반복 판독**해 정한 값이다
  (`marker_mount_instruction.md`가 기존에 쓴 것과 동일한 방법론).
- 각 후보의 `approx_available_width/height_mm`은 color→depth 픽셀 근사 매핑(초점거리
  비율만 사용)으로 얻은 **국소 depth median**(각 후보별 실측, 샘플 수 `depth_sample`에
  기록)과 color fx를 이용한 pinhole 역산이다. **P-007F-06에서 이미 공개된 RGB/Depth
  23.8mm baseline parallax는 보정하지 않았다** — 즉 mm 값은 ESTIMATED이지 실측
  캘리퍼 측정이 아니다. M3는 샘플 윈도우를 ±4px→±10px로 넓혀야 valid depth를 얻을 수
  있었다(좁은 윈도우는 전부 depth hole, 스티커 표면의 저반사 특성으로 추정).
- `rigid_with_arm2`는 전부 **이 프로젝트가 P-007F-03부터 추적해 온 `ARM2_ROI` 관례**를
  기준으로 판단했다. DOFBOT 자체의 내부 joint 번호 체계와 "Arm2"(Jetson controller의
  `SERVO_ID=2`)의 실제 대응관계는 **P-007F-01 `READBACK_SEMANTICS_UNRESOLVED`로 이미
  미해결 상태**이므로, 이번 작업에서도 그 매핑을 새로 확정하지 않는다 — M1만 정의상
  YES이고 M2/M3는 UNCERTAIN으로 공개한다.

## Arm2 구조 식별

- **Arm2 rotation axis**: 원본 좌표 `x≈712` (수직축, 흰색 디스크/호른 중심 통과) —
  `02_arm2_closeup.png` 빨간 수직선
- **Arm2 rigid link**: 흰색 디스크 + 그 위 흰색 fork 브래킷(수직선 아래, `02_arm2_closeup.png`
  하늘색 수평선 아래 전체) — `ARM2_ROI` 관례와 일치
- **다음 joint seam(근사 Arm2/Arm3 경계)**: 원본 좌표 `y≈155` — 이 선 위로는 검은 원통형
  servo 몸체(M2/M3가 위치한 영역)로, DOFBOT 자체 번호 체계상 이것이 "Arm3"인지는
  확정하지 않음(위 불확실성 공개 참조)

## 후보 M1/M2/M3

### M1 — Joint1 disc/horn 상단면 (established ARM2_ROI 영역)

```text
physical_part               = 흰색 회전 디스크/호른 상단 평면 (ARM2_ROI, P-007F-03~06 witness)
rigid_with_arm2              = YES (정의상 — 이 영역 자체가 ARM2_ROI)
surface_shape                = FLAT
approx_available_width_mm    = 72.8 (ESTIMATED, depth 684mm 국소실측)
approx_available_height_mm   = 33.6 (ESTIMATED)
camera_view_angle            = 가파른 하향 oblique, foreshortening 미보정
expected_occlusion           = 케이블/fork 다리가 일부 프레임에서 겹칠 수 있음(이번 프레임엔 경미)
direct_marker_attachment     = YES
largest_directly_attachable  = 30mm (면적만 기준)
```

🔴 **면적은 30mm까지 들어가지만, Phase A 자체 기준(`marker_spec.md`)의 최소 가독
임계값(~30px footprint, 당시 scale 1.110mm/px 기준 ≈33.3mm 이상 필요)에는 **미달**한다 —
M1의 실측 국소 scale(1.120mm/px)로 환산하면 30mm는 약 26.8px로, Phase A가 "minimum만
충족"이라 표시했던 40mm(36px)에도 못 미친다. **즉 M1은 가장 established된 witness
영역이지만, 이 프레임 기준으로는 Phase A 가독성 최소선 미달 가능성이 있다.**

### M2 — Joint2(어깨) 상단 cream/white bracket panel

```text
physical_part               = 검은 원통형 servo 몸체 바로 위 상단 cream/white 사각 브래킷
rigid_with_arm2              = UNCERTAIN (next-joint seam 너머, DOFBOT 번호체계 미확정)
surface_shape                = FLAT
approx_available_width_mm    = 52.3 (ESTIMATED, depth 580mm 국소실측)
approx_available_height_mm   = 52.3 (ESTIMATED)
camera_view_angle            = 가파른 하향 oblique, foreshortening 미보정 (M1보다 더 frontal)
expected_occlusion           = 이 프레임에서는 경미, 전체 동작범위에서는 미검증
direct_marker_attachment     = YES
largest_directly_attachable  = 50mm
```

50mm 적용 시 ≈52.6px footprint로 Phase A "preferred"(45px) 기준을 오히려 상회한다 —
가장 크고 가독성 여유가 큰 후보.

### M3 — Joint2 servo 몸체 라벨 스티커 면

```text
physical_part               = 검은 원통형 servo 몸체의 라벨 스티커 부착면
rigid_with_arm2              = UNCERTAIN (M2와 동일 사유)
surface_shape                = FLAT (스티커 자체는 평평하나 몸체는 원통형)
approx_available_width_mm    = 24.8 (ESTIMATED, depth 541mm 국소실측 — 좁은 윈도우는 depth hole)
approx_available_height_mm   = 15.1 (ESTIMATED)
camera_view_angle            = 가파른 하향 oblique
expected_occlusion           = 경미
direct_marker_attachment     = NO (height 15.1mm < 20mm 최소 테스트 크기)
largest_directly_attachable  = NONE_OF_TESTED_SIZES_FIT (20/25/30/40/50mm 전부 미달)
```

M3는 참고용으로만 남긴다 — 20mm조차 세로로 들어가지 않아 실질 후보에서 제외.

## Recommended candidate by Cursor — CANDIDATE ONLY

**M2를 1순위로 제안한다.** 근거: (1) 3개 후보 중 유일하게 Phase A 가독성 preferred
기준을 여유 있게 충족(50mm≈52.6px), (2) 평면이 가장 넓고 카메라에 더 frontal해
foreshortening 영향이 작다, (3) `rigid_with_arm2=UNCERTAIN` 라벨은 DOFBOT 내부
번호체계를 아직 모른다는 뜻일 뿐 — 실제로는 어차피 이번 실험 프로토콜(§single-motion,
동시에 다른 servo 미명령)에서 "Arm2" 명령 시 이 패널이 움직이는지 여부로 사후 경험적
검증이 가능하다(P-007F-07A static qualification 이후, 실제 motion gate에서).

**M1은 2순위(백업)로 제안한다.** established ARM2_ROI와 정의상 100% 일치하는 유일한
후보라는 강점이 있지만, 이 프레임 기준 최대 부착 가능 크기(30mm)가 Phase A 자체
가독성 최소선에 미달할 수 있어 단독 PRIMARY로는 위험 부담이 있다.

**M3는 제외를 제안한다** (부착 가능 면적 부족).

🔴 최종 선택은 하지 않는다 — 이미지를 검수자에게 전달한 후 결정.

```text
MOTION_AUTHORIZED      = NO
MARKER_LOCATION_FINAL  = NO
```
