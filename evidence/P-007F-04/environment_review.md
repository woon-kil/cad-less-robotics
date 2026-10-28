# P-007F-04 — §12 Workspace/Environment Review

URDF self-collision(§11)은 로봇 자기 자신과의 충돌만 다룬다 — 주변 환경(테이블,
가구, 모니터, 케이블, 카메라 지지대 등)과의 충돌은 별도로 확인해야 한다.

## 사용 데이터

`evidence/P-007F-04/baseline_check_frames/0002-color.png`(이번 gate 신규 캡처,
§5 Baseline B 확인과 동일 frame) + `evidence/P-007F-02B-R2/`의 기존 robot_visibility.md
/occlusion_analysis.md(수정하지 않고 참조만).

## 육안 확인(Read 도구로 이미지 직접 확인)

이미지 구도: DOFBOT Pro 가 나무 작업대 위에 고정되어 있고, Arm2(흰 피벗 브래킷,
base 바로 위)에서 위쪽-좌측으로 팔이 뻗어 그리퍼/DaBai 가 상단에 위치한다.
주변 물체:

- **이불/베개(배경 위쪽)**: 그리퍼/DaBai 바로 뒤쪽-위쪽에 흰색 이불이 있다 —
  육안상 수 cm(대략 3-8cm) 간격으로 보이는 가장 가까운 부드러운 장애물
- **모니터(우측)**: 검은 모니터가 작업대 우측에, 로봇 상단부에서 수평으로
  약 15-20cm 떨어져 있다
- **케이블·보드류**: Arm4/5 부근에 작은 쿨링 팬/라즈베리파이 보드가 근접
  장착되어 있고, 케이블이 책상 위로 늘어져 있다 — 이들은 로봇 자체에 고정된
  부착물(DaBai 마운트 주변)로, Arm2 motion 과 함께 강체로 같이 움직인다(§10
  DaBai 변위 계산에 이미 포함)
- **펜(테이블 위)**: 작업대 중앙에 펜 1자루가 놓여 있다 — 로봇 base 로부터
  약 10-15cm, Arm2/그리퍼의 예상 swept 영역 밖(그리퍼는 테이블 위가 아니라
  테이블보다 높은 위치에 있음)
- **체커보드 캘리브레이션 타깃**: 테이블 위, 로봇과 떨어진 좌측에 위치 —
  명백히 무관

## 예측 envelope 대비 clearance

§10 에서 계산한 D_conservative(모든 candidate, 2.96~10.89mm)는 위에서 육안
관찰한 clearance(이불까지 수 cm, 모니터까지 10cm 이상)보다 **한 자릿수 이상
작다**. ±3° candidate 의 가장 보수적인 envelope(10.89mm)도 1.1cm 수준으로,
육안상 가장 가까운 장애물(이불, 수 cm)과 비교해도 여유가 있다.

## 한계(명시 공개)

- 이것은 **정밀한 3D environment collision model이 아니다** — 캘리브레이션된
  카메라 extrinsic·장애물의 정확한 3D 위치·형상을 사용한 계산이 아니라, 2D RGB
  이미지에 대한 정성적(qualitative) 육안 판단이다
- 이불처럼 부드럽고 형태가 불안정한 물체는 정확한 거리 측정이 원천적으로
  어렵다 — "수 cm"는 대략치다
- 이미지 각도상 가려진(occluded) 영역은 평가하지 못했다(occlusion_analysis.md
  §참조 — self/robot→workspace 겹침이 일부 있음을 P-007F-02B-R2 가 이미 공개)
- 카메라 extrinsic 이 없어 이 2D 판단과 §9/§10 의 3D FK 예측을 수치적으로
  엄밀하게 대조하지는 못했다(§13 visual_detectability.md 의 한계와 동일한
  근본 원인)

## 판정

불확실성이 있는 영역(이불 등)에서도 예측 envelope(≤10.89mm)가 관찰된 clearance
(수 cm 이상)보다 현저히 작으므로, 이번 gate의 후보 범위(±1~±3°)에 대해:

```text
ENVIRONMENT_CLEARANCE_ASSESSED_QUALITATIVELY_SUFFICIENT
```

더 정밀한 environment collision model이 없다는 한계를 숨기지 않고 공개한다.
`±3°`를 넘는 더 큰 candidate 는 이번 gate의 탐색 범위(§6) 밖이므로 평가하지
않았다 — 이 질적 판단이 ±3° 보다 큰 motion 에도 그대로 적용된다고 가정하지
않는다.
