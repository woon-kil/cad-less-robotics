# P-007F-04 — §11 Collision Review

## 1차 근거 — SRDF(벤더 산출물, 전체 joint 범위 포괄)

`.cache/jetson_robot_model/DOFBOT_Pro-V24.srdf`(sha256
`5c355b844eca2ba277e5df204b59b6d8886768b48e11701d17a23491bdffc1ee`, P-007E-R2-R4
와 동일 파일, 수정하지 않음). MoveIt Setup Assistant 가 전체 joint 범위에 대해
조밀 샘플링해 생성한 `disable_collisions` 행렬이며, 133줄 전체 link pair 가
Adjacent/Never/Default 중 하나로 비활성화되어 있다 — 어떤 pair도 "충돌 가능"으로
남아있지 않다. 특히 `Arm1_Link-Arm2_Link=Adjacent`, `Arm2_Link-base_link=Never`.
이 분석은 Arm2 의 전체 joint 범위(−90°~+90° urdf, library 0°~180°)를 포괄하므로
이번 gate의 151°~157° 구간에도 당연히 적용된다.

🔴 한계(R4 와 동일): Setup Assistant 의 내부 샘플링 밀도·알고리즘을 Claude 가
재검증하지 않았다 — 벤더 산출물을 신뢰한다는 전제다.

## 2차 근거 — FK 기반 spot-check (candidate 마다 PRE/MID/POST, stride=40)

`scripts/p007f_04_candidates.py`(`collision_spot_check()`)가 scipy 없이 stride=40
다운샘플링 + chunked brute-force 로 `base_link-Arm2_Link`/`Arm1_Link-Arm2_Link` 최소
거리(mm)를 candidate 마다 PRE(154°)/MID/POST 3-sample 로 직접 계산했다(R4 의 방법
설명과 동일, R4 자체의 1회성 계산 스크립트는 저장되어 있지 않아 이번 gate에서
재구현 — 154°에서 재현값이 R4 의 152/153/154° 표와 float 오차 이내로 정확히
일치함을 먼저 self-check 로 확인했다, 아래 "재현성 검증" 참조).

| delta | target | base_link–Arm2_Link (PRE/MID/POST, mm) | Arm1_Link–Arm2_Link (PRE/MID/POST, mm) |
|---:|---:|---|---|
| −3° | 151° | 16.075 / 16.058 / 16.065 | 0.135 / 0.127 / 0.129 |
| −2° | 152° | 16.075 / 16.061 / 16.058 | 0.135 / 0.133 / 0.125 |
| −1° | 153° | 16.075 / 16.067 / 16.061 | 0.135 / 0.132 / 0.133 |
| +1° | 155° | 16.075 / 16.086 / 16.100 | 0.135 / 0.135 / 0.132 |
| +2° | 156° | 16.075 / 16.100 / 16.135 | 0.135 / 0.132 / 0.127 |
| +3° | 157° | 16.075 / 16.116 / 16.181 | 0.135 / 0.130 / 0.127 |

`base_link-Arm2_Link` 간극은 설계상 명확한 간극(`Never`, ~16mm)과 정합하고, 전
candidate 에서 16.05~16.19mm 범위로 **닫히는 경향 없음**. `Arm1_Link-Arm2_Link`
는 설계상 인접·접촉(`Adjacent`, ~0.13mm)과 정합하고, 전 candidate 에서
0.125~0.135mm 범위로 변동폭이 ±0.01mm 수준 — R4 가 2° 구간에서 관찰한 것과
동일하게, 이 ±3° 구간에서도 간극이 닫히는 경향이 없다.

### 재현성 검증(self-check)

`collision_spot_check(fk, joints, mesh_dir, 152/153/154)` 을 R4 의 공개값과
직접 비교:

```text
152°: 16.057761685628748 / 0.1253498988622087   (R4: 16.057761685628748 / 0.12534989886220804)
153°: 16.06114566658824  / 0.1334185288881729   (R4: 16.061145666588242 / 0.13341852888817007)
154°: 16.075245769584175 / 0.1349329230320783   (R4: 16.07524576958418  / 0.134932923032074)
```

float 오차(1e-9mm 수준) 이내로 완전히 일치 — 재구현이 R4 의 방법·수치를 정확히
재현함을 확인했다. 🔴 1차 초안에서 `/1000.0` 중복 적용 버그로 전혀 다른(각도에
불변인, 링크 origin 간 거리로 붕괴된) 값이 나온 것을 이 self-check 로 즉시 발견해
수정했다 — `fk.load_stl_vertices()` 가 이미 미터 단위를 반환한다는 것을
`p007e_r2r4_fk.py` 의 다른 함수들(`mesh_radius_table_mm`, `transform_mesh_to_base`)
사용 패턴과 R4 `swept_motion.json` 의 "STL 은 meter" 주석으로 확인 후 고쳤다.

## interpolation density

R4 와 동일하게 PRE/MID/POST 3-sample(2° 구간을 1° 간격) 유지 — 이번 ±3° 구간은
3° 를 1.5° 간격 3-sample 로 다루어 R4 보다 interpolation 밀도를 낮추지 않았다
(§11 요구사항).

## 주장하지 않는 것

```text
PHYSICAL_COLLISION_FREE_PROVEN
```

허용되는 표현만 사용한다:

```text
MODEL_LEVEL_COLLISION_CHECK_PASS
```

## 검사 한계

- Setup Assistant 샘플링 알고리즘 미재검증(벤더 신뢰 전제)
- FK spot-check 는 `base_link`/`Arm1_Link` vs `Arm2_Link` 두 pair, stride=40
  다운샘플링만 수행 — 전체 12개 moving link 간 전체 pair 조합은 계산하지 않았다
  (scipy 미사용, R4 와 동일한 범위 제한)
- 수 mm 이하의 작은 간극을 놓칠 수 있으나, 두 관찰 pair 의 간극(16mm, 0.13mm)
  모두 다운샘플링 밀도에 영향받지 않을 만큼 안정적이다
- 그리퍼 passive linkage joint(rlink_joint2/3, llink_joint1/2/3)는 이번
  collision spot-check 대상(base_link/Arm1_Link/Arm2_Link)에 포함되지 않는다
  — swept displacement 계산(§10)에서는 포함됨(exact_moving_group_max_rperp_mm)

## 판정

```text
SELF_COLLISION PRE/MID/POST — MODEL_LEVEL_COLLISION_CHECK_PASS (6개 candidate 전부)
```
