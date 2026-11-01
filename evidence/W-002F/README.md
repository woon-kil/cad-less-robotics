# W-002F — 독립 물리 기준 계약

상태: `PASS_INDEPENDENT_REFERENCE_CONTRACT / CLOSED`

DESIGN / CONTRACT ONLY다. ChArUco를 출력하거나 설치하지 않았고, 촬영과 PnP도 하지 않았다. 이 합격은 등록 합격이 아니다.

질문은 약 -2.4 mm 차이가 RGB fiducial pose인지, registered depth의 작업판 평면인지, 둘 다인지, 기준 자체인지 독립적으로 가를 수 있는가이다. H_RGB, H_DEPTH, H_BOTH, H_REFERENCE 중 어느 것도 고르지 않았다.

## 기준

기준은 rigid backing 위의 ChArUco다. 나무판에 붙인 종이만으로는 만들지 않는다. 역할은 camera geometry의 독립 기준이다. 신발 경계, 작업물, 공정 영역, 로봇 calibration이 아니다.

패턴의 dictionary, 칸 수, 칸 길이는 아직 정하지 않았다. 인쇄 명목 치수를 ground truth로 쓰지 않는다. 촬영 전에 패턴과 실측 치수를 동결한다. 재지 않은 값은 명목과 같다고 적지 않는다.

## 높이

인쇄면을 작업판 z=0으로 두지 않는다. 작업판에 닿는 아랫면에서 실제 인쇄면까지의 총 높이를 직접 잰다. 층마다 나누어 재는 것은 필수가 아니다. 권장 형태는 작업판 위에 직접 놓인 알려진 두께의 rigid plate다. 높이는 아직 `NOT_YET_MEASURED`다.

디지털 캘리퍼스를 쓸 수 있다. 표시 분해능은 측정 정확도가 아니다. 휨은 straightedge나 그에 상당하는 물리 확인이다. 사진으로 µm나 mm 평탄도를 주장하지 않는다.

## 관측

RGB는 raw distorted color와 그 프레임의 K, D, 실측한 target geometry로 reference pose를 낸다. 기존 ID0~ID3 PnP와 따로 한다.

Depth는 같은 인쇄면의 평면을 registered depth로 본다. RGB pose에 맞추지 않는다.

같은 장면에서 reference 옆 bare wood 평면도 보고, 두 평면의 간격을 잰 총 높이와 비교한다.

실패한 W-002E와 R8 변환은 ground truth로 쓰지 않는다. -2.4 mm를 보정값으로 넣지 않는다. ID0~ID3 pose로 reference 위치를 계산하지 않는다.

## 비교

| 시험 | 내용 |
| --- | --- |
| A | RGB ChArUco 평면과 depth reference 평면 |
| B | depth reference 평면과 depth bare wood 평면 |
| C | 물리 총 높이와 depth가 본 간격 |
| D | 기존 ArUco 작업판 pose와 독립적으로 얻은 관계 |

숫자 합격 기준은 이 Gate에서 만들지 않았다. 칸 크기, 거리, 코너 위치, 계측 분해능을 안 다음, 결과를 보기 전에 동결한다. W-002B의 재투영, 평면, 반복 기준은 바꾸지 않는다. SUBPIX를 채택하지 않는다.

ADR-010은 `ACCEPTED / CLOSED`로 남긴다. `VALIDATED_REGISTRATION_TRANSFORM = NONE`이다.
