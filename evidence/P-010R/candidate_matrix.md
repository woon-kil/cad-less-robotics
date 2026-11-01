# P-010R CAD reference asset replacement

P-010의 `freecad-shoe-last-wb` STEP은 고치지 않았다. face를 나누지 않았고, mesh를 STEP으로 바꾸지 않았다. P-010 qualification을 다시 실행하지 않았고 P-011도 열지 않았다.

이번 조사는 자산을 고르지 않는다. 명시적 라이선스가 있는 공개 화형 geometry가 있는지 기록한다.

```text
P-010R = CANDIDATE_FOUND
PRIMARY_ASSET = NONE
READY_FOR_P010B = NO
```

명시적 라이선스와 화형 STEP/BREP를 함께 가진 후보는 없다. STL, OBJ, GLB, Fusion `.f3d`는 후보로만 적고 CAD branch primary로 잡지 않는다.

## 비교

| ID | SOURCE | COMMIT / VERSION | LICENSE | ASSET_NAME | ASSET_TYPE | NATIVE_FORMAT | STEP | BREP | MESH_ONLY | PARAMETRIC | REDISTRIBUTION | ATTRIBUTION | SHOE_LAST | 채택 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | https://github.com/tobiasfs/openshoedesigner | `111912bd4f498cf00b31a8ada4ee18bc4577b3ee` (2026-10-05) | GPL-3.0. 저장소 `LICENSE`가 GPLv3 본문 | `data/Last_Flat_RoundTip.stl`, `Last_Heeled_RoundTip.stl`, `LastScan_Default.stl`, `LastScan_Warped.stl` | 화형 mesh. 제조용 신발 설계 프로그램에 포함 | STL | NO | NO | YES | 프로그램이 last를 불러오거나 insole에서 만들 수 있다고 README에 적혀 있다. Last adaption 상태는 Open. 저장소 안 산출 STEP은 없음 | GPL-3.0 조건을 지키면 가능 | GPL 고지 필요 | YES | 아님 |
| B | https://github.com/les-original/last-shoe | `21d22805f6c15611d550bd30de9b70d700fe6102` (2021-11-30) | CC0-1.0. 저장소 `LICENSE`가 CC0 1.0 본문 | `CORE/last_core.f3d`, `CORE/STL/last_core.stl`, outsole·toe guard의 `.f3d`/STL | 모듈형 자전거 신발 | Fusion `.f3d`, STL, PDF | NO | NO | STL은 있음. 열린 BREP는 없음 | NO | CC0로 가능 | CC0는 요구하지 않음 | NO. 이름은 Last이나 화형이 아니라 신발 제품 | 아님 |
| C | https://github.com/AffinitySpace/new-world-grade branch `sneaker` | `7ec2bf3dd95bc26a80b20bfc9721a343f63cfc12` (2026-04-16) | MIT. `LICENSE`에 Copyright (c) 2025 Affinity Space | `The Mid-Top 80 (280-99-78).obj` (28615 bytes) | footwear last cage | OBJ | NO | NO | YES | README는 lowpoly OBJ를 control cage로 쓴다고 적는다. STEP 생성기는 없음 | MIT 조건으로 가능 | copyright notice 필요 | 부분. last shape라고 부르나 낮은 해상도 OBJ | 아님 |
| C2 | 같은 저장소 branch `minimal` | `03fcbc3144008382e452ed57b58b70a0883ec9c5` (2026-05-04) | MIT. 같은 `LICENSE` | `The Low-Top 00 (280-104-71)` 확장자 없음, 첫 줄이 Blender OBJ | 같은 계열의 cage | OBJ | NO | NO | YES | C와 같음 | MIT | 필요 | 부분 | 아님 |
| D | https://github.com/riccca/shoes_last_data | `67006cced287d5fa0eb9865a8191b3f26c467b3e` (2021-01-09) | GitHub license `null`. `LICENSE` 파일 없음. 작성자 note는 개인 이용은 자유, 상업 이용은 사전 협의 | `shoes.stp`, `shoes.stl` | 화형. README는 둘을 같은 데이터라고 적음 | STEP, STL | YES | 파일은 `.stp`. 적격 검사는 하지 않음 | STL도 있음 | NO | 이 공개 포트폴리오에 대한 허락이 없음 | 작성자가 사용 알림을 원함 | YES | 거절. 라이선스 |
| E | https://doi.org/10.5281/zenodo.10337437 | Zenodo v1, 2020-07-08 | 레코드 License는 CC BY 4.0 | `d0501a5d95894dfa8c26bbfcb56fe296.glb` | 나무 화형의 사진측량 mesh. 출처 표기는 Objaverse / Sketchfab | GLB | NO | NO | YES | NO | 레코드의 CC BY 4.0 조건 | 필요 | YES. CAD가 아니라 스캔 mesh. Sketchfab 원본 라이선스는 따로 확인하지 않음 | 아님 |

`main`의 New World Grade에는 PDF와 README만 있다. OBJ는 `sneaker`와 `minimal`에 있다.

## 거절과 미확인

D는 화형 STEP이 있지만 재배포 조건이 열려 있지 않다. 개인 이용 문구를 GPL이나 CC로 바꾸지 않는다. 파일을 받지 않았다.

Thingiverse `thing:1853417` 은 검색 추출문에 CC BY와 STL 하나가 보였으나, 이 실행에서 페이지 본문이 열리지 않았다. 라이선스를 확정하지 않았고 후보에 넣지 않았다. GrabCAD 파일은 보지 않았다.

## 결론

비교한 공개 후보는 있다. CAD branch primary로 넘길 STEP/BREP 화형은 없다. P-010B와 P-011은 시작하지 않는다.
