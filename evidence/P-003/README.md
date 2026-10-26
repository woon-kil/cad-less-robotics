# P-003 Gemini 335L Camera / PointCloud Bring-up

SOURCE_BASE_COMMIT=8c070d08bba3f4b43ae0425ad50b76f3acf91121
TESTED_COMMIT=NONE
EVIDENCE_GENERATED_AT=2026-10-26 14:51 KST
FINAL_COMMIT=PENDING

Status: BLOCKED

p620 `lsusb`에 Orbbec vendor `2bc5`가 없다. Gemini 335L 공식 product id는 `0804`이다.
USB 3.x root hub(10000M)는 Bus 002, 004, 006, 008에 있고 자식 장치가 없다.
연결된 주변장치는 Bus 003의 Realtek USB 오디오 둘뿐이고 속도는 480M이다.

컨테이너, udev, wrapper build, firmware, launch, topic 측정은 시작하지 않았다.
DOFBOT은 연결하지 않았다. camera→robot TF는 만들지 않았다.
