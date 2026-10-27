# 로그

- `raw_snapshot_before.json` / `raw_snapshot_after.json` — `scripts/p007a_r1_collect.py`의 원본 출력. 15개 섹션.
- `static_detail.txt` / `static_detail2.txt` / `static_detail3.txt` — 종료 핸들러·`Robot_Controller`·OLED 주소·udev·ipykernel·`yb-discover.py` 전문 정적 확인. `static_detail.txt`는 로컬 `head`가 `tee`를 끊어 뒷부분이 유실됐고, 그 항목은 `static_detail2.txt`에서 다시 받았다.
- `pre_isolation_reads.csv` — 격리 전 5라운드 × 6. 격리 후 100라운드는 `../read_regression.csv`에 있다.
- `read_before.stderr` / `read_after.stderr` — 읽기 스크립트의 라운드·오류 수.

쓰기 명령, 토크, 캘리브레이션, i2c probe, 프로세스 그룹 종료는 없다. 자격증명은 남기지 않았다.
