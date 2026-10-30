#!/usr/bin/env bash
# p620 호스트에서 실행한다. USB 노드가 바뀌었으면 카메라 컨테이너만 다시 만든다.
set -euo pipefail
cd /home/woonkil/cad-less-robotics
bash scripts/run_humble_camera.sh bash scripts/p008b_run.sh
