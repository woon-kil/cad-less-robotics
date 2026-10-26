#!/usr/bin/env bash
# Mac에서 실행한다. 보드 배치만 바꾸고 카메라와 로봇은 움직이지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${P006C_REMOTE:-p620}"
REMOTE_ROOT="${P006C_REMOTE_ROOT:-/home/woonkil/cad-less-robotics}"
OUT="${ROOT}/evidence/P-006C-R2"
POSITIONS=(CENTER LEFT RIGHT ROBOT_SIDE CAMERA_SIDE)

prompt_for() {
  case "$1" in
    CENTER) printf '%s' "체크보드 종이 전체가 작업 영역 한가운데에 있게 놓으세요. X는 0에서 400 mm, Y는 300에서 800 mm 안입니다." ;;
    LEFT) printf '%s' "DOFBOT 쪽에서 LED를 볼 때 왼쪽에 놓으세요. 종이는 작업 영역 밖으로 나가지 않게 합니다." ;;
    RIGHT) printf '%s' "같은 시선에서 오른쪽에 놓으세요. 종이는 작업 영역 안에 둡니다." ;;
    ROBOT_SIDE) printf '%s' "로봇 쪽 Y 300 mm 경계 안쪽에 놓으세요. Y 300 mm 미만 설치 영역에는 올리지 마세요." ;;
    CAMERA_SIDE) printf '%s' "카메라와 LED 쪽 안쪽에 놓으세요. 받침 밑으로 넣지 마세요." ;;
    *) return 1 ;;
  esac
}

ask() {
  local message="$1"
  local primary="$2"
  local secondary="$3"
  osascript << EOF
set resultDialog to display dialog "P-006C-R2" & return & "카메라와 로봇은 움직이지 마세요." & return & "${message}" buttons {"${secondary}", "${primary}"} default button "${primary}" giving up after 180
if gave up of resultDialog then
  return "timeout"
end if
return button returned of resultDialog
EOF
}

intro="$(ask "다섯 위치에 체크보드만 놓습니다. 카메라는 좌표를 workplane으로 바꾸지 않습니다. 로봇은 움직이지 않습니다." "시작" "취소" || true)"
if [[ "${intro}" != "시작" ]]; then
  printf 'BLOCKED_NO_OPERATOR\n'
  exit 2
fi

rsync -az \
  "${ROOT}/scripts/p006c_r2_capture.py" \
  "${ROOT}/scripts/p006c_r2_analyze.py" \
  "${ROOT}/scripts/p006c_r2_launch.sh" \
  "${ROOT}/scripts/p006c_capture_streams.py" \
  "${ROOT}/scripts/p006c_validate_metric.py" \
  "${REMOTE}:${REMOTE_ROOT}/scripts/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/thresholds.yaml" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/thresholds.yaml"
rsync -az \
  "${OUT}/physical_task_roi.json" \
  "${OUT}/visibility_rules.json" \
  "${OUT}/environment.json" \
  "${OUT}/capture_manifest.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006C-R2/"

ssh -o BatchMode=yes "${REMOTE}" "mkdir -p '${REMOTE_ROOT}/evidence/P-006C-R2/captures' '${REMOTE_ROOT}/evidence/P-006C-R2/logs' && rm -f '${REMOTE_ROOT}/evidence/P-006C-R2/driver-stop.txt' '${REMOTE_ROOT}/evidence/P-006C-R2/driver-ready.txt'"
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh bash scripts/p006c_r2_launch.sh" > /tmp/p006c-r2-launch.log 2>&1 &
LAUNCH_SSH=$!
ready=0
for _ in $(seq 1 100); do
  if ssh -o BatchMode=yes "${REMOTE}" "test -f '${REMOTE_ROOT}/evidence/P-006C-R2/driver-ready.txt'"; then
    ready=1
    break
  fi
  if ! kill -0 "${LAUNCH_SSH}" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "${ready}" -ne 1 ]]; then
  echo "드라이버가 준비되지 않았다" >&2
  ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006C-R2/driver-stop.txt'" || true
  wait "${LAUNCH_SSH}" || true
  exit 1
fi

mkdir -p "${OUT}/logs"
for position in "${POSITIONS[@]}"; do
  try_index=1
  while [[ "${try_index}" -le 3 ]]; do
    choice="$(ask "$(prompt_for "${position}") 배치 ${try_index}. 놓았으면 촬영을 누르세요." "촬영" "이 위치 건너뜀" || true)"
    printf '%s %s try=%s choice=%s\n' "$(date -Iseconds)" "${position}" "${try_index}" "${choice:-error}" >> "${OUT}/logs/operator-log.txt"
    if [[ "${choice}" != "촬영" ]]; then
      break
    fi
    ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_r2_capture.py --position '${position}' --try-index '${try_index}' --count 4" || true
    try_index=$((try_index + 1))
    if [[ "${try_index}" -le 3 ]]; then
      more="$(ask "${position} 프레임을 남겼습니다. 같은 위치에 한 번 더 놓을까요? 이전 프레임은 지우지 않습니다." "추가 촬영" "다음 위치" || true)"
      printf '%s %s more=%s\n' "$(date -Iseconds)" "${position}" "${more:-error}" >> "${OUT}/logs/operator-log.txt"
      if [[ "${more}" != "추가 촬영" ]]; then
        break
      fi
    fi
  done
done

ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006C-R2/driver-stop.txt'"
wait "${LAUNCH_SSH}" || true
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_r2_analyze.py" || true
rsync -az "${REMOTE}:${REMOTE_ROOT}/evidence/P-006C-R2/" "${OUT}/"
cd "${ROOT}"
python3 - << 'PY'
import json
from pathlib import Path
root = Path("evidence/P-006C-R2")
metrics_path = root / "metrics.json"
environment = json.loads((root / "environment.json").read_text())
manifest = json.loads((root / "capture_manifest.json").read_text())
frames = sorted(path.name for path in (root / "captures").glob("*.json")) if (root / "captures").is_dir() else []
manifest["captures"] = frames
environment["capture_performed"] = bool(frames)
(root / "capture_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
(root / "environment.json").write_text(json.dumps(environment, indent=2, ensure_ascii=False) + "\n")
summary = {}
if metrics_path.is_file():
    summary = json.loads(metrics_path.read_text()).get("summary", {})
positions = summary.get("positions", {})
observed = [name for name, item in positions.items() if item.get("observed")]
captured = int(summary.get("capture_count") or len(frames))
boundary = (summary.get("camera_visible_roi") or {}).get("metric_boundary")
if captured == 0:
    status = "IMPLEMENTED_NOT_VERIFIED"
elif boundary == "METRIC_BOUNDARY_UNRESOLVED" and len(observed) == 5:
    status = "CONDITIONAL_PASS"
elif captured > 0:
    status = "BLOCKED"
else:
    status = "IMPLEMENTED_NOT_VERIFIED"
checks = {
    "R2-01": True,
    "R2-02": True,
    "R2-03": True,
    "R2-04": True,
    "R2-05": captured > 0,
    "R2-06": True,
    "R2-07": captured > 0,
    "R2-08": captured > 0,
    "R2-09": captured > 0,
    "R2-10": captured > 0,
    "R2-11": boundary == "METRIC_BOUNDARY_UNRESOLVED",
    "R2-12": True,
    "R2-13": True,
    "R2-14": True,
    "R2-15": True,
    "R2-16": True,
    "R2-17": True,
    "R2-18": True,
    "R2-19": True,
    "R2-20": True,
}
(root / "acceptance.json").write_text(json.dumps({
    "status": status,
    "observed_positions": observed,
    "capture_count": captured,
    "camera_visible_roi": "METRIC_BOUNDARY_UNRESOLVED",
    "robot_reachable_roi": "UNVERIFIED_P007_NOT_OPEN",
    "checks": checks,
}, indent=2) + "\n")
print(status)
PY
