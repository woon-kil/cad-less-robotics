#!/usr/bin/env python3
"""P-007F-05 — REAL hardware wiring. 이 Mac(coordinator)에서 ssh/scp 로 p620
(Gemini 335L 카메라)과 Jetson(Arm2 controller)을 조율한다.

🔴 이 파일은 scripts/p007f_05_execution.py 와 달리 self_ast_check(하드웨어/
subprocess 금지) 의 적용 대상이 **아니다** — p007e_r2r5_hardware_wiring.py
선례와 동일하게, 실제 장치를 execute_p007f05() 의 주입형 콜백 계약에 맞게
감싸는 것만 한다. 안전 로직(게이트/ledger/승인)은 전부
scripts/p007f_05_execution.py 에 있다.

🔴 이 프로젝트는 2-host 구조다(R3/R4/R5 의 Jetson 단일호스트 설계와 다름) —
Gemini 335L 카메라는 p620(192.168.0.148, docker bytesize-robotics-camera
컨테이너, ROS2 Humble)에 있고, Arm2 controller(Arm_Lib)는 Jetson
(jetson@192.168.0.94)에 있다. 이 파일이 새로 해결해야 하는 유일한 설계
문제가 이 cross-host 조율이다 — 각 host 쪽 실제 호출은 전부 기존에
검증된 스크립트를 그대로 재사용한다(scripts/p007f_03_capture_static.py,
scripts/run_humble_camera.sh, scripts/p007f_05_jetson_controller.py).
"""
import json
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

P620_HOST = "p620"
P620_REPO = "/home/woonkil/cad-less-robotics"
JETSON_HOST = "jetson@192.168.0.94"
JETSON_REPO = "/home/jetson/cad-less-robotics"

GEMINI_EXPECTED_SERIAL = "CP263630009D"
GEMINI_EXPECTED_FIRMWARE = "1.4.60"

MOTION_CAPABLE_PATTERNS = ("arm_driver", "dofbot_pro_driver", "color_follow", "grasp",
                          "apriltag", "KCF", "yolov11", "voice_ctrl", "mediapipe")

SSH_TIMEOUT_S = 30


def _run(cmd, timeout=SSH_TIMEOUT_S):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout, r.stderr


def ssh_p620(remote_cmd, timeout=SSH_TIMEOUT_S):
    return _run(["ssh", P620_HOST, remote_cmd], timeout=timeout)


def ssh_jetson(remote_cmd, timeout=SSH_TIMEOUT_S):
    return _run(["ssh", JETSON_HOST, remote_cmd], timeout=timeout)


# ---------------------------------------------------------------------------
# p620 — Gemini 335L driver lifecycle + windowed capture
# ---------------------------------------------------------------------------
def start_gemini_driver():
    """🔴 2026-10-29 실측 — `nohup cmd & echo started`(세션 내 backgrounding)는
    `ros2 launch`(장수 프로세스) 에 대해 ssh 세션 자체를 **30s 타임아웃까지
    블록**시킨다(docker exec 로 감싼 ROS2 launch 프로세스가 inherited fd를 완전히
    놓지 않는 것으로 추정 — 단순 `sleep`/`echo` 백그라운드는 즉시 반환되어 재현).
    `ssh -f`(ssh 자신이 인증 후 background 로 전환, stdin 자동 /dev/null)로
    바꾸자 0.1s 로 즉시 반환 + 정상 기동을 재현 확인. 매 호출 전 기존
    component_container 중복 프로세스를 pkill 해 누적 인스턴스를 방지한다
    (stale process accumulation — 이 함수가 반복 호출되면 그 수만큼 쌓인다는
    것을 이번에 5개 중복 발생으로 실측)."""
    stop_gemini_driver()
    _run(["ssh", "p620", "sleep 1"], timeout=10)  # pkill 직후 소켓 정리 대기
    cmd = ("cd %s && bash scripts/run_humble_camera.sh "
          "ros2 launch orbbec_camera gemini_330_series.launch.py "
          ">/tmp/p007f05_gemini_driver.log 2>&1" % P620_REPO)
    # 🔴 subprocess.run(capture_output=True) 는 Python 쪽에 PIPE 를 만든다 —
    # `ssh -f` 가 로컬에서 자기 자신을 daemonize 해도(즉시 기동은 완료) 그
    # 백그라운드로 살아남은 ssh 프로세스가 원격 세션이 끝날 때까지 이 PIPE의
    # write-end 를 쥐고 있어 communicate() 가 EOF 를 못 보고 타임아웃까지
    # 블록된다(Bash 로 직접 실행했을 때는 즉시 반환되어 재현 — pty/tty 와
    # PIPE 의 차이). PIPE 대신 DEVNULL 을 써서 이 fd 의존을 없앤다.
    r = subprocess.run(["ssh", "-f", P620_HOST, cmd], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL, timeout=SSH_TIMEOUT_S)
    return r.returncode, None, None


def stop_gemini_driver():
    return ssh_p620("docker exec -u 0 bytesize-robotics-camera pkill -9 -f component_container || true")


def gemini_identity_check():
    """driver 시작 로그에서 serial/firmware 재확인(읽기 전용). P-007F-03/04 와
    동일한 근거 방법 — 새 센서 쿼리 API 를 추가하지 않는다."""
    rc, out, err = ssh_p620("cat /tmp/p007f05_gemini_driver.log 2>/dev/null | "
                            "grep -iE 'serial|firmware' || true")
    log_text = out or ""
    serial_ok = GEMINI_EXPECTED_SERIAL in log_text
    firmware_ok = GEMINI_EXPECTED_FIRMWARE in log_text
    return {"identity_valid": bool(serial_ok and firmware_ok), "serial_found": serial_ok,
            "firmware_found": firmware_ok, "log_excerpt": log_text[:2000]}


BASELINE_B_REFERENCE_PNG = os.path.join(ROOT, "evidence", "P-007F-03", "current_rgb_overview.png")
# evidence/P-007F-04/baseline_b_check.json 의 PASS 판정에 실제로 쓰인 임계값
# (phase-correlation shift magnitude). 이 gate에서 완화하지 않는다.
BASELINE_B_MAX_SHIFT_PX = 1.0


def baseline_b_check(sample_color_png_path):
    """evidence/P-007F-04 §5 에서 ad hoc 로 했던 계산(저장된 스크립트 없음)을
    재사용 가능한 함수로 고정한다 — whole-frame mean-abs-diff + global
    phase-correlation shift, P-007F-03 reference 대비. cv2 가 필요하므로
    .venv-p007f03 인터프리터로 실행되어야 한다."""
    import importlib
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    va = importlib.import_module("p007f_03_visual_analysis")
    cv2, np = va.cv2, va.np
    ref = cv2.cvtColor(cv2.imread(BASELINE_B_REFERENCE_PNG), cv2.COLOR_BGR2GRAY)
    cur = cv2.cvtColor(cv2.imread(sample_color_png_path), cv2.COLOR_BGR2GRAY)
    whole_frame_mean_abs_diff = float(np.mean(np.abs(ref.astype(np.float64) - cur.astype(np.float64))))
    shift, response = cv2.phaseCorrelate(ref.astype(np.float32), cur.astype(np.float32))
    magnitude = (shift[0] ** 2 + shift[1] ** 2) ** 0.5
    unchanged = magnitude <= BASELINE_B_MAX_SHIFT_PX
    return {"unchanged": bool(unchanged), "whole_frame_mean_abs_diff": whole_frame_mean_abs_diff,
            "global_phase_correlation_shift_px": list(shift), "phase_correlation_response": float(response),
            "shift_magnitude_px": magnitude, "max_allowed_px": BASELINE_B_MAX_SHIFT_PX}


def capture_window_on_p620(out_dir_name, frame_count=5):
    """scripts/p007f_03_capture_static.py 를 수정 없이 그대로 재호출한다(SHA-pin,
    §34 point 6).

    🔴 2026-10-29 실측 함정(재발, 기존 메모리 교훈과 동일 패턴) — `/tmp` 를
    출력 경로로 주면 컨테이너 **내부**의 /tmp 에 쓰여서(`-v`로 바인드마운트된
    경로가 아님) host 쪽 plain scp 로는 전혀 보이지 않는다(`ls` 가 "No such
    file or directory"). run_humble_camera.sh 의 `-v ROOT:WORKSPACE` 마운트를
    이용해 **workspace 내부**(host 의 실제 저장소 경로와 동일한 파일)에 쓰면
    docker cp 없이 plain scp 로 바로 가져올 수 있다.
    """
    rel_out = ".cache/p007f05_capture_tmp/%s" % out_dir_name
    remote_out = "%s/%s" % (P620_REPO, rel_out)
    cmd = ("cd %s && bash scripts/run_humble_camera.sh python3 "
          "scripts/p007f_03_capture_static.py %s %d" % (P620_REPO, rel_out, frame_count))
    rc, out, err = ssh_p620(cmd, timeout=90)
    return rc, out, err, remote_out


def fetch_capture_dir(remote_out, local_dir):
    os.makedirs(local_dir, exist_ok=True)
    rc, out, err = _run(["scp", "-r", "%s:%s" % (P620_HOST, remote_out), local_dir], timeout=60)
    return rc, out, err


# ---------------------------------------------------------------------------
# Jetson — controller read/write (scripts/p007f_05_jetson_controller.py 만
# 실제 Arm_Lib 를 호출한다. 이 함수들은 그 결과를 ssh 로 받아오기만 한다)
# ---------------------------------------------------------------------------
def sync_jetson_controller_script():
    local = os.path.join(ROOT, "scripts", "p007f_05_jetson_controller.py")
    rc, out, err = _run(["scp", local, "%s:%s/scripts/p007f_05_jetson_controller.py" % (
        JETSON_HOST, JETSON_REPO)], timeout=30)
    return rc, out, err


def controller_read_once():
    rc, out, err = ssh_jetson("cd %s && python3 scripts/p007f_05_jetson_controller.py --read 1" % JETSON_REPO)
    if rc != 0:
        raise RuntimeError("controller read failed rc=%d stderr=%s" % (rc, err))
    data = json.loads(out.strip().splitlines()[-1])
    return data["values"][0]


def controller_write_once():
    """🔴 이 함수가 실제 Arm2 servo write 를 유발하는 유일한 경로다 — 호출하면
    scripts/p007f_05_jetson_controller.py --write 가 정확히 1회 실행된다. 이
    hardware_wiring.py 자체는 루프/재시도를 전혀 하지 않는다."""
    rc, out, err = ssh_jetson("cd %s && python3 scripts/p007f_05_jetson_controller.py --write" % JETSON_REPO,
                              timeout=15)
    if rc != 0:
        raise RuntimeError("controller write failed rc=%d stderr=%s" % (rc, err))
    return json.loads(out.strip().splitlines()[-1])


def jetson_process_check():
    pattern = "|".join(MOTION_CAPABLE_PATTERNS)
    rc, out, err = ssh_jetson("ps aux | grep -E '%s' | grep -v grep || true" % pattern)
    lines = [l for l in (out or "").splitlines() if l.strip()]
    return lines


def _load_window_mean_gray(local_dir):
    """capture_window_on_p620() 이 받아온 디렉터리에서 capture-summary.json 의
    valid frame 수와, 그 프레임들의 평균(mean) 그레이스케일 이미지를 만든다.
    scripts/p007f_03_visual_analysis.py.mean_frame() 과 동일한 방식(SHA-pin
    import, 재구현하지 않음) — cv2 가 필요하므로 이 함수는 cv2 가 설치된
    인터프리터(.venv-p007f03)로 실행되어야 한다(scripts/p007f_05_run.py 참조)."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import importlib
    va = importlib.import_module("p007f_03_visual_analysis")
    summary_path = os.path.join(local_dir, "capture-summary.json")
    try:
        summary = json.load(open(summary_path, encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, 0
    valid_frames = int(summary.get("frames", 0))
    if valid_frames == 0:
        return None, 0
    frames = []
    for i in range(valid_frames):
        path = os.path.join(local_dir, "%04d-color.png" % i)
        if not os.path.isfile(path):
            continue
        img = va.cv2.imread(path)
        if img is None:
            continue
        frames.append(va.cv2.cvtColor(img, va.cv2.COLOR_BGR2GRAY))
    if not frames:
        return None, 0
    import numpy as np
    mean = np.mean([f.astype(np.float64) for f in frames], axis=0).astype(np.uint8)
    return mean, len(frames)


def build_real_callables(local_run_dir, frame_count=5):
    """execute_p007f05() 에 그대로 주입할 콜백 dict. §34 에서는 이 함수가 실제
    execute_p007f05() 에 주입된 채로 호출되지 않는다 — 준비 단계 산출물로만
    존재한다(이 함수가 끝까지 engage 되는 것은 실제 P-007F-05 실행 시점뿐)."""
    pre_local_dir_holder = {}

    def capture_pre_window_fn():
        rc, out, err, remote = capture_window_on_p620("pre_" + str(int(time.time())), frame_count)
        local_dir = os.path.join(local_run_dir, "pre_frames", os.path.basename(remote))
        fetch_capture_dir(remote, os.path.join(local_run_dir, "pre_frames"))
        pre_local_dir_holder["dir"] = local_dir
        mean, valid = _load_window_mean_gray(local_dir)
        return mean, valid, "ANCHOR"

    def capture_post_window_fn(anchor):
        rc, out, err, remote = capture_window_on_p620("post_" + str(int(time.time())), frame_count)
        local_dir = os.path.join(local_run_dir, "post_frames", os.path.basename(remote))
        fetch_capture_dir(remote, os.path.join(local_run_dir, "post_frames"))
        mean, valid = _load_window_mean_gray(local_dir)
        return mean, valid

    def baseline_b_check_fn():
        """§7 — PRE window 의 첫 프레임을 P-007F-03 reference 와 비교한다. PRE
        capture 가 아직 없으면(= capture_pre_window_fn 이 먼저 호출되지 않았으면)
        독립적으로 1프레임을 새로 받아온다."""
        pre_dir = pre_local_dir_holder.get("dir")
        if pre_dir is None:
            rc, out, err, remote = capture_window_on_p620("baselinecheck_" + str(int(time.time())), 1)
            pre_dir = os.path.join(local_run_dir, "baseline_check_frame", os.path.basename(remote))
            fetch_capture_dir(remote, os.path.join(local_run_dir, "baseline_check_frame"))
        sample = os.path.join(pre_dir, "0000-color.png")
        return baseline_b_check(sample)

    return {
        "gemini_identity_check_fn": gemini_identity_check,
        "baseline_b_check_fn": baseline_b_check_fn,
        "capture_pre_window_fn": capture_pre_window_fn,
        "capture_post_window_fn": capture_post_window_fn,
        "controller_read_fn": controller_read_once,
        "controller_write_fn": controller_write_once,
        "process_check_fn": jetson_process_check,
    }
