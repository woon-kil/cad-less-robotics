#!/usr/bin/env python3
"""P-007E-R2-R5 — REAL hardware wiring. Jetson 전용.

🔴 이 파일은 scripts/p007e_r2r3_motion_preregistration.py / p007e_r2r4_preflight.py /
p007e_r2r5_execution.py 와 달리 self_ast_check(하드웨어 import 금지) 의 적용 대상이
**아니다** — p007e_phase1_command.py 선례와 동일하게, 실제 servo write 를 수행하는
최종 실행 파일은 Arm_Lib 를 직접 import 한다. 이 파일이 하는 일은 오직 "실제 장치를
execute_r5() 의 주입형 콜백 계약에 맞게 감싸는" 것뿐이다 — 안전 로직(게이트/ledger/
승인)은 전부 p007e_r2r5_execution.py(및 그것이 재사용하는 R3/R4)에 있다.

🔴 2026-10-28 환경 조사 기록(비차단, 공개) — `/dev/i2c-7` 를 연 프로세스 중
   `ros2 run dofbot_pro_info kinemarics_dofbot`(및 그 launcher wrapper, sh/python)가
   발견되었다. 소스(dofbot_pro_ws/src/dofbot_pro_info/src/kinemarics_dofbotpro.cpp)를
   직접 읽어 확인한 결과 이 노드는 **KDL 순수 수학 FK/IK 서비스**로 I2C/Arm_Lib
   코드를 전혀 포함하지 않는다. 실제로 spin 중인 컴파일된 바이너리(별도 PID, launcher
   와 다름)는 i2c-7 fd 를 들고 있지 않다 — launcher(sh/ros2-python wrapper)만 상위
   세션에서 상속받은 fd 를 들고 있을 뿐이다(fd 생성 시각이 수 분~수일 전으로, 활성
   트래픽의 증거가 아니다). 능동적으로 I2C 를 쓰는 유일한 프로세스는 기존에 이미
   문서화된 yahboom_oled.py 뿐이다(§10 의 "I2C 버스 전체가 조용하다는 주장이 아니다"
   라는 전제와 일치). 이 파일의 process_check_fn 은 이 i2c-7 holder 목록을 **기록**만
   하고(§13 과 별개의 진단 필드), R3 의 no_unexpected_motion_capable_process 판정은
   기존과 동일하게 camera-device holder + MOTION_CAPABLE_PATTERNS 이름매칭만 쓴다
   (sham.py.env_guards() 를 그대로 재사용 — 수정하지 않음).
"""
import contextlib
import hashlib
import importlib
import io
import json
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

# §7/phase1_command.py 선례 — Arm_Lib 소스/런타임 egg 무결성 고정
ARM_LIB_SRC = "/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py"
ARM_LIB_DISK_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
ARM_LIB_EGG = "/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg"
ARM_LIB_EGG_SHA = "3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371"

# §8 — DaBai identity (2026-10-28 `v4l2-ctl --device=/dev/video0 --info` 실측으로 확인)
DABAI_DEVICE = "/dev/video0"
DABAI_EXPECTED_SERIAL = "CH8R3620170"
DABAI_EXPECTED_WIDTH = 640
DABAI_EXPECTED_HEIGHT = 480
DABAI_EXPECTED_PIXELFORMAT = "MJPG"

# C1-R1 preregistration 의 camera_control_invariant 비교 기준값(Stage B 관측)을 그대로 전재
BASELINE_CONTROLS = {"auto_exposure": "3", "white_balance_automatic": "1",
                     "exposure_dynamic_framerate": "1", "power_line_frequency": "1",
                     "brightness": "30", "contrast": "50"}

SERVO_ID = 2
EXPECTED_BASELINE_ID2 = 152
COMMAND_TARGET_ID2 = 154
COMMAND_DURATION_MS = 2000
PRE_WINDOW_M = 60
POST_WINDOW_M = 60
FRAME_INTERVAL_S = 0.25

MOTION_CAPABLE_PATTERNS = ("arm_driver", "dofbot_pro_driver", "color_follow", "grasp",
                          "apriltag", "KCF", "yolov11", "voice_ctrl", "mediapipe")


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def check_arm_lib_integrity():
    actual_disk = sha256(ARM_LIB_SRC)
    actual_egg = sha256(ARM_LIB_EGG)
    import Arm_Lib
    runtime_is_egg = ".egg" in Arm_Lib.__file__
    return {"disk_sha_match": actual_disk == ARM_LIB_DISK_SHA, "disk_sha_actual": actual_disk,
            "egg_sha_match": actual_egg == ARM_LIB_EGG_SHA, "egg_sha_actual": actual_egg,
            "runtime_is_egg": runtime_is_egg, "runtime_file": Arm_Lib.__file__,
            "pass": (actual_disk == ARM_LIB_DISK_SHA and actual_egg == ARM_LIB_EGG_SHA
                     and runtime_is_egg)}


# ---------------------------------------------------------------------------
# §8 — camera identity + format + control-invariant (read-only queries only)
# ---------------------------------------------------------------------------
def _parse_v4l2_controls(raw_text):
    """`v4l2-ctl --list-ctrls` 출력에서 `<name> ... value=<v> ...` 를 추출한다."""
    values = {}
    for line in raw_text.splitlines():
        m = re.match(r"\s*([a-zA-Z0-9_]+)\s+0x[0-9a-fA-F]+\s+\(\w+\)\s*:\s*(.*)$", line)
        if not m:
            continue
        name, rest = m.group(1), m.group(2)
        vm = re.search(r"value=(\S+)", rest)
        if vm:
            values[name] = vm.group(1)
    return values


def camera_identity_check(device=DABAI_DEVICE, tmp_dir="/tmp/p007e_r5_probe"):
    """§8 — ABORT BEFORE MOTION 대상. v4l2-ctl --info/--list-ctrls 만 쓴다(읽기 전용,
    acquisition.py.ALLOWED_V4L2CTL_FLAGS 화이트리스트에 둘 다 포함됨)."""
    r_info = subprocess.run(["v4l2-ctl", "--device=" + device, "--info"],
                            capture_output=True, text=True, timeout=5.0)
    serial = None
    for line in r_info.stdout.splitlines():
        line = line.strip()
        if line.startswith("Serial"):
            serial = line.split(":", 1)[1].strip()

    acq = importlib.import_module("p007e_r2r2_acquisition")
    reader_probe = acq.NoNegotiationV4L2Reader(device, DABAI_EXPECTED_WIDTH, DABAI_EXPECTED_HEIGHT,
                                               DABAI_EXPECTED_PIXELFORMAT, tmp_dir)
    fmt = reader_probe.check_format_invariant()
    controls_raw = reader_probe.read_controls()
    observed_controls = _parse_v4l2_controls(controls_raw["raw"])
    control_mismatches = {k: {"expected": v, "actual": observed_controls.get(k)}
                          for k, v in BASELINE_CONTROLS.items() if observed_controls.get(k) != v}

    return {
        "dabai_identity_valid": (serial == DABAI_EXPECTED_SERIAL),
        "serial_observed": serial,
        "mjpg_640x480_valid": bool(fmt["match"]),
        "format_invariant": fmt,
        "camera_control_invariant_valid": not control_mismatches,
        "camera_controls_observed": observed_controls,
        "camera_control_mismatches": control_mismatches,
    }


def build_real_reader(run_root):
    acq = importlib.import_module("p007e_r2r2_acquisition")
    return acq.NoNegotiationV4L2Reader(DABAI_DEVICE, DABAI_EXPECTED_WIDTH, DABAI_EXPECTED_HEIGHT,
                                       DABAI_EXPECTED_PIXELFORMAT,
                                       os.path.join(run_root, "_acq_tmp"))


def build_real_estimator_and_mask():
    sham = importlib.import_module("p007e_r2r2_c1r1_sham")
    return sham.build_estimator_and_mask()


def make_capture_pre_window_fn(run_root, reader, estimator, roi_mean_fn, mask, M=PRE_WINDOW_M):
    win = importlib.import_module("p007e_r2r5_window")

    def fn():
        return win.capture_window(reader, estimator, roi_mean_fn, mask, M, run_root, "pre",
                                  anchor_gray=None, frame_interval_s=FRAME_INTERVAL_S)
    return fn


def make_capture_post_window_fn(run_root, reader, estimator, roi_mean_fn, mask, M=POST_WINDOW_M):
    win = importlib.import_module("p007e_r2r5_window")

    def fn(anchor):
        window, _ = win.capture_window(reader, estimator, roi_mean_fn, mask, M, run_root, "post",
                                       anchor_gray=anchor, frame_interval_s=FRAME_INTERVAL_S)
        return window
    return fn


# ---------------------------------------------------------------------------
# §13 — process/device-holder check (sham.py.env_guards() 그대로 재사용, 수정 없음)
# + i2c-7 holder 기록(진단용, gate 대상 아님 — 위 모듈 docstring 참조)
# ---------------------------------------------------------------------------
def i2c7_holder_diagnostic():
    """🔴 진단 전용 — 어떤 gate 값도 여기서 계산하지 않는다. R3 의
    no_unexpected_motion_capable_process 는 sham.py.env_guards() 결과만 쓴다."""
    holders = []
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return {"holders": [], "note": "PROC_LIST_UNAVAILABLE"}
    for pid in pids:
        fdd = "/proc/%s/fd" % pid
        try:
            for fd in os.listdir(fdd):
                if "i2c-7" in (os.readlink(os.path.join(fdd, fd)) or ""):
                    comm_path = "/proc/%s/comm" % pid
                    comm = open(comm_path).read().strip() if os.path.isfile(comm_path) else "?"
                    holders.append({"pid": pid, "comm": comm})
                    break
        except OSError:
            continue
    return {"holders": holders}


def make_process_check_fn(device_path=DABAI_DEVICE):
    sham = importlib.import_module("p007e_r2r2_c1r1_sham")

    def fn():
        g = sham.env_guards(device_path)
        return g["motion_capable_processes"] + g["device_holders"]
    return fn


# ---------------------------------------------------------------------------
# §22 — controller read/write (p007e_phase1_command.py 선례와 동일한 I2C 호출 패턴)
# ---------------------------------------------------------------------------
def make_controller_read_fn(arm):
    def fn():
        return arm.Arm_serial_servo_read(SERVO_ID)
    return fn


def make_controller_write_fn(arm):
    """🔴 Arm_serial_servo_write 는 I2C 예외를 bare except 로 삼킨다(phase1_command.py
    선례) — stdout 에 "I2C error" 가 찍히는 것이 유일한 오류 신호이지만, 이 자체는
    Arm_Lib 내부에서 이미 처리된 신호이지 Python 예외가 아니다. phase1_command.py 가
    그랬던 것처럼 **이 함수도 그것을 보고 인위적으로 예외를 새로 만들어 올리지
    않는다** — 그렇게 하면 R3 가 WRITE_CALL_EXCEPTION/X 로 분류해 POST 관측(카메라/
    controller post-read)을 건너뛰게 되어, 실제로 모터가 움직였는지 확인할 유일한
    기회를 잃는다. 이 함수는 `arm.Arm_serial_servo_write()` 호출 자체에서 발생하는
    **진짜 Python 예외만** 그대로 전파한다(가로채거나 재포장하지 않음) — R3 의
    `except BaseException` 이 그 경우에만 작동하도록 둔다."""
    last_call_log = {}

    def fn(servo_id, target_degrees, duration_ms):
        buf = io.StringIO()
        t0 = time.perf_counter()
        try:
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                arm.Arm_serial_servo_write(servo_id, target_degrees, duration_ms)
        finally:
            last_call_log.clear()
            last_call_log.update({
                "elapsed_s": round(time.perf_counter() - t0, 6),
                "library_stdout": buf.getvalue(),
                "i2c_error_printed": "I2C error" in buf.getvalue(),
            })
    fn.last_call_log = last_call_log
    return fn


# ---------------------------------------------------------------------------
# real-execution entrypoint
# ---------------------------------------------------------------------------
def build_real_callables(run_root, arm):
    reader = build_real_reader(run_root)
    estimator, mask, roi_mean_fn, rules = build_real_estimator_and_mask()
    return {
        "camera_check_fn": lambda: camera_identity_check(),
        "capture_pre_window_fn": make_capture_pre_window_fn(run_root, reader, estimator, roi_mean_fn, mask),
        "capture_post_window_fn": make_capture_post_window_fn(run_root, reader, estimator, roi_mean_fn, mask),
        "controller_read_fn": make_controller_read_fn(arm),
        "controller_write_fn": make_controller_write_fn(arm),
        "process_check_fn": make_process_check_fn(),
    }


# ---------------------------------------------------------------------------
# selftest — 하드웨어 0. v4l2 control 파서만 2026-10-28 Jetson 실측 샘플(고정 문자열)로
# 검증한다. camera_identity_check/controller 쪽은 실제 장치 필요 — Jetson 에서만 검증
# ---------------------------------------------------------------------------
_SAMPLE_V4L2_LIST_CTRLS = """User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=30
                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=50
                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
                            hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=1 (50 Hz)
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
         backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=100 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
                        privacy 0x009a0910 (bool)   : default=0 value=0
"""


def negative_tests():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    parsed = _parse_v4l2_controls(_SAMPLE_V4L2_LIST_CTRLS)
    mism = {k: v for k, v in BASELINE_CONTROLS.items() if parsed.get(k) != v}
    rec("HW-01_control_parser_matches_2026_10_28_jetson_sample", not mism, mism)
    rec("HW-02_control_parser_extracts_all_known_keys",
        all(k in parsed for k in BASELINE_CONTROLS), parsed)

    diag = i2c7_holder_diagnostic()
    rec("HW-03_i2c7_diagnostic_returns_list_without_crashing",
        isinstance(diag.get("holders"), list), diag)

    rec("HW-04_dabai_identity_constant_matches_2026_10_28_measurement",
        DABAI_EXPECTED_SERIAL == "CH8R3620170" and DABAI_EXPECTED_WIDTH == 640
        and DABAI_EXPECTED_HEIGHT == 480 and DABAI_EXPECTED_PIXELFORMAT == "MJPG")

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


def selftest():
    return {"negative_tests_HW_01_HW_04": negative_tests()}


if __name__ == "__main__":
    import json as _json
    print(_json.dumps(selftest(), ensure_ascii=False, indent=1))
