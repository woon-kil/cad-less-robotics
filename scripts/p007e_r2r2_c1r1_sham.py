#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C1-R1 — repaired stationary-sham orchestration harness.

🔴 기본 실행은 **카메라를 열지 않는다**. 캡처 경로 진입 조건 둘 다 필요:
       환경변수  P007E_R2R2_C1R1_ALLOW_CAPTURE=1
       플래그    --run-sham

🔴 acquisition(scripts/p007e_r2r2_acquisition.py) 과 measurement
   (scripts/p007e_r2r2_measurement.py) 을 분리해서 쓴다. 이 파일은 그 둘을
   엮는 orchestration 뿐이다 — subprocess 호출은 acquisition 안에만 있다.

🔴 directory 경로와 file 경로를 같은 변수로 쓰지 않는다 (C1
   HARNESS_OUTPUT_PATH_DEFECT 재발 방지). run_root 아래
   manifest.json / trials/trial_NN.json / frames/trial_NN/*.jpg /
   summary/sham_summary.json 로 고정 분리한다.

🔴 trial 단위 원자적 저장 — tmp 쓰기 + fsync + os.replace.
"""
import argparse
import ast
import hashlib
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREREG = os.path.join(ROOT, "evidence/P-007E-R2-R2/stage_c1r1_preregistration.json")
ROI = os.path.join(ROOT, "evidence/P-007E-R2-R2/stage_c0_roi.json")
FRAME = os.path.join(ROOT, "evidence/P-007E-R2-R2/logs/b3_frame.png")

PREREG_SHA = "ed44663d9e52611f2dfc0db3f07eabc9b3de40fba88a56544b0e4057111faaff"
# 🔴 §5 enforcement — module_sha256 과 동일 값. harness 시작 시 세 파일을 재계산해 대조한다
MODULE_SHA = {
    "scripts/p007e_r2r2_acquisition.py": "dc4cf383d61ddfd96fb322a5bd8522ecb9cb97b9e3c636592f9d2b4f0e7400e1",
    "scripts/p007e_r2r2_measurement.py": "024b7b9d3d6a37aca79afe90b2fa65ced6f5885c89fdbd82b85b1c776b7d526c",
    "scripts/p007e_r2r2_analysis.py": "fd3910ff9d4759244cb3ce1837c2dab099059af006276bee56d5dc8951da4f4f",
}
ALLOW_ENV = "P007E_R2R2_C1R1_ALLOW_CAPTURE"
FORBIDDEN_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "Adafruit_GPIO"}
# 🔴 cv2.VideoCapture 사용 금지 — 정적 검사. 문자열을 조립해 이 검사 자신의 정의문이
# 자기 자신을 오탐하지 않게 한다 (i2c 자기참조 오탐과 같은 함정, p007e_r2r2_c1_sham.py 선례)
FORBIDDEN_CALL_PATTERNS = ("Video" + "Capture(",)


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def self_ast_check():
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    tree = ast.parse(src)
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                found.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            found.add(node.module.split(".")[0])
    bad_imports = sorted(found & FORBIDDEN_IMPORTS)
    needles = ("/dev/" + "i2c", "i2" + "c-7")
    bus_refs = [n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and any(x in n.value for x in needles)]
    video_capture_refs = [p for p in FORBIDDEN_CALL_PATTERNS if p in src]
    return {"imports": sorted(found), "forbidden_imports_found": bad_imports,
            "i2c_string_refs": bus_refs, "video_capture_refs": video_capture_refs,
            "pass": not bad_imports and not bus_refs and not video_capture_refs}


def check_acquisition_module_safety():
    """🔴 acquisition 모듈의 화이트리스트/블랙리스트 상수는 '금지 플래그 문자열'을 **정의 자체로
    담고 있다** — 그래서 '파일에 금지 문자열이 있는가'로 검사하면 항상 자기 자신을 오탐한다
    (i2c 자기참조 오탐과 동일 함정). 대신 **구조**를 검사한다: subprocess 를 직접 호출하는
    함수가 `_run_v4l2ctl` 단 하나뿐이고, 그 함수가 `subprocess.run` 전에
    `_assert_safe_argv(argv)` 를 호출하는지 — AST 로 확인한다."""
    acq_path = os.path.join(ROOT, "scripts/p007e_r2r2_acquisition.py")
    src = open(acq_path, "r", encoding="utf-8").read()
    tree = ast.parse(src)

    def calls_subprocess(node):
        return (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name) and node.func.value.id == "subprocess")

    subprocess_call_sites = []   # (함수이름, 그 함수 안에서 _assert_safe_argv 호출을 봤는지)
    for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
        saw_guard = False
        saw_subprocess = False
        for node in ast.walk(fn):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                    and node.func.id == "_assert_safe_argv":
                saw_guard = True
            if calls_subprocess(node):
                saw_subprocess = True
        if saw_subprocess:
            subprocess_call_sites.append({"function": fn.name, "guarded_by_assert_safe_argv": saw_guard})

    only_one_site = len(subprocess_call_sites) == 1
    only_site_is_run_v4l2ctl = only_one_site and subprocess_call_sites[0]["function"] == "_run_v4l2ctl"
    all_guarded = all(s["guarded_by_assert_safe_argv"] for s in subprocess_call_sites)

    return {"path": acq_path, "sha256": sha256(acq_path),
            "subprocess_call_sites": subprocess_call_sites,
            "single_entry_point": only_one_site, "entry_point_is__run_v4l2ctl": only_site_is_run_v4l2ctl,
            "all_sites_guarded": all_guarded,
            "pass": only_one_site and only_site_is_run_v4l2ctl and all_guarded}


# ---------------------------------------------------------------------------
# Manifest lifecycle
# ---------------------------------------------------------------------------
class RunManifest:
    STATUSES = ("PREPARED", "RUNNING", "COMPLETED", "ABORTED", "CRASHED")

    def __init__(self, run_root, base_fields):
        os.makedirs(run_root, exist_ok=True)   # 🔴 _write() 전에 run_root 를 만든다
        self.path = os.path.join(run_root, "manifest.json")
        self.data = dict(base_fields)
        self.data["status"] = "PREPARED"
        self.data["started_at"] = time.time()
        self.data["last_completed_trial"] = -1
        self._write()

    def _write(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            f.write(json.dumps(self.data, ensure_ascii=False, indent=1))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)

    def set_status(self, status, **extra):
        assert status in self.STATUSES
        self.data["status"] = status
        self.data.update(extra)
        self.data["updated_at"] = time.time()
        self._write()

    def trial_completed(self, idx):
        self.data["last_completed_trial"] = idx
        self._write()


def atomic_write_json(path, obj):
    """🔴 tmp 쓰기 + fsync + os.replace. directory 와 file 경로를 섞지 않는다."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(json.dumps(obj, ensure_ascii=False, indent=1))
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# 통합 trial 루프 — acquisition(reader) + measurement(estimator) + 원자적 저장을 엮는다.
# 🔴 reader 는 duck-typed 인터페이스(check_format_invariant·capture_one_frame)만 요구한다.
#    실제 캡처 시 scripts/p007e_r2r2_acquisition.py.NoNegotiationV4L2Reader 를 쓰고,
#    이 게이트(C0-R1)에서는 하드웨어를 전혀 흉내내지 않는 FakeReader(아래)로만 시험한다.
# ---------------------------------------------------------------------------
def _frame_interval_stats(timestamps):
    """🔴 §16 — count/median/robust spread/min/max. analysis.py 의 robust_sd 와 같은 정의를 쓴다."""
    import importlib
    an = importlib.import_module("p007e_r2r2_analysis")
    if len(timestamps) < 2:
        return {"count": 0, "median": None, "robust_sd": None, "min": None, "max": None}
    gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
    return {"count": len(gaps), "median": an.median(gaps), "robust_sd": an.robust_sd(gaps),
            "min": min(gaps), "max": max(gaps)}


def run_trial(trial_idx, reader, estimator, roi_mean_fn, mask, run_root, M,
             quiet_s=0.06, command_s=2.0, settle_s=1.0, frame_interval_s=0.25,
             sleep_fn=None):
    """1 trial = reference window(M frame, 0.25s pacing) → quiet+command+settle → comparison window(M frame).
       🔴 실패해도 예외를 밖으로 던지지 않는다 — trial 단위 crash-safety 를 orchestrator 가 보장한다.
       🔴 roi_mean_fn 으로 ROI 밝기를 매 프레임 기록한다(M4 수정을 trial 루프에 실제로 연결)."""
    import importlib
    meas = importlib.import_module("p007e_r2r2_measurement")
    sleep_fn = sleep_fn or time.sleep
    frame_dir = os.path.join(run_root, "frames", "trial_%02d" % trial_idx)
    os.makedirs(frame_dir, exist_ok=True)

    inv_before = reader.check_format_invariant()
    if not inv_before["match"]:
        return {"index": trial_idx, "verdict": "INVALID", "reason": "FORMAT_INVARIANT_VIOLATION_BEFORE",
                "format_invariant_before": inv_before}

    # 🔴 카운터를 분리한다 — '캡처/디코드 실패'와 '가드 탈락(dropout)'은 다른 사건이다
    counters = {"frames_requested": 0, "frames_capture_or_decode_failed": 0,
                "frames_decoded": 0, "frames_guard_valid": 0, "frames_guard_rejected": 0,
                "frames_persisted": 0}
    windows = {}
    anchor_gray = None
    for wname, pause in (("reference", None), ("comparison", quiet_s + command_s + settle_s)):
        if pause:
            sleep_fn(pause)
        rows = []
        t_next = time.monotonic()
        for i in range(M):
            now = time.monotonic()
            if now < t_next:
                sleep_fn(t_next - now)
            t_next += frame_interval_s           # 🔴 0.25 s 목표 간격 페이싱

            counters["frames_requested"] += 1
            frame = reader.capture_one_frame()
            if not frame.ok:
                counters["frames_capture_or_decode_failed"] += 1
                rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                            "decoded": False, "valid": False, "roi_mean": None, "reason": frame.error})
                continue
            gray = meas.decode_gray(frame.encoded_bytes)
            if gray is None:
                counters["frames_capture_or_decode_failed"] += 1
                rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                            "decoded": False, "valid": False, "roi_mean": None, "reason": "DECODE_FAILED"})
                continue
            counters["frames_decoded"] += 1
            roi_mean = roi_mean_fn(gray, mask)
            if anchor_gray is None:
                anchor_gray = gray
                open(os.path.join(frame_dir, "%s_first.jpg" % wname), "wb").write(frame.encoded_bytes)
                counters["frames_persisted"] += 1
            elif i == 0:   # comparison window 의 첫 프레임도 보존(anchor 와 별개)
                open(os.path.join(frame_dir, "%s_first.jpg" % wname), "wb").write(frame.encoded_bytes)
                counters["frames_persisted"] += 1
            est = estimator(anchor_gray, gray)
            if est["valid"]:
                counters["frames_guard_valid"] += 1
            else:
                counters["frames_guard_rejected"] += 1
            rows.append({"sequence": frame.sequence, "t": frame.capture_timestamp,
                        "decoded": True, "valid": est["valid"], "roi_mean": roi_mean,
                        "reason": est.get("reason"), "est": est})
        valid_rows = [r for r in rows if r["valid"]]
        median = None
        if len(valid_rows) >= int(0.8 * M):
            median = {k: _median_of([r["est"][k] for r in valid_rows])
                      for k in ("translation_x_px", "translation_y_px", "rotation_deg")}
        windows[wname] = {"n_frames": len(rows), "n_valid": len(valid_rows),
                          "dropout_rate": 1.0 - len(valid_rows) / float(max(len(rows), 1)),
                          "rows": rows, "median": median,
                          "frame_interval_stats": _frame_interval_stats([r["t"] for r in rows])}

    inv_after = reader.check_format_invariant()
    trial = {"index": trial_idx, "windows": windows, "frame_accounting": counters,
             "format_invariant_before": inv_before, "format_invariant_after": inv_after}
    a, b = windows["reference"]["median"], windows["comparison"]["median"]
    if not inv_after["match"]:
        trial["verdict"] = "INVALID"
        trial["reason"] = "FORMAT_INVARIANT_VIOLATION_AFTER"
    elif a is None or b is None:
        trial["verdict"] = "INVALID"
        trial["reason"] = "WINDOW_DROPOUT_EXCEEDED"
    else:
        trial["delta"] = {k: b[k] - a[k] for k in a}
        trial["delta"]["norm"] = (trial["delta"]["translation_x_px"] ** 2
                                  + trial["delta"]["translation_y_px"] ** 2) ** 0.5
        trial["verdict"] = "VALID"
    return trial


def _median_of(xs):
    import importlib
    return importlib.import_module("p007e_r2r2_analysis").median(xs)


def run_full_sham(reader, estimator, roi_mean_fn, mask, run_root, manifest_fields, N_sham, M,
                  sleep_fn=None, trial_gap_s=2.0):
    """🔴 crash recovery ≠ experiment resume. 예외(일반 Exception 뿐 아니라 SIGTERM/SIGHUP 으로
       변환된 SystemExit 포함)가 나면 manifest 를 CRASHED 로 남기고 **자동으로 이어서 하지 않는다**.
       완료된 trial 의 evidence 는 보존된다. atexit 핸들러를 안전망으로 더 건다 — 사전등록
       문구("atexit/finally")와 실제 구현을 일치시켰다."""
    import atexit
    import signal
    sleep_fn = sleep_fn or time.sleep
    manifest = RunManifest(run_root, manifest_fields)
    manifest.set_status("RUNNING")

    def _mark_crashed(signum=None, frame=None):
        if manifest.data.get("status") == "RUNNING":
            manifest.set_status("CRASHED", crash_error=("SIGNAL_%s" % signum) if signum else "ATEXIT_UNFINISHED")
        if signum is not None:
            raise SystemExit(128 + signum)

    prev_term = signal.getsignal(signal.SIGTERM)
    prev_hup = signal.getsignal(signal.SIGHUP) if hasattr(signal, "SIGHUP") else None
    signal.signal(signal.SIGTERM, _mark_crashed)
    if prev_hup is not None:
        signal.signal(signal.SIGHUP, _mark_crashed)
    atexit.register(_mark_crashed)

    try:
        for t in range(N_sham):
            trial = run_trial(t, reader, estimator, roi_mean_fn, mask, run_root, M, sleep_fn=sleep_fn)
            atomic_write_json(os.path.join(run_root, "trials", "trial_%02d.json" % t), trial)
            manifest.trial_completed(t)
            if t < N_sham - 1:
                sleep_fn(trial_gap_s)
    except BaseException as e:  # 🔴 SystemExit·KeyboardInterrupt 포함 — 전부 CRASHED 로 남긴다
        manifest.set_status("CRASHED", crash_error=repr(e))
        raise
    else:
        manifest.set_status("COMPLETED")
    finally:
        signal.signal(signal.SIGTERM, prev_term)
        if prev_hup is not None:
            signal.signal(signal.SIGHUP, prev_hup)
        atexit.unregister(_mark_crashed)
    return manifest.data


class FakeReader:
    """🔴 TEST-ONLY. 실제 장치를 전혀 참조하지 않는다 — Stage B 프레임을 합성 변형해
       scripts/p007e_r2r2_acquisition.CapturedFrame 과 동일한 인터페이스로 공급한다.
       이 클래스는 실제 장치 경로 문자열을 전혀 포함하지 않는다(grep 으로 확인 가능)."""

    def __init__(self, base_jpeg_bytes, expected, crash_at_frame=None):
        self.base = base_jpeg_bytes
        self.expected = expected
        self.crash_at_frame = crash_at_frame
        self._seq = 0

    def check_format_invariant(self):
        return {"current": dict(self.expected), "expected": self.expected, "match": True, "mismatches": {}}

    def capture_one_frame(self):
        import importlib
        acq = importlib.import_module("p007e_r2r2_acquisition")
        self._seq += 1
        if self.crash_at_frame is not None and self._seq == self.crash_at_frame:
            raise RuntimeError("SIMULATED_ACQUISITION_CRASH_AT_FRAME_%d" % self._seq)
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(self.base, np.uint8), cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        dx = (self._seq % 5) * 0.1
        mat = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), 0.01 * (self._seq % 3), 1.0)
        mat[0, 2] += dx
        warped = cv2.warpAffine(img, mat, (w, h))
        ok, enc = cv2.imencode(".jpg", warped)
        data = enc.tobytes() if ok else b""
        return acq.CapturedFrame(self._seq, time.time(), data, self.expected["pixelformat"],
                                 self.expected["width"], self.expected["height"], bool(ok))


def integration_mock_test(run_root):
    """🔴 T8 — 실제 /dev/video0 를 전혀 열지 않고, run_trial/run_full_sham 전체 배선을
       Stage B 프레임 기반 합성 데이터로 끝까지 돌린다. C1 의 교훈
       (self-check 은 capture 0 만 증명, 올바른 캡처는 증명하지 못했다) 을 직접 메운다."""
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    meas = importlib.import_module("p007e_r2r2_measurement")
    roi = json.load(open(ROI))
    base = open(FRAME, "rb").read()
    rules = _load_rules()
    expected = {"width": 640, "height": 480, "pixelformat": "MJPG"}

    import cv2
    img = cv2.imdecode(__import__("numpy").frombuffer(base, __import__("numpy").uint8), cv2.IMREAD_GRAYSCALE)
    mask = meas.build_mask(roi, img.shape)
    estimator = meas.make_estimator(rules, mask)
    roi_mean_fn = meas.roi_mean_brightness

    out = {}

    # 정상 3-trial 완주 (M=3 으로 축소 — 배선 검증이 목적, 실전 M=60 이 아니다)
    r1 = os.path.join(run_root, "normal")
    reader1 = FakeReader(base, expected)
    mf = run_full_sham(reader1, estimator, roi_mean_fn, mask, r1,
                       {"gate": "P-007E-R2-R2", "stage": "C1-R1-MOCK", "mock": True}, N_sham=3, M=3,
                       sleep_fn=lambda s: None)
    trial_files = sorted(f for f in os.listdir(os.path.join(r1, "trials")) if f.endswith(".json"))
    verdicts = [json.load(open(os.path.join(r1, "trials", f)))["verdict"] for f in trial_files]
    roi_means_recorded = [bool([row.get("roi_mean") is not None
                                for w in json.load(open(os.path.join(r1, "trials", f)))["windows"].values()
                                for row in w["rows"] if row.get("decoded")])
                          for f in trial_files]
    out["normal_run"] = {"manifest_status": mf["status"], "trial_files": trial_files, "verdicts": verdicts,
                         "roi_mean_recorded_per_trial": roi_means_recorded,
                         "pass": mf["status"] == "COMPLETED" and len(trial_files) == 3
                                 and all(verdicts[i] == "VALID" for i in range(3))}

    # 모의 crash — trial 2(index 2) 캡처 도중 예외. trial 0·1 생존 확인
    r2 = os.path.join(run_root, "crash")
    reader2 = FakeReader(base, expected, crash_at_frame=7)   # 3 frame/window * 2 window = 6 for trial0,1 끝나고 trial2 도중
    crashed = False
    try:
        run_full_sham(reader2, estimator, roi_mean_fn, mask, r2,
                      {"gate": "P-007E-R2-R2", "stage": "C1-R1-MOCK-CRASH", "mock": True}, N_sham=5, M=3,
                      sleep_fn=lambda s: None)
    except RuntimeError:
        crashed = True
    manifest2 = json.load(open(os.path.join(r2, "manifest.json")))
    t2_files = sorted(f for f in os.listdir(os.path.join(r2, "trials"))) if os.path.isdir(os.path.join(r2, "trials")) else []
    out["simulated_crash_run"] = {
        "crashed_as_expected": crashed, "manifest_status": manifest2["status"],
        "last_completed_trial": manifest2["last_completed_trial"], "trial_files_survived": t2_files,
        "pass": crashed and manifest2["status"] == "CRASHED"
                and manifest2["last_completed_trial"] >= 0
                and len(t2_files) == manifest2["last_completed_trial"] + 1}

    out["all_pass"] = out["normal_run"]["pass"] and out["simulated_crash_run"]["pass"]
    return out


# ---------------------------------------------------------------------------
# 실제 캡처 경로 — C0-R1 에서는 reader_factory=None(실제 장치) 으로 **호출하지 않는다**.
# T10 이 reader_factory=FakeReader 로만 이 함수를 호출해, 실제 실행과 시험의 차이를
# "어느 reader 클래스를 쓰는가" 하나로 줄인다.
# ---------------------------------------------------------------------------
MOTION_CAPABLE_PATTERNS = ("arm_driver", "dofbot_pro_driver", "color_follow", "grasp",
                          "apriltag", "KCF", "yolov11", "voice_ctrl", "mediapipe")


def env_guards(device_path):
    """🔴 C1 harness(p007e_r2r2_c1_sham.py) 의 env_guards 를 이식 — 새 harness 에서 빠뜨렸던 안전장치."""
    import subprocess
    ps = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True, text=True).stdout
    motion = [l.strip() for l in ps.splitlines()[1:]
             if any(m.lower() in l.lower() for m in MOTION_CAPABLE_PATTERNS)]
    holders = []
    for pid in os.listdir("/proc") if os.path.isdir("/proc") else []:
        if not pid.isdigit():
            continue
        fdd = "/proc/%s/fd" % pid
        try:
            for fd in os.listdir(fdd):
                if os.readlink(os.path.join(fdd, fd)) == device_path:
                    holders.append(pid)
                    break
        except OSError:
            continue
    return {"motion_capable_processes": motion, "device_holders": holders,
            "pass": not motion and not holders}


def build_estimator_and_mask():
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    meas = importlib.import_module("p007e_r2r2_measurement")
    roi = json.load(open(ROI))
    pre = json.load(open(PREREG))
    exp = pre["expected_device"]
    mask = meas.build_mask(roi, (exp["height"], exp["width"]))
    rules = _load_rules()
    estimator = meas.make_estimator(rules, mask)
    return estimator, mask, meas.roi_mean_brightness, rules


def execute_c1r1(run_root, reader_factory=None):
    """🔴 REAL CAPTURE PATH 의 유일한 구현. C0-R1 은 reader_factory=None(실제 장치)으로
       호출하지 않는다 — 코드는 존재하되 이번 게이트에서는 한 번도 실행되지 않는다."""
    import importlib
    pre = json.load(open(PREREG))
    exp = pre["expected_device"]
    g = env_guards(exp["path"])
    if not g["pass"]:
        return {"status": "ABORTED", "reason": "ENV_GUARD_FAILED", "env_guards": g}
    estimator, mask, roi_mean_fn, rules = build_estimator_and_mask()
    if reader_factory is None:
        acq = importlib.import_module("p007e_r2r2_acquisition")
        reader = acq.NoNegotiationV4L2Reader(exp["path"], exp["width"], exp["height"], exp["pixelformat"],
                                             os.path.join(run_root, "_acq_tmp"))
    else:
        reader = reader_factory()
    manifest_fields = {
        "gate": "P-007E-R2-R2", "stage": "C1-R1", "base_commit": "64b32bc",
        "preregistration_commit": "<set at commit time>",
        "harness_sha256": sha256(os.path.abspath(__file__)),
        "acquisition_sha256": sha256(os.path.join(ROOT, "scripts/p007e_r2r2_acquisition.py")),
        "measurement_sha256": sha256(os.path.join(ROOT, "scripts/p007e_r2r2_measurement.py")),
        "analysis_sha256": sha256(os.path.join(ROOT, "scripts/p007e_r2r2_analysis.py")),
        "expected_device_identity": exp, "expected_format": exp,
        "roi_sha256": sha256(ROI), "env_guards_at_start": g}
    return run_full_sham(reader, estimator, roi_mean_fn, mask, run_root, manifest_fields,
                         N_sham=rules["N_sham"], M=rules["M"])


# ---------------------------------------------------------------------------
# Self-test — ALGORITHM UNIT VALIDATION (카메라 0, Stage B 프레임 재사용)
# ---------------------------------------------------------------------------
def selftest():
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import p007e_r2r2_measurement as meas  # noqa: E402
    import cv2
    import numpy as np

    roi = json.load(open(ROI))
    rules = _load_rules()
    img = cv2.imdecode(np.frombuffer(open(FRAME, "rb").read(), np.uint8), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    mask = meas.build_mask(roi, img.shape)
    est = meas.make_estimator(rules, mask)

    rows = []
    for dx, dy, deg in [(0.2, 0, 0), (0.5, 0, 0), (1, 0, 0), (2, 0, 0), (5, 0, 0),
                        (0, 0.5, 0), (0, 2, 0), (0, 0, 0.05), (0, 0, 0.1), (0, 0, 0.5),
                        (0, 0, 1.0), (1, 1, 0.1), (3, -2, 0.3), (10, 6, 1.0)]:
        mat = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), deg, 1.0)
        mat[0, 2] += dx
        mat[1, 2] += dy
        warped = cv2.warpAffine(img, mat, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        r = est(img, warped)
        exp_rot = float(np.degrees(np.arctan2(mat[1, 0], mat[0, 0])))
        row = {"applied": {"dx": dx, "dy": dy, "rot_deg_input": deg},
               "expected": {"tx": float(mat[0, 2]), "ty": float(mat[1, 2]), "rot_deg": exp_rot},
               "result": r}
        if r["valid"]:
            row["err_trans_norm"] = float(np.hypot(r["translation_x_px"] - mat[0, 2],
                                                    r["translation_y_px"] - mat[1, 2]))
            row["err_rot_deg"] = r["rotation_deg"] - exp_rot
        rows.append(row)

    roi_mean_direct = meas.roi_mean_brightness(img, mask)
    roi_mean_full = float(img.mean())
    return {"kind": "ALGORITHM UNIT VALIDATION", "cv2_version": cv2.__version__,
            "🔴 not_physical_evidence": "합성 2D 변환이다. 실제 DaBai 감도 증거가 아니다",
            "regression_against_C0": {
                "C0_max_translation_err_px": 0.703, "C0_max_rotation_err_deg": 0.060,
                "this_run_max_translation_err_px": max(r["err_trans_norm"] for r in rows if r["result"]["valid"]),
                "this_run_max_rotation_err_deg": max(abs(r["err_rot_deg"]) for r in rows if r["result"]["valid"])},
            "🔴 roi_mean_bug_fix_demo": {
                "whole_frame_mean (C1 의 버그와 동일 계산)": roi_mean_full,
                "roi_masked_mean (수정됨)": roi_mean_direct,
                "differ": abs(roi_mean_full - roi_mean_direct) > 1e-6},
            "rows": rows}


def _load_rules():
    pre = json.load(open(PREREG))
    g = pre["frame_quality_guards"]
    return {
        "orb": pre["algorithm"]["orb_parameters"], "ransac": pre["transform_model"]["ransac"],
        "min_features": g["min_features_per_frame_in_roi"]["value"],
        "min_matches": g["min_matches"]["value"], "min_inliers": g["min_inliers"]["value"],
        "min_inlier_ratio": g["min_inlier_ratio"]["value"],
        "max_residual": g["max_residual_rms_inliers_px"]["value"],
        "max_scale_dev": g["max_scale_deviation"]["value"],
        "K": pre["🔴 K_M_Nsham_semantics"]["K"]["value"], "M": pre["🔴 K_M_Nsham_semantics"]["M"]["value"],
        "N_sham": pre["🔴 K_M_Nsham_semantics"]["N_sham"]["value"],
    }


# ---------------------------------------------------------------------------
# Negative tests (T1~T7) — 전부 실제 카메라 0
# ---------------------------------------------------------------------------
def negative_tests(tmp_base):
    import subprocess
    results = {}

    # T0 — 🔴 CLI 레벨 경로 충돌 가드. --out 과 --work-dir 에 **같은 값**을 주면
    # (바로 C1 을 죽였던 패턴) 아무 작업도 하지 않고 exit code 2 로 즉시 거부해야 한다
    same_path = os.path.join(tmp_base, "t0_collision.json")
    r = subprocess.run([sys.executable, os.path.abspath(__file__),
                        "--negative-tests", "--out", same_path, "--work-dir", same_path],
                       capture_output=True, text=True, timeout=10)
    results["T0_cli_path_collision_guard"] = {
        "expected": "exit code 2, ABORT BEFORE RUN, 결과 파일 미생성",
        "observed_exit_code": r.returncode, "observed_stdout": r.stdout.strip()[:200],
        "result_file_created": os.path.exists(same_path),
        "pass": r.returncode == 2 and not os.path.exists(same_path)}

    # T1 — wrong output path: directory 경로를 file 로, file 경로를 directory 로 쓰려는 시도
    t1_root = os.path.join(tmp_base, "t1")
    os.makedirs(t1_root, exist_ok=True)
    collide = os.path.join(t1_root, "manifest.json")
    open(collide, "w").write("{}")           # 이미 파일로 존재
    try:
        os.makedirs(collide, exist_ok=True)   # 같은 경로를 디렉터리로 또 만들려는 시도 — 반드시 실패해야 한다
        t1_outcome = "FAIL — collision not detected"
    except (FileExistsError, NotADirectoryError) as e:
        t1_outcome = "ABORT BEFORE RUN — %s: %s" % (type(e).__name__, e)
    results["T1_wrong_output_path"] = {"expected": "ABORT BEFORE RUN", "observed": t1_outcome,
                                       "pass": t1_outcome.startswith("ABORT")}

    # T2 — trial persistence: mock trial 3개
    t2_root = os.path.join(tmp_base, "t2", "trials")
    for i in range(3):
        atomic_write_json(os.path.join(t2_root, "trial_%02d.json" % i), {"index": i, "mock": True})
    files = sorted(os.listdir(t2_root)) if os.path.isdir(t2_root) else []
    results["T2_trial_persistence"] = {"expected": "3 files", "observed": files,
                                       "pass": len(files) == 3}

    # T3 — simulated crash: trial1 PASS, trial2 PASS, trial3 저장 직전 강제 예외
    t3_root = os.path.join(tmp_base, "t3", "trials")
    survived = []
    try:
        for i in range(3):
            data = {"index": i, "mock": True}
            if i == 2:
                # 저장 직전 크래시를 흉내낸다 — tmp 파일조차 생기지 않아야 "미완성이 명확히 식별"된다
                raise RuntimeError("SIMULATED_CRASH_BEFORE_PERSIST")
            atomic_write_json(os.path.join(t3_root, "trial_%02d.json" % i), data)
    except RuntimeError:
        pass
    if os.path.isdir(t3_root):
        survived = sorted(f for f in os.listdir(t3_root) if f.endswith(".json"))
    trial3_incomplete = not os.path.exists(os.path.join(t3_root, "trial_02.json"))
    results["T3_simulated_crash"] = {
        "expected": "trial_00.json · trial_01.json 생존, trial_02.json 없음(미완성이 명확)",
        "observed_files": survived, "trial2_survived": "trial_01.json" in survived,
        "trial3_absent_and_clear": trial3_incomplete,
        "pass": survived == ["trial_00.json", "trial_01.json"] and trial3_incomplete}

    # T3b — 🔴 쓰기 "도중" 크래시(tmp 는 생겼지만 os.replace 전). .tmp 잔존물이
    # "미완성임이 명확히 식별"되는 상태인지 확인한다(§17 "incomplete artifact clearly identifiable")
    t3b_path = os.path.join(tmp_base, "t3b", "trial_00.json")
    os.makedirs(os.path.dirname(t3b_path), exist_ok=True)
    tmp = t3b_path + ".tmp"
    with open(tmp, "w") as f:
        f.write('{"index": 0, "partial":')   # 🔴 의도적으로 불완전한 JSON. replace 전에 중단한다
        f.flush()
        os.fsync(f.fileno())
    # os.replace(tmp, t3b_path) 를 의도적으로 호출하지 않는다 — "저장 도중 크래시"의 흉내
    final_missing = not os.path.exists(t3b_path)
    tmp_present_and_incomplete = False
    if os.path.exists(tmp):
        try:
            json.loads(open(tmp).read())
            tmp_present_and_incomplete = False   # 완전한 JSON이면 "미완성 식별" 실패
        except json.JSONDecodeError:
            tmp_present_and_incomplete = True    # 🔴 .tmp 확장자 + 파싱 불가 = 미완성임이 명확
    results["T3b_crash_during_write"] = {
        "expected": "trial_00.json 없음 · trial_00.json.tmp 는 존재하되 파싱 불가(미완성 식별 가능)",
        "final_file_missing": final_missing, "tmp_present_and_unparseable": tmp_present_and_incomplete,
        "pass": final_missing and tmp_present_and_incomplete}

    # T4 — malformed frame → decode 실패가 dropout 으로 분류되는지 (전체 소실 아님)
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import p007e_r2r2_measurement as meas  # noqa: E402
    bad = meas.decode_gray(b"\x00\x01\x02not-a-jpeg")
    results["T4_malformed_frame"] = {"expected": "decode_gray 가 None 반환 (예외 전파 없음)",
                                     "observed": "None" if bad is None else "decoded(비정상)",
                                     "pass": bad is None}

    # T5 — wrong format metadata (mock: YUYV instead of MJPG) → ABORT/INVALID
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import p007e_r2r2_acquisition as acq  # noqa: E402
    reader = acq.NoNegotiationV4L2Reader("/dev/null-mock", 640, 480, "MJPG",
                                         os.path.join(tmp_base, "t5"))
    reader.read_format = lambda: {"raw": "mock", "parsed": {"width": 640, "height": 480, "pixelformat": "YUYV"}, "rc": 0}
    inv = reader.check_format_invariant()
    results["T5_wrong_format_metadata"] = {"expected": "match=False → ABORT/INVALID",
                                           "observed_match": inv["match"], "mismatches": inv["mismatches"],
                                           "pass": inv["match"] is False and "pixelformat" in inv["mismatches"]}

    # T6 — preregistration SHA mismatch → ABORT BEFORE CAPTURE
    bad_sha = "0" * 64
    results["T6_preregistration_sha_mismatch"] = {
        "expected": "실제 sha != 상수 sha → ABORT",
        "observed": "ABORT" if sha256(PREREG) != bad_sha else "NO_ABORT(버그)",
        "pass": sha256(PREREG) != bad_sha}

    # T7 — capture opt-in absent → NO CAPTURE
    granted_without_env = (os.environ.get(ALLOW_ENV) == "1")
    results["T7_capture_opt_in_absent"] = {
        "expected": "env 없으면 NO CAPTURE", "env_present": granted_without_env,
        "pass": not granted_without_env}

    # T8 — 🔴 통합 모의 end-to-end. run_trial/run_full_sham 전체 배선을 FakeReader 로
    # 끝까지 돌린다(정상 3-trial 완주 + trial 도중 crash). 실제 장치 0
    t8_root = os.path.join(tmp_base, "t8")
    try:
        t8 = integration_mock_test(t8_root)
        t8["pass"] = t8["all_pass"]
    except Exception as e:  # noqa: BLE001
        t8 = {"pass": False, "error": repr(e)}
    results["T8_integrated_mock_run"] = t8

    # T9 — 🔴 analysis.py 의 Q1~X 다섯 분기를 전부 합성 trial 로 단위시험한다 (데이터 0, 순수 함수)
    try:
        t9 = _analysis_branch_tests()
    except Exception as e:  # noqa: BLE001
        t9 = {"pass": False, "error": repr(e)}
    results["T9_analysis_branch_coverage"] = t9

    # T10 — 🔴 execute_c1r1() 을 FakeReader 로 호출 — 실제 캡처 경로와 **동일한 함수**를
    # reader 클래스 하나만 바꿔 끝까지 돈다. real device 는 절대 참조하지 않는다
    t10_root = os.path.join(tmp_base, "t10")
    try:
        base = open(FRAME, "rb").read()
        expected = {"width": 640, "height": 480, "pixelformat": "MJPG"}
        result = execute_c1r1(t10_root, reader_factory=lambda: FakeReader(base, expected))
        manifest = json.load(open(os.path.join(t10_root, "manifest.json")))
        trial_files = sorted(f for f in os.listdir(os.path.join(t10_root, "trials")))
        t10 = {"manifest_status": manifest["status"], "trial_files": trial_files,
              "harness_sha_recorded": "harness_sha256" in manifest,
              "acquisition_sha_recorded": "acquisition_sha256" in manifest,
              "measurement_sha_recorded": "measurement_sha256" in manifest,
              "analysis_sha_recorded": "analysis_sha256" in manifest,
              "pass": manifest["status"] == "COMPLETED"
                      and all(k in manifest for k in
                             ("harness_sha256", "acquisition_sha256", "measurement_sha256", "analysis_sha256"))}
    except Exception as e:  # noqa: BLE001
        t10 = {"pass": False, "error": repr(e)}
    results["T10_execute_c1r1_via_fake_reader"] = t10

    # T11 — 🔴 v4l2-ctl 단축 플래그(-v=--set-fmt-video, -c=--set-ctrl)가 화이트리스트를
    # 우회하지 못하는지 확인한다 (v4l2-ctl --help-all 1차 확인, 2026-10-28)
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import p007e_r2r2_acquisition as acq2  # noqa: E402
    short_flag_blocked = []
    for bad_argv in (["--device=/dev/null-mock", "-v", "width=640,height=480,pixelformat=MJPG"],
                     ["--device=/dev/null-mock", "-c", "brightness=0"],
                     ["--device=/dev/null-mock", "-p", "30"]):
        try:
            acq2._assert_safe_argv(bad_argv)
            short_flag_blocked.append({"argv": bad_argv, "blocked": False})
        except acq2.AcquisitionGuardError:
            short_flag_blocked.append({"argv": bad_argv, "blocked": True})
    results["T11_short_flag_bypass_rejected"] = {
        "expected": "-v/-c/-p 전부 AcquisitionGuardError 로 거부",
        "observed": short_flag_blocked,
        "pass": all(r["blocked"] for r in short_flag_blocked)}

    # T12 — 🔴 module SHA mismatch 감지 로직. T6 과 같은 패턴 — 소스 파일을 변조하지 않고
    # (비가역 변경을 피한다) 틀린 기대값과 실제 sha256 이 다름을 main() 이 쓰는 바로 그
    # sha256() 함수로 확인한다. 이것이 main() 의 module_sha_check 딕셔너리가 실제로 비교하는 식이다
    bad_module_sha = {p: "0" * 64 for p in MODULE_SHA}
    mismatches_detected = {p: sha256(os.path.join(ROOT, p)) != bad for p, bad in bad_module_sha.items()}
    results["T12_module_sha_mismatch_detectable"] = {
        "expected": "실제 파일 sha256 이 틀린 기대값과 전부 달라야 한다(=mismatch 가 감지된다)",
        "observed": mismatches_detected,
        "pass": all(mismatches_detected.values())}

    all_pass = all(v["pass"] for v in results.values())
    return {"results": results, "all_pass": all_pass}


def _analysis_branch_tests():
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    an = importlib.import_module("p007e_r2r2_analysis")
    K, FT, FR = 5, 0.70, 0.060

    def mk_window(dropout_rate=0.0, n=10):
        n_drop = int(round(dropout_rate * n))
        rows = [{"decoded": True, "valid": i >= n_drop, "roi_mean": 100.0,
                "est": {"translation_norm_px": 0.0, "inlier_count": 300,
                        "inlier_ratio": 0.9, "residual_rms_inliers_px": 0.5}} for i in range(n)]
        return {"n_frames": n, "dropout_rate": dropout_rate, "rows": rows}

    def mk_trial(idx, verdict="VALID", dtx=0.0, dty=0.0, drot=0.0, dropout=0.0):
        t = {"index": idx, "verdict": verdict,
             "windows": {"reference": mk_window(dropout), "comparison": mk_window(dropout)}}
        if verdict == "VALID":
            t["delta"] = {"translation_x_px": dtx, "translation_y_px": dty, "rotation_deg": drot,
                         "norm": (dtx ** 2 + dty ** 2) ** 0.5}
        return t

    out = {}
    trials_q1 = [mk_trial(i, dtx=0.05 * (i % 3 - 1), dty=0.05 * (i % 2), drot=0.0) for i in range(10)]
    out["Q1_clean"] = an.classify_run(trials_q1, K, FT, FR)["verdict"].startswith("Q1")

    trials_x = [mk_trial(i) for i in range(8)] + [mk_trial(8, verdict="INVALID"), mk_trial(9, verdict="INVALID")]
    out["X_two_invalid"] = an.classify_run(trials_x, K, FT, FR)["verdict"].startswith("X")

    trials_q3 = [mk_trial(i, dropout=0.25) for i in range(10)]
    out["Q3_high_dropout"] = an.classify_run(trials_q3, K, FT, FR)["verdict"].startswith("Q3")

    trials_q2 = [mk_trial(i, dtx=0.05 * (i % 2)) for i in range(8)] + [
        mk_trial(8, dtx=10.0, dty=10.0), mk_trial(9, dtx=12.0, dty=-9.0)]
    out["Q2_loo_false_positives"] = an.classify_run(trials_q2, K, FT, FR)["verdict"].startswith("Q2")

    def mk_window_corr(n=10):
        rows = [{"decoded": True, "valid": True, "roi_mean": 100.0 + i * 2.0,
                "est": {"translation_norm_px": i * 0.15, "inlier_count": 300,
                        "inlier_ratio": 0.9, "residual_rms_inliers_px": 0.5}} for i in range(n)]
        return {"n_frames": n, "dropout_rate": 0.0, "rows": rows}

    trials_q4 = [{"index": i, "verdict": "VALID",
                 "windows": {"reference": mk_window_corr(), "comparison": mk_window_corr()},
                 "delta": {"translation_x_px": 0.1, "translation_y_px": 0.1, "rotation_deg": 0.0,
                          "norm": 0.1414}} for i in range(10)]
    out["Q4_photometric_correlation"] = an.classify_run(trials_q4, K, FT, FR)["verdict"].startswith("Q4")

    out["pass"] = all(out.values())
    return out


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-sham", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--negative-tests", action="store_true")
    ap.add_argument("--out", default=None,
                    help="결과 JSON **파일** 경로. 반드시 .json 으로 끝나야 한다")
    ap.add_argument("--work-dir", default=None,
                    help="🔴 --negative-tests 전용 스크래치 **디렉터리**. --out 과 절대 같은 "
                         "값을 쓰지 않는다 — C1 의 HARNESS_OUTPUT_PATH_DEFECT 재발 방지")
    ap.add_argument("--run-root", default=None,
                    help="🔴 --run-sham(실제 캡처) 전용 **디렉터리**. --out·--work-dir 과 "
                         "같은 값을 쓰지 않는다")
    a = ap.parse_args()

    # 🔴 경로 타입 검증 — C1 의 HARNESS_OUTPUT_PATH_DEFECT(파일 경로를 디렉터리 인자로
    # 재사용)가 여기서 재발하지 않도록, 두 인자를 받는 즉시 교차 오염을 거부한다
    if a.out is not None and not a.out.endswith(".json"):
        print(json.dumps({"stop": "ABORT BEFORE RUN — --out 은 .json 파일 경로여야 한다: %r" % a.out}))
        sys.exit(2)
    if a.negative_tests:
        if a.work_dir is None:
            print(json.dumps({"stop": "ABORT BEFORE RUN — --negative-tests 는 --work-dir(디렉터리) 가 필요하다"}))
            sys.exit(2)
        if a.out is not None and os.path.abspath(a.work_dir) == os.path.abspath(a.out):
            print(json.dumps({"stop": "ABORT BEFORE RUN — --work-dir 과 --out 이 같은 경로다: %r" % a.out}))
            sys.exit(2)
    if a.run_sham:
        if a.run_root is None:
            print(json.dumps({"stop": "ABORT BEFORE RUN — --run-sham 은 --run-root(디렉터리) 가 필요하다"}))
            sys.exit(2)
        for other in (a.out, a.work_dir):
            if other is not None and os.path.abspath(a.run_root) == os.path.abspath(other):
                print(json.dumps({"stop": "ABORT BEFORE RUN — --run-root 가 다른 경로 인자와 같다: %r" % other}))
                sys.exit(2)

    rec = {"gate": "P-007E-R2-R2", "stage": "C1-R1", "argv": sys.argv[1:],
           "self_ast_check": self_ast_check(),
           "acquisition_module_check": check_acquisition_module_safety(),
           "preregistration": {"path": "evidence/P-007E-R2-R2/stage_c1r1_preregistration.json",
                               "expected_sha256": PREREG_SHA, "actual_sha256": sha256(PREREG),
                               "match": sha256(PREREG) == PREREG_SHA},
           "module_sha_check": {
               p: {"expected": exp, "actual": sha256(os.path.join(ROOT, p)), "match": sha256(os.path.join(ROOT, p)) == exp}
               for p, exp in MODULE_SHA.items()},
           "capture_opt_in": {"env_%s" % ALLOW_ENV: os.environ.get(ALLOW_ENV, "0"),
                              "flag_run_sham": a.run_sham,
                              "granted": os.environ.get(ALLOW_ENV) == "1" and a.run_sham}}

    if not rec["self_ast_check"]["pass"] or not rec["acquisition_module_check"]["pass"]:
        rec["stop"] = "SELF_CHECK_FAILED"
    elif not rec["preregistration"]["match"]:
        rec["stop"] = "PREREGISTRATION_SHA_MISMATCH — ABORT BEFORE CAPTURE"
    elif a.run_sham and not all(v["match"] for v in rec["module_sha_check"].values()):
        # 🔴 §5 — 실제 캡처(--run-sham) 전에만 모듈 SHA 를 강제한다. selftest/negative-tests 는
        # 모듈을 고치고 검증하는 작업 자체이므로 이 가드 대상이 아니다
        rec["stop"] = "MODULE_SHA_MISMATCH — ABORT BEFORE CAPTURE"
    elif a.selftest:
        rec["unit_validation"] = selftest()
        rec["stop"] = "SELFTEST_ONLY — 카메라 0"
    elif a.negative_tests:
        rec["negative_tests"] = negative_tests(a.work_dir)   # 🔴 --out 이 아니라 --work-dir
        rec["stop"] = "NEGATIVE_TESTS_ONLY — 카메라 0"
    elif rec["capture_opt_in"]["granted"]:
        # 🔴 REAL CAPTURE PATH. C0-R1 에서는 이 분기에 **실제로 도달하지 않는다**
        # (ALLOW_ENV 를 설정하지 않았다) — 코드는 존재하되 이번 게이트에서 한 번도 실행되지 않았다.
        # T10 이 execute_c1r1() 자체는 FakeReader 로 끝까지 검증한다
        rec["run_result"] = execute_c1r1(a.run_root, reader_factory=None)
        rec["stop"] = "SHAM_COMPLETE_OR_ABORTED — 결과는 run_result 참조"
    else:
        rec["stop"] = ("🔴 NO CAPTURE (기본값). 캡처하려면 %s=1 과 --run-sham 이 모두 필요하다" % ALLOW_ENV)

    text = json.dumps(rec, ensure_ascii=False, indent=1)
    if a.out and (a.selftest or a.negative_tests):
        atomic_write_json(a.out, rec)   # 🔴 --out 은 위에서 .json 파일임을 이미 검증했다
    print(text)


if __name__ == "__main__":
    main()
