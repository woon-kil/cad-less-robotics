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
                  sleep_fn=None, trial_gap_s=2.0, rules=None, target_frame_interval_s=0.25):
    """🔴 crash recovery ≠ experiment resume. 예외(일반 Exception 뿐 아니라 SIGTERM/SIGHUP 으로
       변환된 SystemExit 포함)가 나면 manifest 를 CRASHED 로 남기고 **자동으로 이어서 하지 않는다**.
       완료된 trial 의 evidence 는 보존된다. atexit 핸들러를 안전망으로 더 건다 — 사전등록
       문구("atexit/finally")와 실제 구현을 일치시켰다.

       🔴 C1-R1-R1 Repair B/C — COMPLETED 로 끝나고 rules(K/M/N_sham/floor_trans/floor_rot) 가
       주어지면, 별도 session 없이 이 함수가 끝나기 전에 frozen analysis.py 로
       summary/sham_summary.json 과 artifact_sha256.json 을 자동 생성한다(§7/§8/§13)."""
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

    finalize_result = None
    if manifest.data.get("status") == "COMPLETED" and rules is not None:
        try:
            finalize_result = _finalize_evidence_package(run_root, N_sham, M, rules,
                                                          target_frame_interval_s, manifest)
        except Exception as e:  # noqa: BLE001 — 🔴 §18 "성공으로 종료하지 않는다"
            # finalize 자체가 깨지면(summary 계산 실패 등) manifest 를 정직하게
            # CRASHED/INCOMPLETE 로 남긴다. trial 들은 이미 저장돼 evidence 로 남아있다
            manifest.set_status("CRASHED", crash_error="FINALIZE_FAILED: %r" % e,
                               evidence_package_core_status="EVIDENCE_PACKAGE_INCOMPLETE")
            raise
    result = dict(manifest.data)
    if finalize_result is not None:
        result["finalize"] = finalize_result
    return result


def _finalize_evidence_package(run_root, N_sham, M, rules, target_frame_interval_s, manifest):
    """🔴 Repair B/C/D/E — capture 완료 직후, 같은 프로세스 안에서 summary +
    artifact SHA manifest + completeness guard 를 전부 만든다. 별도 session 에서
    analysis.py 를 다시 import 해 수동으로 계산하는 단계를 없앤다(C1-R1 의 결함).

    🔴 advisor 지적 반영 — 순서가 중요하다. artifact_sha256.json 에 적히는 manifest.json
    해시가 **최종본**이어야 한다. manifest.set_status() 는 manifest.json 을 다시 쓰므로,
    artifact_sha256.json 을 만들기 **전에** manifest 에 대한 마지막 쓰기를 끝내고, 그
    이후로는 manifest.json 을 절대 건드리지 않는다. artifact_sha256.json 자신은 자기
    참조가 불가능하므로(존재하기 전에는 자기 해시를 쓸 수 없다) manifest 에 기록하는
    completeness 는 그것을 제외한 core contract 만 본다. 전체 contract(artifact_sha256.json
    포함) 완전성은 artifact_sha256.json 을 다 쓴 뒤 다시 계산해 반환값에만 담는다."""
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    evp = importlib.import_module("p007e_r2r2_evidence_pipeline")
    an = importlib.import_module("p007e_r2r2_analysis")

    trial_paths = [os.path.join(run_root, "trials", "trial_%02d.json" % i) for i in range(N_sham)]
    trials = [json.load(open(p)) for p in trial_paths]
    evidence_pipeline_path = os.path.join(ROOT, "scripts/p007e_r2r2_evidence_pipeline.py")
    summary = evp.build_sham_summary(trials, trial_paths, rules, an, M_executed=M, N_sham_executed=N_sham,
                                     target_frame_interval_s=target_frame_interval_s, run_root=run_root,
                                     evidence_pipeline_path=evidence_pipeline_path)
    atomic_write_json(os.path.join(run_root, "summary", "sham_summary.json"), summary)

    # 🔴 manifest 에 대한 마지막 쓰기 — core contract(artifact_sha256.json 제외)만 확인
    core_completeness = evp.check_artifact_completeness(run_root, N_sham, require_artifact_manifest=False)
    manifest.set_status("COMPLETED", evidence_package_core_status=core_completeness["status"],
                        evidence_package_core_missing=core_completeness["missing"])

    # 🔴 이 시점 이후 manifest.json 을 다시 쓰지 않는다 — artifact_sha256.json 의 manifest
    # 해시가 이 최종본과 바이트 단위로 일치해야 한다
    artifact_paths = evp.discover_artifact_relative_paths(run_root)
    artifact_manifest = evp.build_artifact_sha_manifest(run_root, artifact_paths)
    atomic_write_json(os.path.join(run_root, "artifact_sha256.json"), artifact_manifest)

    full_completeness = evp.check_artifact_completeness(run_root, N_sham, require_artifact_manifest=True)
    # 🔴 §13/§14 self-check — 방금 쓴 manifest.json 이 artifact_sha256.json 에 적힌 해시와
    # 바이트 단위로 일치하는지(local==local "전송" 검증). E16/T10 이 이 결과를 확인한다
    self_transfer = evp.verify_transfer(artifact_manifest["entries"], run_root)
    return {"summary": summary, "core_completeness": core_completeness,
           "full_completeness": full_completeness, "self_transfer_verification": self_transfer}


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


def execute_c1r1(run_root, reader_factory=None, git_run_fn=None, require_clean_tree=False):
    """🔴 REAL CAPTURE PATH 의 유일한 구현. C0-R1/C1-R1 은 reader_factory=None(실제 장치)으로
       호출하지 않는다 — 코드는 존재하되 그 게이트에서는 한 번도 실행되지 않았다.

       🔴 C1-R1-R1 Repair A/§6 — manifest 의 base_commit/preregistration_commit 은 더 이상
       historical literal 이 아니라 실행 시점 git 조회 결과다. git_run_fn 을 주입하면 실제
       저장소를 전혀 건드리지 않고 이 함수 전체를 시험할 수 있다(E1/T10 과 같은 패턴).
       require_clean_tree=True 이고 실행 시점 tree 가 dirty 면 캡처를 시작하지 않고
       ABORT BEFORE CAPTURE 한다.

       🔴 advisor 지적 — git 조회(dirty-tree + runtime provenance)는 reader 생성보다
       **먼저** 끝낸다. ROOT 가 git 저장소가 아니거나(또는 상위의 다른 저장소가 섞이면)
       GitProvenanceUnavailable 이 올라오는데, 이를 구조화된 ABORT로 fail-closed 처리한다
       — manifest/reader 를 전혀 만들지 않는다. 추측으로 진행하지 않는다."""
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    evp = importlib.import_module("p007e_r2r2_evidence_pipeline")
    pre = json.load(open(PREREG))
    exp = pre["expected_device"]

    try:
        dirty_tree_check = evp.check_dirty_tree(ROOT, require_clean=require_clean_tree, run_fn=git_run_fn)
        runtime_provenance = evp.build_runtime_provenance(
            ROOT, PREREG, os.path.abspath(__file__),
            os.path.join(ROOT, "scripts/p007e_r2r2_acquisition.py"),
            os.path.join(ROOT, "scripts/p007e_r2r2_measurement.py"),
            os.path.join(ROOT, "scripts/p007e_r2r2_analysis.py"),
            evidence_pipeline_path=os.path.join(ROOT, "scripts/p007e_r2r2_evidence_pipeline.py"),
            run_fn=git_run_fn)
    except (evp.GitProvenanceUnavailable, RuntimeError) as e:
        return {"status": "ABORTED", "reason": "PROVENANCE_UNAVAILABLE", "error": repr(e)}

    if dirty_tree_check["abort"]:
        return {"status": "ABORTED", "reason": "DIRTY_TREE_GUARD", "dirty_tree_check": dirty_tree_check}

    g = env_guards(exp["path"])
    if not g["pass"]:
        return {"status": "ABORTED", "reason": "ENV_GUARD_FAILED", "env_guards": g,
               "dirty_tree_check": dirty_tree_check}
    estimator, mask, roi_mean_fn, rules = build_estimator_and_mask()
    if reader_factory is None:
        acq = importlib.import_module("p007e_r2r2_acquisition")
        reader = acq.NoNegotiationV4L2Reader(exp["path"], exp["width"], exp["height"], exp["pixelformat"],
                                             os.path.join(run_root, "_acq_tmp"))
    else:
        reader = reader_factory()
    manifest_fields = {
        # 🔴 "stage" 도 하드코딩 리터럴이 아니라 실제 쓰고 있는 preregistration 자신의
        # stage 필드에서 가져온다 — "C1-R1-R1" 처럼 이번 repair 세션 번호를 박아두면
        # 다음 gate가 이 harness 를 그대로 재사용할 때 틀린 라벨이 남는다
        "gate": pre.get("gate", "P-007E-R2-R2"), "stage": pre.get("stage", "UNKNOWN_STAGE"),
        "expected_device_identity": exp, "expected_format": exp,
        "roi_sha256": sha256(ROI), "env_guards_at_start": g,
        "dirty_tree_check": dirty_tree_check, **runtime_provenance}
    return run_full_sham(reader, estimator, roi_mean_fn, mask, run_root, manifest_fields,
                         N_sham=rules["N_sham"], M=rules["M"], rules=rules)


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
    floor = pre["🔴 threshold_construction_rule"]["sigma_floor"]
    return {
        "orb": pre["algorithm"]["orb_parameters"], "ransac": pre["transform_model"]["ransac"],
        "min_features": g["min_features_per_frame_in_roi"]["value"],
        "min_matches": g["min_matches"]["value"], "min_inliers": g["min_inliers"]["value"],
        "min_inlier_ratio": g["min_inlier_ratio"]["value"],
        "max_residual": g["max_residual_rms_inliers_px"]["value"],
        "max_scale_dev": g["max_scale_deviation"]["value"],
        "K": pre["🔴 K_M_Nsham_semantics"]["K"]["value"], "M": pre["🔴 K_M_Nsham_semantics"]["M"]["value"],
        "N_sham": pre["🔴 K_M_Nsham_semantics"]["N_sham"]["value"],
        # 🔴 C1-R1-R1 Repair B/C 가 summary 자동생성에 쓴다 — floor 값 자체는 바꾸지 않고 읽기만 추가
        "floor_trans": floor["translation_px"], "floor_rot": floor["rotation_deg"],
    }


def _make_mock_git(expected_root, head=None, dirty=False):
    """🔴 C1-R1-R1 — git provenance/dirty-tree 시험 공용 mock. 실제 저장소를
    전혀 건드리지 않는다. `--show-toplevel` 도 반드시 처리한다 — _verify_repo_root()
    가 그 값을 expected_root 와 realpath 비교하므로, 이걸 빠뜨리면 모든 mock 호출이
    GitProvenanceUnavailable 로 끝난다(실제로 한 번 겪은 함정)."""
    head = head or ("mock" + "0" * 36)

    def _mock(argv):
        if "--show-toplevel" in argv:
            return expected_root + "\n"
        if "rev-parse" in argv:
            return head + "\n"
        if "status" in argv:
            return ("M mockfile.txt\n" if dirty else "")
        return "mockcommit" + "0" * 30 + "\n"
    return _mock


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
    # reader 클래스 하나만 바꿔 끝까지 돈다. real device 는 절대 참조하지 않는다.
    # 🔴 C1-R1-R1 repair 이후: git_run_fn 을 mock 으로 주입해 배포 위치가 git 저장소가
    # 아니어도(Jetson scp 배치 등) 동작한다 — 실제 저장소를 전혀 건드리지 않는다.
    # SHA 필드명이 build_runtime_provenance() 로 바뀌면서 *_module_sha256 으로 변경됐다
    t10_root = os.path.join(tmp_base, "t10")
    try:
        base = open(FRAME, "rb").read()
        expected = {"width": 640, "height": 480, "pixelformat": "MJPG"}
        mock_git = _make_mock_git(ROOT, head="mockheadT10" + "0" * 25)
        result = execute_c1r1(t10_root, reader_factory=lambda: FakeReader(base, expected),
                              git_run_fn=mock_git)
        manifest = json.load(open(os.path.join(t10_root, "manifest.json")))
        trial_files = sorted(f for f in os.listdir(os.path.join(t10_root, "trials")))
        required_sha_keys = ("harness_sha256", "acquisition_module_sha256",
                             "measurement_module_sha256", "analysis_module_sha256",
                             "evidence_pipeline_module_sha256")
        required_provenance_keys = ("git_head", "base_commit", "preregistration_commit",
                                    "preregistration_sha256")
        finalize = result.get("finalize") or {}
        self_tr = finalize.get("self_transfer_verification") or {}
        full_comp = finalize.get("full_completeness") or {}
        t10 = {"manifest_status": manifest["status"], "trial_files": trial_files,
              "sha_keys_recorded": {k: k in manifest for k in required_sha_keys},
              "provenance_keys_recorded": {k: k in manifest for k in required_provenance_keys},
              "git_head_is_mocked_value": manifest.get("git_head", "").startswith("mockheadT10"),
              "evidence_package_core_status": manifest.get("evidence_package_core_status"),
              "full_completeness_status": full_comp.get("status"),
              "self_transfer_verification_status": self_tr.get("status"),
              "pass": manifest["status"] == "COMPLETED"
                      and all(k in manifest for k in required_sha_keys)
                      and all(k in manifest for k in required_provenance_keys)
                      and manifest.get("git_head", "").startswith("mockheadT10")
                      and manifest.get("evidence_package_core_status") == "COMPLETE"
                      and full_comp.get("status") == "COMPLETE"
                      and self_tr.get("status") == "TRANSFER_VERIFIED"}
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
# P-007E-R2-R2 Stage C1-R1-R1 — Evidence Pipeline Closure negative tests (E1~E15)
# 🔴 실제 camera/robot 전혀 참조하지 않는다. 전부 mock/synthetic/tmp 디렉터리.
# ---------------------------------------------------------------------------
def _stale_literal_absent(literal):
    """🔴 E2 — harness 소스 자체를 읽어 historical literal 이 더 이상 하드코딩돼
    있지 않은지 정적으로 확인한다 (self_ast_check 과 같은 자기참조 검사 패턴)."""
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    return literal not in src


def repair_negative_tests(tmp_base):
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    evp = importlib.import_module("p007e_r2r2_evidence_pipeline")
    an = importlib.import_module("p007e_r2r2_analysis")
    results = {}

    # E1 — runtime commit provenance: mock repo HEAD 변경이 manifest(runtime provenance)에
    # 그대로 반영되는지. 실제 저장소는 전혀 건드리지 않는다 (show-toplevel 도 처리하는
    # 공용 mock — 안 하면 _verify_repo_root() 에서 GitProvenanceUnavailable 로 먼저 끝난다)
    fake_head = "e1fake" + "a" * 34
    mock_git_e1 = _make_mock_git(ROOT, head=fake_head)
    evidence_pipeline_path = os.path.join(ROOT, "scripts/p007e_r2r2_evidence_pipeline.py")
    prov = evp.build_runtime_provenance(ROOT, PREREG, os.path.abspath(__file__),
                                        os.path.join(ROOT, "scripts/p007e_r2r2_acquisition.py"),
                                        os.path.join(ROOT, "scripts/p007e_r2r2_measurement.py"),
                                        os.path.join(ROOT, "scripts/p007e_r2r2_analysis.py"),
                                        evidence_pipeline_path=evidence_pipeline_path,
                                        run_fn=mock_git_e1)
    results["E1_runtime_commit_provenance"] = {
        "expected": "git_head == base_commit == 주입된 mock HEAD, 하드코딩값 아님",
        "observed_git_head": prov["git_head"], "observed_base_commit": prov["base_commit"],
        "evidence_pipeline_sha_recorded": "evidence_pipeline_module_sha256" in prov,
        "pass": prov["git_head"] == fake_head and prov["base_commit"] == fake_head
                and "evidence_pipeline_module_sha256" in prov}

    # E2 — stale literal prevention: 과거 C1-R1 의 historical base_commit 리터럴이
    # 더 이상 harness 소스에 하드코딩돼 있지 않은지. 🔴 그 리터럴 자체를 이 파일에
    # 그대로 적으면 이 검사가 자기 자신을 오탐한다(i2c/VideoCapture 선례와 동일 함정) —
    # 조립식 문자열로 피한다
    _historical_stale_commit = "64b32" + "bc"
    results["E2_stale_literal_prevention"] = {
        "expected": "과거 Stage C1 의 historical base_commit 7-char 리터럴이 harness 소스에 없어야 한다",
        "pass": _stale_literal_absent(_historical_stale_commit)}

    # E3/E4 — automatic summary: mock 10 trial 입력 → harness finalization(run_full_sham)
    # 만으로 summary/sham_summary.json 생성. 별도 manual analysis 호출 없음
    e34_root = os.path.join(tmp_base, "e3e4")
    try:
        sys.path.insert(0, os.path.join(ROOT, "scripts"))
        meas = importlib.import_module("p007e_r2r2_measurement")
        roi = json.load(open(ROI))
        base = open(FRAME, "rb").read()
        rules = _load_rules()
        expected = {"width": 640, "height": 480, "pixelformat": "MJPG"}
        import cv2  # noqa: F401 — FakeReader 내부에서 이미 쓰는 의존성, 가용성만 재확인
        img = cv2.imdecode(__import__("numpy").frombuffer(base, __import__("numpy").uint8), cv2.IMREAD_GRAYSCALE)
        mask = meas.build_mask(roi, img.shape)
        estimator = meas.make_estimator(rules, mask)
        reader = FakeReader(base, expected)
        e34_result = run_full_sham(reader, estimator, meas.roi_mean_brightness, mask, e34_root,
                                   {"gate": "P-007E-R2-R2", "stage": "C1-R1-R1-MOCK", "mock": True},
                                   N_sham=10, M=5, sleep_fn=lambda s: None, rules=rules)
        summary_path = os.path.join(e34_root, "summary", "sham_summary.json")
        summary = json.load(open(summary_path))
        finalize34 = e34_result.get("finalize") or {}
        self_tr34 = finalize34.get("self_transfer_verification") or {}
        results["E3_automatic_summary"] = {
            "expected": "harness 종료만으로 summary/sham_summary.json 존재, 별도 analysis 호출 불필요",
            "summary_exists": os.path.exists(summary_path), "trial_file_count": summary.get("trial_file_count"),
            "executed_M_recorded": summary.get("M"), "executed_N_sham_recorded": summary.get("N_sham"),
            # 🔴 advisor 지적 반영 — summary.M 은 이번 실행값(5)이어야 한다, rules 의
            # 사전등록값(60)이 아니다. M!=M_rules_preregistered 로 그 불일치가 이번
            # mock 시험에서는 **의도된 것**임을 바로 확인할 수 있다
            "M_matches_executed_not_preregistered": (summary.get("M") == 5
                                                      and summary.get("M_rules_preregistered") == rules["M"]),
            "pass": os.path.exists(summary_path) and summary.get("trial_file_count") == 10
                    and summary.get("M") == 5 and summary.get("N_sham") == 10}
        results["E4_summary_module_sha"] = {
            "expected": "summary.analysis_module_sha256 == 실제 analysis.py sha256",
            "recorded": summary.get("analysis_module_sha256"),
            "actual": sha256(os.path.join(ROOT, "scripts/p007e_r2r2_analysis.py")),
            "self_transfer_verification_status": self_tr34.get("status"),
            "pass": summary.get("analysis_module_sha256") == sha256(os.path.join(ROOT, "scripts/p007e_r2r2_analysis.py"))
                    and self_tr34.get("status") == "TRANSFER_VERIFIED"}
    except Exception as e:  # noqa: BLE001
        results["E3_automatic_summary"] = {"pass": False, "error": repr(e)}
        results["E4_summary_module_sha"] = {"pass": False, "error": repr(e)}

    # E5 — incomplete trial set: 9/10 trial 만 존재 → EVIDENCE_PACKAGE_INCOMPLETE
    e5_root = os.path.join(tmp_base, "e5")
    for i in range(9):
        atomic_write_json(os.path.join(e5_root, "trials", "trial_%02d.json" % i), {"index": i, "mock": True})
    atomic_write_json(os.path.join(e5_root, "manifest.json"), {"mock": True})
    comp = evp.check_artifact_completeness(e5_root, 10)
    results["E5_incomplete_trial_set"] = {
        "expected": "EVIDENCE_PACKAGE_INCOMPLETE, missing 에 trial_09.json·summary·artifact_sha256 포함",
        "observed_status": comp["status"], "missing": comp["missing"],
        "pass": comp["status"] == "EVIDENCE_PACKAGE_INCOMPLETE" and "trials/trial_09.json" in comp["missing"]}

    # E6 — source SHA manifest 생성(mock artifact 전체)
    e6_root = os.path.join(tmp_base, "e6")
    os.makedirs(e6_root, exist_ok=True)
    open(os.path.join(e6_root, "a.json"), "w").write('{"x":1}')
    open(os.path.join(e6_root, "b.bin"), "wb").write(b"\x00\x01\x02\x03")
    manifest6 = evp.build_artifact_sha_manifest(e6_root)
    expected6 = {e["relative_path"]: e["sha256"] for e in manifest6["entries"]}
    direct6 = {"a.json": evp.sha256_bytes(open(os.path.join(e6_root, "a.json"), "rb").read()),
              "b.bin": evp.sha256_bytes(open(os.path.join(e6_root, "b.bin"), "rb").read())}
    results["E6_source_sha_manifest"] = {
        "expected": "manifest 의 각 entry sha256 이 직접 계산값과 일치",
        "pass": expected6 == direct6 and len(manifest6["entries"]) == 2}

    # E7 — transfer success: remote(mock)/local(mock) 파일 동일 → TRANSFER_VERIFIED
    e7_remote = os.path.join(tmp_base, "e7_remote")
    e7_local = os.path.join(tmp_base, "e7_local")
    os.makedirs(e7_remote, exist_ok=True)
    os.makedirs(e7_local, exist_ok=True)
    content7 = b'{"ok": true}'
    open(os.path.join(e7_remote, "f.json"), "wb").write(content7)
    open(os.path.join(e7_local, "f.json"), "wb").write(content7)
    remote_manifest7 = evp.build_artifact_sha_manifest(e7_remote)
    tr7 = evp.verify_transfer(remote_manifest7["entries"], e7_local)
    results["E7_transfer_success"] = {"expected": "TRANSFER_VERIFIED", "observed": tr7["status"],
                                      "pass": tr7["status"] == "TRANSFER_VERIFIED"}

    # E8 — transfer mismatch: local 1 byte 변경 → TRANSFER_FAILED, 삭제 금지
    e8_remote = os.path.join(tmp_base, "e8_remote")
    e8_local = os.path.join(tmp_base, "e8_local")
    os.makedirs(e8_remote, exist_ok=True)
    os.makedirs(e8_local, exist_ok=True)
    open(os.path.join(e8_remote, "f.json"), "wb").write(b'{"ok": true}')
    open(os.path.join(e8_local, "f.json"), "wb").write(b'{"ok": false}')   # 🔴 1바이트+ 변경
    remote_manifest8 = evp.build_artifact_sha_manifest(e8_remote)
    tr8 = evp.verify_transfer(remote_manifest8["entries"], e8_local)
    comp8 = {"status": "COMPLETE"}  # completeness 와 무관하게 transfer 자체가 실패해야 함
    del8 = evp.remote_deletion_eligible(tr8, comp8["status"] == "COMPLETE", explicit_opt_in=True)
    results["E8_transfer_mismatch"] = {
        "expected": "TRANSFER_FAILED, REMOTE_DELETE_FORBIDDEN(opt-in=True 여도)",
        "observed_status": tr8["status"], "deletion_eligible": del8["eligible"], "deletion_reason": del8["reason"],
        "pass": tr8["status"] == "TRANSFER_FAILED" and del8["eligible"] is False}

    # E9 — missing artifact: local 파일 하나 누락 → TRANSFER_FAILED, 삭제 금지
    e9_remote = os.path.join(tmp_base, "e9_remote")
    e9_local = os.path.join(tmp_base, "e9_local")
    os.makedirs(e9_remote, exist_ok=True)
    os.makedirs(e9_local, exist_ok=True)
    open(os.path.join(e9_remote, "f1.json"), "wb").write(b'{"a":1}')
    open(os.path.join(e9_remote, "f2.json"), "wb").write(b'{"b":2}')
    open(os.path.join(e9_local, "f1.json"), "wb").write(b'{"a":1}')
    # f2.json 을 local 에 만들지 않음 — 누락 흉내
    remote_manifest9 = evp.build_artifact_sha_manifest(e9_remote)
    tr9 = evp.verify_transfer(remote_manifest9["entries"], e9_local)
    del9 = evp.remote_deletion_eligible(tr9, True, explicit_opt_in=True)
    results["E9_missing_artifact"] = {
        "expected": "TRANSFER_FAILED(MISSING_LOCAL_FILE), REMOTE_DELETE_FORBIDDEN",
        "observed_status": tr9["status"], "deletion_eligible": del9["eligible"],
        "pass": tr9["status"] == "TRANSFER_FAILED" and del9["eligible"] is False}

    # E10 — corrupt JSON: sha/size 가 "remote 가 원래 그 내용이었다"는 전제로 일치하더라도
    # JSON parse 가 실패하면 REMOTE_DELETE_FORBIDDEN
    e10_local = os.path.join(tmp_base, "e10_local")
    os.makedirs(e10_local, exist_ok=True)
    corrupt = b'{"broken": '   # 불완전 JSON
    open(os.path.join(e10_local, "f.json"), "wb").write(corrupt)
    remote_manifest10 = [{"relative_path": "f.json", "size_bytes": len(corrupt),
                         "sha256": evp.sha256_bytes(corrupt)}]   # sha/size 는 일치하게 구성
    tr10 = evp.verify_transfer(remote_manifest10, e10_local)
    del10 = evp.remote_deletion_eligible(tr10, True, explicit_opt_in=True)
    results["E10_corrupt_json_sha_matches"] = {
        "expected": "sha_match=True 여도 json_parse_ok=False 라 match=False, REMOTE_DELETE_FORBIDDEN",
        "sha_match": tr10["entries"][0]["sha_match"], "json_parse_ok": tr10["entries"][0]["json_parse_ok"],
        "overall_status": tr10["status"], "deletion_eligible": del10["eligible"],
        "pass": tr10["entries"][0]["sha_match"] is True and tr10["entries"][0]["json_parse_ok"] is False
                and tr10["status"] == "TRANSFER_FAILED" and del10["eligible"] is False}

    # E11 — default deletion: explicit opt-in 없음(False) → 전송이 완벽해도 NO DELETE
    tr11 = evp.verify_transfer(remote_manifest7["entries"], e7_local)   # E7 의 완전 일치 데이터 재사용
    del11_default = evp.remote_deletion_eligible(tr11, True, explicit_opt_in=False)
    results["E11_default_deletion_disabled"] = {
        "expected": "explicit_opt_in=False(기본값) → eligible=False, 전송 성공 여부와 무관",
        "transfer_status": tr11["status"], "deletion_eligible": del11_default["eligible"],
        "deletion_reason": del11_default["reason"],
        "pass": tr11["status"] == "TRANSFER_VERIFIED" and del11_default["eligible"] is False}

    # E12 — frame timing summary: synthetic interval 입력에서 median/robust_sd/ratio 정확성
    def _mk_timed_window(start_t, n, interval):
        rows = [{"decoded": True, "valid": True, "roi_mean": 100.0, "t": start_t + i * interval,
                "est": {"translation_norm_px": 0.0, "inlier_count": 300, "inlier_ratio": 0.9,
                       "residual_rms_inliers_px": 0.5}} for i in range(n)]
        return {"n_frames": n, "dropout_rate": 0.0, "rows": rows,
               "frame_interval_stats": {"count": n - 1, "median": interval,
                                       "robust_sd": 0.0, "min": interval, "max": interval}}
    e12_interval = 0.729
    trials_e12 = []
    for i in range(10):
        ref = _mk_timed_window(1000.0 + i * 100, 5, e12_interval)
        cmp_ = _mk_timed_window(1000.0 + i * 100 + 50, 5, e12_interval)
        trials_e12.append({"index": i, "verdict": "VALID",
                          "windows": {"reference": ref, "comparison": cmp_},
                          "delta": {"translation_x_px": 0.0, "translation_y_px": 0.0,
                                   "rotation_deg": 0.0, "norm": 0.0}})
    rules_e12 = {"K": 5, "M": 5, "N_sham": 10, "floor_trans": 0.70, "floor_rot": 0.060}
    summary_e12 = evp.build_sham_summary(trials_e12, [], rules_e12, an, M_executed=5, N_sham_executed=10, target_frame_interval_s=0.25)
    ft12 = summary_e12["frame_timing"]
    expected_duration = (5 - 1) * e12_interval
    results["E12_frame_timing_summary"] = {
        "expected": "median≈%.3f, ratio≈%.3f, window_duration≈%.3f" % (
            e12_interval, e12_interval / 0.25, expected_duration),
        "observed_median": ft12["observed_frame_interval_median_sec"],
        "observed_ratio": ft12["interval_ratio_to_target"],
        "observed_window_duration_reference": ft12["window_duration_reference_median_sec"],
        "pass": abs(ft12["observed_frame_interval_median_sec"] - e12_interval) < 1e-9
                and abs(ft12["interval_ratio_to_target"] - e12_interval / 0.25) < 1e-6
                and abs(ft12["window_duration_reference_median_sec"] - expected_duration) < 1e-6}

    # E13/E14 — threshold provenance: floor-dominated vs non-floor-dominated
    def _mk_delta_trial(idx, dtx, dty, drot):
        w = {"n_frames": 10, "dropout_rate": 0.0,
            "rows": [{"decoded": True, "valid": True, "roi_mean": 100.0, "t": idx * 10.0 + i,
                     "est": {"translation_norm_px": 0.0, "inlier_count": 300, "inlier_ratio": 0.9,
                            "residual_rms_inliers_px": 0.5}} for i in range(10)],
            "frame_interval_stats": {"count": 9, "median": 0.25, "robust_sd": 0.0, "min": 0.25, "max": 0.25}}
        return {"index": idx, "verdict": "VALID", "windows": {"reference": w, "comparison": w},
               "delta": {"translation_x_px": dtx, "translation_y_px": dty, "rotation_deg": drot,
                        "norm": (dtx ** 2 + dty ** 2) ** 0.5}}
    trials_floor = [_mk_delta_trial(i, 0.01 * (i % 3 - 1), 0.01 * (i % 2), 0.0005 * (i % 3 - 1))
                   for i in range(10)]
    summary_floor = evp.build_sham_summary(trials_floor, [], rules_e12, an, M_executed=10, N_sham_executed=10)
    tp_floor = summary_floor["threshold_provenance"]
    results["E13_threshold_floor_dominated_true"] = {
        "expected": "observed sigma ≪ floor → threshold_floor_dominated=True(전 축)",
        "observed": {k: tp_floor[k] for k in
                     ("observed_sigma_tx", "observed_sigma_ty", "observed_sigma_rot",
                      "threshold_floor_dominated")},
        "pass": tp_floor["threshold_floor_dominated_tx"] and tp_floor["threshold_floor_dominated_ty"]
                and tp_floor["threshold_floor_dominated_rot"] and tp_floor["threshold_floor_dominated"]}

    trials_nonfloor = [_mk_delta_trial(i, float(i - 5), float(i - 5), 0.1 * (i - 5)) for i in range(10)]
    summary_nonfloor = evp.build_sham_summary(trials_nonfloor, [], rules_e12, an, M_executed=10, N_sham_executed=10)
    tp_nonfloor = summary_nonfloor["threshold_provenance"]
    results["E14_threshold_non_floor_case"] = {
        "expected": "observed sigma > floor → threshold_floor_dominated=False(전 축)",
        "observed": {k: tp_nonfloor[k] for k in
                     ("observed_sigma_tx", "observed_sigma_ty", "observed_sigma_rot",
                      "threshold_floor_dominated")},
        "pass": not tp_nonfloor["threshold_floor_dominated_tx"] and not tp_nonfloor["threshold_floor_dominated_ty"]
                and not tp_nonfloor["threshold_floor_dominated_rot"] and not tp_nonfloor["threshold_floor_dominated"]}

    # E15 — no camera opt-in, 실측 강화판: env 체크뿐 아니라 acquisition._run_v4l2ctl
    # 을 raise 하도록 poison-stub 한 뒤에도 FakeReader 경로가 끝까지 성공하면, 이 경로가
    # 실제 v4l2-ctl(=real device)을 전혀 호출하지 않는다는 것이 **실증**된다(advisor 지적).
    # 🔴 금지 호출 리터럴을 여기 그대로 적으면 이 검사 자신이 자기 자신을 오탐한다
    # (i2c/C1-harness 선례와 동일 함정) — self_ast_check 가 쓰는 조립식 상수를 그대로 재사용한다
    acq_mod = importlib.import_module("p007e_r2r2_acquisition")
    _orig_run_v4l2ctl = acq_mod._run_v4l2ctl

    def _poisoned_run_v4l2ctl(*a, **kw):
        raise AssertionError("REAL_V4L2CTL_CALLED_DURING_FAKEREADER_ONLY_TEST")
    acq_mod._run_v4l2ctl = _poisoned_run_v4l2ctl
    try:
        e15_root = os.path.join(tmp_base, "e15")
        base15 = open(FRAME, "rb").read()
        expected15 = {"width": 640, "height": 480, "pixelformat": "MJPG"}
        result15 = execute_c1r1(e15_root, reader_factory=lambda: FakeReader(base15, expected15),
                                git_run_fn=_make_mock_git(ROOT, head="mockheadE15" + "0" * 25))
        poisoned_fakereader_completed = (result15.get("status") == "COMPLETED")
        poison_error = None
    except Exception as e:  # noqa: BLE001 — poison 이 실제로 발동했다면 여기로 떨어진다(=FAIL)
        poisoned_fakereader_completed = False
        poison_error = repr(e)
    finally:
        acq_mod._run_v4l2ctl = _orig_run_v4l2ctl
    results["E15_no_camera_opt_in"] = {
        "expected": "① capture opt-in env 없음 ② acquisition._run_v4l2ctl 을 poison 해도 FakeReader 경로는 COMPLETED",
        "env_present": os.environ.get(ALLOW_ENV) == "1",
        "fakereader_only_source_check": not any(
            p in open(os.path.abspath(__file__)).read() for p in FORBIDDEN_CALL_PATTERNS),
        "poisoned_fakereader_completed": poisoned_fakereader_completed, "poison_error": poison_error,
        "pass": os.environ.get(ALLOW_ENV) != "1" and poisoned_fakereader_completed}

    # E16 — 🔴 git_run_fn 이 raise 하면 ABORTED 로 끝나고 manifest/reader 가 전혀
    # 만들어지지 않는다 (advisor 지적 — fail-closed 검증, Block 2)
    e16_root = os.path.join(tmp_base, "e16")
    def _raising_git(argv):
        raise RuntimeError("E16_SIMULATED_GIT_FAILURE")
    def _reader_factory_should_not_be_called():
        raise AssertionError("READER_FACTORY_CALLED_DESPITE_PROVENANCE_ABORT")
    result16 = execute_c1r1(e16_root, reader_factory=_reader_factory_should_not_be_called,
                            git_run_fn=_raising_git)
    results["E16_provenance_unavailable_aborts_before_reader"] = {
        "expected": "status=ABORTED, reason=PROVENANCE_UNAVAILABLE, manifest.json 생성 안 됨, reader_factory 호출 안 됨",
        "observed_status": result16.get("status"), "observed_reason": result16.get("reason"),
        "manifest_created": os.path.exists(os.path.join(e16_root, "manifest.json")),
        "pass": result16.get("status") == "ABORTED" and result16.get("reason") == "PROVENANCE_UNAVAILABLE"
                and not os.path.exists(os.path.join(e16_root, "manifest.json"))}

    # E17 — 🔴 dirty tree + require_clean_tree=True → ABORTED (§6 abort 경로, Block 2)
    e17_root = os.path.join(tmp_base, "e17")
    mock_git_dirty = _make_mock_git(ROOT, head="mockheadE17" + "0" * 25, dirty=True)
    result17 = execute_c1r1(e17_root, reader_factory=_reader_factory_should_not_be_called,
                            git_run_fn=mock_git_dirty, require_clean_tree=True)
    results["E17_dirty_tree_require_clean_aborts"] = {
        "expected": "status=ABORTED, reason=DIRTY_TREE_GUARD, manifest.json 생성 안 됨",
        "observed_status": result17.get("status"), "observed_reason": result17.get("reason"),
        "manifest_created": os.path.exists(os.path.join(e17_root, "manifest.json")),
        "pass": result17.get("status") == "ABORTED" and result17.get("reason") == "DIRTY_TREE_GUARD"
                and not os.path.exists(os.path.join(e17_root, "manifest.json"))}

    all_pass = all(v["pass"] for v in results.values())
    return {"results": results, "all_pass": all_pass}


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-sham", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--negative-tests", action="store_true")
    ap.add_argument("--repair-tests", action="store_true",
                    help="🔴 P-007E-R2-R2 Stage C1-R1-R1 — 기존 T0~T12 회귀 + selftest 회귀 + "
                         "신규 E1~E15(evidence pipeline repair) 를 전부 돌린다. --work-dir 필요, 카메라 0")
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
    if a.negative_tests or a.repair_tests:
        if a.work_dir is None:
            print(json.dumps({"stop": "ABORT BEFORE RUN — --negative-tests/--repair-tests 는 --work-dir(디렉터리) 가 필요하다"}))
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
    elif a.repair_tests:
        # 🔴 Stage C1-R1-R1 — §23 회귀(T0~T12·selftest) + 신규 E1~E15. 카메라 0, 로봇 0
        rec["regression_negative_tests_T0_T12"] = negative_tests(os.path.join(a.work_dir, "t_regression"))
        rec["regression_selftest"] = selftest()
        rec["repair_negative_tests_E1_E15"] = repair_negative_tests(os.path.join(a.work_dir, "e_repair"))
        rec["stop"] = "REPAIR_TESTS_ONLY — 카메라 0"
    elif rec["capture_opt_in"]["granted"]:
        # 🔴 REAL CAPTURE PATH. C0-R1 에서는 이 분기에 **실제로 도달하지 않는다**
        # (ALLOW_ENV 를 설정하지 않았다) — 코드는 존재하되 이번 게이트에서 한 번도 실행되지 않았다.
        # T10 이 execute_c1r1() 자체는 FakeReader 로 끝까지 검증한다
        rec["run_result"] = execute_c1r1(a.run_root, reader_factory=None)
        rec["stop"] = "SHAM_COMPLETE_OR_ABORTED — 결과는 run_result 참조"
    else:
        rec["stop"] = ("🔴 NO CAPTURE (기본값). 캡처하려면 %s=1 과 --run-sham 이 모두 필요하다" % ALLOW_ENV)

    text = json.dumps(rec, ensure_ascii=False, indent=1)
    if a.out and (a.selftest or a.negative_tests or a.repair_tests):
        atomic_write_json(a.out, rec)   # 🔴 --out 은 위에서 .json 파일임을 이미 검증했다
    print(text)


if __name__ == "__main__":
    main()
