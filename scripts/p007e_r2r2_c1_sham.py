#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C1 — stationary ego-motion sham harness.

🔴 기본 실행은 **카메라를 열지 않는다**. 자기검사만 하고 종료한다.
   캡처 경로 진입 조건은 두 가지가 **모두** 필요하다:
       환경변수  P007E_R2R2_C1_ALLOW_CAPTURE=1
       플래그    --run-sham

🔴 이 harness 는 로봇에 접근하지 않는다 — Arm_Lib · smbus · I2C · ROS 를 쓰지 않는다.
   측정 규칙은 전부 evidence/P-007E-R2-R2/stage_c0_preregistration.json 에서 읽는다.
   그 파일이 바뀌면 sha256 대조에서 멈춘다 (사후 튜닝 차단).
"""
import argparse
import ast
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREREG = os.path.join(ROOT, "evidence/P-007E-R2-R2/stage_c0_preregistration.json")
ROI = os.path.join(ROOT, "evidence/P-007E-R2-R2/stage_c0_roi.json")
FRAME = os.path.join(ROOT, "evidence/P-007E-R2-R2/logs/b3_frame.png")

PREREG_SHA = "048c905488bc9fa1b6b49686cb05ae31c7c9d6cf48b2ea5417d66c52a632502f"

ALLOW_ENV = "P007E_R2R2_C1_ALLOW_CAPTURE"
FORBIDDEN_IMPORTS = {"Arm_Lib", "smbus", "smbus2", "rclpy", "rospy", "Adafruit_GPIO"}
MOTION_PROCS = ("arm_driver", "dofbot_pro_driver", "color_follow", "grasp",
                "apriltag", "KCF", "yolov11", "voice_ctrl", "mediapipe")
DEV = "/dev/video0"


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def self_ast_check():
    """자기 소스에 금지 import 와 로봇 버스 문자열이 없음을 정적으로 확인한다."""
    src = open(os.path.abspath(__file__), "r", encoding="utf-8").read()
    tree = ast.parse(src)
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                found.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            found.add(node.module.split(".")[0])
    bad = sorted(found & FORBIDDEN_IMPORTS)
    # 🔴 검사어 자체가 소스에 리터럴로 남으면 자기 자신을 잡는다. 런타임에 조립한다
    needles = ("/dev/" + "i2c", "i2" + "c-7")
    bus_refs = [n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and any(x in n.value for x in needles)]
    return {"imports": sorted(found), "forbidden_imports_found": bad,
            "i2c_string_refs": bus_refs,
            "pass": not bad and not bus_refs}


def env_guards():
    """캡처 직전 환경 가드. ps 기반 — ros2 CLI 를 쓰지 않는다."""
    ps = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True, text=True).stdout
    motion = [l.strip() for l in ps.splitlines()[1:]
              if any(m.lower() in l.lower() for m in MOTION_PROCS)]
    holders = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fdd = "/proc/%s/fd" % pid
        try:
            for fd in os.listdir(fdd):
                if os.readlink(os.path.join(fdd, fd)) == DEV:
                    holders.append(pid)
                    break
        except OSError:
            continue
    fmt = subprocess.run(["v4l2-ctl", "--device=" + DEV, "--get-fmt-video"],
                         capture_output=True, text=True).stdout.strip()
    return {"motion_capable_processes": motion, "video0_holders": holders,
            "format": fmt, "pass": not motion and not holders}


def load_rules():
    pre = json.load(open(PREREG))
    roi = json.load(open(ROI))
    g = pre["frame_quality_guards"]
    return pre, roi, {
        "orb": pre["algorithm"]["orb_parameters"],
        "ransac": pre["transform_model"]["ransac"],
        "min_features": g["min_features_per_frame_in_roi"]["value"],
        "min_matches": g["min_matches"]["value"],
        "min_inliers": g["min_inliers"]["value"],
        "min_inlier_ratio": g["min_inlier_ratio"]["value"],
        "max_residual": g["max_residual_rms_inliers_px"]["value"],
        "max_scale_dev": g["max_scale_deviation"]["value"],
        "K": pre["🔴 K_M_Nsham_semantics"]["K"]["value"],
        "M": pre["🔴 K_M_Nsham_semantics"]["M"]["value"],
        "N_sham": pre["🔴 K_M_Nsham_semantics"]["N_sham"]["value"],
        "sigma_floor_trans": pre["🔴 threshold_construction_rule"]["sigma_floor"]["translation_px"],
        "sigma_floor_rot": pre["🔴 threshold_construction_rule"]["sigma_floor"]["rotation_deg"],
    }


def build_mask(roi, shape):
    import numpy as np
    h, w = shape
    m = np.zeros((h, w), np.uint8)
    for r in roi["include_regions"]:
        x0, y0, x1, y1 = r["rect"]
        m[y0:y1, x0:x1] = 255
    for r in roi["exclude_regions"]:
        x0, y0, x1, y1 = r["rect"]
        m[y0:y1, x0:x1] = 0
    return m


def make_estimator(rules, mask):
    import cv2
    import numpy as np
    p = rules["orb"]
    orb = cv2.ORB_create(nfeatures=p["nfeatures"], scaleFactor=p["scaleFactor"],
                         nlevels=p["nlevels"], fastThreshold=p["fastThreshold"])
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    rs = rules["ransac"]

    def estimate(gray_a, gray_b):
        ka, da = orb.detectAndCompute(gray_a, mask)
        kb, db = orb.detectAndCompute(gray_b, mask)
        out = {"n_features_a": len(ka), "n_features_b": len(kb)}
        if da is None or db is None or len(ka) < rules["min_features"] or len(kb) < rules["min_features"]:
            return dict(out, valid=False, reason="MIN_FEATURES")
        m = bf.match(da, db)
        out["n_matches"] = len(m)
        if len(m) < rules["min_matches"]:
            return dict(out, valid=False, reason="MIN_MATCHES")
        pa = np.float32([ka[x.queryIdx].pt for x in m]).reshape(-1, 1, 2)
        pb = np.float32([kb[x.trainIdx].pt for x in m]).reshape(-1, 1, 2)
        mat, inl = cv2.estimateAffinePartial2D(
            pa, pb, method=cv2.RANSAC, ransacReprojThreshold=rs["ransacReprojThreshold_px"],
            maxIters=rs["maxIters"], confidence=rs["confidence"])
        if mat is None:
            return dict(out, valid=False, reason="NO_MODEL")
        inl = inl.ravel().astype(bool)
        proj = (mat[:, :2] @ pa.reshape(-1, 2).T).T + mat[:, 2]
        err = np.linalg.norm(proj - pb.reshape(-1, 2), axis=1)
        out.update({
            "translation_x_px": float(mat[0, 2]), "translation_y_px": float(mat[1, 2]),
            "translation_norm_px": float(np.hypot(mat[0, 2], mat[1, 2])),
            "rotation_deg": float(np.degrees(np.arctan2(mat[1, 0], mat[0, 0]))),
            "scale": float(np.hypot(mat[0, 0], mat[1, 0])),
            "inlier_count": int(inl.sum()), "inlier_ratio": float(inl.mean()),
            "residual_rms_inliers_px": float(np.sqrt((err[inl] ** 2).mean())),
            "residual_median_inliers_px": float(np.median(err[inl]))})
        for cond, why in (
                (out["inlier_count"] < rules["min_inliers"], "MIN_INLIERS"),
                (out["inlier_ratio"] < rules["min_inlier_ratio"], "MIN_INLIER_RATIO"),
                (out["residual_rms_inliers_px"] > rules["max_residual"], "MAX_RESIDUAL"),
                (abs(out["scale"] - 1.0) > rules["max_scale_dev"], "SCALE_DEVIATION")):
            if cond:
                return dict(out, valid=False, reason=why)
        return dict(out, valid=True, reason=None)

    return estimate


def selftest(rules, roi):
    """🔴 ALGORITHM UNIT VALIDATION — 카메라를 열지 않는다. Stage B 프레임에 알려진 변환만 가한다."""
    import cv2
    import numpy as np
    img = cv2.imdecode(np.frombuffer(open(FRAME, "rb").read(), np.uint8), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    est = make_estimator(rules, build_mask(roi, img.shape))
    rows = []
    for dx, dy, deg in [(0.5, 0, 0), (2, 0, 0), (0, 2, 0), (0, 0, 0.1), (0, 0, 1.0), (3, -2, 0.3)]:
        mat = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), deg, 1.0)
        mat[0, 2] += dx
        mat[1, 2] += dy
        warped = cv2.warpAffine(img, mat, (w, h), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_REFLECT)
        r = est(img, warped)
        exp_rot = float(np.degrees(np.arctan2(mat[1, 0], mat[0, 0])))
        rows.append({"applied": {"dx": dx, "dy": dy, "rot_deg_input": deg},
                     "expected": {"tx": float(mat[0, 2]), "ty": float(mat[1, 2]), "rot_deg": exp_rot},
                     "result": r,
                     "err_trans_norm": (None if not r["valid"] else
                                        float(np.hypot(r["translation_x_px"] - mat[0, 2],
                                                       r["translation_y_px"] - mat[1, 2]))),
                     "err_rot_deg": (None if not r["valid"] else r["rotation_deg"] - exp_rot)})
    return {"kind": "ALGORITHM UNIT VALIDATION",
            "🔴 not_physical_evidence": "합성 2D 변환이다. 실제 DaBai 감도 증거가 아니다.",
            "rows": rows}


def robust_sd(values):
    import numpy as np
    v = np.asarray(values, float)
    return float(1.4826 * np.median(np.abs(v - np.median(v))))


def run_sham(rules, roi, outdir):
    """🔴 캡처 경로. opt-in 두 조건이 모두 충족된 경우에만 호출된다."""
    import cv2
    import numpy as np
    os.makedirs(outdir, exist_ok=True)
    pre_fmt = subprocess.run(["v4l2-ctl", "--device=" + DEV, "--get-fmt-video"],
                             capture_output=True, text=True).stdout
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)          # 🔴 CAP_PROP 을 하나도 set 하지 않는다
    if not cap.isOpened():
        return {"status": "X", "reason": "CAPTURE_OPEN_FAILED"}
    est = None
    trials = []
    M, N = rules["M"], rules["N_sham"]
    try:
        for t in range(N):
            trial = {"index": t, "windows": {}}
            anchor = None
            for wname, pause in (("reference", None), ("comparison", 3.06)):
                if pause:                              # quiet 0.06 + command slot 2.0 + settle 1.0
                    time.sleep(pause)
                frames = []
                t_next = time.monotonic()
                while len(frames) < M:
                    cap.grab()
                    if time.monotonic() >= t_next:
                        ok, fr = cap.retrieve()
                        if not ok:
                            continue
                        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
                        if anchor is None:
                            anchor = g
                            est = make_estimator(rules, build_mask(roi, g.shape))
                        frames.append({"t": time.time(), "est": est(anchor, g),
                                       "roi_mean": float(g.mean())})
                        if len(frames) == 1:
                            cv2.imwrite(os.path.join(outdir, "t%02d_%s_first.jpg" % (t, wname)), fr)
                        t_next += 0.25
                valid = [f for f in frames if f["est"]["valid"]]
                trial["windows"][wname] = {
                    "n_frames": len(frames), "n_valid": len(valid),
                    "dropout_rate": 1.0 - len(valid) / float(len(frames)),
                    "rows": [f["est"] for f in frames],
                    "t": [f["t"] for f in frames], "roi_mean": [f["roi_mean"] for f in frames],
                    "median": (None if len(valid) < int(0.8 * M) else {
                        k: float(np.median([f["est"][k] for f in valid]))
                        for k in ("translation_x_px", "translation_y_px", "rotation_deg")})}
            a, b = trial["windows"]["reference"]["median"], trial["windows"]["comparison"]["median"]
            if a is None or b is None:
                trial["verdict"] = "INVALID"
            else:
                trial["delta"] = {k: b[k] - a[k] for k in a}
                trial["delta"]["norm"] = float(np.hypot(trial["delta"]["translation_x_px"],
                                                        trial["delta"]["translation_y_px"]))
                trial["verdict"] = "VALID"
            trials.append(trial)
            time.sleep(2.0)
    finally:
        cap.release()
    post_fmt = subprocess.run(["v4l2-ctl", "--device=" + DEV, "--get-fmt-video"],
                              capture_output=True, text=True).stdout
    return {"trials": trials, "format_before": pre_fmt, "format_after": post_fmt,
            "format_unchanged": pre_fmt == post_fmt}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-sham", action="store_true",
                    help="🔴 캡처 경로 진입. 환경변수 %s=1 도 함께 필요하다" % ALLOW_ENV)
    ap.add_argument("--selftest", action="store_true", help="카메라 없이 합성 단위시험만 수행")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    rec = {"gate": "P-007E-R2-R2", "stage": "C1", "argv": sys.argv[1:],
           "self_ast_check": self_ast_check(),
           "preregistration": {"path": "evidence/P-007E-R2-R2/stage_c0_preregistration.json",
                               "expected_sha256": PREREG_SHA, "actual_sha256": sha256(PREREG),
                               "match": sha256(PREREG) == PREREG_SHA},
           "roi_sha256": sha256(ROI),
           "capture_opt_in": {"env_%s" % ALLOW_ENV: os.environ.get(ALLOW_ENV, "0"),
                              "flag_run_sham": a.run_sham,
                              "granted": os.environ.get(ALLOW_ENV) == "1" and a.run_sham}}

    if not rec["self_ast_check"]["pass"]:
        rec["stop"] = "SELF_CHECK_FAILED"
    elif not rec["preregistration"]["match"]:
        rec["stop"] = "PREREGISTRATION_SHA_MISMATCH — 사전등록이 바뀌었다. 사후 튜닝 차단"
    else:
        rules, roi = load_rules()[2], json.load(open(ROI))
        if a.selftest:
            rec["unit_validation"] = selftest(rules, roi)
            rec["stop"] = "SELFTEST_ONLY — 카메라 0"
        elif rec["capture_opt_in"]["granted"]:
            rec["env_guards"] = env_guards()
            if not rec["env_guards"]["pass"]:
                rec["stop"] = "ENV_GUARD_FAILED"
            else:
                rec["sham"] = run_sham(rules, roi, a.out or "/tmp/p007e_r2r2_c1")
                rec["stop"] = "SHAM_COMPLETE"
        else:
            rec["stop"] = ("🔴 NO CAPTURE (기본값). 캡처하려면 %s=1 과 --run-sham 이 모두 필요하다"
                           % ALLOW_ENV)

    text = json.dumps(rec, ensure_ascii=False, indent=1)
    if a.out:
        open(a.out if a.out.endswith(".json") else a.out + "/c1_result.json", "w").write(text)
    print(text)


if __name__ == "__main__":
    main()
