#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C1-R1-R1 — Evidence Pipeline repair module.

🔴 이 모듈은 카메라/로봇을 전혀 참조하지 않는다. git 읽기전용 조회, 파일
sha256/크기 계산, 사전등록된 analysis.py(수정 없이 그대로 import)를 이용한
집계 summary 생성, 전송 무결성 비교, 삭제 적격성 판정만 한다.

🔴 모든 git 호출은 `run_fn` 주입이 가능하다 — 실제 저장소를 건드리지 않는
mock 시험(E1/E2)을 위함이다. 기본값(run_fn=None)은 실제 `git ... -C <repo_root>`
를 읽기전용으로 호출한다(rev-parse/status/log — 쓰기 없음).
"""
import hashlib
import json
import os
import subprocess
import time


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    return sha256_bytes(open(path, "rb").read())


# ---------------------------------------------------------------------------
# Repair A — runtime git provenance (읽기전용: rev-parse / status / log)
# ---------------------------------------------------------------------------
class GitProvenanceUnavailable(RuntimeError):
    """🔴 advisor 지적 — `git -C <dir>` 는 <dir> 이 저장소가 아니면 **상위 디렉터리로
    올라가며** 저장소를 찾는다. 검증 없이 쓰면 엉뚱한 상위 저장소의 HEAD를 조용히
    기록할 위험이 있다(실측: Jetson scp 배치 디렉터리 `/home/jetson/...` 에는 .git 이
    없지만, 그 상위 어딘가에 있다면 그 저장소 상태가 섞여 들어간다). 호출자는 이
    예외를 fail-closed ABORT 로 처리해야 한다 — 추측으로 진행하지 않는다."""
    pass


def _default_git_run(argv):
    r = subprocess.run(argv, capture_output=True, text=True, timeout=10)
    if r.returncode != 0:
        raise RuntimeError("GIT_CMD_FAILED: %r stderr=%s" % (argv, r.stderr))
    return r.stdout


def _verify_repo_root(repo_root, run_fn):
    """🔴 git 이 실제로 찾은 저장소 루트가 repo_root 자신인지 확인한다. 다르면(또는
    저장소를 전혀 못 찾으면) GitProvenanceUnavailable 을 던진다 — 침묵 실패를 막는다."""
    try:
        top = run_fn(["git", "-C", repo_root, "rev-parse", "--show-toplevel"]).strip()
    except Exception as e:  # noqa: BLE001
        raise GitProvenanceUnavailable("NO_GIT_REPO_AT_OR_ABOVE %r: %r" % (repo_root, e))
    if os.path.realpath(top) != os.path.realpath(repo_root):
        raise GitProvenanceUnavailable(
            "WRONG_REPO_ROOT: expected %r, git found %r (상위 디렉터리의 다른 저장소일 수 있다)"
            % (repo_root, top))
    return top


def git_head(repo_root, run_fn=None):
    run_fn = run_fn or _default_git_run
    return run_fn(["git", "-C", repo_root, "rev-parse", "HEAD"]).strip()


def git_status_porcelain(repo_root, run_fn=None):
    run_fn = run_fn or _default_git_run
    return run_fn(["git", "-C", repo_root, "status", "--porcelain"])


def git_last_commit_touching(repo_root, relpath, run_fn=None):
    run_fn = run_fn or _default_git_run
    out = run_fn(["git", "-C", repo_root, "log", "-1", "--format=%H", "--", relpath]).strip()
    return out or None


def check_dirty_tree(repo_root, require_clean=False, run_fn=None):
    """🔴 §6 — porcelain 기준으로 dirty 여부를 기록한다. require_clean=True 이고
    dirty 면 abort 사유를 채운다(실제 abort 집행은 호출자 책임 — 여기선 판정만).
    🔴 repo_root 자체가 git 저장소가 아니면(또는 상위의 다른 저장소가 섞이면)
    GitProvenanceUnavailable 을 던진다 — 호출자가 fail-closed ABORT 해야 한다."""
    run_fn = run_fn or _default_git_run
    _verify_repo_root(repo_root, run_fn)
    porcelain = git_status_porcelain(repo_root, run_fn=run_fn)
    dirty = bool(porcelain.strip())
    result = {"porcelain": porcelain, "dirty": dirty, "require_clean": require_clean}
    result["abort"] = ("ABORT BEFORE CAPTURE — dirty tree, require_clean_tree=True"
                       if (dirty and require_clean) else None)
    return result


def build_runtime_provenance(repo_root, preregistration_path, harness_path,
                             acquisition_path, measurement_path, analysis_path,
                             evidence_pipeline_path=None, run_fn=None):
    """🔴 §5 — 하드코딩된 historical commit literal 을 쓰지 않는다. 전부 실행
    시점에 실제로 조회한다. base_commit == git_head(실행 시점 repo 상태).
    🔴 repo_root 검증 실패 시 GitProvenanceUnavailable 을 던진다(침묵 실패 금지)."""
    run_fn = run_fn or _default_git_run
    _verify_repo_root(repo_root, run_fn)
    head = git_head(repo_root, run_fn=run_fn)
    fields = {
        "git_head": head,
        "base_commit": head,
        "preregistration_commit": git_last_commit_touching(
            repo_root, os.path.relpath(preregistration_path, repo_root), run_fn=run_fn),
        "preregistration_sha256": sha256_file(preregistration_path),
        "harness_sha256": sha256_file(harness_path),
        "acquisition_module_sha256": sha256_file(acquisition_path),
        "measurement_module_sha256": sha256_file(measurement_path),
        "analysis_module_sha256": sha256_file(analysis_path),
    }
    if evidence_pipeline_path:
        fields["evidence_pipeline_module_sha256"] = sha256_file(evidence_pipeline_path)
    return fields


# ---------------------------------------------------------------------------
# §24 — quasi-static observation semantics (architecture note, 그대로 인용)
# ---------------------------------------------------------------------------
QUASI_STATIC_SEMANTICS_NOTE = {
    "qualification_statement": (
        "This observation channel is used for pre/post quasi-static physical "
        "displacement detection, not dynamic trajectory tracking."
    ),
    "qualified_for": ["pre-command stable visual pose", "post-command settled visual pose",
                      "pre/post comparison of the two"],
    "not_qualified_for": ["servo trajectory reconstruction", "motion velocity measurement",
                         "dynamic path tracking", "absolute joint-angle measurement",
                         "absolute robot pose measurement"],
    "channel_independence": (
        "controller-reported servo position and the DaBai visual observation are "
        "independent evidence channels. They are not assumed to be the same physical "
        "quantity."
    ),
}


# ---------------------------------------------------------------------------
# Repair B/C — frozen analysis.py 를 그대로 호출해 summary 를 만든다.
# 분석 공식·estimator 계산 규칙은 수정하지 않는다 — 여기서는 기존
# classify_run() 출력을 provenance 필드로 재구성만 한다.
# ---------------------------------------------------------------------------
def _window_rows_timestamps(trial, wname):
    w = trial.get("windows", {}).get(wname, {})
    return [r["t"] for r in w.get("rows", []) if r.get("t") is not None]


def _anchor_self_comparison_present(trial):
    """🔴 run_trial() 구현상 anchor_gray 는 trial 당 1회, reference window 의
    첫 프레임에서만 설정되고 그 프레임은 자기자신과 비교된다(estimator(anchor,anchor)).
    그 한 행만 self-comparison 이다 — window 마다가 아니라 trial 마다 1개."""
    rows = trial.get("windows", {}).get("reference", {}).get("rows", [])
    if not rows:
        return False
    r0 = rows[0]
    est = r0.get("est") or {}
    return bool(r0.get("decoded") and r0.get("valid")
               and est.get("residual_rms_inliers_px") == 0.0
               and est.get("inlier_ratio") == 1.0)


def build_sham_summary(trials, trial_file_paths, rules, analysis_module, M_executed, N_sham_executed,
                       target_frame_interval_s=0.25, human_trial_offset=1, run_root=None,
                       evidence_pipeline_path=None):
    """trials: trial dict 리스트(저장 순서, index 0..N-1). trial_file_paths: 같은
    순서의 실제 파일 경로(trial sha256 기록용, run_root 가 주어지면 상대경로로 기록).
    rules: {K,floor_trans,floor_rot} — algorithm-level, 변경되지 않는 값만 여기서 온다.
    🔴 advisor 지적 — M/N_sham 은 rules(사전등록값)가 아니라 **이번 실행에서 실제로
    쓴 값**(M_executed/N_sham_executed)을 기록한다. mock/synthetic 시험은 preregistered
    M=60 보다 작은 M 으로 돌 수 있어, rules 값을 그대로 쓰면 summary 가 실행과 불일치한다."""
    an = analysis_module
    K, floor_t, floor_r = rules["K"], rules["floor_trans"], rules["floor_rot"]
    classify = an.classify_run(trials, K, floor_t, floor_r)
    nm, pub = classify["noise_metrics"], classify["publication_thresholds"]

    observed_tx = nm["delta_tx"]["robust_sd"]
    observed_ty = nm["delta_ty"]["robust_sd"]
    observed_rot = nm["delta_rot"]["robust_sd"]
    eff_tx, eff_ty, eff_rot = pub["sigma_tx"], pub["sigma_ty"], pub["sigma_rot"]
    dom_tx = (observed_tx is None) or observed_tx <= floor_t
    dom_ty = (observed_ty is None) or observed_ty <= floor_t
    dom_rot = (observed_rot is None) or observed_rot <= floor_r
    threshold_provenance = {
        "observed_sigma_tx": observed_tx, "sigma_floor_tx": floor_t, "effective_sigma_tx": eff_tx,
        "observed_sigma_ty": observed_ty, "sigma_floor_ty": floor_t, "effective_sigma_ty": eff_ty,
        "observed_sigma_rot": observed_rot, "sigma_floor_rot": floor_r, "effective_sigma_rot": eff_rot,
        "threshold_floor_dominated_tx": dom_tx, "threshold_floor_dominated_ty": dom_ty,
        "threshold_floor_dominated_rot": dom_rot,
        "threshold_floor_dominated": bool(dom_tx or dom_ty or dom_rot),
    }

    ref_medians, cmp_medians, ref_durs, cmp_durs = [], [], [], []
    for t in trials:
        for wname, meds, durs in (("reference", ref_medians, ref_durs),
                                  ("comparison", cmp_medians, cmp_durs)):
            w = t.get("windows", {}).get(wname, {})
            fi = w.get("frame_interval_stats", {})
            if fi.get("median") is not None:
                meds.append(fi["median"])
            ts = _window_rows_timestamps(t, wname)
            if len(ts) >= 2:
                durs.append(max(ts) - min(ts))
    all_medians = ref_medians + cmp_medians
    observed_median = an.median(all_medians) if all_medians else None
    observed_robust_sd = an.robust_sd(all_medians) if all_medians else None
    frame_timing = {
        "target_frame_interval_sec": target_frame_interval_s,
        "observed_frame_interval_median_sec": observed_median,
        "observed_frame_interval_robust_sd_sec": observed_robust_sd,
        "interval_ratio_to_target": (
            (observed_median / target_frame_interval_s) if observed_median else None),
        "window_duration_reference_median_sec": an.median(ref_durs) if ref_durs else None,
        "window_duration_comparison_median_sec": an.median(cmp_durs) if cmp_durs else None,
        "total_capture_duration_sec": (sum(ref_durs) + sum(cmp_durs)) if (ref_durs or cmp_durs) else None,
    }

    anchor_rows = sum(1 for t in trials if _anchor_self_comparison_present(t))
    numbering = [{"storage_index": t["index"], "human_trial_number": t["index"] + human_trial_offset}
                for t in trials]

    brightness_deltas = []
    for t in trials:
        ref_roi = [r["roi_mean"] for r in t.get("windows", {}).get("reference", {}).get("rows", [])
                  if r.get("roi_mean") is not None]
        cmp_roi = [r["roi_mean"] for r in t.get("windows", {}).get("comparison", {}).get("rows", [])
                  if r.get("roi_mean") is not None]
        if ref_roi and cmp_roi:
            brightness_deltas.append(an.median(cmp_roi) - an.median(ref_roi))
    sign = None
    if brightness_deltas:
        if all(d < 0 for d in brightness_deltas):
            sign = "ALL_NEGATIVE"
        elif all(d > 0 for d in brightness_deltas):
            sign = "ALL_POSITIVE"
        else:
            sign = "MIXED"
    photometric = {
        "brightness_delta_median": an.median(brightness_deltas) if brightness_deltas else None,
        "brightness_delta_min": min(brightness_deltas) if brightness_deltas else None,
        "brightness_delta_max": max(brightness_deltas) if brightness_deltas else None,
        "brightness_delta_sign_consistency": sign,
        "photometric_motion_correlation_abs_spearman_rho": classify["q4_photometric"]["abs_spearman_rho"],
    }

    trial_sha_list = [{"path": (os.path.relpath(p, run_root) if run_root else p),
                      "sha256": sha256_file(p)} for p in trial_file_paths]
    out = {
        "schema": "p007e_r2r2_sham_summary_v1",
        "generated_by": "frozen scripts/p007e_r2r2_analysis.py (수정 없음)",
        "analysis_module_sha256": sha256_file(analysis_module.__file__),
        "trial_file_count": len(trials),
        "trial_sha256_list": trial_sha_list,
        "K": K, "M": M_executed, "N_sham": N_sham_executed,
        "M_rules_preregistered": rules.get("M"), "N_sham_rules_preregistered": rules.get("N_sham"),
        "sigma_floor_tx": floor_t, "sigma_floor_ty": floor_t, "sigma_floor_rotation": floor_r,
        "robust_sigma_tx": observed_tx, "robust_sigma_ty": observed_ty, "robust_sigma_rotation": observed_rot,
        "publication_thr_norm": pub["thr_norm"], "publication_thr_rot": pub["thr_rot"],
        "false_positive_count": classify["loo"]["false_positive_count"],
        "false_positive_rate": classify["loo"]["false_positive_rate"],
        "dropout_rate": nm["window_dropout_rate_median"],
        "invalid_trial_rate": nm["trial_invalid_rate"],
        "final_classification": classify["verdict"],
        "threshold_provenance": threshold_provenance,
        "frame_timing": frame_timing,
        "anchor_self_comparison_rows": anchor_rows,
        "anchor_self_comparison_included_in_aggregate_distributions": True,
        "trial_numbering_mapping": numbering,
        "photometric": photometric,
        "quasi_static_semantics": QUASI_STATIC_SEMANTICS_NOTE,
    }
    if evidence_pipeline_path:
        out["evidence_pipeline_module_sha256"] = sha256_file(evidence_pipeline_path)
    return out


# ---------------------------------------------------------------------------
# Repair D/§13 — artifact SHA manifest (source-side, 전송 전)
# ---------------------------------------------------------------------------
def discover_artifact_relative_paths(root_dir):
    paths = []
    for dirpath, _dirs, files in os.walk(root_dir):
        for f in files:
            full = os.path.join(dirpath, f)
            paths.append(os.path.relpath(full, root_dir))
    return sorted(paths)


def build_artifact_sha_manifest(root_dir, relative_paths=None):
    relative_paths = relative_paths if relative_paths is not None else discover_artifact_relative_paths(root_dir)
    entries = []
    for rel in relative_paths:
        full = os.path.join(root_dir, rel)
        data = open(full, "rb").read()
        entries.append({"relative_path": rel, "size_bytes": len(data), "sha256": sha256_bytes(data)})
    return {"schema": "p007e_r2r2_artifact_sha_manifest_v1", "root": root_dir,
            "generated_at": time.time(), "entries": entries}


# ---------------------------------------------------------------------------
# §14/§15 — transfer integrity verification + 삭제 적격성 판정 (전부 로컬
# 디렉터리 2개 비교 — 실제 ssh/scp 는 이 모듈이 전혀 호출하지 않는다)
# ---------------------------------------------------------------------------
def verify_transfer(remote_manifest_entries, local_root):
    results = []
    all_match = True
    for e in remote_manifest_entries:
        local_path = os.path.join(local_root, e["relative_path"])
        row = {"relative_path": e["relative_path"], "expected_sha256": e["sha256"],
              "expected_size": e["size_bytes"]}
        if not os.path.exists(local_path):
            row.update({"present": False, "match": False, "reason": "MISSING_LOCAL_FILE"})
            all_match = False
            results.append(row)
            continue
        data = open(local_path, "rb").read()
        actual_sha = sha256_bytes(data)
        size_match = len(data) == e["size_bytes"]
        sha_match = actual_sha == e["sha256"]
        parse_ok, parse_reason = True, None
        if local_path.endswith(".json"):
            try:
                json.loads(data)
            except Exception as ex:  # noqa: BLE001
                parse_ok, parse_reason = False, repr(ex)
        match = size_match and sha_match and parse_ok
        row.update({"present": True, "actual_sha256": actual_sha, "actual_size": len(data),
                   "size_match": size_match, "sha_match": sha_match,
                   "json_parse_ok": parse_ok, "json_parse_error": parse_reason, "match": match})
        if not match:
            all_match = False
        results.append(row)
    return {"all_match": all_match, "entries": results,
           "status": "TRANSFER_VERIFIED" if all_match else "TRANSFER_FAILED"}


def remote_deletion_eligible(transfer_result, artifact_completeness_ok, explicit_opt_in):
    """🔴 §16 — SHA match 가 성공해도 자동 삭제하지 않는다. explicit_opt_in=False 가
    기본값이며, 이 함수 호출자가 명시적으로 True 를 넘기지 않으면 항상 NO DELETE."""
    if not explicit_opt_in:
        return {"eligible": False, "reason": "NO_DELETE — explicit opt-in absent (default disabled)"}
    if not transfer_result["all_match"]:
        return {"eligible": False, "reason": "TRANSFER_FAILED — REMOTE_DELETE_FORBIDDEN"}
    if not artifact_completeness_ok:
        return {"eligible": False, "reason": "EVIDENCE_PACKAGE_INCOMPLETE — REMOTE_DELETE_FORBIDDEN"}
    return {"eligible": True, "reason": "ALL_CONDITIONS_MET"}


# ---------------------------------------------------------------------------
# Repair E/§18 — 필수 artifact contract 검증
# ---------------------------------------------------------------------------
def check_artifact_completeness(root_dir, n_trials, require_artifact_manifest=True):
    """🔴 advisor 지적 — `artifact_sha256.json` 은 자기 자신이 만들어지기 **전**에는
    존재할 수 없다(자기참조). manifest.json 에 쓰는 `evidence_package_core_status`
    는 require_artifact_manifest=False(manifest/trials/summary 만)로 계산하고,
    artifact_sha256.json 을 다 쓴 **뒤에** require_artifact_manifest=True(전체 contract)
    로 다시 계산해 run_full_sham() 반환값에만 포함한다 — manifest.json 은 그 이후
    다시 쓰지 않는다."""
    expected = (["manifest.json"]
               + ["trials/trial_%02d.json" % i for i in range(n_trials)]
               + ["summary/sham_summary.json"])
    if require_artifact_manifest:
        expected = expected + ["artifact_sha256.json"]
    missing = [p for p in expected if not os.path.exists(os.path.join(root_dir, p))]
    return {"expected": expected, "missing": missing,
           "status": "COMPLETE" if not missing else "EVIDENCE_PACKAGE_INCOMPLETE"}
