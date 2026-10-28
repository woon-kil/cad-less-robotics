#!/usr/bin/env python3
"""P-007F-04 — Single-Motion Execution Safety Review & Preregistration: candidate 생성.

🔴 이 파일은 numpy 외 어떤 하드웨어/네트워크 라이브러리도 import 하지 않는다. 로봇에
   명령을 보내지 않는다(ROBOT_COMMAND_COUNT=0). Arm_serial_servo_write/read, torque,
   reset, calibration write, action playback, PWM write, position write 중 어느 것도
   참조하지 않는다 — p007f_03_hardware_free_check.py 와 동일한 AST 검사 대상이다.

P-007E-R2-R4(scripts/p007e_r2r4_fk.py, sha256 a4e9d2d7891597a6bfa62c8ca87303aeab28ba0a784
f23f2640ca8dc145622cb)를 SHA-pin import 로 그대로 재사용한다 — FK 수학·joint-limit·
mesh/STL 로더를 재구현하지 않는다. 이 게이트가 새로 추가하는 것은:
  1. 현재 controller-reported ID2(154, §3/§4 read-only probe 로 재확인)를 기준으로
     ±1/±2/±3 양방향 candidate 를 비교한다(154→154 가 아니라 R4 의 152→154 를 그대로
     재탕하지 않는다).
  2. self-collision spot-check(PRE/MID/POST, base_link-Arm2_Link / Arm1_Link-Arm2_Link)
     를 scipy 없이 stride=40 chunked brute-force 로 candidate 마다 직접 수행한다
     (R4 는 이 계산을 1회만 했고, 그 계산 자체를 수행한 스크립트는 저장되지 않았다 —
     여기서 재구현한다. 알고리즘은 R4 collision_check.json 의 방법 설명과 동일).

모든 수치는 MODEL_ASSUMPTION_ONLY 다 — controller-reported state 가 physical state 와
대응한다고 가정했을 때의 예측이며, physical pose proof 가 아니다.
"""
import hashlib
import importlib.util
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FK_PATH = os.path.join(ROOT, "scripts", "p007e_r2r4_fk.py")
FK_SHA_EXPECTED = "a4e9d2d7891597a6bfa62c8ca87303aeab28ba0a784f23f2640ca8dc145622cb"

CURRENT_CONTROLLER_REPORTED_ID2 = 154  # §4 read-only probe (이 gate, 2026-10-29) 재확인값
CANDIDATE_DELTAS = [-3, -2, -1, 1, 2, 3]
COLLISION_STRIDE = 40
MODEL_UNCERTAINTY_MARGIN_MM = 2.0  # P-007E-R2-R4 envelope_derivation 과 동일 고정·비실측 pad


def _sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_fk_module():
    actual = _sha256(FK_PATH)
    if actual != FK_SHA_EXPECTED:
        raise SystemExit(f"FK module sha mismatch: expected {FK_SHA_EXPECTED} got {actual}")
    spec = importlib.util.spec_from_file_location("p007e_r2r4_fk_pinned", FK_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, actual


def chunked_min_dist_mm(verts_a_m, verts_b_m, chunk=800):
    """두 정점 집합(미터) 간 최소 유클리드 거리(mm). scipy 없이 메모리 상한을 위해
    chunk 단위로 순회한다(1 pair 당 약 1.3s/600MB 실측, P-007E-R2-R4 collision_check.json
    과 동일한 stride=40 brute-force 방법)."""
    best = np.inf
    for i in range(0, len(verts_a_m), chunk):
        block = verts_a_m[i:i + chunk]
        d = np.linalg.norm(block[:, None, :] - verts_b_m[None, :, :], axis=2)
        best = min(best, float(d.min()))
    return best * 1000.0


def collision_spot_check(fk, joints, mesh_dir, id2_deg, stride=COLLISION_STRIDE):
    """base_link-Arm2_Link / Arm1_Link-Arm2_Link 최소거리(mm) — R4 와 동일 pair,
    동일 downsample 방법. base_link/Arm1_Link 는 고정(FK 가 Arm2 각도에 무관)."""
    poses = fk.fk_tree(joints, fk.full_angles(id2_deg))
    out = {}
    # 🔴 fk.load_stl_vertices() 는 이미 미터 단위를 반환한다(STL 자체가 meter —
    # p007e_r2r4_fk.py 의 mesh_radius_table_mm()/transform_mesh_to_base() 양쪽이
    # 추가 /1000 없이 그대로 poses(미터)와 합성하는 것으로 확인됨, R4 swept_motion.json
    # "mesh_R_vs_precedent" 주석도 동일하게 명시). 1차 초안에서 여기 /1000.0 을
    # 중복 적용해 링크 origin 간 거리로 붕괴하는 버그가 있었다(33mm/122mm 로 전혀
    # 의미없이 angle-불변인 결과가 나와 발견) — 제거한다.
    base_v = fk.load_stl_vertices(os.path.join(mesh_dir, "base_link.STL"))[::stride]
    arm1_v_local = fk.load_stl_vertices(os.path.join(mesh_dir, "Arm1_Link.STL"))[::stride]
    arm2_v_local = fk.load_stl_vertices(os.path.join(mesh_dir, "Arm2_Link.STL"))[::stride]

    p1, R1 = poses["Arm1_Link"]["p"], poses["Arm1_Link"]["R"]
    arm1_v = arm1_v_local @ R1.T + p1
    p2, R2 = poses["Arm2_Link"]["p"], poses["Arm2_Link"]["R"]
    arm2_v = arm2_v_local @ R2.T + p2

    out["min_dist_base_link_Arm2_Link_mm"] = chunked_min_dist_mm(base_v, arm2_v)
    out["min_dist_Arm1_Link_Arm2_Link_mm"] = chunked_min_dist_mm(arm1_v, arm2_v)
    out["id2_lib_deg"] = id2_deg
    out["downsample_stride"] = stride
    return out


def rotation_delta_deg(R_pre, R_post):
    dR = R_post @ R_pre.T
    return fk_module_rotation_angle_deg(dR)


def fk_module_rotation_angle_deg(R):
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(tr))


def build_candidate(fk, joints, mesh_dir, pre_lib_deg, target_lib_deg):
    delta_deg = target_lib_deg - pre_lib_deg
    delta_rad = math.radians(delta_deg)

    pre_poses = fk.fk_tree(joints, fk.full_angles(pre_lib_deg))
    post_poses = fk.fk_tree(joints, fk.full_angles(target_lib_deg))

    # joint-limit margins (PRE 는 공통, POST 는 candidate 마다)
    lim_pre = fk.joint_limit_margins(joints, fk.full_angles(pre_lib_deg))
    lim_post = fk.joint_limit_margins(joints, fk.full_angles(target_lib_deg))

    # swept displacement — exact (chord length, 2*sin(delta/2) * r_perp)
    axis_origin, axis_dir = fk.arm2_axis_params(joints, pre_poses)
    rperp_details, worst_link_exact, worst_rperp_mm = fk.exact_moving_group_max_rperp_mm(
        joints, pre_poses, axis_origin, axis_dir)
    disp_coeff = abs(2 * math.sin(delta_rad / 2.0))
    max_disp_exact_mm = worst_rperp_mm * disp_coeff
    per_link_disp_mm = {link: r * disp_coeff for link, r in rperp_details.items()}

    mesh_radius_mm = fk.mesh_radius_table_mm(fk.ALL_MOVING_LINKS)
    sphere_rows, worst_link_sphere = fk.sphere_bound_table_mm(
        joints, pre_poses, axis_origin, axis_dir, mesh_radius_mm)
    sphere_bound_mm = sphere_rows[worst_link_sphere]["bound_r_mm"] * disp_coeff / 1.0
    # sphere_bound_table_mm 의 bound_r_mm 은 origin_perp+mesh_R 이미 mm 단위 반경 —
    # 실제 displacement 상한은 그 반경에 동일한 disp_coeff 를 곱해야 한다
    sphere_bound_disp_mm = {link: row["bound_r_mm"] * disp_coeff for link, row in sphere_rows.items()}
    worst_sphere_disp_link = max(sphere_bound_disp_mm, key=sphere_bound_disp_mm.get)
    worst_sphere_disp_mm = sphere_bound_disp_mm[worst_sphere_disp_link]

    D_model_mm = max_disp_exact_mm
    D_conservative_mm = D_model_mm + MODEL_UNCERTAINTY_MARGIN_MM

    tcp_link = "Gripping_point_Link"
    dabai_link = "DaBai_DCW2_Link"
    tcp_pre_mm = (pre_poses[tcp_link]["p"] * 1000.0).tolist()
    tcp_post_mm = (post_poses[tcp_link]["p"] * 1000.0).tolist()
    tcp_translation_mm = float(np.linalg.norm(post_poses[tcp_link]["p"] - pre_poses[tcp_link]["p"]) * 1000.0)
    tcp_rotation_deg = rotation_delta_deg(pre_poses[tcp_link]["R"], post_poses[tcp_link]["R"])

    dabai_pre_mm = (pre_poses[dabai_link]["p"] * 1000.0).tolist()
    dabai_post_mm = (post_poses[dabai_link]["p"] * 1000.0).tolist()
    dabai_translation_mm = float(np.linalg.norm(post_poses[dabai_link]["p"] - pre_poses[dabai_link]["p"]) * 1000.0)
    dabai_rotation_deg = rotation_delta_deg(pre_poses[dabai_link]["R"], post_poses[dabai_link]["R"])

    mid_lib_deg = pre_lib_deg + delta_deg / 2.0
    collision = {
        "PRE": collision_spot_check(fk, joints, mesh_dir, pre_lib_deg),
        "MID": collision_spot_check(fk, joints, mesh_dir, mid_lib_deg),
        "POST": collision_spot_check(fk, joints, mesh_dir, target_lib_deg),
    }

    raw_target = fk.raw_from_lib_angle_id234(target_lib_deg)
    raw_pre = fk.raw_from_lib_angle_id234(pre_lib_deg)
    payload = fk.command_payload(2, raw_target, 2000)

    return {
        "delta_deg": delta_deg,
        "pre_lib_deg": pre_lib_deg,
        "target_lib_deg": target_lib_deg,
        "joint_limit_pre": lim_pre,
        "joint_limit_post": lim_post,
        "joint_limit_pass": all(v["status"] == "PASS" for v in lim_post.values()),
        "arm2_margin_to_lower_post_rad": lim_post["Arm2_Joint"]["margin_to_lower_rad"],
        "arm2_margin_to_upper_post_rad": lim_post["Arm2_Joint"]["margin_to_upper_rad"],
        "swept_displacement": {
            "worst_link_exact": worst_link_exact,
            "worst_rperp_mm": worst_rperp_mm,
            "disp_coeff_2sin_half_delta": disp_coeff,
            "max_disp_exact_mm": max_disp_exact_mm,
            "per_link_disp_mm": per_link_disp_mm,
            "worst_link_sphere_bound": worst_sphere_disp_link,
            "sphere_bound_disp_mm": worst_sphere_disp_mm,
            "sphere_geq_exact": worst_sphere_disp_mm >= max_disp_exact_mm,
        },
        "conservative_envelope": {
            "D_model_mm": D_model_mm,
            "model_uncertainty_margin_mm": MODEL_UNCERTAINTY_MARGIN_MM,
            "D_conservative_mm": D_conservative_mm,
        },
        "tcp": {
            "pre_mm": tcp_pre_mm, "post_mm": tcp_post_mm,
            "translation_mm": tcp_translation_mm, "rotation_deg": tcp_rotation_deg,
        },
        "dabai": {
            "pre_mm": dabai_pre_mm, "post_mm": dabai_post_mm,
            "translation_mm": dabai_translation_mm, "rotation_deg": dabai_rotation_deg,
        },
        "collision": collision,
        "command_encoding": {
            "api": "Arm_serial_servo_write", "id": 2, "angle": target_lib_deg, "time_ms": 2000,
            "raw_pre": raw_pre, "raw_target": raw_target,
            "register_hex": hex(payload["register"]), "payload_bytes_hex": [hex(b) for b in payload["bytes"]],
        },
    }


def main():
    fk, fk_sha = load_fk_module()
    joints, _ = fk.load_joints()
    mesh_dir = os.path.join(ROOT, ".cache", "jetson_robot_model", "meshes")

    candidates = []
    for delta in CANDIDATE_DELTAS:
        target = CURRENT_CONTROLLER_REPORTED_ID2 + delta
        candidates.append(build_candidate(fk, joints, mesh_dir, CURRENT_CONTROLLER_REPORTED_ID2, target))

    out = {
        "gate": "P-007F-04",
        "fk_module_sha256": fk_sha,
        "current_controller_reported_id2": CURRENT_CONTROLLER_REPORTED_ID2,
        "model_assumption_only": True,
        "candidates": candidates,
    }
    out_dir = os.path.join(ROOT, "evidence", "P-007F-04")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "candidate_raw.json"), "w") as f:
        json.dump(out, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else o)
        f.write("\n")
    print(json.dumps({"candidates_built": len(candidates), "fk_module_sha256": fk_sha}, indent=2))


if __name__ == "__main__":
    main()
