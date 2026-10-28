#!/usr/bin/env python3
"""P-007E-R2-R4 — tree FK + STL geometry helpers (kinematics/collision 계산 전용).

🔴 scripts/p007b_fk.py 는 **직렬 체인**(CHAIN 리스트)만 다룬다. 이 로봇의 실제 URDF는
   Arm4_Link 에서 DaBai_DCW2_Link(fixed)로, Arm5_Link 에서 grip_joint/llink_joint1/
   rlink_joint3/llink_joint3/Gripping_point_Joint 로 **가지(branch)가 갈라진다**.
   이 파일은 parent/child 관계를 따라 트리 전체를 순회하는 FK를 구현한다 — p007b_fk.py
   의 H0 convention·회전행렬 수학은 그대로 재사용(복붙 아님, 동일 공식 재구현 후
   self-check로 일치 확인)한다.

NO CAMERA CAPTURE / NO CONTROLLER ACCESS / NO SERVO READ / NO SERVO WRITE / NO ROBOT
MOTION — 이 파일은 numpy 외 어떤 하드웨어/네트워크 라이브러리도 import 하지 않는다.

모든 계산 결과는 EXPECTED / MODEL-PREDICTED 다. 실측이 아니다.
"""
import json
import math
import os
import struct

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_JOINTS_PATH = os.path.join(ROOT, "evidence/P-007B/urdf_joints.json")
MESH_DIR = os.path.join(ROOT, ".cache/jetson_robot_model/meshes")

H0_OFFSET_DEG = 90.0


def h0(lib_deg):
    """벤더 convention. q_urdf = (q_library_deg - 90) * pi/180. 보정값이 아니다."""
    return (lib_deg - H0_OFFSET_DEG) * math.pi / 180.0


def rot_matrix(axis, q):
    """Rodrigues 축-각 회전행렬. axis 는 단위벡터여야 한다(URDF axis_xyz 관례)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def rpy_matrix(r, p, y):
    return rot_matrix((0, 0, 1), y) @ rot_matrix((0, 1, 0), p) @ rot_matrix((1, 0, 0), r)


def load_joints(path=URDF_JOINTS_PATH):
    data = json.load(open(path, encoding="utf-8"))
    joints = {j["name"]: j for j in data["joints"]}
    return joints, data


def build_tree(joints):
    """parent link -> [joint_name, ...] 인접리스트. base_link 가 root(암묵, URDF에 joint 없음)."""
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    return children


def fk_tree(joints, angles_deg, children=None, gripper_passive_deg=None):
    """전체 트리 FK. angles_deg: {joint_name: library_degree}(revolute 관절만, 나머지는
    URDF 고정/기본값 0 사용). gripper_passive_deg: rlink_joint2/3, llink_joint1/2/3 같은
    continuous(비구동) 관절의 라디안 값을 override 하고 싶을 때 {joint_name: rad}.
    반환: {link_name: {"p": np.array(3,), "R": np.array(3,3)}} — base_link 포함."""
    children = children or build_tree(joints)
    gripper_passive_deg = gripper_passive_deg or {}
    poses = {"base_link": {"p": np.zeros(3), "R": np.eye(3)}}

    def visit(parent_link):
        for jname in children.get(parent_link, []):
            j = joints[jname]
            p_parent, R_parent = poses[parent_link]["p"], poses[parent_link]["R"]
            oxyz = np.array([float(x) for x in j["origin_xyz"].split()])
            orpy = [float(x) for x in (j["origin_rpy"] or "0 0 0").split()]
            p = p_parent + R_parent @ oxyz
            R = R_parent @ rpy_matrix(*orpy)
            if j["type"] == "revolute" and jname in angles_deg:
                q = h0(angles_deg[jname])
                axis = [float(x) for x in j["axis_xyz"].split()]
                R = R @ rot_matrix(axis, q)
            elif j["type"] == "continuous" and jname in gripper_passive_deg:
                axis = [float(x) for x in j["axis_xyz"].split()]
                R = R @ rot_matrix(axis, gripper_passive_deg[jname])
            elif j["type"] == "revolute" and jname in gripper_passive_deg:
                axis = [float(x) for x in j["axis_xyz"].split()]
                R = R @ rot_matrix(axis, gripper_passive_deg[jname])
            poses[j["child"]] = {"p": p, "R": R}
            visit(j["child"])

    visit("base_link")
    return poses


# ---------------------------------------------------------------------------
# STL (binary) vertex loader — numpy only
# ---------------------------------------------------------------------------
def load_stl_vertices(path):
    """binary STL (80-byte header + uint32 count + 50 bytes/triangle). 중복 정점 허용
    (dedup 하지 않는다 — max-radius/거리 계산에는 불필요하고, dedup 자체가 버그 표면이다)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        buf = f.read(n * 50)
    if len(buf) != n * 50:
        raise ValueError("STL truncated: %s expected %d got %d" % (path, n * 50, len(buf)))
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(n, 50)
    # 각 triangle record: 12 bytes normal(f32x3) + 36 bytes vertices(3 x f32x3) + 2 bytes attr
    tri_vertex_bytes = arr[:, 12:48]
    verts = tri_vertex_bytes.reshape(n * 3, 12).view("<f4").reshape(n * 3, 3).astype(np.float64)
    return verts  # millimeters (STL 단위, URDF meshes 는 mm 단위로 export 된 것으로 보임 — 검증은 호출측)


def mesh_bounding_sphere_radius(vertices_local_mm, origin_mm=(0.0, 0.0, 0.0)):
    """링크 origin 기준 최대 정점 거리(mm) — fk_clearance.json 의 mesh_R_mm 과 동일 정의."""
    origin = np.asarray(origin_mm, dtype=float)
    d = np.linalg.norm(vertices_local_mm - origin, axis=1)
    return float(d.max())


def point_line_perp_distance(points, line_point, line_dir):
    """points: (N,3) — line_dir 는 단위벡터. 각 점의 직선으로부터 수직거리 배열 반환."""
    line_dir = np.asarray(line_dir, dtype=float)
    line_dir = line_dir / np.linalg.norm(line_dir)
    v = points - np.asarray(line_point, dtype=float)
    proj = (v @ line_dir)[:, None] * line_dir[None, :]
    perp = v - proj
    return np.linalg.norm(perp, axis=1)


def rotation_angle_deg(R):
    """회전행렬의 축-각 표현에서 각(deg)만 추출. tr(R) = 1 + 2cos(theta)."""
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(tr))


def rotation_axis(R):
    """회전행렬의 회전축(단위벡터). theta≈0 이면 정의되지 않으므로 호출측에서 가드."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(w)
    if n < 1e-12:
        return None
    return w / n


# ---------------------------------------------------------------------------
# §5/§6/§7 — command geometry 상수 + joint-limit/axis 계산
# ---------------------------------------------------------------------------
BASE_JOINT_ANGLES_LIB = {"Arm1_Joint": 86, "Arm3_Joint": 0, "Arm4_Joint": 18, "Arm5_Joint": 93}
PRE_LIB_ID2, POST_LIB_ID2 = 152, 154

RIGID_MOVING_LINKS = ("Arm2_Link", "Arm3_Link", "Arm4_Link", "Arm5_Link",
                      "DaBai_DCW2_Link", "Gripping_point_Link")
GRIPPER_GRIP_JOINT_DEPENDENT_LINKS = ("rlink1", "rlink2")
GRIPPER_DEFAULT_ONLY_LINKS = ("llink1", "llink2", "llink3", "rlink3")
ALL_MOVING_LINKS = RIGID_MOVING_LINKS + GRIPPER_GRIP_JOINT_DEPENDENT_LINKS + GRIPPER_DEFAULT_ONLY_LINKS
FIXED_LINKS = ("base_link", "Arm1_Link")


def full_angles(id2_lib_deg, base=None):
    base = dict(base or BASE_JOINT_ANGLES_LIB)
    base["Arm2_Joint"] = id2_lib_deg
    return base


def arm2_axis_params(joints, poses):
    """Arm2 조인트의 base-frame 축 원점·방향. Arm1 이 변하지 않는 한 PRE/POST 동일하다."""
    arm1_R = poses["Arm1_Link"]["R"]
    axis_local = [float(x) for x in joints["Arm2_Joint"]["axis_xyz"].split()]
    axis_dir = arm1_R @ np.array(axis_local)
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    axis_origin = poses["Arm2_Link"]["p"]
    return axis_origin, axis_dir


def transform_mesh_to_base(link, poses, stride=1):
    """STL local-frame 정점(m)을 해당 link 의 FK pose 로 base frame 으로 옮긴다."""
    verts = load_stl_vertices(os.path.join(MESH_DIR, link + ".STL"))
    if stride > 1:
        verts = verts[::stride]
    p, R = poses[link]["p"], poses[link]["R"]
    return verts @ R.T + p


def mesh_radius_table_mm(links):
    """각 link 의 mesh_R_mm(링크 origin 기준 최대 정점거리) — STL 에서 직접 계산."""
    out = {}
    for link in links:
        verts = load_stl_vertices(os.path.join(MESH_DIR, link + ".STL"))
        out[link] = mesh_bounding_sphere_radius(verts) * 1000.0
    return out


def exact_moving_group_max_rperp_mm(joints, pre_poses, axis_origin, axis_dir,
                                    grip_joint_samples=9):
    """§6 — moving-group 전체 정점의 Arm2 축 수직거리 최대값(mm, EXACT — sphere 근사 아님).
    grip_joint(그리퍼 주 구동축)은 자신의 URDF limit 구간을 샘플링해 최댓값을 취한다.
    나머지 passive linkage joint(rlink_joint2/3, llink_joint1/2/3)는 URDF 기본값(0 rad)
    에서만 평가한다 — 이 점은 호출측이 명시적으로 공개해야 하는 잔여 한계다."""
    details = {}
    for link in RIGID_MOVING_LINKS:
        v = transform_mesh_to_base(link, pre_poses)
        details[link] = float(point_line_perp_distance(v, axis_origin, axis_dir).max() * 1000.0)

    grip_joint = joints["grip_joint"]
    lo, hi = float(grip_joint["limit_lower"]), float(grip_joint["limit_upper"])
    samples = np.linspace(lo, hi, grip_joint_samples)
    for link in GRIPPER_GRIP_JOINT_DEPENDENT_LINKS:
        best = 0.0
        for s in samples:
            poses_s = fk_tree(joints, full_angles(PRE_LIB_ID2), gripper_passive_deg={"grip_joint": s})
            v = transform_mesh_to_base(link, poses_s)
            best = max(best, float(point_line_perp_distance(v, axis_origin, axis_dir).max() * 1000.0))
        details[link] = best

    for link in GRIPPER_DEFAULT_ONLY_LINKS:
        v = transform_mesh_to_base(link, pre_poses)
        details[link] = float(point_line_perp_distance(v, axis_origin, axis_dir).max() * 1000.0)

    worst_link = max(details, key=details.get)
    return details, worst_link, details[worst_link]


def sphere_bound_table_mm(joints, pre_poses, axis_origin, axis_dir, mesh_radius_mm):
    """§6 cross-check — fk_clearance.json 선례와 동일한 origin_perp+mesh_R 구-경계 방법."""
    rows = {}
    for link in ALL_MOVING_LINKS:
        origin_mm = pre_poses[link]["p"] * 1000.0
        origin_perp_mm = float(point_line_perp_distance(
            origin_mm[None, :], axis_origin * 1000.0, axis_dir)[0])
        bound_r_mm = origin_perp_mm + mesh_radius_mm[link]
        rows[link] = {"origin_perp_mm": origin_perp_mm, "mesh_R_mm": mesh_radius_mm[link],
                     "bound_r_mm": bound_r_mm}
    worst_link = max(rows, key=lambda k: rows[k]["bound_r_mm"])
    return rows, worst_link


def joint_limit_margins(joints, lib_angles):
    """§7 — 5개 arm joint 의 margin_to_lower/upper(rad). PASS/VIOLATION 판정 포함."""
    out = {}
    for jname in ("Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"):
        j = joints[jname]
        lower, upper = float(j["limit_lower"]), float(j["limit_upper"])
        q = h0(lib_angles[jname])
        margin_lo, margin_hi = q - lower, upper - q
        out[jname] = {"library_deg": lib_angles[jname], "q_urdf_rad": q,
                      "lower_rad": lower, "upper_rad": upper,
                      "margin_to_lower_rad": margin_lo, "margin_to_upper_rad": margin_hi,
                      "status": "PASS" if (margin_lo >= 0 and margin_hi >= 0) else "VIOLATION"}
    return out


# ---------------------------------------------------------------------------
# §11/§12 — Arm_serial_servo_write/read raw conversion (id in {2,3,4})
#   source: /home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py (sha256
#   edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c, read-only cat 로
#   재확인, import 하지 않음)
# ---------------------------------------------------------------------------
def raw_from_lib_angle_id234(lib_angle):
    angle = 180 - lib_angle
    return int((3100 - 900) * (angle - 0) / (180 - 0) + 900)


def lib_angle_from_raw_id234(raw):
    pos = int((180 - 0) * (raw - 900) / (3100 - 900) + 0)
    if pos > 180 or pos < 0:
        return None
    return 180 - pos


def command_payload(id_, raw_pos, duration_ms):
    value_H = (raw_pos >> 8) & 0xFF
    value_L = raw_pos & 0xFF
    time_H = (duration_ms >> 8) & 0xFF
    time_L = duration_ms & 0xFF
    return {"register": 0x10 + id_, "bytes": [value_H, value_L, time_H, time_L]}


def readback_bucket_for_value(target_read, raw_lo=1100, raw_hi=1400):
    matches = [r for r in range(raw_lo, raw_hi) if lib_angle_from_raw_id234(r) == target_read]
    return (matches[0], matches[-1]) if matches else None
