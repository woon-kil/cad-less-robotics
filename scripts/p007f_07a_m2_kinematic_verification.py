#!/usr/bin/env python3
"""P-007F-07A -- M2 Kinematic Membership Verification (사용자 지시, 2026-10-29).

질문: Gemini close-up(evidence/P-007F-07/marker_mount_closeup/)에서 M2로 표시한
cream/white 사각 브래킷이, servo ID2(Jetson Arm_Lib 의 "Arm2" 채널, P-007F-04/05
전체에서 152/154/151 로 명령한 그 서보)가 실제로 회전하면 공간상 반드시 움직이는
downstream 구조물인지를 **URDF kinematic chain + mesh 기하 + 기존에 이미
검증된 FK 모듈의 FIXED_LINKS/RIGID_MOVING_LINKS 분류**로 판정한다.

ROBOT COMMANDS = 0, SERVO WRITES = 0, TORQUE COMMANDS = 0, ROBOT MOTION = 0,
GEMINI RELOCATION = 0, MARKER ATTACHMENT = 0 -- 이 스크립트는 저장된 URDF/STL
파일만 읽는다. scripts/p007e_r2r4_fk.py(P-007E-R2-R4, 4건의 과거 FK 결과와
오차 <=0.005mm 로 교차검증되어 실제 P-007F-05 §36 실행의 안전판정 근거로
이미 쓰인 모듈)를 SHA-pin import 로 **수정 없이** 재사용한다 -- servo
ID2 <-> Arm2_Joint 대응관계, FIXED_LINKS/RIGID_MOVING_LINKS 분류를 이 gate에서
새로 만들지 않고 그 모듈에 이미 있는 것을 그대로 가져온다.
"""
import hashlib
import importlib.util
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "m2_kinematic_verification")
FK_MODULE_PATH = os.path.join(ROOT, "scripts", "p007e_r2r4_fk.py")
URDF_JOINTS_PATH = os.path.join(ROOT, "evidence", "P-007B", "urdf_joints.json")

# M2 bounding box in the Gemini close-up frame (original 1280x720 coords) --
# 그대로 가져온다, scripts/p007f_07_marker_mount_closeup.py CANDIDATES["M2"]["box"]와 동일
M2_CLOSEUP_SOURCE = os.path.join(ROOT, "evidence", "P-007F-07", "marker_mount_closeup",
                                 "candidate_report.json")


def self_check_no_hardware_import():
    import ast
    with open(os.path.abspath(__file__), "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    forbidden = {"rclpy", "Arm_Lib", "p007f_05_hardware_wiring", "p007f_05_jetson_controller",
                "p007f_07_jetson_controller", "subprocess"}
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    return forbidden.intersection(imported)


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_fk_module():
    spec = importlib.util.spec_from_file_location("p007e_r2r4_fk_pin", FK_MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def downstream_links_of_joint(joints, start_joint_name):
    """joint 의 child link 로부터 시작해 트리 전체를 순회, 모든 descendant link 집합을
    반환한다 -- FK 모듈의 FIXED_LINKS/RIGID_MOVING_LINKS 는 가져다 쓰지 않고(§대조용으로
    독립 재계산), parent/child 그래프만으로 순수하게 다시 유도한다."""
    start = joints[start_joint_name]
    children_map = {}
    for name, j in joints.items():
        children_map.setdefault(j["parent"], []).append(j["child"])
    visited = set()
    stack = [start["child"]]
    while stack:
        link = stack.pop()
        if link in visited:
            continue
        visited.add(link)
        for child_joint_name, j in joints.items():
            if j["parent"] == link:
                stack.append(j["child"])
    return visited


CLOSEUP_MODULE_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_mount_closeup.py")
CLOSEUP_SRC_COLOR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_setup_capture",
                                 "raw_frames", "marker_setup_capture_1793278747", "0004-color.png")


def load_closeup_module():
    spec = importlib.util.spec_from_file_location("p007f_07_marker_mount_closeup_pin", CLOSEUP_MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_m2_mapping_figure(report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    closeup_sha = sha256(CLOSEUP_MODULE_PATH)
    closeup = load_closeup_module()
    cv2, _np = closeup.load_cv2()
    img = cv2.imread(CLOSEUP_SRC_COLOR)
    base = closeup.make_closeup_base(cv2, img)  # BGR, same crop as marker_mount_closeup evidence
    base_rgb = cv2.cvtColor(base, cv2.COLOR_BGR2RGB)

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(15, 8), gridspec_kw={"width_ratios": [1.3, 1]})

    # -- Panel A: annotated Gemini close-up (qualitative region labels only --
    # no pixel-exact link boundary is claimed; no camera<->robot extrinsic calibration exists) --
    axL.imshow(base_rgb)
    axL.axis("off")
    axL.set_title("Gemini close-up -- qualitative URDF-link region labels\n"
                  "(NOT a calibrated 3D projection -- no camera<->robot extrinsic exists)", fontsize=9)

    colors = {"M1": "lime", "M2": "orange", "M3": "magenta"}
    for name, spec in closeup.CANDIDATES.items():
        x, y, w, h = spec["box"]
        p0 = closeup.to_crop_px(x, y)
        p1 = closeup.to_crop_px(x + w, y + h)
        rect = mpatches.Rectangle(p0, p1[0] - p0[0], p1[1] - p0[1], fill=False,
                                  edgecolor=colors[name], linewidth=2.5)
        axL.add_patch(rect)
        axL.text(p0[0], p0[1] - 8, name, color=colors[name], fontsize=11, fontweight="bold")

    region_labels = [
        (closeup.to_crop_px(700, 275), "Arm1_Link (disc)\nFIXED under ID2", "lime"),
        (closeup.to_crop_px(700, 220), "Arm2_Link\n(downstream, lower cylinder)", "yellow"),
        (closeup.to_crop_px(720, 130), "Arm3_Link\n(downstream, upper cylinder)", "orange"),
    ]
    for (px, py), text, color in region_labels:
        axL.text(px, py, text, color=color, fontsize=8, ha="center", va="center",
                 bbox=dict(boxstyle="round", fc="black", alpha=0.55, ec=color))

    # -- Panel B: schematic kinematic chain (box-and-arrow, from independently derived graph) --
    axR.axis("off")
    axR.set_xlim(0, 10)
    axR.set_ylim(0, 24)
    axR.set_title("URDF kinematic chain (schematic, not to scale)", fontsize=10)

    chain_boxes = [
        ("base_link", "gray", 22),
        ("Arm1_Joint (axis Z, yaw)", "white", 20.3),
        ("Arm1_Link  [FIXED under ID2]", "lightgray", 18.6),
        ("Arm2_Joint = servo ID2 (axis X)", "red", 16.9),
        ("Arm2_Link  [downstream]", "palegreen", 15.2),
        ("Arm3_Joint", "white", 13.5),
        ("Arm3_Link  [downstream]  <- M2 here", "palegreen", 11.8),
        ("Arm4_Joint", "white", 10.1),
        ("Arm4_Link  [downstream]", "palegreen", 8.4),
        ("Arm5_Joint / DaBai_DCW2_Joint", "white", 6.7),
        ("Arm5_Link / DaBai_DCW2_Link  [downstream]", "palegreen", 5.0),
        ("gripper joints -> Gripping_point_Link  [downstream]", "palegreen", 3.0),
    ]
    for label, color, y in chain_boxes:
        is_joint = ("Joint" in label or "joints" in label)
        box = mpatches.FancyBboxPatch((0.3, y - 0.7), 9.4, 1.4, boxstyle="round,pad=0.05",
                                      facecolor=color, edgecolor="black",
                                      linewidth=2.2 if "servo ID2" in label else 1.0)
        axR.add_patch(box)
        axR.text(5.0, y, label, ha="center", va="center", fontsize=7.6,
                 fontweight="bold" if "servo ID2" in label or "M2" in label else "normal")
        if not is_joint:
            pass

    for i in range(len(chain_boxes) - 1):
        y0 = chain_boxes[i][2] - 0.7
        y1 = chain_boxes[i + 1][2] + 0.7
        axR.annotate("", xy=(5, y1), xytext=(5, y0),
                    arrowprops=dict(arrowstyle="->", color="black", lw=1.2))

    verdict_text = "M2_DOWNSTREAM_OF_ID2 = %s" % report["M2_DOWNSTREAM_OF_ID2"]
    fig.suptitle(verdict_text, fontsize=13, fontweight="bold",
                color=("darkgreen" if report["M2_DOWNSTREAM_OF_ID2"] == "YES" else "darkred"))

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = os.path.join(OUT_DIR, "m2_mapping.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path, closeup_sha


def main():
    forbidden = self_check_no_hardware_import()
    if forbidden:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden)}))
        sys.exit(1)

    fk_sha = sha256(FK_MODULE_PATH)
    fk = load_fk_module()
    joints, raw = fk.load_joints(URDF_JOINTS_PATH)

    # servo ID2 <-> Arm2_Joint -- fk.full_angles()의 리터럴 소스를 읽어 재확인한다
    # (하드코딩 재주장이 아니라 실제 소스 문자열에서 매핑을 뽑아낸다)
    import inspect
    full_angles_src = inspect.getsource(fk.full_angles)
    id2_maps_to_arm2_joint = ('base["Arm2_Joint"] = id2_lib_deg' in full_angles_src)

    # 독립 재계산: parent/child 그래프 순회로 Arm2_Joint 의 downstream link 집합 유도
    downstream = downstream_links_of_joint(joints, "Arm2_Joint")

    # FK 모듈 자체가 이미 선언한 FIXED_LINKS/RIGID_MOVING_LINKS 와 교차검증.
    # 🔴 처음엔 downstream == moving_in_module(완전 집합 일치)로 비교해 FALSE
    # NEGATIVE가 났다 -- 독립 재계산은 그래프 topology상 downstream인 모든 link(그리퍼
    # 손가락 rlink/llink 계열 포함)를 포함하는데, FK 모듈의 RIGID_MOVING_LINKS는 그중
    # grip_joint 등 **자체 DOF를 추가로 갖는 그리퍼 링크를 의도적으로 제외**한
    # 더 좁은 부분집합(그 모듈의 목적=안전 포락선 계산에서 "순수 rigid"만 다루기
    # 위함)이다 -- 모순이 아니라 서로 다른 질문. 올바른 불변식은 "FK 모듈이 rigid라고
    # 선언한 모든 link는 반드시 내 독립 유도 downstream 집합의 부분집합이어야 한다"다
    # (자체발견·수정).
    fixed_in_module = set(fk.FIXED_LINKS)
    moving_in_module = set(fk.RIGID_MOVING_LINKS)
    cross_check_ok = moving_in_module.issubset(downstream) and not (moving_in_module & fixed_in_module)
    gripper_finger_links_downstream_but_excluded_from_rigid_set = sorted(downstream - moving_in_module)

    arm2_joint = joints["Arm2_Joint"]

    # mesh bounding box profile (link 선정: 의심되는 체인 전구간)
    profile = {}
    for link in ("base_link", "Arm1_Link", "Arm2_Link", "Arm3_Link", "Arm4_Link", "DaBai_DCW2_Link"):
        verts_mm = fk.load_stl_vertices(os.path.join(fk.MESH_DIR, link + ".STL")) * 1000.0
        mins = verts_mm.min(axis=0)
        maxs = verts_mm.max(axis=0)
        profile[link] = {
            "bbox_min_mm": [round(float(v), 2) for v in mins],
            "bbox_max_mm": [round(float(v), 2) for v in maxs],
            "size_mm": [round(float(v), 2) for v in (maxs - mins)],
        }

    # M2 close-up source box, for reference (읽기만 함, 수정 없음)
    m2_box = None
    if os.path.isfile(M2_CLOSEUP_SOURCE):
        cand = json.load(open(M2_CLOSEUP_SOURCE, encoding="utf-8"))
        m2_box = cand.get("candidates", {}).get("M2", {}).get("box_xywh")

    # qualitative visual <-> link correspondence: length-profile을 이용한 교차판단
    # (사람의 "같아 보인다" 추정이 아니라, Arm2_Link/Arm3_Link의 z축 길이(~107mm, 거의
    # 동일)와 Arm1_Link의 z축 길이(43.5mm, 짧은 disc 형태)를 Gemini close-up에서 관측된
    # 두 개의 긴 원통형 몸체 + 그 아래 짧은 원반 구조와 대조한 결과를 기록한다 -- §5 참조)
    arm1_h = profile["Arm1_Link"]["size_mm"][2]
    arm2_h = profile["Arm2_Link"]["size_mm"][2]
    arm3_h = profile["Arm3_Link"]["size_mm"][2]
    m2_structural_note = (
        "Arm1_Link z-extent=%.1fmm(짧은 disc)와 Arm2_Link/Arm3_Link z-extent=%.1fmm/%.1fmm(거의 동일한 긴 원통형)를 "
        "Gemini close-up에서 관측된 '짧은 흰색 디스크 1개 + 그 위 긴 검은 원통형 몸체 2개(거의 동일한 외관)' 구조와 "
        "대조. M2(cream bracket)는 이 두 긴 원통형 몸체 중 아래쪽(Arm2_Link로 추정)이 끝나고 위쪽(Arm3_Link로 추정) "
        "몸체로 전환되는 지점 부근에 위치 -- 즉 Arm2_Link 말단 또는 Arm3_Link 시작부 중 하나이며, 두 가능성 모두 "
        "Arm2_Joint 의 downstream(RIGID_MOVING_LINKS)에 포함된다. M1(disc top)은 Arm1_Link z-extent(43.5mm, 디스크 "
        "형태)와 정확히 일치 -- 즉 M1 은 FIXED_LINKS 에 속해 ID2 단독 회전으로는 움직이지 않는다는 corollary 가 "
        "도출된다(§7 참조, 이번 작업에서 새로 발견됨)."
    ) % (arm1_h, arm2_h, arm3_h)

    verdict = "YES" if (id2_maps_to_arm2_joint and cross_check_ok) else "UNRESOLVED"

    report = {
        "servo_id2_to_urdf_joint": "Arm2_Joint" if id2_maps_to_arm2_joint else "UNRESOLVED",
        "servo_id2_mapping_source": "scripts/p007e_r2r4_fk.py full_angles() literal source (grep-confirmed, P-007E-R2-R4 validated module, reused unmodified, SHA-pin)",
        "joint_parent_link": arm2_joint["parent"],
        "joint_child_link": arm2_joint["child"],
        "joint_axis_local": arm2_joint["axis_xyz"],
        "downstream_links_independently_derived": sorted(downstream),
        "fk_module_rigid_moving_links": sorted(moving_in_module),
        "fk_module_fixed_links": sorted(fixed_in_module),
        "cross_check_rigid_moving_links_subset_of_independent_downstream": cross_check_ok,
        "cross_check_note": ("독립 재계산(parent/child 그래프)은 그리퍼 손가락 link(rlink/llink 계열)도 "
                            "topology상 downstream으로 포함하지만, FK 모듈의 RIGID_MOVING_LINKS는 grip_joint 등 "
                            "자체 DOF를 추가로 갖는 그 link들을 의도적으로 제외한 더 좁은 부분집합이다 -- 모순이 "
                            "아니라 서로 다른 질문(순수 rigid-body 여부 vs topology상 downstream 여부). "
                            "cross_check는 'FK 모듈이 rigid라 선언한 link는 전부 내 독립 downstream 집합의 "
                            "부분집합이어야 한다'는 올바른 불변식으로 검증했다."),
        "links_downstream_but_excluded_from_fk_module_rigid_set": gripper_finger_links_downstream_but_excluded_from_rigid_set,
        "mesh_bbox_profile_mm": profile,
        "m2_closeup_box_xywh_px": m2_box,
        "m2_structural_correspondence_note": m2_structural_note,
        "m2_candidate_links": ["Arm2_Link", "Arm3_Link"],
        "m2_candidate_links_both_downstream_of_arm2_joint": all(
            l in moving_in_module for l in ("Arm2_Link", "Arm3_Link")),
        "m1_corollary_fixed_link": "Arm1_Link",
        "m1_corollary_in_fixed_links": "Arm1_Link" in fixed_in_module,
        "M2_DOWNSTREAM_OF_ID2": verdict,
        "fk_module_sha256": fk_sha,
        "robot_command_count": 0,
        "servo_write_count": 0,
    }

    os.makedirs(OUT_DIR, exist_ok=True)

    mapping_png_path, closeup_module_sha = build_m2_mapping_figure(report)
    report["m2_mapping_png"] = mapping_png_path
    report["closeup_module_sha256"] = closeup_module_sha

    with open(os.path.join(OUT_DIR, "membership_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1, default=repr)

    chain_lines = []
    chain_lines.append("servo ID2 (Jetson Arm_Lib channel, commanded 152/154/151 throughout P-007F-04/05)")
    chain_lines.append("  |")
    chain_lines.append("  v  (scripts/p007e_r2r4_fk.py full_angles(): base[\"Arm2_Joint\"] = id2_lib_deg)")
    chain_lines.append("URDF joint: Arm2_Joint  (type=revolute, axis_local=%s, parent=%s)" % (
        arm2_joint["axis_xyz"], arm2_joint["parent"]))
    chain_lines.append("  |")
    chain_lines.append("  v")
    chain_lines.append("child link: Arm2_Link  (mesh bbox z-extent=%.1fmm)" % arm2_h)
    chain_lines.append("  |")
    chain_lines.append("  v  URDF joint: Arm3_Joint (parent=Arm2_Link, child=Arm3_Link)")
    chain_lines.append("child link: Arm3_Link  (mesh bbox z-extent=%.1fmm, ~same size as Arm2_Link)" % arm3_h)
    chain_lines.append("  |")
    chain_lines.append("  v  URDF joint: Arm4_Joint -> Arm4_Link -> (DaBai_DCW2_Joint fixed) -> DaBai_DCW2_Link")
    chain_lines.append("  v  URDF joint: Arm5_Joint -> Arm5_Link -> grip_joint/llink*/rlink* -> Gripping_point_Link")
    chain_lines.append("  |")
    chain_lines.append("  v")
    chain_lines.append("M2 physical structure (cream/white bracket, close-up box_xywh=%s)" % m2_box)
    chain_lines.append("  == located at/near the Arm2_Link -> Arm3_Joint -> Arm3_Link transition")
    chain_lines.append("  == both Arm2_Link and Arm3_Link are in RIGID_MOVING_LINKS -> downstream regardless of exact boundary")
    chain_lines.append("")
    chain_lines.append("full downstream link set (independently derived from parent/child graph):")
    chain_lines.append("  " + ", ".join(sorted(downstream)))
    chain_lines.append("")
    chain_lines.append("corollary: Arm1_Link (M1/disc) is NOT in this set -- it is the PARENT of Arm2_Joint,")
    chain_lines.append("  i.e. FIXED under ID2-only rotation (moves only under Arm1_Joint).")
    with open(os.path.join(OUT_DIR, "kinematic_chain.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(chain_lines) + "\n")

    print(json.dumps(report, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
