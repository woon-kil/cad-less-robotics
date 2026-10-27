#!/usr/bin/env python3
"""P-007B 전방 기구학 — 벤더 convention H0로 라이브러리 각도를 URDF 포즈로 옮긴다.

사용: python3 scripts/p007b_fk.py <urdf_joints.json> [--self-check]

이 계산은 candidate 포즈다. measured TCP도, calibrated TCP도 아니다.
H0: q_urdf_rad = (q_library_deg - 90) * pi/180  (evidence/P-006B/resume-04/convention_decision.json)

--self-check 는 P-007A가 남긴 candidate_fk.json 값을 재현하는지 검사한다.
재현하지 못하면 이 도구로 방향 예측을 하지 않는다.
"""
import json, math, sys

CHAIN = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "Gripping_point_Joint"]
# P-007A candidate_fk.json 의 링크 좌표 (library [86,150,0,18,93])
P007A_REF = {
    "Arm1_Link": [0.0, 0.0, 0.0895],
    "Arm2_Link": [0.0, 0.0, 0.1224999999999998],
    "Arm3_Link": [-0.005553701498180669, -0.07153705875047323, 0.16392499999999802],
}


def rot(axis, q):
    """축-각 회전행렬. axis는 단위벡터 (URDF axis는 단위축이다)."""
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def rpy(r, p, y):
    return matmul(rot((0, 0, 1), y), matmul(rot((0, 1, 0), p), rot((1, 0, 0), r)))


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def h0(lib_deg):
    """벤더 convention. 보정값이 아니다."""
    return (lib_deg - 90.0) * math.pi / 180.0


def fk(joints, lib_angles):
    """lib_angles: {관절명: library degree}. 고정 관절은 각도 0으로 둔다."""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    out = []
    for name in CHAIN:
        j = joints[name]
        oxyz = [float(x) for x in j["origin_xyz"].split()]
        orpy = [float(x) for x in (j["origin_rpy"] or "0 0 0").split()]
        p = [p[i] + apply(R, oxyz)[i] for i in range(3)]
        R = matmul(R, rpy(*orpy))
        if j["type"] == "revolute" and name in lib_angles:
            q = h0(lib_angles[name])
            R = matmul(R, rot([float(x) for x in j["axis_xyz"].split()], q))
        out.append({"link": j["child"], "joint": name, "xyz_m": p[:]})
    return out


def main():
    data = json.load(open(sys.argv[1], encoding="utf-8"))
    joints = {j["name"]: j for j in data["joints"]}

    if "--self-check" in sys.argv:
        got = {r["link"]: r["xyz_m"] for r in fk(joints, dict(zip(CHAIN[:5], [86, 150, 0, 18, 93])))}
        worst = 0.0
        for link, ref in P007A_REF.items():
            d = max(abs(got[link][i] - ref[i]) for i in range(3))
            worst = max(worst, d)
            print("%-12s 기대 %s" % (link, ["%.9f" % v for v in ref]))
            print("%-12s 계산 %s  최대오차 %.3e m" % ("", ["%.9f" % v for v in got[link]], d))
        print("\nself-check %s (허용 1e-9 m, 최대 %.3e m)" % ("PASS" if worst < 1e-9 else "FAIL", worst))
        sys.exit(0 if worst < 1e-9 else 1)

    libs = [float(x) for x in sys.argv[2:7]] if len(sys.argv) > 6 else [86, 150, 0, 18, 93]
    print(json.dumps({"input_library_deg": libs,
                      "convention": "q_urdf = (q_library_deg - 90) * pi/180",
                      "convention_status": "VENDOR_DEFINED_CONVENTION",
                      "not": ["measured TCP", "calibrated TCP"],
                      "urdf_sha256": data["sha256"],
                      "links": fk(joints, dict(zip(CHAIN[:5], libs)))},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
