#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C0 — offline ROI/algorithm analysis + synthetic unit validation.
🔴 카메라를 열지 않는다. Arm_Lib·I2C·ROS 를 import 하지 않는다. Stage B 프레임 1장만 읽는다."""
import json, sys, hashlib
import numpy as np
import cv2

IMG = sys.argv[1]
SPEC = sys.argv[2]
OUT = sys.argv[3]

spec = json.load(open(SPEC))
raw = open(IMG, 'rb').read()
img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
H, W = img.shape
rec = {"cv2_version": cv2.__version__, "image_sha256": hashlib.sha256(raw).hexdigest(),
       "image_shape": [W, H]}

mask = np.zeros((H, W), np.uint8)
for r in spec["include"]:
    x0, y0, x1, y1 = r["rect"]; mask[y0:y1, x0:x1] = 255
for r in spec["exclude"]:
    x0, y0, x1, y1 = r["rect"]; mask[y0:y1, x0:x1] = 0
rec["mask_pixels"] = int((mask > 0).sum())
rec["mask_fraction_pct"] = round(100.0 * (mask > 0).sum() / (W * H), 2)

ORB_N = 3000
orb = cv2.ORB_create(nfeatures=ORB_N, scaleFactor=1.2, nlevels=8, fastThreshold=12)
kp_full = orb.detect(img, None)
kp_roi, des_roi = orb.detectAndCompute(img, mask)
rec["orb_params"] = {"nfeatures": ORB_N, "scaleFactor": 1.2, "nlevels": 8, "fastThreshold": 12}
rec["features_full_frame"] = len(kp_full)
rec["features_in_roi"] = len(kp_roi)

def est(imgA, imgB):
    """ORB + BFMatcher(Hamming, crossCheck) + estimateAffinePartial2D(RANSAC)."""
    kA, dA = orb.detectAndCompute(imgA, mask)
    kB, dB = orb.detectAndCompute(imgB, mask)
    if dA is None or dB is None or len(kA) < 8 or len(kB) < 8:
        return {"ok": False, "reason": "FEATURES"}
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    m = bf.match(dA, dB)
    if len(m) < 8:
        return {"ok": False, "reason": "MATCHES", "n_feat_a": len(kA), "n_feat_b": len(kB), "n_match": len(m)}
    pA = np.float32([kA[x.queryIdx].pt for x in m]).reshape(-1, 1, 2)
    pB = np.float32([kB[x.trainIdx].pt for x in m]).reshape(-1, 1, 2)
    Mx, inl = cv2.estimateAffinePartial2D(pA, pB, method=cv2.RANSAC,
                                          ransacReprojThreshold=1.5, maxIters=5000, confidence=0.995)
    if Mx is None:
        return {"ok": False, "reason": "NO_MODEL", "n_match": len(m)}
    inl = inl.ravel().astype(bool)
    tx, ty = float(Mx[0, 2]), float(Mx[1, 2])
    rot = float(np.degrees(np.arctan2(Mx[1, 0], Mx[0, 0])))
    scale = float(np.hypot(Mx[0, 0], Mx[1, 0]))
    proj = (Mx[:, :2] @ pA.reshape(-1, 2).T).T + Mx[:, 2]
    err = np.linalg.norm(proj - pB.reshape(-1, 2), axis=1)
    return {"ok": True, "n_feat_a": len(kA), "n_feat_b": len(kB), "n_match": len(m),
            "inliers": int(inl.sum()), "inlier_ratio": round(float(inl.mean()), 4),
            "tx": round(tx, 4), "ty": round(ty, 4), "rot_deg": round(rot, 5),
            "scale": round(scale, 6),
            "residual_rms_inliers": round(float(np.sqrt((err[inl] ** 2).mean())), 4),
            "residual_median_inliers": round(float(np.median(err[inl])), 4)}

def warp_matrix(dx, dy, deg):
    c = (W / 2.0, H / 2.0)
    R = cv2.getRotationMatrix2D(c, deg, 1.0)
    R[0, 2] += dx; R[1, 2] += dy
    return R

def warp(im, R):
    return cv2.warpAffine(im, R, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)

# [1] identity — 같은 이미지 두 장
rec["identity_pair"] = est(img, img)

# [2] synthetic sweep — 알려진 변환 회수 시험
cases = [(0.2,0,0),(0.5,0,0),(1,0,0),(2,0,0),(5,0,0),(0,0.5,0),(0,2,0),(0,0,0.05),
         (0,0,0.1),(0,0,0.5),(0,0,1.0),(1,1,0.1),(3,-2,0.3),(10,6,1.0)]
sweep=[]
for dx,dy,deg in cases:
    R = warp_matrix(dx, dy, deg)
    r = est(img, warp(img, R))
    # 기대값은 estimator 와 **같은 식**으로 warp 행렬에서 뽑는다 (부호 규약 추측 금지)
    exp = {"tx": round(float(R[0,2]),4), "ty": round(float(R[1,2]),4),
           "rot_deg": round(float(np.degrees(np.arctan2(R[1,0], R[0,0]))),5)}
    row={"applied":{"dx":dx,"dy":dy,"rot_deg_input":deg}, "expected":exp, **r}
    if r.get("ok"):
        row["err_tx"]=round(r["tx"]-exp["tx"],4)
        row["err_ty"]=round(r["ty"]-exp["ty"],4)
        row["err_rot_deg"]=round(r["rot_deg"]-exp["rot_deg"],5)
        row["err_trans_norm"]=round(float(np.hypot(row["err_tx"], row["err_ty"])),4)
    sweep.append(row)
rec["synthetic_sweep"]=sweep

# [3] 광도 변화 내성 — 밝기/대비 섭동만 가한 동일 장면
photo=[]
for g,b in [(1.0,5),(1.0,15),(1.05,0),(0.95,0),(1.1,10)]:
    pert=np.clip(img.astype(np.float32)*g+b,0,255).astype(np.uint8)
    r=est(img,pert); photo.append({"gain":g,"bias":b, **r})
rec["photometric_perturbation"]=photo

json.dump(rec, open(OUT,'w'), ensure_ascii=False, indent=1)
print(json.dumps({k:rec[k] for k in ["cv2_version","image_sha256","image_shape","mask_pixels",
      "mask_fraction_pct","features_full_frame","features_in_roi"]}, ensure_ascii=False, indent=1))
print("identity:", json.dumps(rec["identity_pair"], ensure_ascii=False))
