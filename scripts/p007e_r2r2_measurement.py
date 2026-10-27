#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C0-R1 — Measurement Layer.

🔴 acquisition 메커니즘을 전혀 모른다. decode → ROI/mask → ORB → matching →
   RANSAC → metrics 만 한다. 입력은 바이트열(인코딩된 프레임)과 ROI 명세뿐이다.

🔴 알고리즘·파라미터는 Stage C0(121dc43)에서 동결된 값 그대로다 — 이번 repair 로
   바꾸지 않았다 (작업의뢰서 §11). 수정한 것은 단 하나, roi_mean 버그(§12)뿐이다:
   C1 harness(p007e_r2r2_c1_sham.py:235)는 `g.mean()`(전체 프레임 평균)을 썼다.
   이 모듈은 마스킹된 영역의 평균만 쓴다.
"""


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


def roi_mean_brightness(gray, mask):
    """🔴 수정됨(M4 버그 수정) — ROI(마스킹된 영역)만의 평균. 전체 프레임 평균이 아니다."""
    import numpy as np
    sel = gray[mask > 0]
    if sel.size == 0:
        return None
    return float(np.mean(sel))


def make_estimator(rules, mask):
    """🔴 C0 동결값 그대로 — ORB(3000,1.2,8,12) + BFMatcher(Hamming,crossCheck) +
       estimateAffinePartial2D(RANSAC, 1.5px, 5000, 0.995). 변경 0."""
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


def decode_gray(encoded_bytes):
    """🔴 malformed frame 은 예외를 던지지 않고 None 을 반환한다 (T4 음성시험)."""
    import cv2
    import numpy as np
    try:
        arr = np.frombuffer(encoded_bytes, np.uint8)
        if arr.size == 0:
            return None
        img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        return img
    except Exception:
        return None
