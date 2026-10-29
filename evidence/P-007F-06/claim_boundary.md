# P-007F-06 — §20/§25 Claim Boundary

## §25 — Relationship to P-007F-05 (unchanged)

```text
P-007F-05 PREREGISTERED RESULT (unchanged, historical):
CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION

P-007F-06 POST-HOC FORENSIC RESULT (this gate):
POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED (F2)
```

P-007F-05's preregistered classification is **not** retroactively changed by this analysis. The
two results are reported side by side, as separate gates with separate methodologies and separate
(non-preregistered vs. preregistered) status.

## Maximum claim this gate supports

> Offline forensic re-analysis of the stored P-007F-05 PRE/POST Gemini RGB frames, using five
> detection methods beyond the preregistered ORB/RANSAC primary (dense optical flow, raw
> registered absolute difference, Canny edge centroid shift, local phase correlation, and ORB
> itself re-applied per-ROI), found that the **ARM2_ROI signal exceeds its own within-window
> (PRE-PRE / POST-POST) noise floor on 3 of 6 independent metrics, while the STATIC_REFERENCE_ROI
> background does not exceed its noise floor on any metric** -- indicating a real, non-global,
> statistically-detectable visual change localized to the robot-occupied region of the frame.
> However, this change is **not confined to ARM2_ROI alone**: most other robot-geometry ROIs
> (ROBOT_ROI, ARM3_ROI, ARM4_ROI, DABAI_ROI, END_EFFECTOR_ROI) and one adjacent non-robot ROI
> (DOWNSTREAM_ROI) also exceed their own noise floors, while only ARM1_ROI and
> STATIC_REFERENCE_ROI do not. The evidence therefore supports "a real, non-background,
> statistically significant visual change exists in the robot-occupied image region" but does
> **not** establish that this change is specifically rigid single-link Arm2 motion as opposed to
> a diffuse effect (vibration/backlash propagation, shadow/lighting correlated with robot
> position, or imprecision in the unmeasured approximate ROIs).

## Forbidden claims (still forbidden, unchanged from P-007F-05)

```text
Physical Arm2 angle = 151° or 152°
Physical Arm2 moved exactly -3° (or any exact degree value)
Controller readback = shaft encoder angle
Servo positioning accuracy verified
Absolute robot positioning verified
```

## Additionally forbidden by this gate specifically

```text
"Forensic analysis proves the robot physically moved"          -- F2, not F1; not established
"The primary P-007F-03/05 detector was simply wrong"            -- it used a different, narrower
                                                                      ROI/metric combination; this
                                                                      gate does not establish ORB
                                                                      was in error, only that other
                                                                      metrics detect something ORB
                                                                      did not flag as significant
"Operator's YES observation is confirmed by this analysis"      -- UNKNOWN per
                                                                      operator_observation_review.md
"Operator's YES observation is refuted by this analysis"        -- same, UNKNOWN
"Depth confirms a robot-region displacement of N mm"            -- DEPTH_SPATIAL_LOCALIZATION_LIMITED
                                                                      (rgb_depth_alignment.json);
                                                                      no depth-based distance claim
                                                                      is made
```

## What this gate does support, stated carefully

A statistically real (noise-exceeding), spatially non-global (background-excluded) visual change
exists in the stored P-007F-05 evidence, broadly distributed across the robot-occupied image
region. Its physical cause (rigid Arm2 rotation, backlash/vibration propagation through the
downstream chain, a lighting/shadow effect correlated with robot position, or ROI-boundary
imprecision in the unmeasured approximate ROIs) is **not established** by this gate.
