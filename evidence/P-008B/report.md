# P-008B — Provenance-Complete RGB-D Acquisition

Dataset v1은 수정하지 않았다. 역할은 `HISTORICAL_DIAGNOSTIC`이다. 이번 게이트는 촬영과 provenance 검증만 했다. segmentation, PCA, boundary, path, robot 명령은 없다.

판정은 저장 파일을 다시 읽어 재계산한 결과다. `scripts/p008b_readback.py`.

## 1–4. 카메라

드라이버 로그 실측:

```text
Device Orbbec Gemini 335L connected
Serial number: CP263630009D
Firmware version: 1.4.60
ROS Wrapper version: 2.10.6
SDK version: 2.10.6
```

기대 reference와 같다. `CAMERA_IDENTITY = VERIFIED`.

## 5. Acquisition contract

`evidence/P-008B/acquisition_contract.json`

촬영 전에 고정했고, 결과를 본 뒤 바꾸지 않았다.

## 6–7. Color / Depth

드라이버 로그:

```text
color Frame - Width: 1280 Height: 720 fps: 30 Format: MJPG
depth Frame - Width: 1280 Height: 720 fps: 30 Format: Y16
```

ROS 저장본은 color `rgb8`, depth `16UC1`이다. 프레임은 둘 다 `camera_color_optical_frame`이다.

계약의 native depth `848×480`은 P-008A의 비정렬 기본값이다. 이번 세션은 unaligned topic을 구독하지 않았고, 드라이버가 찍은 depth frame은 정렬 후 `1280×720`이다.

## 8–9. Alignment

사용 모드: software depth→color. launch `align_mode:=SW`, `align_target_stream:=COLOR`, `depth_registration:=true`.

로그 `set align mode to SW`가 있다. launch 인자만으로 통과시키지 않았다. 발행 depth는 `1280×720`, frame은 `camera_color_optical_frame`이다. depth CameraInfo의 K는 color K와 같다.

```text
fx 610.5073852539062
fy 610.349853515625
cx 641.7918090820312
cy 364.4433288574219
```

depth distortion은 `plumb_bob`이고 계수는 0이다. color distortion은 `rational_polynomial`이고 0이 아니다. hardware D2C는 쓰지 않았다.

`Enable frame sync` 로그는 있다. 이것을 hardware sync로 보지 않는다. `HARDWARE_SYNC = NOT_VERIFIED`. ROS stamp 결합만 기록한다. 다섯 장의 color–depth 차이는 18–20 µs이고, depth–cloud 차이는 0이다. 결합 한도는 0.05 s다.

## 10–12. Extrinsic

이번 세션에서 읽었다. 다른 세션 값을 복사하지 않았다.

- source: `tf2 lookup_transform`
- target / to: `camera_color_optical_frame`
- source / from: `camera_depth_optical_frame`
- TF stamp: 0 (static)
- 세션 로그 시각: `1793367535.718` (`Serial number` 줄)

translation m:

```text
-0.023803766630847746
-0.00019296251462793627
 0.00017839526009778665
```

quaternion xyzw:

```text
-0.00023462735471307952
-0.0013354254224321462
 0.004136464569787003
 0.9999905255800219
```

행렬은 `preflight/color_depth_extrinsic.json`에 있다. 단위행렬 fallback이 아니다. `/camera/depth_to_color`도 같은 세션에서 읽었고 translation은 위 값과 1e-9 m 안에서 같다. 그 메시지 header frame은 `depth_to_color_extrinsics`라서 프레임 의미는 optical TF를 기준으로 했다.

다섯 capture의 `extrinsic_reference.json`은 이 preflight 값과 바이트가 같다. 캡처 사이에 다시 샘플하지 않았다.

## 13–15. Point cloud

topic `/camera/depth_registered/points`

fields `x y z rgb`, float32, little-endian. `point_step` 20, `rgb` offset 16. frame `camera_color_optical_frame`. 단위는 미터다. 드라이버가 xyz를 1000으로 나눈다.

color field가 메시지에 실제로 있다. 저장 PCD header도 `FIELDS x y z rgb`다. `COLORED_POINTCLOUD = AVAILABLE`.

## 16–20. Preflight depth↔cloud

기준은 R1G와 같다. 양수 비율은 in-bounds 투영 점 가운데 depth가 양수인 비율이고, p95는 3D 거리다. 결과를 보고 바꾸지 않았다.

```text
sample_count          756742
positive_depth_ratio  1.0
mean                  2.8573905798977176e-08 m
median                2.5281561956768074e-08 m
p95                   6.245397937012279e-08 m
max                   1.538630431765747e-07 m
```

이미지 전체에서 depth가 0보다 큰 픽셀 비율은 약 0.82다. 이것은 게이트 기준이 아니다. `preflight/capture_loop.json`의 `positive_depth_ratio`가 그 픽셀 비율이다.

3D 오차가 1e-7 m 근처인 것은 같은 frameset에서 만든 depth와 cloud의 float 왕복이다. 물리 등록 정확도로 읽지 않는다.

## 21. Color correspondence

공식 colored cloud의 `rgb`와, color K로 투영한 픽셀을 비교했다.

```text
in_bounds_ratio     1.0
median_abs_channel  0.0
```

고정 기준(in_bounds ≥ 0.999, median ≤ 15)은 통과한다. checkerboard가 없으므로 `REGISTRATION_ACCURACY = NOT_CLAIMED`. 새 calibration은 하지 않았다.

## 22–27. 다섯 장

preflight가 모두 통과한 뒤에만 저장했다. capture_01은 preflight에 쓴 그 frameset이다. capture_02–05는 그 다음 stamp다. 다섯 파일 해시는 서로 다르다.

| capture | color stamp | depth–color | points | p95 m | pixel depth>0 |
| --- | --- | --- | --- | --- | --- |
| capture_01 | 1793367536.606351 | 20.0 µs | 756742 | 6.245e-08 | 0.8211 |
| capture_02 | 1793367536.706443 | 18.8 µs | 754900 | 6.263e-08 | 0.8191 |
| capture_03 | 1793367538.341757 | 18.8 µs | 757495 | 6.267e-08 | 0.8219 |
| capture_04 | 1793367539.810164 | 18.1 µs | 756329 | 6.252e-08 | 0.8207 |
| capture_05 | 1793367543.514476 | 17.9 µs | 757058 | 6.245e-08 | 0.8215 |

depth–cloud stamp 차이는 다섯 장 모두 0이다. color–cloud 차이는 color–depth와 같다.

SHA256은 `readback.json`에 있다. RGB, depth, cloud 각각 5개가 서로 다르다.

저장 파일을 다시 읽어 resolution, encoding, fields, frame, 유한 점 수를 확인했다. depth↔cloud와 color 대응을 같은 기준으로 다시 계산했다. 5/5 통과. live JSON과 p95가 같다. mean은 float32를 다시 읽으며 마지막 자리만 다르다.

alignment frame, resolution, CameraInfo K는 다섯 장이 같다.

신발 FOV는 RGB를 눈으로 봤다. segmentation은 하지 않았다. 다섯 장 모두 앞코, 뒤꿈치, 좌우 경계가 화면 안에 있고 실루엣이 잘리지 않는다. `shoe_fov.json`.

## 28. Manifest

`evidence/P-008B/manifest.sha256`. 핵심 artifact를 포함한다. 목록을 만든 뒤 같은 파일을 다시 해시했고 목록과 일치한다. `MANIFEST_RECOMPUTE = PASS`.

## 분할

```text
capture_01 = DEVELOPMENT / REFERENCE
capture_02 capture_03 capture_04 capture_05 = HOLDOUT
```

이 분할은 여기서 고정한다.

## 하지 않은 것

segmentation, RGB threshold, ROI, ML, PCA, hull, boundary, normal, ProcessPath, MoveIt, robot, servo. capture_01 기하는 FOV 확인 외에는 보지 않았다. capture_02–05로 파라미터를 맞추지 않았다. P-008A와 R1D–R1G evidence는 바꾸지 않았다.

R1H, R1 retry, R2, P-009는 열지 않는다.
