# P-003 Gemini 335L PointCloud2

SOURCE_BASE_COMMIT=8c070d08bba3f4b43ae0425ad50b76f3acf91121
TESTED_COMMIT=NONE
EVIDENCE_GENERATED_AT=2026-10-26 15:13 KST
FINAL_COMMIT=PENDING

Status: PASS

공식 호스트는 p620 Docker다. 컨테이너는 `bytesize-robotics-camera`, 이미지는 `cad-less-robotics:humble-p003`다.
P-000 컨테이너 `bytesize-robotics-humble`는 privileged 없이 그대로 두고, USB 장치를 붙이지 않았다.

재현:

```bash
bash scripts/validate_gemini_pointcloud.sh
```

이 스크립트는 `2bc5:0804`의 usbfs 노드만 `--device`로 넘긴다. `--privileged`와 `-v /dev:/dev`는 쓰지 않는다.

## Stream

토픽 `/camera/depth/points`를 구독 QoS RELIABLE로 34.865초 측정했다.
메시지 1047, 평균 30.002 Hz, 끊김(간격 2초 초과) 0, frame_id 변경 0, stamp 역행 0.
frame_id는 `camera_depth_optical_frame`.
첫 메시지는 height 1, width 262790, point_step 16, xyz float32, is_dense true.
창 전체 finite XYZ 합은 275746263이고 비율은 1.0이다. 드라이버가 유효 점만 담은 비정렬 클라우드를 보낸다.
depth 이미지 자체는 launch 로그 기준 848x480 Y16 30 fps다. 그 해상도를 PointCloud2 width로 적지 않는다.

`camera_node` relay는 파라미터 `input_topic:=/camera/depth/points`로 `/vision/camera/points`를 한 프레임 받았다.
width 263779, 같은 frame_id. path_extractor는 실행하지 않았다.

## Firmware

읽은 버전은 1.4.60이다. wrapper release v2.10.6이 Gemini 335L에 적은 권장값은 1.8.10이다.
스트림이 측정 기준을 넘겨서 펌웨어는 쓰지 않았다.

## TF

Orbbec 내부 프레임만 있다. camera→robot TF는 만들지 않았다. DOFBOT은 연결하지 않았고 execute는 0이다.
