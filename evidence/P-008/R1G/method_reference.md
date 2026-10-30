# P-008-R1G 방법 근거

설치본은 p620 컨테이너 `bytesize-robotics-camera`의 `/ws/cad-less-robotics/third_party/OrbbecSDK_ROS2`다. `orbbec_camera/package.xml`의 version은 `2.10.6`이다.

| 확인 항목 | 위치 | 관측 |
|---|---|---|
| color/depth alignment | `orbbec_camera/launch/gemini_330_series.launch.py:85` | `depth_registration` 기본값 `false` |
| align mode | 같은 파일 `:281-282` | `align_mode` 기본값 `SW`, target `COLOR`. registration이 꺼져 있으면 적용되지 않는다 |
| depth point cloud frame | `orbbec_camera/src/ob_camera_node.cpp:6301-6303` | registration이 꺼지면 `optical_frame_id_[DEPTH]` |
| point cloud 단위 | 같은 파일 `:6281` | SDK point filter 출력을 1000으로 나눠 미터로 발행 |
| depth 영상 scale | 같은 파일 `:7490-7491` | 발행 전에 `getValueScale()`을 곱한다. 저장 메시지에는 scale 필드가 없다 |
| aligned depth topic | 같은 파일 `:6112` 근처 `publishRawDepthImage` | registration이 켜진 경우에만 unaligned raw를 따로 낸다. 이번 저장 토픽은 `/camera/depth/image_raw` |
| RGB point cloud | launch `:100`, `ob_camera_node.cpp:4833` | `enable_colored_point_cloud` 기본값 `false` |
| color undistortion | launch `:297` | `enable_color_undistortion` 기본값 `false` |
| extrinsic topic | launch `:219`, `ob_camera_node.cpp:5144`, `:5997-5999`, `:8018-8029` | `enable_publish_extrinsic` 기본값 `false`. 켜지면 `/camera/depth_to_color`에 `getExtrinsicTo` 결과를 낸다 |
| static TF | `ob_camera_node.cpp:7974` 이후 `calcAndPublishStaticTransform` | 스트림 profile의 extrinsic으로 static TF를 만든다. capture_01은 이 TF를 저장하지 않았다 |
| CameraInfo | `orbbec_camera/src/utils.cpp:91-131` | K와 왜곡만 채운다. color/depth `P`의 이동 성분은 0이다 |

P-008A 런치 기록은 인자 없는 `ros2 launch orbbec_camera gemini_330_series.launch.py`다. 따라서 위 기본값이 이번 저장본의 configuration evidence다.

공식 `ob::Align`을 저장된 raw 프레임만으로 재현하려면 장치에 있는 calibration이 필요하다. 그 행렬은 capture_01에 없다. `OFFICIAL_ALIGNMENT_COMPARISON = NOT_AVAILABLE_FROM_SAVED_CAPTURE`.
