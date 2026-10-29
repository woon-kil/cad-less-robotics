# P-007F-07 Phase A — §9/§10 Timestamp Design

## §9 — Per-frame timestamp record

`scripts/p007f_07_continuous_capture.py.record_continuous_window()`가 매 frame마다 기록:

```text
frame_index
host_monotonic_ns      <- time.monotonic_ns(), 저장/분류의 기준 clock
device_color_stamp     <- ROS2 메시지 헤더의 device timestamp(참고용)
device_depth_stamp
elapsed_s_since_start
rgb_filename
depth_filename
```

절대 wall-clock(`time.time()`)만을 기준으로 삼지 않는다 — `host_monotonic_ns`가
1차 기준이며, device timestamp는 "참고용"으로만 별도 보존한다(§9 요구 그대로).

## §10 — Command event timestamp 설계

`CommandEventRecorder`(`p007f_07_continuous_capture.py`)가 Phase B의 유일한 write 호출
직전/직후를 같은 clock domain(`host_monotonic_ns`)에 기록하도록 설계되어 있다:

```text
command_prepare_timestamp_ns      (recorder.record 호출 시점, capture 루프 안)
command_call_enter_timestamp_ns   (scripts/p007f_07_jetson_controller.py.do_write() 내부)
command_call_exit_timestamp_ns    (do_write() 내부, write 호출 직후)
```

`do_write()`는 Jetson에서 실행되고 `time.monotonic_ns()`를 반환 — 이 값 자체는 **Jetson의
monotonic clock**이고, capture 쪽(`p620`)의 `host_monotonic_ns`와는 **다른 machine의 서로
다른 monotonic clock**이다(§11에서 다룸 — 단순 비교 금지).

## Phase A에서 실제로 검증한 것

실제 하드웨어 없이 `time.monotonic_ns()`의 단조증가성만 synthetic으로 검증 가능하다(아래
구조만 명시, Phase A 코드 자체는 실제 clock 호출 테스트를 포함하지 않음 — `monotonic_ns()`는
Python 표준 라이브러리 보증사항이므로 재검증하지 않는다). 실제 두 machine 간 clock 비교는
§11에서 다룬다.
