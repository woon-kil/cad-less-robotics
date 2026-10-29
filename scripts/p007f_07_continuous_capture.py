#!/usr/bin/env python3
"""P-007F-07 Phase A — §8/§9 continuous Gemini RGB+Depth recording pipeline
(design/implementation, NOT executed against live hardware in Phase A).

scripts/p007f_03_capture_static.py(SHA-pin, 수정 없음)의 Grab 노드/image_array/
write_png 를 그대로 재사용하고, 여기서는 "N프레임 한 번에 받기" 대신
"T-3s ~ T+5s 연속 기록 + command event timestamp 삽입" 으로 확장한다.

🔴 이 파일은 rclpy 를 import 한다 -- Phase A 의 self_ast_check 대상인
scripts/p007f_07_execution.py 와 달리, 이 파일은 "camera wiring" 계층이고
(p007e_r2r5_hardware_wiring.py/p007f_05_hardware_wiring.py 선례와 동일 원칙),
실제 실행 자체가 이 gate에서 일어나지 않는다(코드만 준비, §20 "camera startup"
은 unattended 단계의 설계일 뿐 이번 세션에서 실행하지 않음).
"""
import json
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

PRE_SECONDS_MIN = 3.0
POST_SECONDS_MIN = 3.0
TARGET_FPS_HINT = 10  # Gemini native 30fps -- 저장 안정성을 위해 다운샘플 권장치(§8)


def qos(reliable):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def write_png(path, image):
    """scripts/p007f_03_capture_static.py.write_png() 와 동일 알고리즘(재구현,
    SHA-pin import 대신 복제한 이유: 그 파일은 main()이 CLI 진입점이라 함수
    단위 재사용이 어색함 -- 동작은 1:1 동일, 바이트 단위 비교로 검증 가능)."""
    height, width = image.shape[:2]
    if image.ndim == 2:
        if image.dtype == np.uint16:
            color_type, bit_depth, rows = 0, 16, image.astype(">u2")
        else:
            color_type, bit_depth, rows = 0, 8, image
    else:
        color_type, bit_depth, rows = 2, 8, image
    raw = b"".join(b"\x00" + np.ascontiguousarray(rows[row]).tobytes() for row in range(height))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    path.write_bytes(payload)


class ContinuousGrab(Node):
    """§9 -- 각 frame 마다 host monotonic timestamp 를 함께 기록한다."""

    def __init__(self, on_frame_fn):
        super().__init__("p007f07_continuous_grab")
        self.color = None
        self.depth = None
        self.on_frame_fn = on_frame_fn
        for reliable in (True, False):
            self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos(reliable))
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos(reliable))

    def on_color(self, message):
        if message.width > 0 and message.height > 0:
            self.color = message
            self._maybe_emit()

    def on_depth(self, message):
        if message.width > 0 and message.height > 0:
            self.depth = message

    def _maybe_emit(self):
        if self.color is None or self.depth is None:
            return
        host_ts_ns = time.monotonic_ns()
        self.on_frame_fn(self.color, self.depth, host_ts_ns)


def image_array(message):
    if message.encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
        if message.encoding == "bgr8":
            array = array[:, :, ::-1]
        return np.ascontiguousarray(array)
    if message.encoding == "16UC1":
        return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
    raise SystemExit("unsupported encoding: %s" % message.encoding)


class CommandEventRecorder:
    """§10 -- Phase B 의 유일한 write 직전/직후에 기록될 monotonic timestamp 를
    이 recorder 가 받아 영상 timestamp 와 같은 clock_domain(host monotonic)에
    기록한다. Phase A 에서는 실제 command 가 없으므로 이 클래스는 설계/테스트
    대상일 뿐 실행되지 않는다."""

    def __init__(self):
        self.events = []

    def record(self, label, timestamp_ns=None):
        self.events.append({"label": label, "timestamp_ns": timestamp_ns or time.monotonic_ns()})

    def to_json(self):
        return {"clock_domain": "host_monotonic_ns", "events": list(self.events)}


def record_continuous_window(out_dir, pre_seconds=PRE_SECONDS_MIN, post_seconds=POST_SECONDS_MIN,
                             target_fps=TARGET_FPS_HINT, command_event_recorder=None,
                             trigger_command_fn=None):
    """§8 -- 연속 기록 메인 루프 설계.

    `trigger_command_fn`: PRE 구간이 끝나면 호출되는 콜백(Phase B에서 실제
    1회 write 를 수행하는 함수가 주입된다 -- 이 모듈 자체는 그 함수를 정의하지
    않는다, scripts/p007f_07_jetson_controller.py.do_write() 가 유일한 정의처).
    Phase A 에서는 이 함수를 전혀 호출하지 않는다(코드 경로 준비만).
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    recorder = command_event_recorder or CommandEventRecorder()
    frames_meta = []
    min_interval_s = 1.0 / target_fps

    rclpy.init()
    state = {"last_save": 0.0, "index": 0, "command_fired": False, "start_ns": None}

    def on_frame(color_msg, depth_msg, host_ts_ns):
        if state["start_ns"] is None:
            state["start_ns"] = host_ts_ns
        elapsed_s = (host_ts_ns - state["start_ns"]) / 1e9
        if elapsed_s - state["last_save"] < min_interval_s:
            return
        state["last_save"] = elapsed_s
        idx = state["index"]
        state["index"] += 1
        color = image_array(color_msg)
        depth = image_array(depth_msg)
        write_png(out / ("%06d-color.png" % idx), color)
        write_png(out / ("%06d-depth.png" % idx), depth)
        frames_meta.append({
            "frame_index": idx, "host_monotonic_ns": host_ts_ns,
            "device_color_stamp": stamp_sec(color_msg.header.stamp),
            "device_depth_stamp": stamp_sec(depth_msg.header.stamp),
            "elapsed_s_since_start": elapsed_s,
            "rgb_filename": "%06d-color.png" % idx, "depth_filename": "%06d-depth.png" % idx,
        })
        if not state["command_fired"] and elapsed_s >= pre_seconds and trigger_command_fn is not None:
            state["command_fired"] = True
            recorder.record("command_prepare_timestamp")
            trigger_command_fn()
            recorder.record("command_call_exit_observed_by_recorder")

    node = ContinuousGrab(on_frame)
    deadline = time.monotonic() + pre_seconds + post_seconds + 10.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if state["start_ns"] is not None:
            elapsed = (time.monotonic_ns() - state["start_ns"]) / 1e9
            if elapsed >= pre_seconds + post_seconds:
                break
    node.destroy_node()
    rclpy.shutdown()

    (out / "frames_meta.json").write_text(json.dumps(frames_meta, indent=1))
    (out / "command_events.json").write_text(json.dumps(recorder.to_json(), indent=1))
    return {"frame_count": len(frames_meta), "out_dir": str(out)}
