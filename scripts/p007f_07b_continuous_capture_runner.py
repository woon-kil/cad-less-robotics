#!/usr/bin/env python3
"""P-007F-07B Phase B4 -- CLI entrypoint for the real continuous-capture run.

이 파일은 p620 위(ROS2 Humble 컨테이너 내부)에서 직접 실행된다.
scripts/p007f_07_continuous_capture.py(Phase A, 설계만 되어있고 미실행이던
파일, 수정 없이 SHA-pin import)의 record_continuous_window() 를 호출만
한다 -- trigger_command_fn 은 주지 않는다(이번 gate는 완전 정지 상태 촬영,
어떤 command 도 발사하지 않음).

사용: python3 scripts/p007f_07b_continuous_capture_runner.py <out_dir> <pre_seconds>
"""
import importlib.util
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CC_PATH = os.path.join(ROOT, "scripts", "p007f_07_continuous_capture.py")


def main():
    if len(sys.argv) < 3:
        raise SystemExit("사용: p007f_07b_continuous_capture_runner.py <out_dir> <pre_seconds>")
    out_dir = sys.argv[1]
    pre_seconds = float(sys.argv[2])

    spec = importlib.util.spec_from_file_location("p007f_07_continuous_capture_pin", CC_PATH)
    cc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cc)

    result = cc.record_continuous_window(out_dir, pre_seconds=pre_seconds, post_seconds=0.0,
                                         target_fps=cc.TARGET_FPS_HINT,
                                         command_event_recorder=None, trigger_command_fn=None)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
