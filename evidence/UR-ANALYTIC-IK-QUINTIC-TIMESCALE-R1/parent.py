#!/usr/bin/env python3
"""구간별 5차 시간 스케일링 단일 실행의 주소 공간과 전체 시간 상한."""

import resource
import subprocess
import sys

LIMIT_BYTES = 8589934592
OVERALL_S = 300


def main() -> int:
    if len(sys.argv) < 2:
        print("EXECUTION_ERROR parent_arguments", file=sys.stderr)
        return 4
    resource.setrlimit(resource.RLIMIT_AS, (LIMIT_BYTES, LIMIT_BYTES))
    try:
        completed = subprocess.run(sys.argv[1:], timeout=OVERALL_S, check=False)
    except subprocess.TimeoutExpired:
        print("WALL_KILL overall", file=sys.stderr)
        return 3
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
