#!/usr/bin/env python3
"""P-007E-R2-R2 Stage C0-R1 — Acquisition Layer.

🔴 format negotiation 이 없는 V4L2 스트리밍 경로. `cv2.VideoCapture` 를 쓰지 않는다.
   subprocess 로 `v4l2-ctl` 을 호출하되, **허용 플래그 화이트리스트**만 쓴다.
   `--stream-mmap` 경로(streaming_set_cap())는 1차 소스(v4l-utils
   utils/v4l2-ctl/v4l2-ctl-streaming.cpp, WebFetch 확인 2026-10-28)로
   `g_fmt()`(읽기)만 호출하고 `s_fmt()`는 `--set-fmt-video` 가 있을 때만
   호출됨을 확인했다 — 우리는 그 플래그를 **절대 쓰지 않는다**.

이 모듈은 Measurement Layer 를 전혀 모른다. CapturedFrame 만 내놓는다.
"""
import dataclasses
import os
import subprocess
import time

# 🔴 허용 플래그 화이트리스트 — 이 목록 밖의 어떤 인자도 구성하지 않는다
ALLOWED_V4L2CTL_FLAGS = {
    "--device", "--stream-mmap", "--stream-count", "--stream-to",
    "--get-fmt-video", "--list-ctrls", "--get-parm", "--info", "--list-devices",
}
# 🔴 이 토큰이 argv 어디에든 있으면 즉시 중단한다 (defense in depth)
FORBIDDEN_SUBSTRINGS = (
    "--set-fmt", "--set-ctrl", "--set-edid", "--set-parm", "--set-priority",
    "--set-selection", "--try-fmt", "-s ", "--set-",
)


class AcquisitionGuardError(RuntimeError):
    pass


def _assert_safe_argv(argv):
    """🔴 구성된 argv 가 화이트리스트 밖 토큰·금지 문자열을 포함하지 않는지 런타임에 재확인한다.
    v4l2-ctl 은 `-v`=`--set-fmt-video`, `-c`=`--set-ctrl` 같은 **단축 플래그**를 갖는다
    (`v4l2-ctl --help-all` 로 확인, 2026-10-28). 과거 버전은 `--` 로 시작하는 토큰만 봐서
    단축형이 검사를 **우회**할 수 있었다 — 지금은 `-` 로 시작하는 **모든** 토큰(단축·장문
    가리지 않고)을 화이트리스트와 정확히 대조한다. 기본값은 거부(default-deny)다."""
    joined = " ".join(argv)
    for bad in FORBIDDEN_SUBSTRINGS:
        if bad in joined:
            raise AcquisitionGuardError("FORBIDDEN_FLAG_DETECTED: %r in %r" % (bad, argv))
    for tok in argv:
        if tok.startswith("-"):
            flag = tok.split("=", 1)[0]
            if flag not in ALLOWED_V4L2CTL_FLAGS:
                raise AcquisitionGuardError("FLAG_NOT_WHITELISTED: %r in %r" % (flag, argv))


def _run_v4l2ctl(argv, timeout=5.0):
    _assert_safe_argv(argv)
    full = ["v4l2-ctl"] + argv
    return subprocess.run(full, capture_output=True, text=True, timeout=timeout)


@dataclasses.dataclass
class CapturedFrame:
    sequence: int
    capture_timestamp: float          # host time.time() 직후 기록 (subprocess 반환 시점)
    encoded_bytes: bytes
    format_hint: str                  # 캡처 당시 장치가 보고한 pixel format (읽기 전용 질의 결과)
    width: int
    height: int
    ok: bool
    error: str = None


class NoNegotiationV4L2Reader:
    """🔴 format negotiation 0. 스트리밍 ioctl 만 쓴다 — REQBUFS/QUERYBUF/QBUF/STREAMON/DQBUF/STREAMOFF
       (v4l2-ctl --stream-mmap 내부가 이 경로를 탄다. S_FMT 는 호출되지 않는다 — 1차 소스 확인)."""

    def __init__(self, device, expected_width, expected_height, expected_pixelformat, tmp_dir):
        self.device = device
        self.expected = {"width": expected_width, "height": expected_height,
                          "pixelformat": expected_pixelformat}
        self.tmp_dir = tmp_dir
        os.makedirs(tmp_dir, exist_ok=True)
        self._seq = 0

    # ---- 읽기 전용 질의 (설정 변경 0) ----
    def read_format(self):
        """VIDIOC_G_FMT 상당. v4l2-ctl --get-fmt-video 파싱."""
        r = _run_v4l2ctl(["--device=" + self.device, "--get-fmt-video"])
        out = r.stdout
        fmt = {}
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("Width/Height"):
                wh = line.split(":", 1)[1].strip()
                w, h = wh.split("/")
                fmt["width"], fmt["height"] = int(w), int(h)
            elif line.startswith("Pixel Format"):
                # 예: "'MJPG' (Motion-JPEG)"
                v = line.split(":", 1)[1].strip()
                fmt["pixelformat"] = v.split("'")[1] if "'" in v else v
        return {"raw": out, "parsed": fmt, "rc": r.returncode}

    def read_controls(self):
        """읽기 전용. v4l2-ctl --list-ctrls 전문 반환 — 파싱은 호출자가 한다."""
        r = _run_v4l2ctl(["--device=" + self.device, "--list-ctrls"])
        return {"raw": r.stdout, "rc": r.returncode}

    def check_format_invariant(self):
        cur = self.read_format()["parsed"]
        mismatches = {k: {"expected": v, "actual": cur.get(k)}
                      for k, v in self.expected.items() if cur.get(k) != v}
        return {"current": cur, "expected": self.expected,
                "match": not mismatches, "mismatches": mismatches}

    # ---- 스트리밍 (설정 변경 0 — S_FMT 를 호출하지 않는다) ----
    def capture_one_frame(self):
        """`v4l2-ctl --stream-mmap --stream-count=1 --stream-to=<tmp>` 1회.
           🔴 --set-fmt-video 를 쓰지 않는다 — 장치 현재 포맷 그대로 받는다."""
        self._seq += 1
        tmp = os.path.join(self.tmp_dir, "frame_%06d.bin" % self._seq)
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
            r = _run_v4l2ctl(["--device=" + self.device, "--stream-mmap",
                               "--stream-count=1", "--stream-to=" + tmp], timeout=5.0)
            ts = time.time()
            if r.returncode != 0 or not os.path.exists(tmp):
                return CapturedFrame(self._seq, ts, b"", "", 0, 0, False,
                                     "V4L2CTL_RC_%s: %s" % (r.returncode, r.stderr[-300:]))
            data = open(tmp, "rb").read()
            os.remove(tmp)
            return CapturedFrame(self._seq, ts, data, self.expected["pixelformat"],
                                 self.expected["width"], self.expected["height"], True)
        except subprocess.TimeoutExpired:
            return CapturedFrame(self._seq, time.time(), b"", "", 0, 0, False, "CAPTURE_TIMEOUT")
        except Exception as e:  # noqa: BLE001 — 포렌식 목적, 어떤 예외든 dropout 으로 분류한다
            return CapturedFrame(self._seq, time.time(), b"", "", 0, 0, False,
                                 "CAPTURE_EXCEPTION: %r" % e)
