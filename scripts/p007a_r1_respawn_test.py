#!/usr/bin/env python3
"""P-007A-R1 respawn 관찰. Mac에서 돈다. 읽기만 한다.

사용: python3 scripts/p007a_r1_respawn_test.py [관찰초=72]

격리 직후 CONTROL_WRITER와 Jupyter 표면이 되살아나는지 본다. 60초 이상 관찰한다.
pgrep 패턴은 [j]oystick 식으로 적어 관찰 명령 자신이 잡히지 않게 한다.
"""
import json, os, re, subprocess, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "evidence", "P-007A-R1")
REMOTE = os.environ.get("P007A_REMOTE", "jetson@192.168.0.94")
INTERVAL = 3

REMOTE_LOOP = r"""
for i in $(seq 1 %(n)d); do
  echo "T|$(date -Is)"
  pgrep -af '[j]oystick\.py|[Y]ahboomArm|[i]pykernel_launcher|[s]tart_app\.sh' | sed 's/^/P|/' || echo "P|none"
  ss -ltn 2>/dev/null | grep -E ':(6500|8888)\b' | sed 's/^/L|/' || echo "L|none"
  systemctl is-active jupyterlab 2>/dev/null | sed 's/^/J|/'
  echo "---"
  sleep %(iv)d
done
"""


def main():
    seconds = int(sys.argv[1]) if len(sys.argv) > 1 else 72
    n = max(1, seconds // INTERVAL)
    cmd = REMOTE_LOOP % {"n": n, "iv": INTERVAL}
    print("respawn 관찰 %d초 (%d회 x %ds)" % (n * INTERVAL, n, INTERVAL))
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", REMOTE, cmd],
                       capture_output=True, text=True, timeout=n * INTERVAL + 60)
    polls, cur = [], None
    for line in p.stdout.splitlines():
        if line.startswith("T|"):
            cur = {"at": line[2:], "processes": [], "listeners": [], "jupyterlab": ""}
        elif line == "---" and cur:
            polls.append(cur); cur = None
        elif cur is None:
            continue
        elif line.startswith("P|") and line != "P|none":
            cur["processes"].append(line[2:])
        elif line.startswith("L|") and line != "L|none":
            cur["listeners"].append(" ".join(line[2:].split()))
        elif line.startswith("J|"):
            cur["jupyterlab"] = line[2:]

    respawned = sorted({proc for poll in polls for proc in poll["processes"]})
    ports = sorted({l for poll in polls for l in poll["listeners"]})
    result = {
        "gate": "P-007A-R1",
        "observed_seconds": len(polls) * INTERVAL,
        "poll_count": len(polls),
        "interval_s": INTERVAL,
        "control_writer_respawn_count": len(respawned),
        "respawned_processes": respawned,
        "listener_6500_8888": ports,
        "jupyterlab_states": sorted({poll["jupyterlab"] for poll in polls if poll["jupyterlab"]}),
        "verdict": "NO_RESPAWN" if not respawned and not ports else "RESPAWN_DETECTED",
        "polls": polls,
    }
    with open(os.path.join(OUT, "respawn_test.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
        f.write("\n")
    print("관찰 %d회 / %d초" % (len(polls), len(polls) * INTERVAL))
    print("재생성 프로세스: %s" % (respawned or "없음"))
    print("6500/8888 listener: %s" % (ports or "없음"))
    print("jupyterlab 상태: %s" % (result["jupyterlab_states"] or "unknown"))
    print("판정: %s" % result["verdict"])
    sys.exit(0 if result["verdict"] == "NO_RESPAWN" else 1)


if __name__ == "__main__":
    main()
