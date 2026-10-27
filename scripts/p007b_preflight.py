#!/usr/bin/env python3
"""P-007B preflight — 모션 직전 배타적 제어 환경을 새로 측정한다. Mac에서 돈다.

사용: python3 scripts/p007b_preflight.py

읽기만 한다. 서보 쓰기 0, 프로세스 종료 0, 설정 변경 0, i2c probe 0.
P-007A-R1의 PASS를 재사용하지 않는다 (스펙 §3). 하나라도 실패하면 BLOCKED.
원격 명령에 로그인 셸(`bash -lc`)을 쓰지 않는다 — 로그인 파일이 벤더 앱을 띄울 수 있다.
"""
import json, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 출력 경로는 환경변수로 바꿀 수 있다. 기본값을 그대로 쓰면 닫힌 P-007B 증거를 덮어쓴다.
# 2026-10-27 P-007C 에서 실제로 그 사고가 났다. 다른 게이트에서는 P007_OUT 을 반드시 지정한다.
OUT = os.environ.get("P007_OUT") or os.path.join(ROOT, "evidence", "P-007B")
REMOTE = os.environ.get("P007A_REMOTE", "jetson@192.168.0.94")
# P-007A-R1 측정값 — 재부팅·재로그인 감지 기준선
R1_BTIME = "1793005682"
R1_SESSION_PID = "2969"          # gnome-terminal-server
R1_SESSION_STARTTIME = "2433"
R1_AUTOSTART_BAK_SHA = "416f63d965ce008b06ce0835f1573c75eebffeb219009fb0acac14dacdde56af"
ARM_LIB_SHA = "edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c"
ARM_LIB = "/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py"
AUTOSTART = "/home/jetson/.config/autostart"

REMOTE_SCRIPT = r"""
set -u
echo "BTIME|$(awk '/^btime/{print $2}' /proc/stat)"
if [ -d /proc/%(spid)s ]; then
  echo "SESSION_START|$(sed 's/.*) //' /proc/%(spid)s/stat | awk '{print $20}')"
  echo "SESSION_CMD|$(tr '\0' ' ' < /proc/%(spid)s/cmdline)"
else
  echo "SESSION_START|ABSENT"
  echo "SESSION_CMD|ABSENT"
fi
echo "PROC_YAHBOOMARM|$(pgrep -cf '[Y]ahboomArm')"
echo "PROC_JOYSTICK|$(pgrep -cf '[j]oystick\.py')"
echo "PROC_ARMDRIVER|$(pgrep -cf '[a]rm_driver\.py')"
echo "PROC_IPYKERNEL|$(pgrep -cf '[i]pykernel_launcher')"
echo "JUPYTER_ACTIVE|$(systemctl is-active jupyterlab 2>&1)"
echo "JUPYTER_ENABLED|$(systemctl is-enabled jupyterlab 2>&1)"
echo "PORT_6500|$(ss -ltn 2>/dev/null | grep -cE ':6500\b')"
echo "PORT_8888|$(ss -ltn 2>/dev/null | grep -cE ':8888\b')"
echo "AUTOSTART_LIST|$(ls -1 %(autostart)s 2>/dev/null | tr '\n' ',')"
echo "AUTOSTART_DESKTOP_COUNT|$(ls -1 %(autostart)s/*.desktop 2>/dev/null | wc -l | tr -d ' ')"
echo "AUTOSTART_BAK_SHA|$(sha256sum %(autostart)s/bash.desktop.p007ar1.bak 2>/dev/null | awk '{print $1}')"
echo "ARMLIB_SHA|$(sha256sum %(armlib)s 2>/dev/null | awk '{print $1}')"
for d in /proc/[0-9]*; do
  pid=${d#/proc/}
  n=0
  for fd in $d/fd/*; do
    t=$(readlink "$fd" 2>/dev/null) || continue
    case "$t" in */dev/i2c-*) n=$((n+1));; esac
  done
  [ "$n" -gt 0 ] && echo "I2C_FD|$pid|$n|$(tr '\0' ' ' < $d/cmdline 2>/dev/null)"
done
echo "DATE|$(date -Is)"
"""


def rsh(script, timeout=60):
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", REMOTE, "bash -s"],
                       input=script, capture_output=True, text=True, timeout=timeout)
    if p.returncode != 0 and not p.stdout:
        raise SystemExit("원격 실행 실패 rc=%d: %s" % (p.returncode, p.stderr.strip()))
    return p.stdout


def main():
    os.makedirs(os.path.join(OUT, "logs"), exist_ok=True)
    raw = rsh(REMOTE_SCRIPT % {"spid": R1_SESSION_PID, "autostart": AUTOSTART, "armlib": ARM_LIB})
    with open(os.path.join(OUT, "logs", "preflight_raw.txt"), "w", encoding="utf-8") as f:
        f.write(raw)

    kv, i2c = {}, []
    for line in raw.splitlines():
        if line.startswith("I2C_FD|"):
            _, pid, n, cmd = line.split("|", 3)
            i2c.append({"pid": int(pid), "i2c_fd_count": int(n), "cmdline": cmd.strip()})
        elif "|" in line:
            k, v = line.split("|", 1)
            kv[k] = v.strip()

    # 서보 쓰기가 가능한 i2c 보유자만 센다. 분류 근거는 P-007A-R1 control_authority_inventory.json.
    known_non_writers = ("yahboom_oled.py", "ros2 run dofbot_pro_info", "ros2", "kinemarics_dofbot")
    writers = [h for h in i2c if not any(t in h["cmdline"] for t in known_non_writers)]

    checks = []

    def ck(cid, name, ok, got, expect):
        checks.append({"id": cid, "name": name, "result": "PASS" if ok else "FAIL",
                       "measured": got, "expected": expect})
        return ok

    reboot = kv.get("BTIME") != R1_BTIME
    relogin = kv.get("SESSION_START") != R1_SESSION_STARTTIME
    ck("PF-01", "재부팅 없음 (btime)", not reboot, kv.get("BTIME"), R1_BTIME)
    ck("PF-02", "세션 재시작 없음 (gnome-terminal-server starttime)", not relogin,
       kv.get("SESSION_START"), R1_SESSION_STARTTIME)
    ck("PF-03", "YahboomArm absent", kv.get("PROC_YAHBOOMARM") == "0", kv.get("PROC_YAHBOOMARM"), "0")
    ck("PF-04", "joystick.py absent", kv.get("PROC_JOYSTICK") == "0", kv.get("PROC_JOYSTICK"), "0")
    ck("PF-05", "arm_driver.py absent", kv.get("PROC_ARMDRIVER") == "0", kv.get("PROC_ARMDRIVER"), "0")
    ck("PF-06", "ipykernel absent", kv.get("PROC_IPYKERNEL") == "0", kv.get("PROC_IPYKERNEL"), "0")
    ck("PF-07", "jupyterlab inactive", kv.get("JUPYTER_ACTIVE") == "inactive", kv.get("JUPYTER_ACTIVE"), "inactive")
    ck("PF-08", "TCP 6500 absent", kv.get("PORT_6500") == "0", kv.get("PORT_6500"), "0")
    ck("PF-09", "TCP 8888 absent", kv.get("PORT_8888") == "0", kv.get("PORT_8888"), "0")
    ck("PF-10", "autostart 활성 .desktop 0", kv.get("AUTOSTART_DESKTOP_COUNT") in ("0", "1"),
       kv.get("AUTOSTART_LIST"), "bash-1.desktop(기동 불가) 외 활성 .desktop 없음")
    ck("PF-11", "autostart 백업 원본 보존", kv.get("AUTOSTART_BAK_SHA") == R1_AUTOSTART_BAK_SHA,
       kv.get("AUTOSTART_BAK_SHA"), R1_AUTOSTART_BAK_SHA)
    ck("PF-12", "Arm_Lib 무결성", kv.get("ARMLIB_SHA") == ARM_LIB_SHA, kv.get("ARMLIB_SHA"), ARM_LIB_SHA)
    ck("PF-13", "uncontrolled servo writer 0", len(writers) == 0,
       [w["cmdline"][:60] for w in writers], "0건")

    failed = [c for c in checks if c["result"] != "PASS"]
    verdict = "READY" if not failed else "BLOCKED"

    with open(os.path.join(OUT, "preflight.json"), "w", encoding="utf-8") as f:
        json.dump({"gate": "P-007B", "measured_at_device": kv.get("DATE"),
                   "reuse_of_p007a_r1_pass": False,
                   "reboot_detected": reboot, "relogin_detected": relogin,
                   "session_marker_cmdline": kv.get("SESSION_CMD"),
                   "verdict": verdict, "failed_count": len(failed), "checks": checks,
                   "operator_confirmations": "PENDING — 홀드 포인트 전에 새로 받는다",
                   "servo_writes_by_preflight": 0, "i2c_probe": 0},
                  f, ensure_ascii=False, indent=1)
        f.write("\n")

    with open(os.path.join(OUT, "control_ownership.json"), "w", encoding="utf-8") as f:
        json.dump({"gate": "P-007B", "measured_at_device": kv.get("DATE"),
                   "i2c_fd_holders": i2c,
                   "servo_capable_uncontrolled_writers": writers,
                   "writer_count": len(writers),
                   "non_writer_classification_source": "evidence/P-007A-R1/control_authority_inventory.json",
                   "scope": "이 측정 시점에 열린 fd 기준. 필요할 때 장치를 여는 프로그램까지 전수 증명한 것은 아니다."},
                  f, ensure_ascii=False, indent=1)
        f.write("\n")

    with open(os.path.join(OUT, "source_integrity.json"), "w", encoding="utf-8") as f:
        json.dump({"path": ARM_LIB, "sha256_measured": kv.get("ARMLIB_SHA"),
                   "sha256_expected": ARM_LIB_SHA,
                   "matches_p007a_r1": kv.get("ARMLIB_SHA") == ARM_LIB_SHA,
                   "checked_before_command": True},
                  f, ensure_ascii=False, indent=1)
        f.write("\n")

    print("== P-007B preflight: %s (실패 %d건)" % (verdict, len(failed)))
    for c in checks:
        mark = "OK " if c["result"] == "PASS" else "FAIL"
        print("  [%s] %-8s %-42s 측정=%s" % (mark, c["id"], c["name"], c["measured"]))
    print("\ni2c fd 보유 %d건, 그중 서보 쓰기 가능 %d건" % (len(i2c), len(writers)))
    sys.exit(0 if verdict == "READY" else 1)


if __name__ == "__main__":
    main()
