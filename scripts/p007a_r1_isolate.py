#!/usr/bin/env python3
"""P-007A-R1 가역 격리. Mac에서 돈다.

사용: python3 scripts/p007a_r1_isolate.py [--dry-run]

순서를 고정한다. 롤백 준비 확인 → 재기동원 차단 → 차단 확인 → writer 종료.
신호는 SIGKILL만 쓴다. SIGINT는 KeyboardInterrupt로 cleanup 코드를 돌려 write6를
낼 수 있다. 종료 직전에 /proc의 cmdline과 starttime으로 신원을 다시 확인한다(PID 재사용 방어).

한계: SIGKILL은 이미 컨트롤러로 전송된 이동 명령을 취소하지 않는다.
원복: bash scripts/p007a_r1_restore.sh (구성만 되돌리고 프로세스는 띄우지 않는다)
"""
import json, os, subprocess, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "evidence", "P-007A-R1")
REMOTE = os.environ.get("P007A_REMOTE", "jetson@192.168.0.94")
AUTOSTART = "/home/jetson/.config/autostart"
BAK = ".p007ar1.bak"
# 종료 대상. snapshot의 cmdline 패턴으로 찾는다. pid를 코드에 박지 않는다.
TARGETS = [
    ("joystick.py", "CONTROL_WRITER. main이 move_init_pose()를 호출한다"),
    ("YahboomArm.pyc", "CONTROL_WRITER + NETWORK_CONTROL_ENTRYPOINT (Flask 6500)"),
    ("ipykernel_launcher", "UNCONTROLLED_EXECUTION_SURFACE. cwd가 dofbot_color_sorting/scripts"),
]
DRY = "--dry-run" in sys.argv
actions = []


def rsh(cmd, check=True, timeout=30):
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", REMOTE, cmd],
                       capture_output=True, text=True, timeout=timeout)
    if check and p.returncode != 0:
        raise SystemExit("원격 명령 실패 rc=%d: %s\n%s" % (p.returncode, cmd, p.stderr.strip()))
    return p


def log(kind, detail, result):
    actions.append({"at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "action": kind,
                    "detail": detail, "result": result, "dry_run": DRY})
    print("  [%s] %s -> %s" % (kind, detail, result))


def preflight():
    print("== 0. 사전 검증")
    if rsh("hostname").stdout.strip() != "yahboom":
        raise SystemExit("원격 hostname이 yahboom이 아니다")
    conf = json.load(open(os.path.join(OUT, "operator_confirmations.json"), encoding="utf-8"))
    answers = {c["id"]: c["answer"] for c in conf["confirmations"]}
    for key in ("R1-20", "R1-21", "R1-19-precondition"):
        if not str(answers.get(key, "")).startswith("YES"):
            raise SystemExit("운영자 확인 %s 가 YES가 아니다: %r" % (key, answers.get(key)))
    if not os.path.exists(os.path.join(ROOT, "scripts", "p007a_r1_restore.sh")):
        raise SystemExit("롤백 스크립트가 없다")
    snap = json.load(open(os.path.join(OUT, "process_before.json"), encoding="utf-8"))
    print("  운영자 확인 3종 YES, 롤백 스크립트 있음, snapshot 프로세스 %d건" % snap["total_process_count"])
    return snap


def block_respawn_sources():
    """autostart를 .desktop이 아닌 이름으로 옮긴다. 원본 바이트는 보존한다."""
    print("== 1. 재기동원 차단 (autostart)")
    before = rsh("sha256sum %s/* 2>/dev/null; echo '---'; ls -1 %s/" % (AUTOSTART, AUTOSTART)).stdout
    log("autostart_before", "%s 목록·해시" % AUTOSTART, before.replace("\n", " | "))
    # start_app.sh를 띄우는 항목만 옮긴다. setting.sh 항목은 대상 파일이 없어 기동할 수 없다.
    cmd = ("set -e; f=%s/bash.desktop; "
           "if [ -e \"$f\" ]; then mv -v \"$f\" \"$f%s\"; else echo 'already isolated'; fi" % (AUTOSTART, BAK))
    if DRY:
        log("autostart_rename", cmd, "DRY_RUN_SKIPPED")
    else:
        log("autostart_rename", "bash.desktop -> bash.desktop%s" % BAK, rsh(cmd).stdout.strip())


def verify_respawn_blocked():
    print("== 2. 차단 확인")
    active = rsh("ls -1 %s/*.desktop 2>/dev/null | xargs -r -n1 basename; true" % AUTOSTART, check=False).stdout.strip()
    remaining = [x for x in active.splitlines() if x]
    detail = "활성 .desktop: %s" % (remaining or "없음")
    launchable = []
    for entry in remaining:
        execline = rsh("grep -h '^Exec=' %s/%s 2>/dev/null || true" % (AUTOSTART, entry)).stdout.strip()
        target = execline.split()[-1] if execline else ""
        exists = rsh("test -e %s && echo yes || echo no" % target, check=False).stdout.strip() if target else "no"
        detail += " | %s -> %s (대상 존재: %s)" % (entry, execline, exists)
        if exists == "yes":
            launchable.append(entry)
    log("autostart_after", detail, "기동 가능 항목: %s" % (launchable or "없음"))
    if launchable and not DRY:
        raise SystemExit("기동 가능한 autostart 항목이 남았다: %s" % launchable)
    after = rsh("sha256sum %s/* 2>/dev/null" % AUTOSTART).stdout
    log("autostart_hash_after", "해시 재확인", after.replace("\n", " | "))


def kill_writers(snap):
    print("== 3. writer 종료 (SIGKILL, 신원 재확인 후)")
    index = {}
    for p in snap["processes_of_interest"] + snap["all_processes"]:
        argv0 = p["cmdline"].split()[0] if p["cmdline"] else ""
        # 래퍼 셸("bash -c python3 ...YahboomArm.pyc;exec bash")을 고르면 실제 writer가 살아남는다.
        # argv0가 python3인 프로세스만 종료 대상으로 본다.
        if not argv0.endswith("python3"):
            continue
        for pat, _ in TARGETS:
            if pat in p["cmdline"] and pat not in index:
                index[pat] = p
    for pat, role in TARGETS:
        p = index.get(pat)
        if not p:
            log("kill_skip", pat, "snapshot에 없음")
            continue
        pid, exp = p["pid"], p["starttime_ticks"]
        check = ("if [ ! -d /proc/%(pid)d ]; then echo GONE; exit 3; fi; "
                 "ST=$(sed 's/.*) //' /proc/%(pid)d/stat | awk '{print $20}'); "
                 "CMD=$(tr '\\0' ' ' < /proc/%(pid)d/cmdline); "
                 "echo \"ST=$ST CMD=$CMD\"" % {"pid": pid})
        got = rsh(check, check=False)
        if got.returncode == 3:
            log("kill_skip", "%s pid %d" % (pat, pid), "이미 없음")
            continue
        line = got.stdout.strip()
        st = line.split("ST=")[1].split()[0] if "ST=" in line else ""
        if st != str(exp) or pat not in line:
            log("identity_mismatch", "%s pid %d 기대 starttime %s" % (pat, pid, exp), line[:200])
            raise SystemExit("신원 불일치. PID 재사용 가능성. 종료하지 않았다.")
        if DRY:
            log("kill", "%s pid %d (%s)" % (pat, pid, role), "DRY_RUN_SKIPPED. 신원 일치 확인: %s" % line[:120])
            continue
        rsh("kill -9 %d" % pid, check=False)
        time.sleep(1.5)
        gone = rsh("test -d /proc/%d && echo ALIVE || echo GONE" % pid, check=False).stdout.strip()
        log("kill", "%s pid %d SIGKILL (%s)" % (pat, pid, role), gone)
        if gone != "GONE":
            raise SystemExit("%s pid %d 가 SIGKILL 후에도 남아 있다" % (pat, pid))


def main():
    print("P-007A-R1 격리%s" % (" (--dry-run)" if DRY else ""))
    snap = preflight()
    block_respawn_sources()
    verify_respawn_blocked()
    kill_writers(snap)
    path = os.path.join(OUT, "isolation_actions.json")
    payload = {
        "gate": "P-007A-R1",
        "dry_run": DRY,
        "signal_used": "SIGKILL only. SIGINT·SIGTERM 미사용",
        "vendor_source_deleted": 0,
        "package_removed": 0,
        "vendor_source_modified": 0,
        "firmware_changed": 0,
        "process_group_kill": 0,
        "i2c_probe": 0,
        "reversible": "scripts/p007a_r1_restore.sh 가 autostart 구성만 되돌린다. 프로세스는 재기동하지 않는다.",
        "not_stopped_and_why": {
            "yahboom_oled.py": "NON_SERVO_I2C_CLIENT. Adafruit_SSD1306 기본 주소 0x3C이고 서보 쓰기 호출이 없다",
            "yb-discover.py": "UDP 8000 discovery 응답기. exec·Arm 호출이 없어 servo-capable 진입점이 아니다",
            "kinemarics_dofbot": "NON_CONTROLLER_PROCESS. 바이너리에 i2c·smbus·Arm_Lib 참조 0",
            "sh -c ros2 run / ros2 python wrapper": "상속받은 descriptor만 들고 있고 서보 쓰기 코드가 없다",
            "bash-1.desktop": "Exec 대상 setting.sh 가 존재하지 않아 기동 불가. 원본 보존",
        },
        "manual_steps_required": [
            "sudo systemctl stop jupyterlab  (system unit, NOPASSWD sudo 없음 — 운영자 실행)",
        ],
        "actions": actions,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)
        f.write("\n")
    print("\n기록: %s" % path)
    print("원복: bash scripts/p007a_r1_restore.sh")


if __name__ == "__main__":
    main()
