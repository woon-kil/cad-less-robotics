#!/usr/bin/env python3
# P-007A-R1 읽기 전용 수집기. Jetson에서 `ssh jetson@HOST python3 - < 이 파일`로 돈다.
# 서보 쓰기, i2c probe(i2cdetect/i2cget/i2cset), 프로세스 종료, 설정 변경을 하지 않는다.
# 표준출력은 JSON 하나다. 실패한 항목은 지우지 않고 error로 남긴다.
import json, os, re, subprocess, sys, glob, hashlib, marshal

APP = "/home/jetson/dofbot_pro/APP_DOFBOT_PRO"
ARM_LIB = "/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py"
EXIT_PATTERNS = ["signal.signal", "atexit", "KeyboardInterrupt", "finally", "__del__", "QUIT", "sys.exit", "os._exit"]
WRITE_PATTERNS = ["write6", "Arm_serial_servo_write", "set_torque", "offset_switch", "Arm_reset", "move_init_pose", "arm_move"]


def run(cmd, timeout=20):
    """셸 명령 하나를 돌리고 rc·stdout·stderr를 그대로 담는다."""
    try:
        p = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)
        return {"cmd": cmd, "rc": p.returncode, "out": p.stdout.strip(), "err": p.stderr.strip()[:2000]}
    except subprocess.TimeoutExpired:
        return {"cmd": cmd, "rc": None, "out": "", "err": "timeout %ds" % timeout}
    except Exception as e:  # noqa: BLE001
        return {"cmd": cmd, "rc": None, "out": "", "err": "%s: %s" % (type(e).__name__, e)}


def read_text(path, limit=200000):
    try:
        with open(path, "r", errors="replace") as f:
            return f.read(limit)
    except Exception as e:  # noqa: BLE001
        return None


def sha256(path):
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return {"sha256": h.hexdigest(), "bytes": os.path.getsize(path)}
    except Exception as e:  # noqa: BLE001
        return {"error": str(e)}


def grep_file(path, patterns):
    """정적 grep. 줄 번호와 줄 내용을 남긴다."""
    text = read_text(path)
    if text is None:
        return {"path": path, "error": "unreadable"}
    hits = []
    for i, line in enumerate(text.splitlines(), 1):
        for pat in patterns:
            if pat in line:
                hits.append({"line": i, "pattern": pat, "text": line.strip()[:200]})
                break
    return {"path": path, "exists": True, "hit_count": len(hits), "hits": hits[:120]}


def proc_table():
    """/proc에서 pid·ppid·시작시각(clock ticks)·cmdline을 직접 읽는다. PID 재사용 방어용 신원이다."""
    rows = []
    for d in sorted(glob.glob("/proc/[0-9]*"), key=lambda p: int(os.path.basename(p))):
        pid = int(os.path.basename(d))
        stat = read_text(os.path.join(d, "stat"), 8192)
        if not stat:
            continue
        # comm에 공백·괄호가 있을 수 있어 마지막 ')' 뒤부터 쪼갠다
        try:
            rest = stat[stat.rindex(")") + 2:].split()
            ppid = int(rest[1])
            starttime = int(rest[19])
        except Exception:  # noqa: BLE001
            ppid, starttime = None, None
        cmdline = read_text(os.path.join(d, "cmdline"), 8192) or ""
        cmdline = cmdline.replace("\x00", " ").strip()
        if not cmdline:
            continue
        try:
            uid = os.stat(d).st_uid
        except Exception:  # noqa: BLE001
            uid = None
        rows.append({"pid": pid, "ppid": ppid, "starttime_ticks": starttime, "uid": uid, "cmdline": cmdline[:400]})
    return rows


def i2c_fd_holders():
    """/proc/*/fd에서 i2c 장치를 가리키는 descriptor를 센다. 권한이 없으면 error로 남긴다."""
    holders, errors = [], []
    for d in glob.glob("/proc/[0-9]*"):
        pid = os.path.basename(d)
        fdd = os.path.join(d, "fd")
        try:
            fds = os.listdir(fdd)
        except Exception as e:  # noqa: BLE001
            errors.append({"pid": int(pid), "error": type(e).__name__})
            continue
        count = 0
        for fd in fds:
            try:
                target = os.readlink(os.path.join(fdd, fd))
            except Exception:  # noqa: BLE001
                continue
            if "/dev/i2c-" in target:
                count += 1
        if count:
            cmdline = (read_text(os.path.join(d, "cmdline"), 4096) or "").replace("\x00", " ").strip()
            holders.append({"pid": int(pid), "i2c_fd_count": count, "cmdline": cmdline[:300]})
    return {"holders": sorted(holders, key=lambda r: r["pid"]), "unreadable_pid_count": len(errors)}


def pyc_vs_py(py_path, pyc_path):
    """실행 중인 .pyc가 분석한 .py에서 나왔는지 확인한다. 불일치·확인 불가면 UNKNOWN 분류 근거다."""
    out = {"py": py_path, "pyc": pyc_path}
    src = None
    try:
        with open(py_path, "rb") as f:
            src = f.read()
    except Exception as e:  # noqa: BLE001
        out["error"] = "py unreadable: %s" % e
        return out
    try:
        with open(pyc_path, "rb") as f:
            data = f.read()
    except Exception as e:  # noqa: BLE001
        out["error"] = "pyc unreadable: %s" % e
        return out
    import importlib.util
    out["magic_runtime"] = importlib.util.MAGIC_NUMBER.hex()
    out["magic_pyc"] = data[:4].hex()
    out["magic_match"] = data[:4] == importlib.util.MAGIC_NUMBER
    if not out["magic_match"]:
        out["verdict"] = "UNKNOWN_DIFFERENT_PYTHON_MAGIC"
        return out
    try:
        compiled = compile(src, py_path, "exec")
        loaded = marshal.loads(data[16:])
    except Exception as e:  # noqa: BLE001
        out["error"] = "compare failed: %s" % e
        out["verdict"] = "UNKNOWN"
        return out

    def digest(code):
        parts = [code.co_code, repr(sorted(code.co_names)).encode(), repr(code.co_varnames).encode()]
        for c in code.co_consts:
            if hasattr(c, "co_code"):
                parts.append(digest(c).encode())
            else:
                parts.append(repr(c).encode())
        return hashlib.sha256(b"|".join(parts)).hexdigest()

    out["digest_py"] = digest(compiled)
    out["digest_pyc"] = digest(loaded)
    out["verdict"] = "MATCH" if out["digest_py"] == out["digest_pyc"] else "MISMATCH"
    return out


def login_configs():
    """로그인 가능한 모든 사용자의 로그인 스크립트와 시스템 로그인 경로에서 기동 줄을 찾는다."""
    users = []
    for line in (read_text("/etc/passwd") or "").splitlines():
        f = line.split(":")
        if len(f) < 7:
            continue
        if f[6].rstrip().endswith(("nologin", "false")):
            continue
        users.append({"user": f[0], "home": f[5], "shell": f[6]})
    files = []
    for u in users:
        for name in [".bashrc", ".profile", ".bash_profile", ".xsessionrc", ".xprofile", ".config/autostart"]:
            p = os.path.join(u["home"], name)
            if os.path.exists(p):
                files.append(p)
    for pat in ["/etc/profile", "/etc/profile.d/*", "/etc/X11/Xsession.d/*", "/etc/xdg/autostart/*"]:
        files.extend(sorted(glob.glob(pat)))
    found = []
    for p in files:
        if os.path.isdir(p):
            found.append({"path": p, "dir_entries": sorted(os.listdir(p))[:50]})
            continue
        hit = grep_file(p, ["start_app", "YahboomArm", "joystick", "yb-discover", "arm_driver", "dofbot"])
        if hit.get("hit_count"):
            found.append(hit)
    return {"login_users": users, "scanned_file_count": len(files), "startup_hits": found}


def main():
    data = {"gate": "P-007A-R1", "role": "read_only_snapshot", "writes_performed": 0}

    data["environment"] = {
        "hostname": run("hostname"),
        "uname": run("uname -a"),
        "os_release": run("cat /etc/os-release | head -4"),
        "date_iso": run("date -Is"),
        "whoami": run("id"),
        "sudo_nopasswd": run("sudo -n true; echo rc=$?", timeout=15),
    }

    data["processes"] = proc_table()
    data["i2c"] = {
        "devices": run("ls -l /dev/i2c-*"),
        "group_i2c": run("getent group i2c"),
        "adapters_sysfs": run("for d in /sys/class/i2c-dev/*; do echo \"$(basename $d) $(cat $d/name 2>/dev/null)\"; done"),
        "bus_devices": run("ls -l /sys/bus/i2c/devices/ 2>/dev/null"),
        "fd_holders": i2c_fd_holders(),
        "probe_commands_used": [],
    }

    data["autostart"] = {
        "user_dir": run("ls -la /home/jetson/.config/autostart/ 2>/dev/null"),
        "system_dir": run("ls -la /etc/xdg/autostart/ 2>/dev/null"),
        "user_entries": [
            dict({"path": p, "content": read_text(p, 8000)}, **sha256(p))
            for p in sorted(glob.glob("/home/jetson/.config/autostart/*"))
        ],
        "system_same_name": sorted(
            os.path.basename(p) for p in glob.glob("/etc/xdg/autostart/*")
            if os.path.basename(p) in {os.path.basename(q) for q in glob.glob("/home/jetson/.config/autostart/*")}
        ),
        "xdg_config_dirs": run("echo \"${XDG_CONFIG_DIRS:-unset}\""),
    }

    data["login_configs"] = login_configs()

    data["cron"] = {
        "user_crontab": run("crontab -l 2>&1 | head -40"),
        "etc_crontab": run("cat /etc/crontab 2>/dev/null | tail -10"),
        "cron_d": run("ls -la /etc/cron.d/ /etc/cron.hourly/ /etc/cron.daily/ 2>/dev/null"),
        "all_user_crontabs": run("sudo -n ls -la /var/spool/cron/crontabs/ 2>&1 | head -20"),
    }

    data["systemd"] = {
        "services_running": run("systemctl list-units --type=service --state=running --no-pager --plain 2>/dev/null | head -60"),
        "timers": run("systemctl list-timers --all --no-pager 2>/dev/null | head -30"),
        "sockets": run("systemctl list-units --type=socket --all --no-pager --plain 2>/dev/null | head -30"),
        "paths": run("systemctl list-units --type=path --all --no-pager --plain 2>/dev/null | head -20"),
        "user_units": run("systemctl --user list-units --all --no-pager --plain 2>&1 | head -40"),
        "unit_detail": {
            u: run("systemctl show %s -p Restart -p ExecStart -p WantedBy -p ActiveState -p UnitFileState --no-pager 2>/dev/null" % u)
            for u in ["jupyterlab.service", "yahboom_oled.service", "docker.service"]
        },
        "reverse_deps_jupyter": run("systemctl list-dependencies --reverse jupyterlab.service --no-pager 2>/dev/null | head -20"),
        "enabled_grep_dofbot": run("systemctl list-unit-files --no-pager --plain 2>/dev/null | grep -iE 'dofbot|yahboom|arm|jupyter' | head -20"),
    }

    data["docker"] = {
        "ps_all": run("docker ps -a --format '{{.ID}} {{.Image}} {{.Status}} {{.Names}}' 2>&1 | head -20"),
        "restart_policies": run("for c in $(docker ps -aq 2>/dev/null); do echo \"$c $(docker inspect -f '{{.HostConfig.RestartPolicy.Name}} {{.HostConfig.Privileged}} {{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' $c 2>/dev/null)\"; done 2>&1 | head -20"),
    }

    data["udev"] = {
        "i2c_rules": run("grep -rln -i i2c /etc/udev/rules.d /lib/udev/rules.d 2>/dev/null | head -20"),
        "i2c_rule_lines": run("grep -rn -i i2c /etc/udev/rules.d 2>/dev/null | head -20"),
    }

    data["network"] = {
        "listeners": run("ss -ltnp 2>&1 | head -40"),
        "listeners_sudo": run("sudo -n ss -ltnp 2>&1 | head -40"),
    }

    data["ros"] = {
        "node_list": run("source /opt/ros/humble/setup.bash 2>/dev/null; source /home/jetson/dofbot_pro_ws/install/setup.bash 2>/dev/null; timeout 15 ros2 node list 2>&1 | head -30", timeout=30),
        "service_list": run("source /opt/ros/humble/setup.bash 2>/dev/null; source /home/jetson/dofbot_pro_ws/install/setup.bash 2>/dev/null; timeout 15 ros2 service list 2>&1 | head -40", timeout=30),
        "topic_list": run("source /opt/ros/humble/setup.bash 2>/dev/null; source /home/jetson/dofbot_pro_ws/install/setup.bash 2>/dev/null; timeout 15 ros2 topic list 2>&1 | head -40", timeout=30),
    }

    start_app = os.path.join(APP, "start_app.sh")
    data["sources"] = {
        "start_app_sh": dict({"path": start_app, "content": read_text(start_app, 20000)}, **sha256(start_app)),
        "arm_lib_integrity": sha256(ARM_LIB),
        "arm_lib_constructor": grep_file(ARM_LIB, ["def __init__", "def __del__", "write", "bus ="]),
        "exit_paths": {
            name: grep_file(os.path.join(APP, name), EXIT_PATTERNS)
            for name in ["joystick.py", "YahboomArm.py", "yb-discover.py"]
        },
        "write_paths": {
            name: grep_file(os.path.join(APP, name), WRITE_PATTERNS)
            for name in ["joystick.py", "YahboomArm.py", "yb-discover.py"]
        },
        "app_dir_listing": run("ls -la %s 2>/dev/null | head -40" % APP),
        "oled_source": grep_file("/home/jetson/dofbot_pro/Yahboom_Oled/yahboom_oled.py", ["0x", "SMBus", "address", "write", "i2c"]),
        "robot_controller_grep": run("grep -rn -E 'class Robot_Controller|def move_init_pose|P_LOOK_AT' %s 2>/dev/null | head -20" % APP),
        "yb_discover_head": {"path": os.path.join(APP, "yb-discover.py"), "head": (read_text(os.path.join(APP, "yb-discover.py"), 3000) or "")[:3000]},
        "kinematics_i2c_refs": run("strings /home/jetson/dofbot_pro_ws/install/dofbot_pro_info/lib/dofbot_pro_info/kinemarics_dofbot 2>/dev/null | grep -icE 'i2c|smbus|Arm_Lib' ; echo '---'; ldd /home/jetson/dofbot_pro_ws/install/dofbot_pro_info/lib/dofbot_pro_info/kinemarics_dofbot 2>/dev/null | grep -icE 'i2c|arm_lib'"),
        "pyc_match": pyc_vs_py(os.path.join(APP, "YahboomArm.py"), os.path.join(APP, "YahboomArm.pyc")),
        "writer_file_perms": run("ls -l %s/joystick.py %s/YahboomArm.pyc %s/start_app.sh /home/jetson/dofbot_pro_ws/src/dofbot_pro_driver/dofbot_pro_driver/arm_driver.py 2>&1" % (APP, APP, APP)),
    }

    json.dump(data, sys.stdout, ensure_ascii=False, indent=1)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
