#!/usr/bin/env python3
# P-007A-R1 수집 결과를 증거 파일로 나눈다. Mac에서 돈다. 원격에 아무것도 하지 않는다.
# 사용: python3 scripts/p007a_r1_split.py <raw.json> <out_dir> <before|after>
import json, sys, os

WRITER_HINTS = ("joystick.py", "YahboomArm", "arm_driver.py", "yb-discover.py", "start_app.sh")


def main():
    raw_path, out_dir, phase = sys.argv[1], sys.argv[2], sys.argv[3]
    assert phase in ("before", "after"), phase
    data = json.load(open(raw_path, encoding="utf-8"))
    procs = data["processes"]

    def write(name, obj):
        path = os.path.join(out_dir, name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=1)
            f.write("\n")
        return path

    # 관심 프로세스는 cmdline 힌트로 고른다. 종료 대상 신원(pid+starttime+cmdline)을 그대로 남긴다.
    of_interest = [p for p in procs if any(h in p["cmdline"] for h in WRITER_HINTS)]
    write("process_%s.json" % phase, {
        "phase": phase,
        "collected_at": data["environment"]["date_iso"]["out"],
        "total_process_count": len(procs),
        "processes_of_interest": of_interest,
        "identity_note": "pid 하나만으로 종료하지 않는다. cmdline과 starttime_ticks가 같을 때만 같은 프로세스로 본다.",
        "all_processes": procs,
    })

    write("autostart_%s.json" % phase, dict(data["autostart"], phase=phase))
    write("i2c_%s.json" % phase, dict(data["i2c"], phase=phase))
    write("network_entrypoints.json" if phase == "before" else "network_entrypoints_after.json",
          dict(data["network"], phase=phase, ros=data["ros"]))

    if phase == "before":
        write("environment.json", {
            "gate": "P-007A-R1",
            "robot_host": "yahboom",
            "robot_user": "jetson",
            "robot_address": "192.168.0.94",
            "path": "Mac SSH to Jetson, then /dev/i2c-7 address 0x15",
            "robot_env": {k: v["out"] for k, v in data["environment"].items()},
            "servo_position_writes": 0,
            "gripper_command": 0,
            "torque_change": 0,
            "offset_write": 0,
            "calibration_executed": 0,
            "homing_reset_init": 0,
            "arm_driver_executed": 0,
            "moveit_executed": 0,
            "i2c_probe_commands": 0,
            "reboot_relogin_powercycle": 0,
            "vendor_source_modified": 0,
            "vendor_package_removed": 0,
            "firmware_changed": 0,
            "p007b_executed": 0,
            "p006d_started": 0,
            "t_base_camera_computed": 0,
            "credentials_recorded": False,
        })

    # 요약 출력. 큰 raw를 다시 읽지 않고 판단할 수 있게 한다.
    s = data["sources"]
    print("## 요약 (%s)" % phase)
    print("- hostname: %s / sudo: %s" % (data["environment"]["hostname"]["out"], data["environment"]["sudo_nopasswd"]["out"]))
    print("- 전체 프로세스 %d, 관심 프로세스 %d" % (len(procs), len(of_interest)))
    for p in of_interest:
        print("  - pid %s ppid %s start %s :: %s" % (p["pid"], p["ppid"], p["starttime_ticks"], p["cmdline"][:110]))
    print("- i2c fd holders: %s" % [(h["pid"], h["i2c_fd_count"], h["cmdline"][:48]) for h in data["i2c"]["fd_holders"]["holders"]])
    print("- user autostart: %s" % [os.path.basename(e["path"]) for e in data["autostart"]["user_entries"]])
    print("- system 동명 autostart: %s" % data["autostart"]["system_same_name"])
    print("- XDG_CONFIG_DIRS: %s" % data["autostart"]["xdg_config_dirs"]["out"])
    print("- 로그인 설정 기동 hit: %d건" % len(data["login_configs"]["startup_hits"]))
    print("- pyc 대응: %s" % s["pyc_match"].get("verdict"))
    print("- Arm_Lib 해시: %s" % s["arm_lib_integrity"].get("sha256"))
    for name, g in s["exit_paths"].items():
        print("- exit 경로 %s: %s" % (name, g.get("hit_count", g.get("error"))))
    for name, g in s["write_paths"].items():
        print("- write 경로 %s: %s" % (name, g.get("hit_count", g.get("error"))))
    print("- jupyter unit: %s" % data["systemd"]["unit_detail"]["jupyterlab.service"]["out"].replace("\n", " | "))
    print("- docker ps -a: %s" % data["docker"]["ps_all"]["out"][:200])
    print("- udev i2c rules: %s" % data["udev"]["i2c_rules"]["out"][:200])
    print("- ROS nodes: %s" % data["ros"]["node_list"]["out"][:300].replace("\n", " "))


if __name__ == "__main__":
    main()
