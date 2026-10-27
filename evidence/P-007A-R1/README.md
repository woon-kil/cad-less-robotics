# P-007A-R1 — 배타적 제어 환경 · 안전 인터록 기준선

P-007A가 BLOCKED로 닫힌 이유는 통신이 아니라 control ownership이었다. 이 게이트는 `0x15`에 서보 명령을 쓸 수 있는 소프트웨어를 전부 식별해 **가역적으로** 격리했다. 로봇은 움직이지 않았다.

판정은 **PASS**다. P-007B는 **설계만** 가능하고 실행 허가는 아니다.

| 항목 | 결과 |
| --- | --- |
| CONTROL_WRITER (격리 후) | 0 |
| servo-capable network entrypoint | 0 (6500·8888 없음) |
| UNKNOWN servo writer | 0 |
| 72초 respawn | 0 |
| 읽기 회귀 | 600/600, error 0, empty 0 |
| 격리 전후 각도 | 동일 (86, 150, 0, 18, 93, 178) |
| 운영자 무동작 확인 | 움직이지 않았다 |
| 물리 전원 접근 | 확인 (차단 시험은 안 함) |
| 작업대 clearance | 확인 |
| 이 게이트가 내린 이동 명령 | 0 |
| 벤더 소프트웨어 삭제·firmware 변경 | 0 |
| P-007B | MAY BE DESIGNED, NOT AUTHORIZED |

## 무엇을 멈췄는가

| 대상 | 방법 | 이유 |
| --- | --- | --- |
| `joystick.py` pid 3072 | SIGKILL (신원 재확인 후) | main이 `move_init_pose()`로 `[90,164,18,0,90,30]`을 쓴다 |
| `YahboomArm.pyc` pid 3022 | SIGKILL (신원 재확인 후) | 17개 지점에서 `write6`·`set_torque`·`offset_switch`. Flask 6500 |
| `jupyterlab.service` | 운영자 `sudo systemctl stop` | 서버가 죽은 커널을 자동 재시작한다. 커널만 죽이면 되살아난다 |
| `~/.config/autostart/bash.desktop` | `.p007ar1.bak`로 rename (해시 동일) | 재로그인·재부팅 시 `start_app.sh`가 위 둘을 다시 띄운다 |

신호는 **SIGKILL만** 썼다. SIGINT는 KeyboardInterrupt로 cleanup 코드를 돌려 `write6`를 낼 수 있다. 프로세스 그룹 일괄 종료와 `gnome-terminal-server` 종료는 하지 않았다. 종료 직전에 `/proc`의 `cmdline`과 `starttime`을 다시 읽어 PID 재사용을 막았다.

## 무엇을 남겼는가

| 대상 | 분류 | 근거 |
| --- | --- | --- |
| `yahboom_oled.py` | NON_SERVO_I2C_CLIENT | `Adafruit_SSD1306` 기본 주소 `0x3C`, 주소 인자를 넘기지 않음, 서보 쓰기 호출 0 |
| `yb-discover.py` | servo 권한 없는 네트워크 응답기 | 전문 21줄. UDP 8000에서 `YAHBOOMARM_FIND`에 `DOFBOT-Pro`만 응답. exec·Arm 호출 없음 |
| `kinemarics_dofbot` | NON_CONTROLLER_PROCESS | 바이너리에 i2c·smbus·Arm_Lib 참조 0, i2c fd 없음 |
| `sh -c ros2 run` / ros2 CLI wrapper | 상속 descriptor 보유 | 서보 쓰기 코드 없음 |
| `bash-1.desktop` | 기동 불가 | `Exec` 대상 `setting.sh`가 존재하지 않는다. 원본 보존 |

## UNKNOWN 해소

before 스냅샷의 ROS 그래프에 `/arm_grap`, `/dofbot_garbage`(중복), `/dofbot_snake`, `/color_identify1`이 있었다. P-007A는 ROS 그래프를 보지 않았다. 격리 후 `ros2 node list`가 `/kinemarics_dofbot` 하나만 반환했다. 이 실측이 보이는 것은 그 노드들이 **종료된 집합(pid 3022 + ipykernel)** 소유였다는 것까지다. Jupyter 중지와 kill 사이에 node list를 다시 뜨지 않았으므로 둘을 분리하지 못한다. pid 3022 귀속은 정적 근거다 — 노드 이름이 APP·`ws/src` 범위에서 `YahboomArm.py`에만 나오고 `rclpy.init`이 `YahboomArm.py:55`이며, 커널의 cwd `dofbot_color_sorting`은 그 grep 결과에 없었다. 어느 쪽이든 격리 후 서보 쓰기 가능 프로세스는 0이다. `YahboomArm.pyc`는 `.py`와 code object digest가 일치해(MATCH) `.py` 분석을 적용할 수 있다.

## 남은 한계

- `jupyterlab.service`는 `inactive`지만 `enabled`다. **재부팅하면 Jupyter 표면이 돌아온다.** P-007B 직전에 다시 확인해야 한다.
- autostart 격리는 정적 확인이다. 재로그인 검증은 재로그인 자체가 `move_init_pose`를 유발하므로 하지 않았다.
- `writer = 0`은 "이 시점 열린 writer 0 + 72초 재생성 0 + 식별된 재기동원 차단" 범위다. 필요할 때 장치를 여는 프로그램까지 전수 증명한 것은 아니다.
- 정지 경로는 어느 것도 시험하지 않았다. Normal Stop은 이미 전송된 명령을 취소하지 못한다. `stop_strategy.md` 참조.
- 컨트롤러 보고 각도 고정은 물리 무동작의 완전한 증명이 아니다. 운영자 관찰과 함께만 읽는다.

## 원복

```bash
bash scripts/p007a_r1_restore.sh          # autostart 구성만 되돌린다
sudo systemctl start jupyterlab           # Jetson에서 (운영자)
```

롤백은 **프로세스를 다시 띄우지 않는다.** 복원 후 재로그인·재부팅·전원 재투입은 작업대를 비우고 전원 차단에 손이 닿는 상태에서만 한다. 그 순간 `joystick.py`가 팔을 `[90,164,18,0,90,30]`으로 움직인다.

## 산출물

`acceptance.json`(R1-01~34) · `environment.json` · `process_dependency_map.json` · `control_authority_inventory.json` · `process_before.json` · `process_after.json` · `autostart_before.json` · `autostart_after.json` · `i2c_before.json` · `i2c_after.json` · `network_entrypoints.json` · `network_entrypoints_after.json` · `isolation_actions.json` · `respawn_test.json` · `read_regression.csv` · `read_metrics.json` · `operator_confirmations.json` · `stop_strategy.md` · `logs/`

도구는 `scripts/p007a_r1_{collect,read,split,isolate,respawn_test}.py`, `p007a_r1_snapshot.sh`, `p007a_r1_restore.sh`다. 수집기는 `bash -lc`를 쓰지 않는다. 로그인 파일 소싱이 벤더 앱을 띄울 수 있다. `i2cdetect`·`i2cget`·`i2cset`은 쓰지 않았다.
