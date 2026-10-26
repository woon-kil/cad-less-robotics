# 연결

이번 게이트에서 확인한 읽기 경로다.

```text
Mac / Cursor
      |
      | SSH
      v
Jetson Orin Nano  hostname yahboom  192.168.0.94
      |
      | /dev/i2c-7
      v
DOFBOT controller address 0x15
      |
      v
Servo ID1 .. ID6
```

P620은 이 읽기에 쓰지 않았다. 팔 버스는 Orin에 있다. 카메라는 이 경로에 없다.

Orin은 Ubuntu 22.04.5 LTS, 커널 `5.15.148-tegra`, 사용자 `jetson`이 `i2c` 그룹에 있다. 장치 시각대는 `+08:00`이다.
