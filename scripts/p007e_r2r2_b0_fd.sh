#!/bin/bash
# P-007E-R2-R2 Stage B0 — inherited /dev/i2c-7 fd read-only classification
# 🔴 READ-ONLY. /proc 읽기만. kill·fd close·restart 금지. I2C 트랜잭션 0
echo "### B0 — inherited i2c fd provenance (read-only /proc)"
echo "### host=$(hostname) date=$(date -Is)"

echo; echo "## [0] /dev/i2c-7 fd 를 보유한 프로세스 전수 재확인"
for q in $(ls /proc | grep -E '^[0-9]+$'); do
  hits=$(ls -l /proc/$q/fd 2>/dev/null | grep "i2c-7")
  if [ -n "$hits" ]; then
    echo "--- PID $q"
    echo "$hits" | sed 's/^/    /'
  fi
done
echo "## (전수 스캔 끝)"

echo; echo "## [1] 조상 체인 — PID 1 까지"
p=3091
while [ "$p" != "0" ] && [ "$p" != "1" ]; do
  ppid=$(awk '{print $4}' /proc/$p/stat 2>/dev/null)
  echo "PID $p  ppid=$ppid"
  echo "   comm    : $(cat /proc/$p/comm 2>/dev/null)"
  echo "   cmdline : $(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)"
  echo "   exe     : $(readlink /proc/$p/exe 2>/dev/null)"
  echo "   cwd     : $(readlink /proc/$p/cwd 2>/dev/null)"
  echo "   start   : $(ps -o lstart= -p $p 2>/dev/null)"
  echo "   i2c fd  : $(ls -l /proc/$p/fd 2>/dev/null | grep -c i2c-7)"
  p=$ppid
done
echo "PID 1 : $(tr '\0' ' ' < /proc/1/cmdline)  i2c fd=$(ls -l /proc/1/fd 2>/dev/null | grep -c i2c-7)"

echo; echo "## [2] 3091 / 3092 / 3103 상세"
for q in 3091 3092 3103; do
  echo "--- PID $q"
  echo "   cmdline : $(tr '\0' ' ' < /proc/$q/cmdline 2>/dev/null)"
  echo "   exe     : $(readlink /proc/$q/exe 2>/dev/null)"
  echo "   ppid    : $(awk '{print $4}' /proc/$q/stat 2>/dev/null)"
  echo "   fd 전부 :"
  ls -l /proc/$q/fd 2>/dev/null | awk '{print "      "$9" -> "$11}' | grep -v '^      ->'
  echo "   i2c-7 fdinfo:"
  for f in $(ls /proc/$q/fd 2>/dev/null); do
    tgt=$(readlink /proc/$q/fd/$f 2>/dev/null)
    if [ "$tgt" = "/dev/i2c-7" ]; then
      echo "      fd=$f target=$tgt"
      sed 's/^/         /' /proc/$q/fdinfo/$f 2>/dev/null
    fi
  done
done

echo; echo "## [3] 자식 프로세스"
for q in 3091 3092 3103; do
  echo "PID $q children: $(cat /proc/$q/task/*/children 2>/dev/null | tr '\n' ' ')"
done

echo; echo "## [4] 기동 환경 — ROS 관련 환경변수만 (read-only)"
for q in 3091 3092 3103; do
  echo "--- PID $q"
  tr '\0' '\n' < /proc/$q/environ 2>/dev/null | grep -E '^(ROS_|AMENT_|PYTHONPATH|LD_LIBRARY_PATH)' | sed 's/^/   /' | head -12
done

echo; echo "## [5] 읽기/쓰기 흔적 — io 통계 (누적값. 어떤 fd 인지 구분하지는 못한다)"
for q in 3091 3092 3103; do
  echo "--- PID $q  $(cat /proc/$q/comm)"
  grep -E '^(rchar|wchar|syscr|syscw)' /proc/$q/io 2>/dev/null | sed 's/^/   /'
done

echo; echo "## [6] i2c 장치 노드 · 드라이버 측 참고"
ls -l /dev/i2c-*
echo "-- i2c 그룹 소속 확인:"; id | tr ',' '\n' | grep i2c
echo; echo "### B0 END"
