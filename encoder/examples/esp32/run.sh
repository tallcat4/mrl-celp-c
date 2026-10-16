#!/bin/bash
# ESP32 でビルド・書き込みして結果を読む。  ./run.sh [port] [ram]
#   ram を付けると、コードと表を内部 RAM に置く(MRL_RAM=1)
set -e
port=${1:-/dev/cu.usbserial-1110} ram=$2
. ~/esp/idf_env.sh
b=build${ram:+_ram}
idf.py -B $b -DSDKCONFIG=$b/sdkconfig -DMRL_RAM=${ram:+1} -DMRL_PROFILE=${MRL_PROFILE} -DMRL_MICRO=${MRL_MICRO} ${MRL_ENC_FLAGS:+-DMRL_ENC_FLAGS="$MRL_ENC_FLAGS"} set-target esp32 >/dev/null 2>&1 || true
idf.py -B $b -DSDKCONFIG=$b/sdkconfig -DMRL_RAM=${ram:+1} -DMRL_PROFILE=${MRL_PROFILE} -DMRL_MICRO=${MRL_MICRO} ${MRL_ENC_FLAGS:+-DMRL_ENC_FLAGS="$MRL_ENC_FLAGS"} build > $b.log 2>&1 || { tail -30 $b.log; exit 1; }
idf.py -B $b -DSDKCONFIG=$b/sdkconfig -p $port flash > $b.flash.log 2>&1 || { tail -20 $b.flash.log; exit 1; }
python - "$port" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=1)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
t0 = time.time()
while time.time() - t0 < 120:
    line = s.readline().decode(errors="replace").rstrip()
    if line.startswith("  ") or any(k in line for k in ("calibration", "micro", "state", "encoder:", "decoder:", "too small", "abort", "Guru", "Backtrace")):
        print(line, flush=True)
    if "MRL_TEST_DONE" in line or "Guru" in line:
        break
PY
