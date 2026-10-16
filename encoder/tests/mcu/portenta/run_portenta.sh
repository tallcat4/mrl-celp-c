#!/bin/bash
# Portenta H7(M7 コア)向けにビルド・書き込みして結果を表示する。  tests/mcu/portenta/run_portenta.sh [port]
# arduino-cli と arduino:mbed_portenta のコア、pyserial の入った python3 が要る。
set -e
port=${1:-$(arduino-cli board list | awk '/mbed_portenta/ {print $1; exit}')}
here=$(cd "$(dirname "$0")" && pwd)
c=$(cd "$here/../../.." && pwd)
dec=$(cd "$c/../decoder" && pwd)
sk=$here/build/mrl_test
rm -rf "$sk" && mkdir -p "$sk"
cp "$here/mrl_test.ino" "$here/glue.cpp" "$c/tests/mcu/expect_hash.h" "$sk/"
# 1 ソース 1 翻訳単位。デコーダの状態を進める dsp.c は原本とビット一致させるため積和の融合を禁止(encoder.c は融合してよい)
printf '#pragma GCC optimize ("fp-contract=off")\n#include "dsp.c"\n' > "$sk/w_dsp.c"
for s in bits tables decoder; do echo "#include \"$s.c\"" > "$sk/w_$s.c"; done
printf '#pragma GCC optimize ("fp-contract=fast")\n#include "encoder.c"\n' > "$sk/w_encoder.c"
printf '#define printf mcu_printf\nint mcu_printf(const char *f, ...);\n#include "mcu_test.c"\n' > "$sk/w_mcu_test.c"
(cd "$c/tests/vectors" && { xxd -i speech.pcm; xxd -i speech.expected.mrl; }) |
    sed 's/^unsigned char/const unsigned char/; s/^unsigned int/const unsigned int/' > "$sk/vectors.c"
flags="-O2 -I$c/include -I$c/src -I$c/tests/mcu -I$dec/include -I$dec/src"
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 --build-property "compiler.c.extra_flags=$flags" \
    --build-property "compiler.cpp.extra_flags=$flags" "$sk" 2>&1 | grep -E "error|Sketch uses|Global variables" || true
arduino-cli upload --fqbn arduino:mbed_portenta:envie_m7 -p "$port" "$sk" >/dev/null 2>&1 || arduino-cli upload --fqbn arduino:mbed_portenta:envie_m7 -p "$port" "$sk"
for i in $(seq 30); do   # 書き込みの後、ボードはつなぎ直される
    sleep 1
    port=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    [ -n "$port" ] && break
done
py=python3; python3 -c "import serial" 2>/dev/null || py=$(ls ~/.espressif/python_env/*/bin/python 2>/dev/null | head -1)
$py - "$port" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=1)
t0 = time.time()
while time.time() - t0 < 300:
    line = s.readline().decode(errors="replace").rstrip()
    if line:
        print(line, flush=True)
    if "MRL_TEST_DONE" in line or "too small" in line:
        break
PY
