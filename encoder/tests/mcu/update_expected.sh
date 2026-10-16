#!/bin/bash
# デスクトップの C 版で、マイコンのテストの期待値(tests/vectors/speech.expected.mrl と、復号した PCM のハッシュ)を作り直す
set -e
cd "$(dirname "$0")/../.."
make -s wav2mrl ../decoder/mrl2wav
./wav2mrl tests/vectors/speech.pcm tests/vectors/speech.expected.mrl
../decoder/mrl2wav tests/vectors/speech.expected.mrl /tmp/mrl_expected.raw
h=$(python3 -c "
d=open('/tmp/mrl_expected.raw','rb').read(); h=2166136261
for b in d: h=((h^b)*16777619)&0xffffffff
print('0x%08x' % h)")
printf '/* デスクトップの C 版で復号した PCM のハッシュ(tests/mcu/update_expected.sh が作る) */\n#define MRL_EXPECT_HASH %su\n' "$h" > tests/mcu/expect_hash.h
echo "hash $h"
