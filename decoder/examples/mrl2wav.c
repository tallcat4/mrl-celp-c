/* ペイロード(20 byte ずつ)→ 8 kHz・16 bit・モノラルの WAV(出力の名前が .raw なら見出しなしの PCM)。
 * 使い方: mrl2wav in.payload out.wav [--pre](出力フィルタの前の合成) [--bench]
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "mrlcelp.h"

int main(int argc, char **argv) {
    static double mem[1024];
    FILE *in, *out;
    uint8_t p[20];
    int16_t pcm[320], pre[320];
    int want_pre = 0, bench = 0, wav = 0, i;
    long n = 0;
    clock_t t0;
    mrl_decoder *d;
    if (argc < 3) return 2;
    for (i = 3; i < argc; i++) {
        want_pre |= !strcmp(argv[i], "--pre");
        bench |= !strcmp(argv[i], "--bench");
    }
    in = fopen(argv[1], "rb");
    out = fopen(argv[2], "wb");
    if (!in || !out) return 2;
    if (!strstr(argv[2], ".raw")) {   /* WAV の見出し(長さは最後に書き直す) */
        static const uint8_t head[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 1, 0,
                                         0x40, 0x1F, 0, 0, 0x80, 0x3E, 0, 0, 2, 0, 16, 0, 'd', 'a', 't', 'a', 0, 0, 0, 0};
        fwrite(head, 1, 44, out);
        wav = 1;
    }
    d = mrl_decoder_init(mem);
    t0 = clock();
    while (fread(p, 1, 20, in) == 20) {
        if (bench) {   /* 100 回ずつ(状態は複製して) */
            static double tmp[1024];
            int k;
            for (k = 0; k < 100; k++) {
                memcpy(tmp, mem, mrl_decoder_size());
                mrl_decode((mrl_decoder *)tmp, p, pcm);
            }
        }
        mrl_decode_ex(d, p, pcm, pre);
        fwrite(want_pre ? pre : pcm, 2, 320, out);
        n++;
    }
    if (bench) fprintf(stderr, "%.1f us/frame\n", (double)(clock() - t0) / CLOCKS_PER_SEC / (n * 101) * 1e6);
    if (wav) {
        uint32_t data = (uint32_t)n * 640, riff = data + 36;
        uint8_t b[4];
        int k;
        fseek(out, 4, SEEK_SET);
        for (k = 0; k < 4; k++) b[k] = (uint8_t)(riff >> (8 * k));
        fwrite(b, 1, 4, out);
        fseek(out, 40, SEEK_SET);
        for (k = 0; k < 4; k++) b[k] = (uint8_t)(data >> (8 * k));
        fwrite(b, 1, 4, out);
    }
    fclose(out);
    return 0;
}
