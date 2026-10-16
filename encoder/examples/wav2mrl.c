/* 8 kHz・16 bit・モノラルの WAV(または生の PCM)→ ペイロード(20 byte ずつ)。使い方: wav2mrl in.wav out.payload */
/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "mrlcelp_enc.h"

int main(int argc, char **argv) {
    static double mem[8192];
    FILE *in, *out;
    int16_t pcm[320];
    uint8_t p[20];
    long frames = 0;
    size_t got;
    clock_t t0, used = 0;
    char head[44];
    mrl_encoder *e;
    float te, ee;
    if (argc < 3 || mrl_encoder_size() > sizeof mem) return 2;
    in = fopen(argv[1], "rb");
    out = fopen(argv[2], "wb");
    if (!in || !out) return 2;
    if (strstr(argv[1], ".wav") && fread(head, 1, 44, in) != 44) return 2;   /* 44 byte の素の WAV の見出しを飛ばす */
    e = mrl_encoder_init(mem, NULL);
    while ((got = fread(pcm, 2, 320, in)) > 0) {
        if (got < 320) memset(pcm + got, 0, 2 * (320 - got));
        t0 = clock();
        if (mrl_encode_push(e, pcm, p)) fwrite(p, 1, 20, out), frames++;
        used += clock() - t0;
    }
    t0 = clock();
    if (mrl_encode_flush(e, p)) fwrite(p, 1, 20, out), frames++;
    used += clock() - t0;
    mrl_encoder_stats(e, &te, &ee);
    fprintf(stderr, "%ld frames, synthesis SNR %.2f dB, %.1f us/frame (state %zu bytes)\n", frames, 10 * log10f(te / ee),
            (double)used / CLOCKS_PER_SEC / frames * 1e6, mrl_encoder_size());
    fclose(out);
    return 0;
}
