/* 原本(jr-voice の Rust 版)の参照ベクトル(tests/vectors/celp_reference.txt)とビット一致すること。
 * 使い方: test_vectors tests/vectors/celp_reference.txt
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mrlcelp.h"

static int hexbyte(const char *s) {
    int v;
    sscanf(s, "%2x", &v);
    return v;
}

int main(int argc, char **argv) {
    static char line[4096];
    static double mem[1024];
    FILE *f = fopen(argc > 1 ? argv[1] : "tests/vectors/celp_reference.txt", "r");
    mrl_decoder *d = NULL;
    long frames = 0, bad = 0;
    int maxdiff = 0;
    if (!f || mrl_decoder_size() > sizeof mem) return 2;
    while (fgets(line, sizeof line, f)) {
        uint8_t payload[20];
        int16_t pcm[320];
        char *sp;
        int i, diff = 0;
        if (!strncmp(line, "reset", 5)) {
            d = mrl_decoder_init(mem);
            continue;
        }
        sp = strchr(line, ' ');
        for (i = 0; i < 20; i++) payload[i] = (uint8_t)hexbyte(line + 2 * i);
        mrl_decode(d, payload, pcm);
        for (i = 0; i < 320; i++) {
            int want = (int16_t)(hexbyte(sp + 1 + 4 * i) | hexbyte(sp + 3 + 4 * i) << 8), dd = abs(want - pcm[i]);
            if (dd > diff) diff = dd;
        }
        if (diff > maxdiff) maxdiff = diff;
        bad += diff != 0;
        frames++;
    }
    printf("%ld frames, %ld differ, max |diff| %d (decoder state %zu bytes)\n", frames, bad, maxdiff, mrl_decoder_size());
    return bad != 0;
}
