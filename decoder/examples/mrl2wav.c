/* ペイロード(20 byte ずつ)→ 8 kHz・16 bit・モノラルの WAV(出力の名前が .raw なら見出しなしの PCM)。
 * 使い方: mrl2wav in.payload out.wav [--pre](出力フィルタの前の合成) [--bench]
 *         mrl2wav in.payload out.wav --mpf   三菱 CELP のポストフィルタを入れた復号(固定小数点)
 *         mrl2wav in.payload out.wav --enhance [--no-postfilter] [--no-antisparse] [--antisparse-all] [--no-limiter]
 *                 [--gain=0.5] [--lowcut=HZ] [--level[=RMS]] [--no-pitchenh] [--no-noiseenh] [--no-bass]
 *                 [--postfilter=g729|mpf] [--formant=0.55,0.7](短期ポストフィルタの γn,γd。差が大きいほどフォルマントが強調される)
 *                 聞きやすさ優先の復号(ビット一致ではない)
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "mrlcelp.h"

int main(int argc, char **argv) {
    static double mem[1024], emem[2048], mmem[1024];
    mrl_mpf_decoder *md = NULL;
    mrl_enh_config cfg;
    mrl_enhanced *e = NULL;
    int enhance = 0;
    FILE *in, *out;
    uint8_t p[20];
    int16_t pcm[320], pre[320];
    int want_pre = 0, bench = 0, wav = 0, i;
    long n = 0;
    clock_t t0;
    mrl_decoder *d;
    if (argc < 3) return 2;
    mrl_enh_default_config(&cfg);
    for (i = 3; i < argc; i++) {
        want_pre |= !strcmp(argv[i], "--pre");
        bench |= !strcmp(argv[i], "--bench");
        enhance |= !strcmp(argv[i], "--enhance");
        if (!strcmp(argv[i], "--mpf")) md = (mrl_mpf_decoder *)mmem;
        if (!strcmp(argv[i], "--no-postfilter")) cfg.postfilter = 0;
        if (!strcmp(argv[i], "--postfilter=mpf")) cfg.postfilter = 2;
        if (!strcmp(argv[i], "--postfilter=g729")) cfg.postfilter = 1;
        if (!strcmp(argv[i], "--no-antisparse")) cfg.anti_sparse = 0;
        if (!strcmp(argv[i], "--antisparse-all")) cfg.anti_sparse = 1;
        if (!strcmp(argv[i], "--no-limiter")) cfg.limiter = 0;
        if (!strncmp(argv[i], "--gain=", 7)) cfg.gain = (float)atof(argv[i] + 7);
        if (!strncmp(argv[i], "--lowcut=", 9)) cfg.low_cut_hz = (float)atof(argv[i] + 9);
        if (!strcmp(argv[i], "--level")) cfg.level_rms = 2000.0f;
        if (!strcmp(argv[i], "--no-pitchenh")) cfg.pitch_enhance = 0;
        if (!strcmp(argv[i], "--no-noiseenh")) cfg.noise_enhance = 0;
        if (!strcmp(argv[i], "--no-bass")) cfg.bass_postfilter = 0;
        if (!strncmp(argv[i], "--formant=", 10)) sscanf(argv[i] + 10, "%f,%f", &cfg.gamma_n, &cfg.gamma_d);
        if (!strncmp(argv[i], "--level=", 8)) cfg.level_rms = (float)atof(argv[i] + 8);
    }
    if (md) {
        if (mrl_mpf_decoder_size() > sizeof mmem) return 2;
        md = mrl_mpf_decoder_init(mmem);
    }
    if (enhance) {
        if (mrl_enhanced_size() > sizeof emem) return 2;
        e = mrl_enhanced_init(emem, &cfg);
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
        if (e) {   /* 20 ms 遅れるので、最初の 160 sample を捨てて最後に flush で足す(入力と時刻をそろえる) */
            mrl_decode_enhanced(e, p, pcm);
            fwrite(n == 0 ? pcm + 160 : pcm, 2, n == 0 ? 160 : 320, out);
        } else if (md) {
            mrl_decode_mpf(md, p, pcm);
            fwrite(pcm, 2, 320, out);
        } else {
            mrl_decode_ex(d, p, pcm, pre);
            fwrite(want_pre ? pre : pcm, 2, 320, out);
        }
        n++;
    }
    if (e && n > 0) {
        mrl_enhanced_flush(e, pcm);
        fwrite(pcm, 2, 160, out);
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
