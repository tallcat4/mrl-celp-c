/* マイコン用のテスト: 音声を符号化してデスクトップと同じフレームか確かめ、続けてそのフレームを復号して PCM のハッシュを確かめる。
 * 1 フレーム(40 ms)あたりのカウンタのティックを出す。データ・カウンタ(read_ticks)・printf はプラットフォームが用意する。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <stdio.h>
#include <string.h>

#include "mcu_test.h"
#include "mrlcelp_enc.h"

static double enc_mem[10240 / sizeof(double)];
static double dec_mem[1024 / sizeof(double)];

#ifdef MRL_PROFILE
extern void (*mrl_prof_hook)(int stage);
static const char *const STAGE[] = {"lpc+lsf analysis", "lsf quantiser", "open-loop pitch", "target+impulse", "pitch search",
                                    "codebook shapes", "codebook alt", "codebook gains", "state update",
                                    "  shapes: g_k", "  shapes: corr", "  shapes: cross", "  shapes: combos", "  alt: phi",
                                    "  alt: rows", "  pitch: backward", "  pitch: integer", "  pitch: frac", "  lsf: stage1",
                                    "  lsf: stage2"};
static uint32_t (*prof_ticks)(void), prof_last;
static uint64_t prof_acc[20];
static void prof_mark(int stage) {
    uint32_t now = prof_ticks();
    prof_acc[stage] += now - prof_last;
    prof_last = now;
}
#endif

static uint32_t fnv(uint32_t h, const void *p, size_t n) {
    const uint8_t *b = (const uint8_t *)p;
    while (n--) h = (h ^ *b++) * 16777619u;
    return h;
}

#ifdef MRL_MICRO
#include "core.h"
static void micro_dec(uint32_t (*read_ticks)(void)) {   /* デコーダの部品ごとのティック(100 回の平均) */
    static mrl_state st;
    int16_t lpc[11], lsp[10], code[80], y[80], blk[160];
    uint32_t c;
    int k, pg, cg;
    mrl_lag lag = {45, 1};
    mrl_state_init(&st);
    for (k = 0; k < 314; k++) st.excitation[k] = (int16_t)((k * 7919) % 6000 - 3000);
    mrl_lsp_to_lpc(st.previous_lsp, lpc);
#define T(name, expr) c = read_ticks(); for (k = 0; k < 100; k++) { expr; } printf("micro: %-16s %lu ticks\n", name, (unsigned long)((read_ticks() - c) / 100));
    T("predict", mrl_predict(st.excitation, 154, lag));
    T("reflection", (void)mrl_reflection_first(lpc));
    T("lsf_to_lsp", mrl_lsf_to_lsp((const int16_t *)st.previous_lsp, lsp));
    T("lsp_to_lpc", mrl_lsp_to_lpc(st.previous_lsp, lpc));
    T("codebook", (void)mrl_codebook_vector(20000, code));
    T("sharpen", mrl_sharpen(code, lag, 8000));
    T("gain_decode", mrl_gain_decode(st.gain_history, 60, code, &pg, &cg));
    T("mix", mrl_mix(st.excitation, 154, code, 8000, 3000));
    T("synthesis", mrl_synthesis(lpc, st.excitation + 154, st.synthesis_memory, y, 80));
    for (k = 0; k < 160; k++) blk[k] = st.excitation[k];
    T("output_filter160", mrl_output_filter(st.fir, st.iir, blk, 160));
#undef T
}
static float fa[80], fb[80];
static volatile float sink;
static float dot4(const float *a, const float *b, int n) {
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    int i;
    for (i = 0; i + 4 <= n; i += 4) {
        s0 += a[i] * b[i];
        s1 += a[i + 1] * b[i + 1];
        s2 += a[i + 2] * b[i + 2];
        s3 += a[i + 3] * b[i + 3];
    }
    return (s0 + s1) + (s2 + s3);
}
#if defined(__XTENSA__)
/* lsip(読み込みとポインタの加算)と madd.s で、累積 4 本 */
static float dot_asm(const float *a, const float *b, int n) {
    float s0, s1, s2, s3, t0, t1, t2, t3, u0, u1, u2, u3;
    int k = n >> 2;
    __asm__ volatile(
        "wfr %[s0], %[zero]\n wfr %[s1], %[zero]\n wfr %[s2], %[zero]\n wfr %[s3], %[zero]\n"
        "loopnez %[k], 1f\n"
        "lsi %[t0], %[a], 0\n lsi %[u0], %[b], 0\n"
        "lsi %[t1], %[a], 4\n lsi %[u1], %[b], 4\n"
        "madd.s %[s0], %[t0], %[u0]\n"
        "lsi %[t2], %[a], 8\n lsi %[u2], %[b], 8\n"
        "madd.s %[s1], %[t1], %[u1]\n"
        "lsi %[t3], %[a], 12\n lsi %[u3], %[b], 12\n"
        "madd.s %[s2], %[t2], %[u2]\n"
        "addi %[a], %[a], 16\n addi %[b], %[b], 16\n"
        "madd.s %[s3], %[t3], %[u3]\n"
        "1:\n"
        : [s0] "=&f"(s0), [s1] "=&f"(s1), [s2] "=&f"(s2), [s3] "=&f"(s3), [t0] "=&f"(t0), [t1] "=&f"(t1), [t2] "=&f"(t2),
          [t3] "=&f"(t3), [u0] "=&f"(u0), [u1] "=&f"(u1), [u2] "=&f"(u2), [u3] "=&f"(u3), [a] "+r"(a), [b] "+r"(b), [k] "+r"(k)
        : [zero] "r"(0)
        : "memory");
    return (s0 + s1) + (s2 + s3);
}
#endif

static void micro(uint32_t (*read_ticks)(void)) {
    uint32_t c;
    int i, k;
    float s = 0;
    int32_t ia[80], ib[80];
    int64_t acc64 = 0;
    for (i = 0; i < 80; i++) fa[i] = fb[i] = 0.01f * i, ia[i] = ib[i] = i * 300;
    c = read_ticks();
    for (k = 0; k < 1000; k++) s += dot4(fa, fb, 80);
    c = read_ticks() - c;
    sink = s;
    printf("micro: float dot80 %lu ticks (%.2f per MAC)\n", (unsigned long)(c / 1000), c / 80000.0);
#if defined(__XTENSA__)
    c = read_ticks();
    s = 0;
    for (k = 0; k < 1000; k++) s += dot_asm(fa, fb, 80);
    c = read_ticks() - c;
    printf("micro: asm dot80 %lu ticks (%.2f per MAC), equal %d\n", (unsigned long)(c / 1000), c / 80000.0, dot_asm(fa, fb, 80) == dot4(fa, fb, 80));
#endif
    c = read_ticks();
    for (k = 0; k < 1000; k++)
        for (i = 0; i < 80; i++) acc64 += (int64_t)ia[i] * ib[i];
    c = read_ticks() - c;
    sink = (float)acc64;
    printf("micro: int64 MAC80 %lu ticks (%.2f per MAC)\n", (unsigned long)(c / 1000), c / 80000.0);
    c = read_ticks();
    {
        int32_t a32 = 0;
        for (k = 0; k < 1000; k++)
            for (i = 0; i < 80; i++) a32 += ia[i] * ib[i];
        sink = (float)a32;
    }
    c = read_ticks() - c;
    printf("micro: int32 MAC80 %lu ticks (%.2f per MAC)\n", (unsigned long)(c / 1000), c / 80000.0);
}
#endif

void mrl_mcu_test(const uint8_t *pcm, long bytes, const uint8_t *expect, long expect_bytes, uint32_t expect_hash,
                  uint32_t (*read_ticks)(void)) {
    long k, n = bytes / 640, out = 0, same = 0;
    uint32_t c, cmax = 0, h = 2166136261u;
    uint64_t total = 0;
    uint8_t fr[20];
    int16_t x[320];
    mrl_encoder *e;
    mrl_decoder *d;
    c = read_ticks();
    for (k = 0; k < 1000; k++) __asm__ volatile(".rept 100\n nop\n .endr");
    printf("calibration: %lu ticks for 100000 nops\n", (unsigned long)(read_ticks() - c));
#ifdef MRL_MICRO
    micro(read_ticks);
    micro_dec(read_ticks);
#endif
    printf("encoder state %u bytes, decoder state %u bytes\n", (unsigned)mrl_encoder_size(), (unsigned)mrl_decoder_size());
    if (mrl_encoder_size() > sizeof enc_mem || mrl_decoder_size() > sizeof dec_mem) {
        printf("state buffer too small\n");
        return;
    }
    e = mrl_encoder_init(enc_mem, NULL);
    for (k = 0; k <= n; k++) {
        int got;
        if (k < n) memcpy(x, pcm + 640 * k, 640);
        c = read_ticks();
#ifdef MRL_PROFILE
        prof_ticks = read_ticks;
        prof_last = c;
        mrl_prof_hook = prof_mark;
#endif
        got = k < n ? mrl_encode_push(e, x, fr) : mrl_encode_flush(e, fr);
        c = read_ticks() - c;
        total += c;
        if (c > cmax) cmax = c;
        if (got) {
            if (20 * out + 20 <= expect_bytes) same += !memcmp(fr, expect + 20 * out, 20);
            out++;
        }
    }
    printf("encoder: %ld/%ld frames identical to the desktop ones; ticks/frame mean %lu max %lu\n", same, out,
           (unsigned long)(total / (uint64_t)n), (unsigned long)cmax);
#ifdef MRL_PROFILE
    for (k = 0; k < 20; k++) printf("  %-18s %9lu ticks/frame\n", STAGE[k], (unsigned long)(prof_acc[k] / (uint64_t)n));
    mrl_prof_hook = 0;
#endif
    d = mrl_decoder_init(dec_mem);
    total = cmax = 0;
    for (k = 0; k < expect_bytes / 20; k++) {
        int16_t y[320];
        c = read_ticks();
        mrl_decode(d, expect + 20 * k, y);
        c = read_ticks() - c;
        total += c;
        if (c > cmax) cmax = c;
        h = fnv(h, y, sizeof y);
    }
    printf("decoder: PCM hash %08lx (%s); ticks/frame mean %lu max %lu\n", (unsigned long)h, h == expect_hash ? "matches" : "DIFFERS",
           (unsigned long)(total / (uint64_t)(expect_bytes / 20)), (unsigned long)cmax);
    printf("MRL_TEST_DONE\n");
}
