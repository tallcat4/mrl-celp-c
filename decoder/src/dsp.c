/* 固定小数点の DSP 部品とデコーダの状態遷移。研究用の Python 版(mrl-celp の src/mrlcelp/dsp.py・decoder.py)と同じ式(原本: jr-voice src/celp)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#include <math.h>
#include <string.h>

#include "core.h"
#include "fixed.h"
#include "tables.h"

/* ---------------------------------------------------------------- 除算・正規化 */

static int64_t nexp(int64_t v) {   /* bit 31 に正規化する左シフト量 */
    int64_t mag;
    int top = -1;
    v = acc(v);
    if (v == 0) return 0;
    mag = v < 0 ? ~v : v;
    while (mag) {
        mag >>= 1;
        top++;
    }
    return 30 - top;
}

static int64_t restoring_divide(int64_t num, int64_t den, int steps) {
    int i;
    for (i = 0; i < steps; i++) {
        int64_t alu = num - shl(den, 15);
        num = alu >= 0 ? acc(shl(alu, 1) + 1) : acc(shl(num, 1));
    }
    return num;
}

static int64_t normalised_reciprocal(int64_t value, int *shift_out) {
    int64_t e = nexp(value), normalised, m, q;
    normalised = shift(value, (int)e);
    m = normalised < 0 ? -normalised : normalised;
    q = restoring_divide(shl(16383, 16), wrap16(sat(m) >> 16), 15);
    q = acc(shl(q & 0xFFFF, 16));
    *shift_out = (int)e + 1;
    return hi(normalised) < 0 ? acc(-q) : q;
}

/* ---------------------------------------------------------------- LSP / LPC */

void mrl_lsf_to_lsp(const int16_t lsf[10], int16_t lsp[10]) {
    int i;
    for (i = 0; i < 10; i++) {
        int64_t scaled = (int64_t)lsf[i] * 20861 * 2;
        int64_t s8 = shift(scaled, -8), cap = (int64_t)63 << 16;
        int64_t index = hi(s8 < cap ? s8 : cap);
        int64_t frac = hi(scaled) & 0xFF;
        lsp[i] = (int16_t)hi(acc(shl(MRL_COS[index], 16) + shl(frac, 3) * MRL_COS_SLOPE[index] * 2));
    }
}

static void lsp_polynomial(const int16_t lsp[10], int first, int64_t f[6]) {
    int i, j;
    f[0] = (int64_t)1 << 24;
    f[1] = trunc32(-shl(lsp[first], 10));
    for (i = 2; i < 6; i++) {
        int64_t l = wrap16(-(int64_t)lsp[first + 2 * (i - 1)]);
        f[i] = trunc32(acc(2 * f[i - 2] + shl(mul32x16(f[i - 1], l), 1)));
        for (j = i - 1; j > 1; j--) f[j] = trunc32(acc(f[j] + shl(mul32x16(f[j - 1], l), 1) + f[j - 2]));
        f[1] = trunc32(acc(f[1] + shl(l, 10)));
    }
}

void mrl_lsp_to_lpc(const int16_t lsp[10], int16_t a[11]) {
    int64_t f1[6], f2[6];
    int i;
    lsp_polynomial(lsp, 0, f1);
    lsp_polynomial(lsp, 1, f2);
    for (i = 5; i > 0; i--) {
        f1[i] = trunc32(acc(f1[i] + f1[i - 1]));
        f2[i] = trunc32(acc(f2[i] - f2[i - 1]));
    }
    a[0] = 4096;
    for (i = 1; i < 6; i++) {
        a[i] = (int16_t)hi(acc(shift(shift(acc(f1[i] + f2[i]), -1), 4) + (1 << 15)));
        a[11 - i] = (int16_t)hi(acc(shift(shift(acc(f1[i] - f2[i]), -1), 4) + (1 << 15)));
    }
}

void mrl_midpoint(const int16_t a[10], const int16_t b[10], int16_t out[10]) {
    int i;
    for (i = 0; i < 10; i++) out[i] = (int16_t)hi(acc(shl((int64_t)a[i] + b[i], 15)));
}

int mrl_reflection_first(const int16_t lpc[11]) {
    int64_t k[10];
    int step, i;
    for (i = 0; i < 10; i++) k[i] = lpc[i + 1];
    for (step = 0; step < 9; step++) {
        int order = 10 - step, top = order - 1, p, scale, exponent;
        int64_t kv = k[top];
        int64_t residual = acc(shl(16384, 16) - shift(acc(kv * kv * 2), 5));
        int64_t quotient = normalised_reciprocal(residual, &exponent), inverse;
        scale = exponent & 31;
        if (scale >= 16) scale -= 32;
        inverse = hi(shift(quotient, -1));
        for (p = 0; p < ((order - 2) >> 1) + 1; p++) {
            int ii = p, jj = top - 1 - p;
            int64_t lo = acc(shl(k[ii], 16) - shift(acc(k[jj] * kv * 2), 3));
            int64_t up_low = hi(shift(mul32x16(sat(lo), inverse), scale));
            int64_t hh = acc(shl(k[jj], 16) - shift(acc(kv * k[ii] * 2), 3));
            int64_t up_high = hi(shift(mul32x16(sat(hh), inverse), scale));
            k[ii] = up_low;
            k[jj] = up_high;
        }
    }
    return (int)k[0];
}

/* ---------------------------------------------------------------- ピッチ */

mrl_lag mrl_decode_absolute(int code) {
    mrl_lag l;
    int q;
    if (code >= 197) {
        l.integer = code - 112;
        l.frac = 0;
        return l;
    }
    q = (int)wrap16(restoring_divide(code + 2, 3, 16));
    l.integer = q + 19;
    l.frac = code - (int)wrap16(((int64_t)l.integer * 3 * 2) >> 1) + 58;
    return l;
}

mrl_lag mrl_decode_relative(int code, int prev) {
    mrl_lag l;
    int lo = prev - 5 > 20 ? prev - 5 : 20, q;
    if (lo + 9 > 143) lo = 134;
    q = (int)wrap16(restoring_divide(code + 2, 3, 16)) - 1;
    l.integer = q + lo;
    l.frac = code - 2 - (int)wrap16(((int64_t)q * 3 * 2) >> 1);
    return l;
}

/* 16 bit × 16 bit の積の和の 2 倍(小数乗算)。40 bit の累算器の巻き戻しは、和がその範囲に収まる所(積 |x·c·2| < 2^31、
 * 20 項で 2^36 未満)では起きないので省く。ESP32 などの Xtensa(MRL_MAC16)では MAC16 の 40 bit 累算器を使う。 */
#if defined(__XTENSA__) && defined(MRL_MAC16)
static inline int64_t dot2(const int16_t *x, const int16_t *c, int n) {
    int32_t t0, t1, hi_;
    uint32_t lo_;
    __asm__ volatile(
        "movi %[t0], 0\n wsr %[t0], acclo\n wsr %[t0], acchi\n"
        "loopnez %[n], 1f\n"
        "l16si %[t0], %[x], 0\n l16si %[t1], %[c], 0\n addi %[x], %[x], 2\n addi %[c], %[c], 2\n mula.aa.ll %[t0], %[t1]\n"
        "1:\n rsr %[lo], acclo\n rsr %[hi], acchi\n"
        : [t0] "=&r"(t0), [t1] "=&r"(t1), [x] "+r"(x), [c] "+r"(c), [n] "+r"(n), [lo] "=&r"(lo_), [hi] "=&r"(hi_)
        :
        : "memory");
    return (int64_t)(((uint64_t)(int64_t)(int8_t)hi_ << 32) | lo_) * 2;
}
#else
static inline int64_t dot2(const int16_t *x, const int16_t *c, int n) {
    int64_t v = 0;
    int k;
    for (k = 0; k < n; k++) v += (int32_t)(x[k] * c[k]);
    return v * 2;
}
#endif

/* 補間フィルタを、窓 x[c − 9 .. c + 10] に掛ける 20 タップの形に並べ直したもの(位相ごと) */
static int16_t TAPS[3][20];
static int taps_ready;
static void init_taps(void) {
    int ph, j;
    if (taps_ready) return;
    for (ph = 0; ph < 3; ph++)
        for (j = 0; j < 10; j++) {
            TAPS[ph][j] = MRL_PITCH_INTERP[ph + 3 * (9 - j)];
            TAPS[ph][10 + j] = MRL_PITCH_INTERP[3 - ph + 3 * j];
        }
    taps_ready = 1;
}

static inline int64_t interp(const int16_t *sig, int centre, int phase) { return dot2(sig + centre - 9, TAPS[phase], 20); }

void mrl_predict(int16_t *exc, int at, mrl_lag lag) {
    int phase = -lag.frac, delay = lag.integer, n;
    if (phase < 0) {
        phase += 3;
        delay++;
    }
    for (n = 0; n < 80; n++) exc[at + n] = (int16_t)hi(sat(acc(interp(exc, at + n - delay, phase) + 0x8000)));
}

void mrl_sharpen(int16_t code[80], mrl_lag lag, int gain) {
    int first = lag.integer - 10, phase = -lag.frac, n;
    int16_t buf[101];
    if (phase < 0) {
        phase += 3;
        first++;
    }
    if (first >= 80) return;
    memset(buf, 0, sizeof buf);
    memcpy(buf + 21, code, sizeof(int16_t) * first);
    for (n = first; n < 80; n++) {
        int64_t scaled = mul32x16(interp(buf, 21 + (n - first) - 10, phase), gain);
        code[n] = (int16_t)hi(acc(shift(scaled, 1) + shl(code[n], 16) + (1 << 15)));
        buf[21 + n] = code[n];
    }
}

/* ---------------------------------------------------------------- 固定符号帳 */

int mrl_codebook_vector(int index, int16_t out[80]) {
    int cls = 0, k, s, pulses;
    int64_t rest, value, signs[3] = {0}, positions[3] = {0};
    if (index > MRL_FCB_INDEX_LIMIT[0] - 1) index = MRL_FCB_INDEX_LIMIT[0] - 1;
    for (k = 5; k >= 0; k--)
        if (index >= MRL_FCB_CLASS_BASE[k]) {
            cls = k;
            break;
        }
    memset(out, 0, sizeof(int16_t) * 80);
    if (cls == 5) {
        int t;
        rest = index - MRL_FCB_CLASS_BASE[5];
        for (t = 0; t < 2; t++) {
            const int16_t *base = t ? MRL_FCB_ALT_B : MRL_FCB_ALT_A;
            int offset = t ? (int)(rest - ((rest >> 7) << 7)) : (int)(rest >> 7);
            for (k = 0; k < 5; k++) {
                int e = base[5 * offset + k], pos = (e < 0 ? -e : e) - 1;
                int64_t x = shl(out[pos], 16), amp = (int64_t)8192 << 16;
                out[pos] = (int16_t)(e <= 0 ? hi(acc(x - amp)) : hi(acc(x + amp)));
            }
        }
        return cls;
    }
    pulses = MRL_FCB_PULSES[cls];
    rest = index - MRL_FCB_CLASS_BASE[cls];
    for (s = 0; s < pulses; s++) {
        signs[s] = rest & 1;
        rest = shift(rest, -1);
    }
    value = shl(rest, 16);
    for (k = pulses - 1; k >= 0; k--) {
        int64_t recip = MRL_FCB_RADIX_INV[3 * cls + k], radix = MRL_FCB_RADIX[3 * cls + k];
        int64_t quotient = hi(shift(acc(hi(value) * recip * 2), -2));
        value = acc(value - shl(quotient * radix * 2, 15));
        positions[k] = hi(value);
        value = shl(quotient, 16);
    }
    {
        int first_shape = MRL_FCB_SHAPE_SEL[3 * cls], n;
        for (k = 0; k < pulses; k++) {
            int pos = MRL_FCB_POSITIONS[40 * (3 * cls + k) + positions[k]];
            for (n = pos; n < 80 && n < pos + 20; n++) {
                int64_t x = shl(out[n], 16), y = shl(MRL_FCB_SHAPES[20 * (first_shape + k) + n - pos], 16);
                out[n] = (int16_t)(signs[k] ? hi(acc(x + y)) : hi(acc(x - y)));
            }
        }
    }
    return cls;
}

/* ---------------------------------------------------------------- 周期強調の利得・合成・励振・出力フィルタ */

void mrl_interpolation_gains(int reflection, int previous_gain, int lag, int previous_lag, int *first, int *second) {
    const int64_t HALF = 16384, WAS_STRONG = 13107, SAME = 24576;
    int peaky = acc(shift(shl(reflection, 16), 3) - shl(HALF, 16)) > 0;
    int strong = acc(shift(shl(previous_gain, 16), 1) - shl(WAS_STRONG, 16)) > 0;
    int above = acc(shl(lag, 16) - acc(SAME * previous_lag * 2)) > 0;
    int below = acc(shift(acc((int64_t)previous_lag * SAME * 2), 1) - shl(lag, 16)) > 0;
    *first = (int)HALF;
    *second = peaky ? 0 : (strong && above && below ? (int)HALF : previous_gain);
}

void mrl_synthesis(const int16_t a[11], const int16_t *in, int16_t mem[10], int16_t *out, int n) {
    int16_t window[10 + 160], ar[10];
    int i, j;
    for (j = 0; j < 10; j++) ar[j] = a[10 - j];   /* window[i + m] に a[10 − m] を掛ける */
    memcpy(window, mem, sizeof(int16_t) * 10);
    for (i = 0; i < n; i++) {
        int64_t s = -dot2(window + i, ar, 10);   /* 10 項、各 |a·y·2| < 2^31 なので 40 bit を超えない */
        window[10 + i] = out[i] = (int16_t)hi(sat(acc(shift(s, 3) + shl(in[i], 16) + (1 << 15))));
    }
    memcpy(mem, window + n, sizeof(int16_t) * 10);
    (void)j;
}

void mrl_mix(int16_t *exc, int at, const int16_t code[80], int pitch_gain, int code_gain) {
    int n;
    for (n = 0; n < 80; n++) {
        int64_t adaptive = (int64_t)(exc[at + n] * pitch_gain) * 2, fixed = (int64_t)(code[n] * code_gain) * 2;
        exc[at + n] = (int16_t)hi(sat(acc(shift(acc(adaptive + shift(fixed, 1)), 1) + 0x8000)));
    }
}

/* 出力フィルタ。途中の値は |b| < 2^35、shift(b, 2) < 2^37 なので 40 bit の巻き戻しは起きない(32 bit への切り詰めと飽和は残す) */
void mrl_output_filter(int32_t fir[6], int32_t iir[6], int16_t *block, int n) {
    int i, section, k;
    for (i = 0; i < n; i++) {
        int64_t v = block[i];
        for (section = 0; section < 2; section++) {
            int at = section * 3;
            int64_t b = (int64_t)((int32_t)v * MRL_OUT_FIR[0]) * 2;   /* 積は 32 bit に収まる(fir は 16 bit の値) */
            for (k = 0; k < 3; k++) b += (int64_t)(fir[at + k] * MRL_OUT_FIR[k + 1]) * 2;
            fir[at + 2] = fir[at + 1];
            fir[at + 1] = fir[at];
            fir[at] = (int32_t)v;
            for (k = 0; k < 3; k++) b -= mul32x16(iir[at + k], MRL_OUT_IIR[k]) * 2;
            iir[at + 2] = iir[at + 1];
            iir[at + 1] = iir[at];
            iir[at] = (int32_t)trunc32(b * 2);
            v = hi(sat(b * 4 + (1 << 15)));
        }
        block[i] = (int16_t)hi(sat((int64_t)((int32_t)v * 19661) * 4 + (1 << 15)));
    }
}

/* ---------------------------------------------------------------- ゲイン(f32、decoder.GainState) */

int mrl_quantize(float x) {
    float r = rintf(x);   /* 偶数への丸め(既定の丸めの向き) */
    return r < 0 ? 0 : (r > 32767 ? 32767 : (int)r);
}

float mrl_code_energy(const int16_t code[80]) {
    int64_t s = 0;
    int i;
    for (i = 0; i < 80; i++) s += (int64_t)code[i] * code[i];
    return (float)s / 80.0f;
}

float mrl_gain_prediction(const float h[4], float energy) {
    float s = h[0] * 0.68f;
    s = s + h[1] * 0.5f;
    s = s + h[2] * 0.33f;
    s = s + h[3] * 0.16f;
    return (s - 0.5f * logf(energy > 1.0f ? energy : 1.0f)) + MRL_GAIN_BIAS1[0];
}

void mrl_gain_decode(float history[4], int index, const int16_t code[80], int *pitch_gain, int *code_gain) {
    float prediction = mrl_gain_prediction(history, mrl_code_energy(code)), correction = MRL_LOG_GAIN[index];
    *pitch_gain = mrl_quantize(MRL_PITCH_GAIN[index]);
    *code_gain = mrl_quantize(expf(prediction + correction));
    history[3] = history[2];
    history[2] = history[1];
    history[1] = history[0];
    history[0] = correction;
}

/* ---------------------------------------------------------------- スペクトル(decoder.Spectral) */

void mrl_lsf_decode(const float history[3][10], const mrl_half *h, int16_t lsf[10], float residual[10]) {
    const float *m = MRL_MA + 40 * h->mode;
    float out[10];
    int k, j, prior = 0;
    for (k = 0; k < 10; k++)
        residual[k] = (float)MRL_LSF_CB1[h->stage1 * 10 + k] +
                      (k < 5 ? MRL_STAGE2_LOW[5 * h->stage2_low + k] : MRL_STAGE2_HIGH[5 * h->stage2_high + k - 5]);
    for (k = 0; k < 10; k++) {
        float v = residual[k] * m[k];
        v = v + history[0][k] * m[10 + k];
        v = v + history[1][k] * m[20 + k];
        out[k] = v + history[2][k] * m[30 + k];
    }
    for (k = 1; k < 10; k++)   /* 挿入整列 */
        for (j = k; j > 0 && out[j - 1] > out[j]; j--) {
            float t = out[j];
            out[j] = out[j - 1];
            out[j - 1] = t;
        }
    if (out[0] < 41.0f) out[0] = 41.0f;
    for (k = 1; k < 10; k++)
        if (out[k] < out[k - 1] + 321.0f) out[k] = out[k - 1] + 321.0f;
    if (out[9] > 25682.0f) out[9] = 25682.0f;
    for (k = 0; k < 10; k++) {   /* 偶数丸め → 単調・範囲 */
        int r = (int)rintf(out[k]);
        if (r < prior) r = prior;
        if (r < 0) r = 0;
        if (r > 25682) r = 25682;
        lsf[k] = (int16_t)(prior = r);
    }
}

void mrl_spectral_push(float history[3][10], const float residual[10]) {
    memmove(history[1], history[0], sizeof(float) * 20);
    memcpy(history[0], residual, sizeof(float) * 10);
}

/* ---------------------------------------------------------------- 状態 */

void mrl_state_init(mrl_state *s) {
    float initial = 0.0f;
    int i, k;
    init_taps();
    memset(s, 0, sizeof *s);
    memcpy(s->previous_lsp, MRL_LSP_MEAN, sizeof s->previous_lsp);
    s->sharpen_fixed = s->sharpen_voiced = 3277;
    s->last_half_lag = s->previous_lag = 60;
    for (i = 0; i < 128; i++) initial = initial + MRL_LOG_GAIN[i];
    for (i = 0; i < 4; i++) s->gain_history[i] = initial / 128.0f;
    for (i = 0; i < 3; i++)
        for (k = 0; k < 10; k++) s->spectral_history[i][k] = (float)MRL_LSF_MEAN[k];
}

void mrl_begin_half(mrl_state *s, const int16_t lsp[10], int16_t lpc0[11], int16_t lpc1[11]) {
    int16_t mid[10];
    mrl_midpoint(s->previous_lsp, lsp, mid);
    mrl_lsp_to_lpc(mid, lpc0);
    mrl_lsp_to_lpc(lsp, lpc1);
    memcpy(s->previous_lsp, lsp, sizeof s->previous_lsp);
}

void mrl_prepare(mrl_state *s, int sub, const int16_t lpc[11], mrl_lag lag) {
    s->previous_lag = lag.integer;
    mrl_predict(s->excitation, 154 + 80 * sub, lag);
    mrl_interpolation_gains(mrl_reflection_first(lpc), s->sharpen_fixed, lag.integer, s->last_half_lag, &s->sharpen_fixed,
                            &s->sharpen_voiced);
}

void mrl_innovation(const mrl_state *s, int code_index, mrl_lag lag, int16_t code[80]) {
    int cls = mrl_codebook_vector(code_index, code);
    if (lag.integer < 80) mrl_sharpen(code, lag, (cls == 5 || (cls >= 1 && cls < 3)) ? s->sharpen_voiced : s->sharpen_fixed);
}

void mrl_finish(mrl_state *s, int sub, const int16_t lpc[11], const int16_t code[80], int pitch_gain, int code_gain,
                int16_t synth[80]) {
    int at = 154 + 80 * sub;
    s->sharpen_fixed = pitch_gain < 3277 ? 3277 : (pitch_gain > 13017 ? 13017 : pitch_gain);
    mrl_mix(s->excitation, at, code, pitch_gain, code_gain);
    mrl_synthesis(lpc, s->excitation + at, s->synthesis_memory, synth, 80);
}

void mrl_finish_half(mrl_state *s, const int16_t synth[160], int16_t pcm[160]) {
    memmove(s->excitation, s->excitation + 160, sizeof(int16_t) * 154);
    s->last_half_lag = s->previous_lag;
    memcpy(pcm, synth, sizeof(int16_t) * 160);
    mrl_output_filter(s->fir, s->iir, pcm, 160);
}
