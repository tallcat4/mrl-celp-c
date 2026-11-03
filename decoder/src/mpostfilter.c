/* 三菱 CELP の適応ポストフィルタ(固定小数点)。mcelp-codec(Rust、MIT、© 2026 KIRISHIKI Yudai)の
 * src/postfilter.rs と src/ltp.rs(commit 1ffc2f5)を C99 に移植したもの。
 * MRL-CELP の出力フィルタは三菱 CELP のものと係数まで同一で、三菱 CELP ではその直前にこのポストフィルタがある。
 *   1. サブフレームの LSF を平らなバイアスへ 2 通りの度合いで寄せ、分子 A(z/g2)・分母 A(z/g1) を作る
 *   2. 合成音声を分子に通して残差 → 長期ポストフィルタ(ラグ ±1、両側 7 位相、16 タップでの作り直し)
 *   3. 分母で合成し直し、利得の正規化・傾きの補正・AGC
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#include <string.h>

#include "core.h"
#include "fixed.h"
#include "mpostfilter.h"
#include "tables.h"

#define ORDER 10
#define SUB 80
#define WINDOW (MRL_MPF_RES_HISTORY + SUB)
#define PHASES 7
#define INTERP_LEN (SUB + 1)

static const int16_t PF_BIAS[10] = {2340, 4679, 7019, 9359, 11698, 14038, 16377, 18717, 21057, 23396};
static const int16_t PF_WEIGHT_DEN = 12780, PF_WEIGHT_NUM = 19988;
static const int16_t ACOS_TAB[64] = {
    -26887, -8812, -5323, -3813, -2979, -2444, -2081, -1811, -1608, -1450, -1322, -1219, -1132, -1059, -998, -946,
    -901,   -861,  -827,  -797,  -772,  -750,  -730,  -713,  -699,  -687,  -677,  -668,  -662,  -657,  -654, -652,
    -652,   -654,  -657,  -662,  -668,  -677,  -687,  -699,  -713,  -730,  -750,  -772,  -797,  -827,  -861, -901,
    -946,   -998,  -1059, -1132, -1219, -1322, -1450, -1608, -1811, -2081, -2444, -2979, -3813, -5323, -8812, -26887};
static const int16_t SEARCH_FILTER[28] = {-189,  2873,  31650, -1598, -485,  7041,  28469, -2148, -934, 12266,
                                          23705, -1993, -1493, 18050, 18050, -1493, -1993, 23705, 12266, -934,
                                          -2148, 28469, 7041,  -485,  -1598, 31650, 2873,  -189};
static const int16_t REFINE_FILTER[112] = {
    -41,   72,   -157, 315,  -580,  1023,  -1875, 4439,  31915, -3391, 1595,  -888,  501,   -267,  130,   -60,
    -78,   147,  -318, 631,  -1151, 2030,  -3774, 9639,  29436, -5580, 2727,  -1528, 859,   -454,  218,   -102,
    -107,  212,  -456, 892,  -1615, 2850,  -5393, 15206, 25569, -6550, 3303,  -1861, 1041,  -544,  258,   -123,
    -124,  253,  -539, 1044, -1877, 3319,  -6414, 20676, 20676, -6414, 3319,  -1877, 1044,  -539,  253,   -124,
    -123,  258,  -544, 1041, -1861, 3303,  -6550, 25569, 15206, -5393, 2850,  -1615, 892,   -456,  212,   -107,
    -102,  218,  -454, 859,  -1528, 2727,  -5580, 29436, 9639,  -3774, 2030,  -1151, 631,   -318,  147,   -78,
    -60,   130,  -267, 501,  -888,  1595,  -3391, 31915, 4439,  -1875, 1023,  -580,  315,   -157,  72,    -41};

/* ---------------------------------------------------------------- 固定小数点の補助(mcelp の fixed.rs) */

static inline int16_t h16(int64_t v) { return (int16_t)(v >> 16); }
static inline int16_t l16(int64_t v) { return (int16_t)(uint16_t)(v & 0xFFFF); }
static inline int64_t fmul(int16_t x, int16_t y) { return (int64_t)x * y * 2; }
static inline int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

static int fexp(int64_t v) {   /* bit 31 に正規化する左シフト量(0 なら 0) */
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

static int64_t rdiv(int64_t num, int16_t den, int steps) {
    int i;
    for (i = 0; i < steps; i++) {
        int64_t alu = num - shl(den, 15);
        num = acc(alu >= 0 ? shl(alu, 1) + 1 : shl(num, 1));
    }
    return num;
}

static int64_t nreciprocal(int64_t value, int *shift_out) {
    int e = fexp(value);
    int64_t normalised = shift(value, e), m, q;
    *shift_out = e + 1;
    m = sat(normalised < 0 ? -normalised : normalised) >> 16;
    q = rdiv(shl(16383, 16), (int16_t)m, 15);
    q = acc(shl(q & 0xFFFF, 16));
    return h16(normalised) < 0 ? acc(-q) : q;
}

static int64_t fdivide(int64_t value, int16_t numerator) {   /* numerator / value */
    int negative = value < 0, e;
    int64_t magnitude = negative ? acc(-value) : value, normalised, product;
    int16_t quotient;
    e = fexp(magnitude);
    normalised = shift(magnitude, e);
    quotient = l16(rdiv(shl(16383, 16), h16(normalised), 15));
    product = acc((int64_t)quotient * numerator * 2);
    return shift(negative ? acc(-product) : product, e + 1);
}

static int64_t fmul32(int64_t x, int64_t y) {   /* ltp.rs の mul32 */
    int64_t xh = h16(x), xl = (uint16_t)l16(x), yh = h16(y), yl = (uint16_t)l16(y), a;
    a = acc(xl * yl);
    a = shift(a, -16);
    a = acc(a + yl * xh * 2);
    a = acc(a + xl * yh * 2);
    a = shift(a, -16);
    return acc(a + xh * yh * 2);
}

/* ---------------------------------------------------------------- LSP / フィルタ */

static void lsp_to_lsf(const int16_t lsp[10], int16_t lsf[10]) {
    int index = 63, i;
    for (i = ORDER - 1; i >= 0; i--) {
        int64_t diff, slope, pos, angle;
        for (;;) {
            diff = (int64_t)lsp[i] - MRL_COS[index];
            if (diff <= 0 || index == 0) break;
            index--;
        }
        slope = ACOS_TAB[index];
        pos = acc(shl(index, 25) + shl(diff * slope * 2, 4));
        angle = acc((pos >> 16) * 25736 * 2 + 0x8000);
        lsf[i] = h16(angle);
    }
}

static void weighted_lpc(const int16_t lsf[10], int16_t w, int16_t a[11]) {
    int16_t pulled[10], lsp[10];
    int i;
    for (i = 0; i < ORDER; i++) {
        int64_t v = shl(lsf[i], 16);
        v = acc(v - fmul(lsf[i], w));
        v = acc(v + fmul(PF_BIAS[i], w));
        pulled[i] = h16(v);
    }
    mrl_lsf_to_lsp(pulled, lsp);
    mrl_lsp_to_lpc(lsp, a);
}

static void synth(const int16_t a[11], const int16_t *in, int16_t *out, int n, int16_t mem[10]) {
    int16_t win[ORDER + SUB];
    int i, k;
    memcpy(win, mem, sizeof(int16_t) * ORDER);
    for (i = 0; i < n; i++) {
        int64_t sum = 0;
        for (k = 1; k <= ORDER; k++) sum = acc(sum - fmul(a[k], win[ORDER + i - k]));
        win[ORDER + i] = out[i] = h16(sat(acc(shift(sum, 3) + shl(in[i], 16) + (1 << 15))));
    }
    memcpy(mem, win + n, sizeof(int16_t) * ORDER);
}

static void inverse_filter(const int16_t a[11], const int16_t *speech, int16_t *out) {   /* speech: 10 の過去 + 80 */
    int n, i;
    for (n = 0; n < SUB; n++) {
        int64_t sum = 0;
        for (i = 1; i <= ORDER; i++) sum = acc(sum + fmul(a[i], speech[ORDER + n - i]));
        out[n] = h16(sat(acc(shift(sum, 3) + shl(speech[ORDER + n], 16) + 0x8000)));
    }
}

/* ---------------------------------------------------------------- 長期ポストフィルタ(ltp.rs) */

typedef struct {
    int64_t numerator, denominator;
    int16_t correlation, phase, direction;
} score;

typedef struct {
    int16_t correlation, energy, corr_exp, energy_exp;
} stats;

static int64_t energy_of(const int16_t *s, int n) {
    int64_t e = 0;
    int i;
    for (i = 0; i < n; i++) e = acc(e + (int64_t)s[i] * s[i] * 2);
    return e;
}

static int64_t correlate(const int16_t *a, const int16_t *b, int n) {
    int64_t c = 0;
    int i;
    for (i = 0; i < n; i++) c = acc(c + fmul(a[i], b[i]));
    return c;
}

static int better_ratio(int64_t new_num, int64_t best_den, int64_t best_num, int16_t new_den, int peak_exp) {
    int64_t lhs = shift(fmul32(best_den, new_num), -peak_exp);
    int64_t rhs = mul32x16(best_num, new_den);
    return acc(lhs - rhs) > 0;
}

typedef struct {
    int found;
    int lag;
    int16_t phase, direction, correlation, energy, corr_exp, energy_exp;
    int16_t interpolated[SUB];
} search_out;

static void ltp_search(const int16_t res[WINDOW], int lag0, search_out *out) {
    const int here = MRL_MPF_RES_HISTORY;
    const int16_t *cur = res + here;
    int64_t e0, best_corr, delayed_energy, peak;
    int e0_exp, k, best_k = 0, lag, ph, dir, peak_exp, corr_shift;
    int16_t e0_mant, samples[PHASES][INTERP_LEN], energies[2][PHASES];
    score best;
    out->found = 0;
    /* 今のブロックのエネルギー */
    e0 = energy_of(cur, SUB);
    if (acc(e0 - shl(1, 16)) < 0) return;
    e0_exp = fexp(e0);
    e0_mant = h16(shift(e0, e0_exp));
    /* 整数ラグ ±1 */
    best_corr = -shl(32768, 16);
    for (k = 0; k < 3; k++) {
        int delay = lag0 - 1 + k;
        int64_t c = correlate(cur, res + here - delay, SUB);
        if (c >= best_corr) {
            best_corr = c;
            best_k = k;
        }
        best_corr = sat(best_corr);
    }
    if (best_corr <= 0) return;
    lag = lag0 - 1 + best_k;
    delayed_energy = energy_of(res + here - lag, SUB);
    if (acc(delayed_energy - shl(1, 16)) < 0) return;
    delayed_energy = sat(delayed_energy);
    /* 短いフィルタの 7 位相 */
    peak = delayed_energy;
    {
        int base = here - lag + 1, n, t;
        for (ph = 0; ph < PHASES; ph++) {
            const int16_t *c = SEARCH_FILTER + 4 * ph;
            int64_t edge;
            for (n = 0; n < INTERP_LEN; n++) {
                int64_t v = 0;
                for (t = 0; t < 4; t++) v = acc(v + fmul(c[t], res[base + n - t]));
                samples[ph][n] = h16(acc(v + 0x8000));
            }
            energies[0][ph] = h16(sat(energy_of(samples[ph], SUB)));
            energies[1][ph] = h16(sat(energy_of(samples[ph] + 1, SUB)));
            edge = abs64(samples[ph][0]) > abs64(samples[ph][SUB]) ? energies[0][ph] : energies[1][ph];
            edge = shl(edge, 16);
            if (edge > peak) peak = edge;
        }
    }
    if (acc(peak - shl(1, 16)) < 0) return;
    peak_exp = fexp(peak);
    corr_shift = peak_exp < e0_exp ? peak_exp : e0_exp;
    /* 整数の候補 */
    {
        int16_t c = h16(shift(best_corr, corr_shift));
        best.correlation = c;
        best.numerator = acc((int64_t)c * c * 2);
        best.denominator = shift(delayed_energy, peak_exp);
        best.phase = PHASES;
        best.direction = 1;
    }
    for (ph = 0; ph < PHASES; ph++)
        for (dir = 0; dir < 2; dir++) {
            int64_t scaled = shift(correlate(cur, samples[ph] + dir, SUB), corr_shift);
            score cand;
            if (scaled < 0) scaled = 0;
            cand.correlation = h16(scaled);
            cand.numerator = acc((int64_t)cand.correlation * cand.correlation * 2);
            cand.denominator = shift(shl(energies[dir][ph], 16), peak_exp);
            cand.phase = (int16_t)(PHASES - 1 - ph);
            cand.direction = (int16_t)dir;
            if (better_ratio(cand.numerator, best.denominator, best.numerator, energies[dir][ph], peak_exp)) best = cand;
        }
    /* 有意か */
    if (best.correlation == 0 || acc(best.denominator - shl(1, 16)) <= 0) return;
    {
        int64_t scaled = shift(shift(acc(mul32x16(best.denominator, e0_mant)), -1), 2 * corr_shift - peak_exp - e0_exp);
        if (acc(scaled - best.numerator) > 0) return;
    }
    out->found = 1;
    out->phase = (int16_t)(PHASES - best.phase);
    memset(out->interpolated, 0, sizeof out->interpolated);
    if (out->phase != 0) memcpy(out->interpolated, samples[out->phase - 1] + best.direction, sizeof(int16_t) * SUB);
    out->lag = lag + 1 - best.direction;
    out->direction = best.direction;
    out->correlation = best.correlation;
    out->energy = h16(best.denominator);
    out->corr_exp = (int16_t)corr_shift;
    out->energy_exp = (int16_t)peak_exp;
}

static stats refine(const int16_t res[WINDOW], int lag, int phase, int16_t out[SUB]) {
    const int here = MRL_MPF_RES_HISTORY;
    const int16_t *h = REFINE_FILTER + 16 * (phase - 1);
    int base = here + 8 - lag, n, k, e;
    int64_t c, en;
    stats st = {0, 0, 0, 0};
    for (n = 0; n < SUB; n++) {
        int64_t a = 0;
        for (k = 0; k < 16; k++) a = acc(a + fmul(h[k], res[base + n - k]));
        out[n] = h16(sat(acc(a + 0x8000)));
    }
    c = correlate(out, res + here, SUB);
    if (c >= 0) {
        e = fexp(c);
        st.corr_exp = (int16_t)e;
        st.correlation = h16(shift(c, e));
    }
    en = energy_of(out, SUB);
    e = fexp(en);
    st.energy_exp = (int16_t)e;
    st.energy = h16(shift(en, e));
    return st;
}

static int refinement_wins(const stats *nw, const stats *old) {
    int64_t num_new, num_old, challenger, incumbent, diff;
    if (nw->energy == 0) return 0;
    num_new = acc((int64_t)nw->correlation * nw->correlation * 2);
    num_old = acc((int64_t)old->correlation * old->correlation * 2);
    challenger = mul32x16(num_new, old->energy);
    incumbent = mul32x16(num_old, nw->energy);
    diff = 2 * (int64_t)nw->corr_exp - 2 * (int64_t)old->corr_exp + old->energy_exp - nw->energy_exp;
    if (diff > 0)
        challenger = shift(challenger, -(int)diff);
    else
        incumbent = shift(incumbent, (int)diff);
    return acc(challenger - incumbent) > 0;
}

static int16_t mix_gain(const stats *st) {
    int sa = st->corr_exp - st->energy_exp;
    int64_t scaled_energy = shift(shl(st->energy, 16), sa), den;
    if (acc(shl(st->correlation, 16) - scaled_energy) > 0) return 21845;   /* 2/3 */
    den = acc(scaled_energy + (int64_t)st->correlation * 16384 * 2);
    return h16(shift(fdivide(den, st->energy), sa));
}

/* 長期ポストフィルタ: residual は未調整の残差の窓(最後の 80 が今のサブフレーム)。決めたラグ(0 = 切)を返す */
static int ltp_filter(const int16_t residual[WINDOW], int lag0, int16_t out[SUB]) {
    int16_t scaled[WINDOW], delayed[SUB];
    int64_t peak = 0;
    int headroom, n;
    search_out found;
    stats st;
    for (n = 0; n < WINDOW; n++) {
        int64_t m = abs64(shl(residual[n], 16));
        if (m > peak) peak = m;
    }
    peak = sat(peak);
    headroom = peak == 0 ? 12 : fexp(peak) - 3;
    for (n = 0; n < WINDOW; n++) scaled[n] = h16(shift(shl(residual[n], 16), headroom));
    ltp_search(scaled, lag0, &found);
    if (!found.found) {
        memcpy(out, residual + MRL_MPF_RES_HISTORY, sizeof(int16_t) * SUB);
        return 0;
    }
    st.correlation = found.correlation;
    st.energy = found.energy;
    st.corr_exp = found.corr_exp;
    st.energy_exp = found.energy_exp;
    if (found.phase == 0) {
        memcpy(delayed, residual + MRL_MPF_RES_HISTORY - found.lag, sizeof delayed);
    } else {
        int16_t refined[SUB];
        stats rs = refine(scaled, found.lag, found.phase, refined);
        memcpy(delayed, found.interpolated, sizeof delayed);
        if (refinement_wins(&rs, &st)) {
            st = rs;
            memcpy(delayed, refined, sizeof delayed);
        }
        for (n = 0; n < SUB; n++) delayed[n] = h16(sat(shift(shl(delayed[n], 16), -headroom)));
    }
    {
        int16_t gain = mix_gain(&st), complement = h16(acc(shl(32768, 16) - shl(gain, 16)));
        for (n = 0; n < SUB; n++)
            out[n] = h16(acc(fmul(gain, residual[MRL_MPF_RES_HISTORY + n]) + fmul(complement, delayed[n]) + 0x8000));
    }
    return found.lag;
}

/* ---------------------------------------------------------------- 短期の後半(postfilter.rs) */

static void probe_response(const int16_t den[11], const int16_t num[11], int16_t probe[MRL_MPF_PROBE]) {
    int16_t in[MRL_MPF_PROBE], mem[10];
    memset(in, 0, sizeof in);
    memcpy(in, num, sizeof(int16_t) * 11);
    memset(mem, 0, sizeof mem);
    synth(den, in, probe, MRL_MPF_PROBE, mem);
}

static int16_t spectral_tilt(const int16_t probe[MRL_MPF_PROBE]) {
    int64_t energy = 0, corr = 0, mag, energy_hi;
    int k, e;
    for (k = 0; k < MRL_MPF_PROBE; k++) energy = acc(energy + (int64_t)probe[k] * probe[k] * 2);
    if (energy == 0) return 0;
    e = fexp(energy);
    energy = shift(energy, e);
    for (k = 0; k < MRL_MPF_PROBE - 1; k++) corr = acc(corr + fmul(probe[k], probe[k + 1]));
    corr = shift(corr, e);
    mag = corr < 0 ? acc(-corr) : corr;
    energy_hi = shl(h16(energy), 16);
    if (acc(mag - energy_hi) > 0) return 0;
    return h16(acc(-fdivide(energy_hi, h16(corr))));
}

static void normalise_gain(const int16_t probe[MRL_MPF_PROBE], int16_t block[SUB]) {
    int64_t sum = 0, q, gain;
    int k, e;
    for (k = 0; k < MRL_MPF_PROBE; k++) sum = acc(sum + abs64(shl(probe[k], 16)));
    if (acc(shift(sum, 2) - shl(16384, 16)) <= 0) return;
    q = nreciprocal(sum, &e);
    gain = shift(shift(q, e), -3);
    for (k = 0; k < SUB; k++) block[k] = h16(mul32x16(gain, block[k]));
}

static void compensate_tilt(int16_t tilt, int16_t prev, const int16_t block[SUB], int16_t out[SUB]) {
    int16_t strength = tilt > 0 ? 6554 : 16384, k, direct;
    int64_t scaled = acc((int64_t)tilt * strength * 2), magnitude, gain, combined;
    int n;
    k = h16(shift(scaled, 1));
    magnitude = scaled < 0 ? acc(-scaled) : scaled;
    gain = fdivide(acc(shl(32768, 16) - magnitude), 16384);
    combined = acc((int64_t)h16(gain) * k * 2);
    direct = h16(gain);
    for (n = 0; n < SUB; n++) {
        int16_t previous = n == 0 ? prev : block[n - 1];
        int64_t a = mul32x16(combined, previous), b = acc((int64_t)block[n] * direct * 2);
        out[n] = h16(sat(acc(a + shift(b, 1) + (1 << 15))));
    }
}

static int64_t absolute_sum(const int16_t *b, int n) {
    int64_t s = 0;
    int i;
    for (i = 0; i < n; i++) s = acc(s + abs64(shl(b[i], 16)));
    return s;
}

static void agc(const int16_t speech[SUB], int16_t block[SUB], int16_t *gain) {
    int64_t reference = absolute_sum(speech, SUB), filtered, scaled_ref, normalised, q, ratio, scaled, running;
    int e_ref, e_filt, e_recip, n;
    int16_t target;
    if (reference == 0) {
        target = 0;
    } else {
        e_ref = fexp(reference);
        scaled_ref = shift(shift(reference, e_ref), -1);
        filtered = absolute_sum(block, SUB);
        if (filtered == 0) {
            *gain = 0;
            return;
        }
        e_filt = fexp(filtered);
        normalised = shift(filtered, e_filt);
        q = nreciprocal(normalised, &e_recip);
        ratio = sat(shift(mul32x16(scaled_ref, h16(q)), e_recip));
        scaled = mul32x16(ratio, 410);
        target = h16(sat(shift(shift(scaled, e_filt - e_ref + 1), -1)));
    }
    running = shl(*gain, 16);
    for (n = 0; n < SUB; n++) {
        int64_t smoothed = acc((int64_t)h16(running) * 32358 * 2), s2;
        running = sat(acc(smoothed + shl(target, 16)));
        s2 = acc((int64_t)h16(running) * block[n] * 2);
        block[n] = h16(sat(acc(shift(s2, 1) + 0x8000)));
    }
    *gain = h16(running);
}

/* ---------------------------------------------------------------- 公開 */

void mrl_mpf_init(mrl_mpf *p) {
    memset(p, 0, sizeof *p);
    p->agc_gain = 16384;
}

int mrl_mpf_subframe(mrl_mpf *p, const int16_t lsp[10], int lag0, const int16_t speech[10 + 80], int16_t out[80]) {
    int16_t lsf[10], den[11], num[11], res[SUB], filtered[SUB], probe[MRL_MPF_PROBE], resyn[SUB], tilt, prev;
    int lag;
    lsp_to_lsf(lsp, lsf);
    weighted_lpc(lsf, PF_WEIGHT_DEN, den);
    weighted_lpc(lsf, PF_WEIGHT_NUM, num);
    inverse_filter(num, speech, res);
    memcpy(p->residual + MRL_MPF_RES_HISTORY, res, sizeof res);
    lag = ltp_filter(p->residual, lag0, filtered);
    probe_response(den, num, probe);
    tilt = spectral_tilt(probe);
    normalise_gain(probe, filtered);
    prev = p->previous;
    synth(den, filtered, resyn, SUB, p->synthesis);
    p->previous = p->synthesis[ORDER - 1];
    compensate_tilt(tilt, prev, resyn, out);
    agc(speech + ORDER, out, &p->agc_gain);
    memmove(p->residual, p->residual + SUB, sizeof(int16_t) * MRL_MPF_RES_HISTORY);
    return lag;
}

/* ---------------------------------------------------------------- 三菱型のデコーダ(本体 + このポストフィルタ + 出力フィルタ) */

#include "mrlcelp.h"

struct mrl_mpf_decoder {
    mrl_state s;
    mrl_mpf pf;
    int16_t speech[10 + 160];
    int32_t fir[6], iir[6];
};

size_t mrl_mpf_decoder_size(void) { return sizeof(struct mrl_mpf_decoder); }

mrl_mpf_decoder *mrl_mpf_decoder_init(void *mem) {
    mrl_mpf_decoder *d = (mrl_mpf_decoder *)mem;
    memset(d, 0, sizeof *d);
    mrl_state_init(&d->s);
    mrl_mpf_init(&d->pf);
    return d;
}

int mrl_decode_mpf(mrl_mpf_decoder *d, const uint8_t payload[20], int16_t pcm[320]) {
    mrl_state *s = &d->s;
    mrl_half halves[2];
    int h, sub;
    if (payload[19] & 0xC0) return -1;
    mrl_unpack(payload, halves);
    for (h = 0; h < 2; h++) {
        int16_t lsf[10], lsp[2][10], lpc[2][11], code[80], dummy[160];
        float residual[10];
        int lag0 = 60;
        mrl_lsf_decode((const float(*)[10])s->spectral_history, &halves[h], lsf, residual);
        mrl_spectral_push(s->spectral_history, residual);
        mrl_lsf_to_lsp(lsf, lsp[1]);
        mrl_midpoint(s->previous_lsp, lsp[1], lsp[0]);   /* 1 つ目のサブフレームは前の半フレームとの中点 */
        mrl_begin_half(s, lsp[1], lpc[0], lpc[1]);
        for (sub = 0; sub < 2; sub++) {
            mrl_lag lag = sub == 0 ? mrl_decode_absolute(halves[h].absolute_pitch)
                                   : mrl_decode_relative(halves[h].relative_pitch, s->previous_lag);
            int pg, cg;
            if (sub == 0) lag0 = lag.integer;
            mrl_prepare(s, sub, lpc[sub], lag);
            mrl_innovation(s, halves[h].code[sub], lag, code);
            mrl_gain_decode(s->gain_history, halves[h].gain[sub], code, &pg, &cg);
            mrl_finish(s, sub, lpc[sub], code, pg, cg, d->speech + 10 + 80 * sub);
        }
        for (sub = 0; sub < 2; sub++) mrl_mpf_subframe(&d->pf, lsp[sub], lag0, d->speech + 80 * sub, pcm + 160 * h + 80 * sub);
        mrl_finish_half(s, d->speech + 10, dummy);   /* 本体の状態を進める(本体の出力フィルタの出力は使わない) */
        memmove(d->speech, d->speech + 160, sizeof(int16_t) * 10);
        mrl_output_filter(d->fir, d->iir, pcm + 160 * h, 160);
    }
    return 0;
}
