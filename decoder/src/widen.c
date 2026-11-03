/* 帯域拡張(8 kHz → 16 kHz)。復号した 8 kHz の音声の後ろに付ける、コーデックに依存しない処理。
 * 4 kHz より上はビット列に無いので、声としてもっともらしい高域を作って足す(本物の高域ではない):
 *   1. 8 → 16 kHz(Kaiser 窓の 97 タップ、鏡像を抑える)
 *   2. 高域の音源 = 決まった種の雑音 × 2.5〜3.8 kHz の帯域の rms 包絡(既定 150 Hz で平滑。音節の単位で変わる)
 *   3. 大きさ = 3〜3.8 kHz から途切れない密度 × 傾きの係数(AMR-WB の高域と同じ 1 − tilt を既定で 2 乗。母音ではほぼ足さない)
 *      2.5〜3.8 kHz のエネルギーは直近 30 ms の中央値(元の音の 1 ブロックだけの跳ねを高域に広げない)。
 *      高域の大きさは、上がるときは約 30 ms、下がるときは約 6 ms で追う(孤立した数 ms の雑音のかたまり = パチパチ感を作らない)
 *   4. 上へ行くほど弱くなる傾き(1 次の低域通過、既定 4.5 kHz)を付け、3.9〜7 kHz の帯域通過に通して足す
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <math.h>
#include <string.h>

#include "mrlcelp.h"

#define UP_TAPS 97    /* 遅れ 48 */
#define BP_TAPS 65    /* 遅れ 32 */
#define DELAY_U (2 * (BP_TAPS / 2))   /* 高域の経路(帯域通過 2 回)に合わせる u の遅れ */

typedef struct {
    float taps[UP_TAPS];
    float hist[UP_TAPS];
    int n, pos;
} fir;

struct mrl_widen {
    mrl_widen_config cfg;
    fir up, band, rband, high;
    float udelay[DELAY_U];
    int udpos;
    float env, scale, env_a, slope_a, slope_y, fac_s, eb_hist[2], rect_ms, mix_s, sq_ms;
    uint32_t seed;
    float nb_prev;
};

void mrl_widen_default_config(mrl_widen_config *c) {
    c->gain = 1.5f;
    c->low_hz = 3900.0f;
    c->high_hz = 6000.0f;   /* 7 kHz までだと高いパチパチ感が残った */
    c->env_hz = 150.0f;
    c->fac_power = 2.0f;
    c->slope_hz = 4500.0f;
    c->source = 1;          /* 全波整流(雑音だけの音源はザラつき・パチパチ感があった) */
    c->aperiodic = 0.1f;    /* 有声音にも雑音を 10 %(倍音だけだとロボット感) */
    c->square = 1;          /* |x| でなく x² */
}

size_t mrl_widen_size(void) { return sizeof(struct mrl_widen); }

static double bessel_i0(double x) {
    double s = 1.0, t = 1.0;
    int k;
    for (k = 1; k < 30; k++) {
        t *= (x / (2.0 * k)) * (x / (2.0 * k));
        s += t;
    }
    return s;
}

/* Kaiser 窓の sinc の低域通過(fc は fs = 16 kHz に対する Hz)。lo > 0 なら lo〜fc の帯域通過 */
static void design(fir *f, int n, double lo, double fc, double beta, double gain) {
    int i, m = n / 2;
    memset(f, 0, sizeof *f);
    f->n = n;
    for (i = 0; i < n; i++) {
        double t = i - m, r = (double)t / m, w = bessel_i0(beta * sqrt(1.0 - r * r)) / bessel_i0(beta);
        double hp = t == 0 ? 2.0 * fc / 16000.0 : sin(2.0 * 3.14159265358979 * fc / 16000.0 * t) / (3.14159265358979 * t);
        double lp = lo <= 0 ? 0.0 : (t == 0 ? 2.0 * lo / 16000.0 : sin(2.0 * 3.14159265358979 * lo / 16000.0 * t) / (3.14159265358979 * t));
        f->taps[i] = (float)((hp - lp) * w * gain);
    }
}

static float run(fir *f, float x) {
    float v = 0.0f;
    int k, p;
    f->hist[f->pos] = x;
    for (k = 0, p = f->pos; k < f->n; k++) {
        v += f->taps[k] * f->hist[p];
        p = p == 0 ? f->n - 1 : p - 1;
    }
    f->pos = f->pos + 1 == f->n ? 0 : f->pos + 1;
    return v;
}

mrl_widen *mrl_widen_init(void *mem, const mrl_widen_config *cfg) {
    mrl_widen *w = (mrl_widen *)mem;
    memset(w, 0, sizeof *w);
    if (cfg)
        w->cfg = *cfg;
    else
        mrl_widen_default_config(&w->cfg);
    design(&w->up, UP_TAPS, 0, 3800.0, 8.0, 2.0);   /* 0 を挟むので 2 倍 */
    design(&w->band, BP_TAPS, 2500.0, 3800.0, 6.0, 1.0);
    /* 全波整流の元: 1.2〜3.8 kHz(和の周波数が 2.4〜7.6 kHz になり、4 kHz のすぐ上も埋まる。2.5 kHz からだと 4〜5 kHz が空く) */
    design(&w->rband, BP_TAPS, 1200.0, 3800.0, 6.0, 1.0);
    design(&w->high, BP_TAPS, w->cfg.low_hz, w->cfg.high_hz, 6.0, 1.0);
    w->seed = 0x6d726c77u;
    w->env_a = 1.0f - expf(-2.0f * 3.14159265f * w->cfg.env_hz / 16000.0f);
    w->slope_a = w->cfg.slope_hz > 0.0f ? 1.0f - expf(-2.0f * 3.14159265f * w->cfg.slope_hz / 16000.0f) : 1.0f;
    return w;
}

int mrl_widen_delay(void) { return UP_TAPS / 2 + DELAY_U; }

static float noise(mrl_widen *w) {
    float s = 0.0f;
    int k;
    for (k = 0; k < 4; k++) {   /* 一様 4 個の和(ほぼ正規分布、分散 1) */
        w->seed = w->seed * 1664525u + 1013904223u;
        s += (float)(w->seed >> 8) / 16777216.0f - 0.5f;
    }
    return s * 1.7320508f;
}

/* 80 sample(8 kHz、10 ms)→ 160 sample(16 kHz) */
void mrl_widen_process(mrl_widen *w, const int16_t in[80], int16_t out[160]) {
    float r0 = 0.0f, r1 = 0.0f, tilt, fac, raw_fac, u[160], b[160], rb, hr[160], eb = 0.0f, er = 0.0f, target;
    int n;
    for (n = 0; n < 80; n++) {   /* 傾き(1 次の正規化自己相関) */
        float x = in[n];
        r0 += x * x;
        r1 += x * w->nb_prev;
        w->nb_prev = x;
    }
    tilt = r0 > 0.0f ? r1 / r0 : 1.0f;
    fac = 1.0f - tilt;   /* AMR-WB の高域の利得と同じ考え方 */
    if (fac < 0.0f) fac = 0.0f;
    if (fac > 1.0f) fac = 1.0f;
    raw_fac = fac;   /* 無声の度合い(音源を混ぜる比に使う) */
    fac = powf(fac, w->cfg.fac_power);
    w->fac_s += 0.4f * (fac - w->fac_s);   /* 10 ms ごとに更新、時定数 約 20 ms(1 ブロックだけの傾きの落ち込みで高域を出さない) */
    fac = w->fac_s;
    for (n = 0; n < 160; n++) {
        float e;
        u[n] = run(&w->up, (n & 1) ? 0.0f : (float)in[n >> 1]);
        b[n] = run(&w->band, u[n]);
        rb = run(&w->rband, u[n]);
        e = b[n] * b[n];
        w->env += w->env_a * (e - w->env);   /* rms 包絡(2 乗を 1 次で平滑) */
        /* 音源: 0 = 雑音 × rms 包絡、1 = 全波整流(|b| の和と差の周波数。有声音では F0 の間隔の倍音が上に続く)、
         * 2 = 有声の度合いで混ぜる(有声 = 全波整流、無声 = 雑音)。全波整流は rms で正規化してから混ぜる */
        {
            float src, nz = noise(w) * sqrtf(w->env), rect;
            if (w->cfg.square) {   /* 2 乗: 和と差の周波数(2 次)だけを作る。|x| の折れ目が作る高い次数の成分(ザラつき)が出ない */
                w->sq_ms += w->env_a * (rb * rb - w->sq_ms);
                rect = w->sq_ms > 1e-6f ? rb * rb / sqrtf(w->sq_ms) : 0.0f;
            } else {
                rect = fabsf(rb);
            }
            if (w->cfg.source == 2) {
                w->rect_ms += w->env_a * (rect * rect - w->rect_ms);
                w->mix_s += 0.0005f * (raw_fac - w->mix_s);   /* 約 120 ms でならす */
                rect = w->rect_ms > 1e-6f ? rect * sqrtf(w->env / w->rect_ms) : 0.0f;
                src = sqrtf(1.0f - w->mix_s) * rect + sqrtf(w->mix_s) * nz;
            } else if (w->cfg.source == 1 && w->cfg.aperiodic > 0.0f) {   /* 全波整流に、有声音でも一部の雑音を混ぜる(混合励振) */
                float m = w->cfg.aperiodic + (1.0f - w->cfg.aperiodic) * w->mix_s;
                w->rect_ms += w->env_a * (rect * rect - w->rect_ms);
                w->mix_s += 0.0005f * (raw_fac - w->mix_s);
                rect = w->rect_ms > 1e-6f ? rect * sqrtf(w->env / w->rect_ms) : 0.0f;
                src = sqrtf(1.0f - m) * rect + sqrtf(m) * nz;
            } else {
                src = w->cfg.source == 1 ? rect : nz;
            }
            w->slope_y += w->slope_a * (src - w->slope_y);   /* 上ほど弱く */
        }
        hr[n] = run(&w->high, w->slope_y);
        eb += b[n] * b[n];
        er += hr[n] * hr[n];
    }
    {   /* 2.5〜3.8 kHz のエネルギーは直近 3 ブロック(30 ms)の中央値を使う: 元の音の 1 ブロックだけの跳ねを高域に広げない */
        float a = w->eb_hist[0], b2 = w->eb_hist[1], c = eb, med = a > b2 ? (b2 > c ? b2 : (a > c ? c : a)) : (a > c ? a : (b2 > c ? c : b2));
        w->eb_hist[0] = b2;
        w->eb_hist[1] = c;
        eb = med;
    }
    /* 密度をそろえる: 高域の帯域幅 / 2.5〜3.8 kHz の帯域幅 */
    target = er > 1e-6f ? sqrtf(w->cfg.gain * fac * ((w->cfg.high_hz - w->cfg.low_hz) / 1300.0f) * eb / er) : 0.0f;
    for (n = 0; n < 160; n++) {
        float d = w->udelay[w->udpos], y;
        w->udelay[w->udpos] = u[n];
        w->udpos = w->udpos + 1 == DELAY_U ? 0 : w->udpos + 1;
        /* 上がるときはゆっくり(約 30 ms)、下がるときは速く(約 6 ms): 数 ms だけの雑音のかたまりを作らない */
        w->scale += (target > w->scale ? 0.002f : 0.01f) * (target - w->scale);
        y = d + w->scale * hr[n];
        out[n] = (int16_t)(y > 32767.0f ? 32767 : (y < -32768.0f ? -32768 : lrintf(y)));
    }
}
