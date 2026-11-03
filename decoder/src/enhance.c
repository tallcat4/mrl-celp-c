/* 聞きやすさ優先の復号(ビット一致ではない)。
 * パラメータの復号(LSF・ピッチ・固定符号帳・ゲイン)は CELP 本体(core.h)をそのまま使い、
 * 励振と合成を浮動小数でやり直して、その後に CELP で定番の後処理を掛ける:
 *   1. 浮動小数の励振・合成(16 bit の飽和が無いので、クリップと飽和の帰還による崩れが起きない)
 *   2. 固定符号帳の疎さの緩和(AMR 型の位相分散。ピッチ利得が低いときだけ、エネルギーは変えない)
 *   3. 適応ポストフィルタ(G.729 型:長期 = ピッチ、短期 = A(z/γn)/A(z/γd)、傾きの補正、AGC)
 *   4. 出力のハイパス(原本の出力フィルタと同じ特性)と、ソフトリミッタ
 *   5. AMR-WB のピッチ強調(有声のとき固定符号帳の寄与の低域を弱める)とノイズ強調(安定した無声で符号帳ゲインの揺れをならす)。
 *      2 と 5 は合成に使う励振だけに掛け、適応符号帳の記憶は本体と同じ(加工しない)励振で進める(AMR-WB と同じ)
 *   6. G.718 / EVS のバスポストフィルタ(1 kHz より下の倍音の間の雑音を、ピッチ周期の両側の平均で消す)。先読み 160 sample
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <math.h>
#include <string.h>

#include "core.h"
#include "mpostfilter.h"
#include "mrlcelp.h"
#include "tables.h"

#define EXC_LEN 314   /* core と同じ並び: 154 sample の履歴 + 160 sample(半フレーム) */
#define RES_HIST 160  /* 長期ポストフィルタが参照する残差の履歴(ラグ 143 + 探索幅) */
#define DISP_TAPS 80
#define BPF_LP 16     /* バスポストフィルタの低域通過の片側のタップ数 */

struct mrl_enhanced {
    mrl_state s;
    mrl_enh_config cfg;
    float exc[EXC_LEN];
    float syn_mem[10];
    /* ポストフィルタ */
    float pf_sig[10];             /* A(z/γn) に通す合成音声の過去 */
    float pf_res[RES_HIST + 80];  /* 残差(履歴 + 今のサブフレーム) */
    float pf_den[10];             /* 1/A(z/γd) の過去 */
    float pf_tilt;                /* 傾き補正の 1 sample 前 */
    float pf_agc;                 /* AGC の利得 */
    /* 位相分散 */
    float disp[2][DISP_TAPS];     /* 0 = 強い、1 = 中くらい */
    float prev_code_gain;
    int prev_level;
    /* 出力のハイパス(2 段) */
    float hp_x[2][3], hp_y[2][3];
    /* 任意の低域カット(2 次の Butterworth) */
    float lc_b[3], lc_a[3], lc_x[2], lc_y[2];
    /* 声の大きさの調整(ゆっくりした AGC) */
    float lv_level, lv_gain;
    /* ノイズ強調 */
    float prev_lsf_hz[10], ne_gain;
    int have_prev_lsf;
    /* バスポストフィルタ: 過去 160 + 遅らせて出す 160 + 先読み 160、サブフレームごとのラグ */
    float bpf_buf[480], bpf_lp[2 * BPF_LP + 1], bpf_alpha;
    int bpf_lag[6], frames_out;
    /* 三菱 CELP のポストフィルタ(postfilter == 2): 合成を ×0.5 して 16 bit に収めて通す */
    mrl_mpf mpf;
    int16_t mpf_speech[10 + 80];
    long limited;
};

void mrl_enh_default_config(mrl_enh_config *c) {
    c->float_synthesis = 1;
    c->anti_sparse = 2;
    c->postfilter = 2;   /* 三菱 CELP 型(聴き比べで G.729 型よりわずかに良かった) */
    c->limiter = 1;
    c->gamma_n = 0.55f;
    c->gamma_d = 0.70f;
    c->gamma_p = 0.50f;
    c->gamma_t = 0.80f;
    c->gain = 0.5f;   /* 実データの合成は 16 bit の約 2.4 倍まで振れる(99.99 % 点が約 50000)ので −6 dB */
    c->low_cut_hz = 0.0f;
    c->level_rms = 0.0f;
    c->pitch_enhance = 1;
    c->noise_enhance = 1;
    c->bass_postfilter = 1;
    c->bass_strength = 0.25f;   /* 0.5 だと低域が周期的すぎてブザーのような声になった(実データの聴き比べ) */
}

size_t mrl_enhanced_size(void) { return sizeof(struct mrl_enhanced); }

/* 平らな振幅・決まった乱数の位相の、長さ 80 の巡回の全域通過の応答。low_hz より下は位相を回さない */
static void design_dispersion(float h[DISP_TAPS], float low_hz, float spread, uint32_t seed) {
    float phase[DISP_TAPS / 2 + 1];
    int k, n;
    for (k = 0; k <= DISP_TAPS / 2; k++) {
        float f = 8000.0f * k / DISP_TAPS, w;
        seed = seed * 1664525u + 1013904223u;
        w = f <= low_hz ? 0.0f : (f - low_hz) / 500.0f;
        if (w > 1.0f) w = 1.0f;
        phase[k] = (k == 0 || k == DISP_TAPS / 2) ? 0.0f : w * spread * ((float)(seed >> 8) / 16777216.0f * 2.0f - 1.0f) * 3.14159265f;
    }
    for (n = 0; n < DISP_TAPS; n++) {   /* 実の逆 DFT */
        double v = 1.0 + cos(3.14159265358979 * n);   /* k = 0 と k = N/2 */
        for (k = 1; k < DISP_TAPS / 2; k++) v += 2.0 * cos(2.0 * 3.14159265358979 * k * n / DISP_TAPS + phase[k]);
        h[n] = (float)(v / DISP_TAPS);
    }
}

mrl_enhanced *mrl_enhanced_init(void *mem, const mrl_enh_config *cfg) {
    mrl_enhanced *e = (mrl_enhanced *)mem;
    memset(e, 0, sizeof *e);
    mrl_state_init(&e->s);
    if (cfg)
        e->cfg = *cfg;
    else
        mrl_enh_default_config(&e->cfg);
    e->pf_agc = 1.0f;
    design_dispersion(e->disp[0], 1000.0f, 1.0f, 0x6d726c31u);
    design_dispersion(e->disp[1], 2000.0f, 0.6f, 0x6d726c32u);
    e->prev_level = 2;
    e->lv_level = e->cfg.level_rms;
    e->lv_gain = 1.0f;
    mrl_mpf_init(&e->mpf);
    {   /* 1 kHz の低域通過(Hann 窓の sinc、DC 利得 1) */
        float sum = 0.0f;
        int n;
        for (n = -BPF_LP; n <= BPF_LP; n++) {
            float x = 2.0f * 3.14159265f * 1000.0f / 8000.0f * n;
            float w = 0.5f + 0.5f * cosf(3.14159265f * n / (BPF_LP + 1));
            sum += e->bpf_lp[n + BPF_LP] = (n == 0 ? 1.0f : sinf(x) / x) * w;
        }
        for (n = 0; n <= 2 * BPF_LP; n++) e->bpf_lp[n] /= sum;
        for (n = 0; n < 6; n++) e->bpf_lag[n] = 60;
    }
    if (e->cfg.low_cut_hz > 0.0f) {   /* 双一次変換の 2 次 Butterworth のハイパス */
        float k = tanf(3.14159265f * e->cfg.low_cut_hz / 8000.0f), q = 1.41421356f, d = 1.0f + q * k + k * k;
        e->lc_b[0] = 1.0f / d;
        e->lc_b[1] = -2.0f / d;
        e->lc_b[2] = 1.0f / d;
        e->lc_a[1] = 2.0f * (k * k - 1.0f) / d;
        e->lc_a[2] = (1.0f - q * k + k * k) / d;
    }
    return e;
}

long mrl_enhanced_limited(const mrl_enhanced *e) { return e->limited; }

/* core の mrl_predict と同じ補間(20 タップ、1/3 sample)を浮動小数で */
static void predict_float(float *exc, int at, mrl_lag lag) {
    int phase = -lag.frac, delay = lag.integer, n, j;
    float taps[20];
    if (phase < 0) {
        phase += 3;
        delay++;
    }
    for (j = 0; j < 10; j++) {
        taps[j] = MRL_PITCH_INTERP[phase + 3 * (9 - j)] / 32768.0f;
        taps[10 + j] = MRL_PITCH_INTERP[3 - phase + 3 * j] / 32768.0f;
    }
    for (n = 0; n < 80; n++) {
        const float *x = exc + at + n - delay - 9;
        float v = 0.0f;
        for (j = 0; j < 20; j++) v += x[j] * taps[j];
        exc[at + n] = v;
    }
}

/* AMR の anti-sparseness と同じ考え方: ピッチ利得が低いほど強く分散。立ち上がりでは弱める */
static void disperse(mrl_enhanced *e, float code[80], int pitch_gain, int code_gain) {
    float gp = pitch_gain / 16384.0f, out[80];
    int level = gp < 0.6f ? 0 : (gp < 0.9f ? 1 : 2), n, k;
    if (e->cfg.anti_sparse == 2) {   /* 疎なベクトル(エネルギーの 85 % 以上が上位 10 sample)だけ */
        float mag[80], total = 0.0f, top = 0.0f;
        int i, j;
        for (n = 0; n < 80; n++) total += mag[n] = code[n] * code[n];
        for (i = 0; i < 10; i++) {
            for (j = 0, k = 0; j < 80; j++)
                if (mag[j] > mag[k]) k = j;
            top += mag[k];
            mag[k] = 0.0f;
        }
        if (total <= 0.0f || top < 0.85f * total) level = 2;
    }
    if (code_gain > 2.0f * e->prev_code_gain && e->prev_code_gain > 0.0f) level = level < 2 ? level + 1 : 2;   /* 立ち上がり */
    if (level == 0 && e->prev_level == 2) level = 1;   /* 急に強くしない */
    e->prev_level = level;
    e->prev_code_gain = (float)code_gain;
    if (level == 2) return;
    for (n = 0; n < 80; n++) {
        float v = 0.0f;
        for (k = 0; k < 80; k++) v += code[k] * e->disp[level][(n - k + 80) % 80];
        out[n] = v;
    }
    memcpy(code, out, sizeof out);
}

/* AMR-WB のピッチ強調: c'(n) = c(n) − fac·(c(n−1) + c(n+1))、fac = 0.125·(1 + rv) */
static void pitch_enhance(float code[80], float rv) {
    float fac = 0.125f * (1.0f + rv), out[80];
    int n;
    for (n = 0; n < 80; n++) out[n] = code[n] - fac * ((n > 0 ? code[n - 1] : 0.0f) + (n < 79 ? code[n + 1] : 0.0f));
    memcpy(code, out, sizeof out);
}

/* AMR-WB のノイズ強調: スペクトルが安定(θ)で無声(rv が低い)ほど、符号帳ゲインを ±1.5 dB の範囲で前の値に寄せる */
static float noise_enhance(mrl_enhanced *e, float gc, float rv, float theta) {
    float sm = 0.5f * (1.0f - rv) * theta, g0;
    if (gc < e->ne_gain) {
        g0 = gc * 1.19f;
        if (g0 > e->ne_gain) g0 = e->ne_gain;
    } else {
        g0 = gc / 1.19f;
        if (g0 < e->ne_gain) g0 = e->ne_gain;
    }
    e->ne_gain = g0;
    return sm * g0 + (1.0f - sm) * gc;
}

/* G.718 型のバスポストフィルタ。bpf_buf[160..320) を、前後 160 sample を使って処理する。
 * e(n) = s(n) − (s(n−T) + s(n+T))/2 を 1 kHz の低域通過に通し、有声度に応じた α で引く */
static void bass_postfilter(mrl_enhanced *e, float out[160]) {
    float *b = e->bpf_buf, err[160 + 2 * BPF_LP];
    int n, k, sub;
    for (n = 0; n < 160 + 2 * BPF_LP; n++) {
        int i = 160 - BPF_LP + n, sf = i / 80, t;
        if (sf > 5) sf = 5;
        t = e->bpf_lag[sf];
        err[n] = b[i] - 0.5f * (b[i - t] + b[i + t]);
    }
    for (sub = 0; sub < 2; sub++) {
        float num = 0.0f, den = 0.0f, rho, target;
        int t = e->bpf_lag[2 + sub], base = 160 + 80 * sub;
        for (n = 0; n < 80; n++) {
            float avg = 0.5f * (b[base + n - t] + b[base + n + t]);
            num += b[base + n] * avg;
            den += 0.5f * (b[base + n] * b[base + n] + avg * avg);
        }
        rho = den > 0.0f ? num / den : 0.0f;
        target = e->cfg.bass_strength * (rho - 0.3f) / 0.4f;
        if (target < 0.0f) target = 0.0f;
        if (target > e->cfg.bass_strength) target = e->cfg.bass_strength;
        for (n = 0; n < 80; n++) {
            float lp = 0.0f;
            int m = 80 * sub + n;
            for (k = 0; k <= 2 * BPF_LP; k++) lp += e->bpf_lp[k] * err[m + k];
            e->bpf_alpha += 0.01f * (target - e->bpf_alpha);
            out[m] = b[160 + m] - e->bpf_alpha * lp;
        }
    }
}

static void weight_lpc(const float a[11], float g, float out[11]) {
    float p = 1.0f;
    int k;
    for (k = 0; k <= 10; k++, p *= g) out[k] = a[k] * p;
}

/* G.729 型の適応ポストフィルタ(80 sample) */
static void postfilter(mrl_enhanced *e, const float a[11], int lag, const float *in, float *out) {
    const mrl_enh_config *c = &e->cfg;
    float an[11], ad[11], *res = e->pf_res + RES_HIST, rp[80], h[22], rh0 = 0, rh1 = 0, k1, ein = 0, eout = 0, g;
    int n, k, best = lag, t;
    weight_lpc(a, c->gamma_n, an);
    weight_lpc(a, c->gamma_d, ad);
    /* 残差 r = A(z/γn) s */
    for (n = 0; n < 80; n++) {
        float v = in[n];
        for (k = 1; k <= 10; k++) v += an[k] * (n - k >= 0 ? in[n - k] : e->pf_sig[10 + n - k]);
        res[n] = v;
    }
    for (k = 0; k < 10; k++) e->pf_sig[k] = in[70 + k];
    /* 長期: 復号したラグの近くで正規化相関が最大の整数ラグ */
    {
        float cmax = -1e30f, en = 0, en0 = 0, gl = 0;
        int lo = lag - 3 < 20 ? 20 : lag - 3, hi = lag + 3 > RES_HIST - 1 ? RES_HIST - 1 : lag + 3;
        for (t = lo; t <= hi; t++) {
            float cc = 0;
            for (n = 0; n < 80; n++) cc += res[n] * res[n - t];
            if (cc > cmax) {
                cmax = cc;
                best = t;
            }
        }
        for (n = 0; n < 80; n++) {
            en += res[n - best] * res[n - best];
            en0 += res[n] * res[n];
        }
        if (cmax > 0 && cmax * cmax >= 0.5f * en * en0 && en > 0) gl = cmax / en > 1.0f ? 1.0f : cmax / en;
        gl *= c->gamma_p;
        for (n = 0; n < 80; n++) rp[n] = (res[n] + gl * res[n - best]) / (1.0f + gl);
    }
    memmove(e->pf_res, e->pf_res + 80, sizeof(float) * RES_HIST);
    /* 短期: 1/A(z/γd) */
    for (n = 0; n < 80; n++) {
        float v = rp[n];
        for (k = 1; k <= 10; k++) v -= ad[k] * (n - k >= 0 ? out[n - k] : e->pf_den[10 + n - k]);
        out[n] = v;
    }
    for (k = 0; k < 10; k++) e->pf_den[k] = out[70 + k];
    /* 傾きの補正: A(z/γn)/A(z/γd) の応答の 1 次の相関から */
    for (n = 0; n < 22; n++) {
        float v = n <= 10 ? an[n] : 0.0f;
        for (k = 1; k <= 10 && k <= n; k++) v -= ad[k] * h[n - k];
        h[n] = v;
    }
    for (n = 0; n < 22; n++) rh0 += h[n] * h[n];
    for (n = 0; n < 21; n++) rh1 += h[n] * h[n + 1];
    k1 = rh0 > 0 ? -rh1 / rh0 : 0.0f;
    g = k1 < 0 ? c->gamma_t * k1 : 0.0f;
    for (n = 0; n < 80; n++) {
        float x = out[n];
        out[n] = x + g * e->pf_tilt;
        e->pf_tilt = x;
    }
    /* AGC: 入力とエネルギーをそろえる(1 sample ごとに滑らかに) */
    for (n = 0; n < 80; n++) {
        ein += in[n] * in[n];
        eout += out[n] * out[n];
    }
    g = eout > 1e-3f ? sqrtf(ein / eout) : 0.0f;
    for (n = 0; n < 80; n++) {
        e->pf_agc = 0.9f * e->pf_agc + 0.1f * g;
        out[n] *= e->pf_agc;
    }
}

/* 声の大きさをそろえる: 声のあるサブフレーム(rms が目標の 1/30 以上)の rms を約 0.5 s で追い、
 * 目標との比を −6〜+12 dB に制限した利得を 1 sample ごとに滑らかに掛ける。無音は持ち上げない */
static void leveler(mrl_enhanced *e, float *y, int n_samples) {
    float target = e->cfg.level_rms, ms = 0.0f, g;
    int n;
    for (n = 0; n < n_samples; n++) ms += y[n] * y[n];
    ms = sqrtf(ms / n_samples);
    if (ms > target / 30.0f) e->lv_level = 0.98f * e->lv_level + 0.02f * ms;   /* 80 sample ごと → 時定数 0.4 s */
    g = e->lv_level > 0.0f ? target / e->lv_level : 1.0f;
    if (g < 0.5f) g = 0.5f;
    if (g > 4.0f) g = 4.0f;
    for (n = 0; n < n_samples; n++) {
        e->lv_gain += 0.002f * (g - e->lv_gain);
        y[n] *= e->lv_gain;
    }
}

/* 原本の出力フィルタ(2 段の 3 次のハイパス、通過域 ×1.2)を浮動小数で */
static float highpass(mrl_enhanced *e, float x) {
    int s;
    for (s = 0; s < 2; s++) {
        float b = 0.0f, *px = e->hp_x[s], *py = e->hp_y[s];
        b = 4.0f * (MRL_OUT_FIR[0] * x + MRL_OUT_FIR[1] * px[0] + MRL_OUT_FIR[2] * px[1] + MRL_OUT_FIR[3] * px[2]) / 32768.0f;
        b -= 4.0f * (MRL_OUT_IIR[0] * py[0] + MRL_OUT_IIR[1] * py[1] + MRL_OUT_IIR[2] * py[2]) / 32768.0f;
        px[2] = px[1];
        px[1] = px[0];
        px[0] = x;
        py[2] = py[1];
        py[1] = py[0];
        py[0] = b;
        x = b;
    }
    return x * (19661.0f * 4.0f / 65536.0f);
}

static int16_t limit(mrl_enhanced *e, float x) {
    const float knee = 0.7f * 32767.0f, room = 32767.0f - 0.7f * 32767.0f;
    float m = fabsf(x);
    if (e->cfg.limiter && m > knee) {
        m = knee + room * tanhf((m - knee) / room);
        x = x < 0 ? -m : m;
        e->limited++;
    }
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)lrintf(x);
}

/* 遅らせた 160 sample(bpf_buf[160..320))を、バスポストフィルタとリミッタを通して出す */
static void emit(mrl_enhanced *e, int16_t pcm[160]) {
    float out[160];
    int n;
    if (e->cfg.bass_postfilter)
        bass_postfilter(e, out);
    else
        memcpy(out, e->bpf_buf + 160, sizeof out);
    for (n = 0; n < 160; n++) pcm[n] = limit(e, out[n]);
}

int mrl_decode_enhanced(mrl_enhanced *e, const uint8_t payload[20], int16_t pcm[320]) {
    mrl_state *s = &e->s;
    mrl_half halves[2];
    int h, sub, n, k;
    if (payload[19] & 0xC0) return -1;
    mrl_unpack(payload, halves);
    for (h = 0; h < 2; h++) {
        int16_t lsf[10], lsp[10], sub_lsp[2][10], lpc[2][11], synth[160], code[80];
        int lag0 = 60;
        float residual[10], fsyn[160], post[160], theta = 0.0f;
        mrl_lsf_decode((const float(*)[10])s->spectral_history, &halves[h], lsf, residual);
        {   /* スペクトルの安定度(AMR-WB の stab_fac): θ = 1.25 − Σ(Δf[Hz])² / 400000 を 0〜1 に */
            float dist = 0.0f;
            for (k = 0; k < 10; k++) {
                float hz = lsf[k] * (4000.0f / 25736.0f);
                if (e->have_prev_lsf) dist += (hz - e->prev_lsf_hz[k]) * (hz - e->prev_lsf_hz[k]);
                e->prev_lsf_hz[k] = hz;
            }
            theta = e->have_prev_lsf ? 1.25f - dist / 400000.0f : 0.0f;
            if (theta < 0.0f) theta = 0.0f;
            if (theta > 1.0f) theta = 1.0f;
            e->have_prev_lsf = 1;
        }
        memmove(e->bpf_buf, e->bpf_buf + 160, sizeof(float) * 320);
        memmove(e->bpf_lag, e->bpf_lag + 2, sizeof(int) * 4);
        mrl_spectral_push(s->spectral_history, residual);
        mrl_lsf_to_lsp(lsf, lsp);
        mrl_midpoint(s->previous_lsp, lsp, sub_lsp[0]);
        memcpy(sub_lsp[1], lsp, sizeof lsp);
        mrl_begin_half(s, lsp, lpc[0], lpc[1]);
        for (sub = 0; sub < 2; sub++) {
            mrl_lag lag = sub == 0 ? mrl_decode_absolute(halves[h].absolute_pitch)
                                   : mrl_decode_relative(halves[h].relative_pitch, s->previous_lag);
            int pg, cg, at = 154 + 80 * sub;
            float a[11], fcode[80];
            if (sub == 0) lag0 = lag.integer;
            mrl_prepare(s, sub, lpc[sub], lag);
            mrl_innovation(s, halves[h].code[sub], lag, code);
            mrl_gain_decode(s->gain_history, halves[h].gain[sub], code, &pg, &cg);
            mrl_finish(s, sub, lpc[sub], code, pg, cg, synth + 80 * sub);   /* 本体の状態も進める(ラグ・周期強調の利得のため) */
            for (k = 0; k <= 10; k++) a[k] = lpc[sub][k] / 4096.0f;
            e->bpf_lag[4 + sub] = lag.integer < 20 ? 20 : lag.integer;   /* integer + frac/3 の最も近い整数 = integer */
            if (e->cfg.float_synthesis) {
                float *y = fsyn + 80 * sub, exc2[80], gp = pg / 16384.0f, gc = cg / 8192.0f, ev = 0.0f, ec = 0.0f, rv;
                for (n = 0; n < 80; n++) fcode[n] = (float)code[n];
                predict_float(e->exc, at, lag);
                for (n = 0; n < 80; n++) {
                    ev += (gp * e->exc[at + n]) * (gp * e->exc[at + n]);
                    ec += (gc * fcode[n]) * (gc * fcode[n]);
                }
                rv = ev + ec > 0.0f ? (ev - ec) / (ev + ec) : 0.0f;   /* 有声度 −1(無声)〜 1(有声) */
                for (n = 0; n < 80; n++) exc2[n] = e->exc[at + n] * gp;   /* 適応符号帳の寄与 */
                for (n = 0; n < 80; n++) e->exc[at + n] = e->exc[at + n] * gp + fcode[n] * gc;   /* 記憶は本体と同じ式 */
                if (e->cfg.anti_sparse) disperse(e, fcode, pg, cg);
                if (e->cfg.pitch_enhance) pitch_enhance(fcode, rv);
                if (e->cfg.noise_enhance) gc = noise_enhance(e, gc, rv, theta);
                for (n = 0; n < 80; n++) exc2[n] += fcode[n] * gc;
                for (n = 0; n < 80; n++) {
                    float v = exc2[n];
                    for (k = 1; k <= 10; k++) v -= a[k] * (n - k >= 0 ? y[n - k] : e->syn_mem[10 + n - k]);
                    y[n] = v;
                }
                for (k = 0; k < 10; k++) e->syn_mem[k] = y[70 + k];
            } else {
                for (n = 0; n < 80; n++) fsyn[80 * sub + n] = synth[80 * sub + n];
            }
            if (e->cfg.postfilter == 2) {
                int16_t pf_out[80];
                for (n = 0; n < 80; n++) {
                    float v = fsyn[80 * sub + n] * 0.5f;
                    e->mpf_speech[10 + n] = (int16_t)(v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : lrintf(v)));
                }
                mrl_mpf_subframe(&e->mpf, sub_lsp[sub], lag0, e->mpf_speech, pf_out);
                memmove(e->mpf_speech, e->mpf_speech + 80, sizeof(int16_t) * 10);
                for (n = 0; n < 80; n++) post[80 * sub + n] = pf_out[n] * 2.0f;
            } else if (e->cfg.postfilter)
                postfilter(e, a, lag.integer, fsyn + 80 * sub, post + 80 * sub);
            else
                memcpy(post + 80 * sub, fsyn + 80 * sub, sizeof(float) * 80);
        }
        memmove(e->exc, e->exc + 160, sizeof(float) * 154);
        {   /* 本体の半フレームの終わり(出力フィルタの状態は使わない) */
            int16_t dummy[160];
            mrl_finish_half(s, synth, dummy);
        }
        for (n = 0; n < 160; n++) post[n] = highpass(e, post[n] * e->cfg.gain);
        if (e->cfg.level_rms > 0.0f) {
            leveler(e, post, 80);
            leveler(e, post + 80, 80);
        }
        for (n = 0; n < 160; n++) {
            float y = post[n];
            if (e->cfg.low_cut_hz > 0.0f) {
                float v = e->lc_b[0] * y + e->lc_b[1] * e->lc_x[0] + e->lc_b[2] * e->lc_x[1] - e->lc_a[1] * e->lc_y[0] - e->lc_a[2] * e->lc_y[1];
                e->lc_x[1] = e->lc_x[0];
                e->lc_x[0] = y;
                e->lc_y[1] = e->lc_y[0];
                e->lc_y[0] = v;
                y = v;
            }
            e->bpf_buf[320 + n] = y;
        }
        emit(e, pcm + 160 * h);
    }
    return 0;
}

int mrl_enhanced_flush(mrl_enhanced *e, int16_t pcm[160]) {
    memmove(e->bpf_buf, e->bpf_buf + 160, sizeof(float) * 320);
    memmove(e->bpf_lag, e->bpf_lag + 2, sizeof(int) * 4);
    memset(e->bpf_buf + 320, 0, sizeof(float) * 160);
    emit(e, pcm);
    return 160;
}
