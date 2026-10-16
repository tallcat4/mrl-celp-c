/* 改良型 RL-CELP のエンコーダ(MCU 向けの軽い A-b-S、float)。研究用の Python 版(mrl-celp の src/mrlcelp/encoder.py)の考え方を、探索を絞って C にしたもの。
 *
 * 状態はデコーダと同じ整数の処理(dsp.c)で進めるので、エンコーダとデコーダの状態は常に一致する
 * (出力フィルタの状態だけは符号化に使わないので進めない)。
 * 半フレーム(20 ms)ごと:
 *   1. 線形予測: サブフレームの中心に 240 sample の Hamming 窓(後半は 10 ms の先読み)、ラグ窓、Levinson、
 *      LSF は Chebyshev 多項式の根の探索
 *   2. LSF の量子化: デコーダの移動平均予測(モード 0/1)を裏返した目標に対して 1 段の上位 lsf_stage1 個 × 2 段の上位 lsf_stage2 個ずつ
 *      (途中で打ち切る部分距離探索)。誤差は後半の LSF と、前半(前の半フレームとの中点)の LSF の両方で測る
 *   3. 開ループのピッチ: 重みつきの音声の正規化相関(偶数のラグ・1 つおきの標本で粗く → 前後を詰める。短いラグを優先)
 * サブフレーム(10 ms)ごと:
 *   4. 目標: 聴感の重み付け W(z) = A(z/γ1)/A(z/γ2) をかけた入力から零入力応答を引く
 *   5. ピッチ: 開ループの候補 ± pitch_w0(後半は前のラグ ± pitch_w2)の整数のラグを、分母は補間フィルタを通した励振の
 *      再帰(G.729 と同じ)、分子は正確な適応符号帳で比べる。小数(±1/3)は、整数のラグの前後の得点が高い側だけ調べる
 *   6. 固定符号帳: パルスの形をフィルタに通したもの g_k = h * shape_k を位置をずらして使い、相関(d = H^T x と形の内積)の
 *      大きい位置だけ組み合わせる(3 パルスの区分は m3 個ずつ、2 パルスは m2 個ずつ、単位パルス 5 + 5 個の区分は表の行を malt 個ずつ)
 *   7. 上位 keep 個について、g_k を足し合わせた近似のベクトルで 128 通りのゲインを比べ、選んだものだけ正確な符号ベクトル
 *      (周期強調つき)を作って、ゲインを正確な値で選び直す
 *   8. 選んだパラメータでデコーダの状態を進める
 * 計算量は ESP32(240 MHz)で 1 フレーム 40 ms あたり約 320 万サイクル(CPU の 33%)、デコーダと合わせて 37%(README.md)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include <math.h>
#include <string.h>

#include "core.h"
#include "mrlcelp_enc.h"
#include "tables.h"

#define PI_F 3.14159265358979f
#define Q13_PER_RAD (25736.0f / PI_F)
#define BUF 960   /* 前処理した信号のバッファ: 絶対位置 [320F − 640, 320F + 320) */
#define MAXC 16

/* 段階ごとの計測(MRL_PROFILE を定義したときだけ。tests/mcu/mcu_test.c が mrl_prof_hook を入れる) */
#ifdef MRL_PROFILE
void (*mrl_prof_hook)(int stage);
#define PROF(n) do { if (mrl_prof_hook) mrl_prof_hook(n); } while (0)
#else
#define PROF(n) do { } while (0)
#endif
enum { P_LPC, P_LSF, P_OL, P_TARGET, P_PITCH, P_SHAPES, P_ALT, P_GAINS, P_STATE,
       P_SH_G, P_SH_CORR, P_SH_CROSS, P_SH_COMB, P_ALT_PHI, P_ALT_ROWS, P_P_BACK, P_P_INT, P_P_FRAC, P_LSF_S1, P_LSF_S2, P_N };

struct mrl_encoder {
    mrl_enc_config cfg;
    mrl_state s;
    float hp_x[2], hp_y[2];        /* 高域通過の状態 */
    float sp[BUF], sw[BUF];        /* 前処理した信号、重みつきの信号 */
    float lpc[4][11];              /* 符号化するフレームの 4 サブフレームの LPC(量子化前) */
    float lsf[4][10];              /* 同じく LSF(Q13) */
    float prev_lsf_rad[10];        /* 根が求まらないときに使う */
    float prev_q[10];              /* 前の半フレームの量子化した LSF(Q13) */
    float sw_x[10], sw_y[10];      /* 重みつきの音声のフィルタの状態 */
    float w_in[10], w_out[10];     /* 誤差の重み付けフィルタの状態 */
    int16_t last_synth[160];       /* 半フレームの合成(出力フィルタの前) */
    long frames;                   /* push されたフレーム数 */
    float target_energy, error_energy;
};

static inline float dot(const float *a, const float *b, int n) {   /* 累積を 4 本に分けて、加算の待ちを減らす */
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    int i;
    for (i = 0; i + 4 <= n; i += 4) {
        s0 += a[i] * b[i];
        s1 += a[i + 1] * b[i + 1];
        s2 += a[i + 2] * b[i + 2];
        s3 += a[i + 3] * b[i + 3];
    }
    for (; i < n; i++) s0 += a[i] * b[i];
    return (s0 + s1) + (s2 + s3);
}

/* ---------------------------------------------------------------- 表(初回に作る) */

static float HAMMING[240], LAGWIN[11], PITCH_GAINS[128], EXP_LG[128], COS_GRID[129], SHAPES[240], INTERP[31], CB1[1280];
static float FWD[3][10], BWD[3][10], SHAPES_R[240];   /* 補間フィルタ(前向きは逆順)、逆順のパルスの形 */
static int16_t ABS_CODE[144][3];
static mrl_lag ABS_LAG[256];   /* 絶対ピッチの符号 → ラグ(mrl_decode_absolute の表) */
static uint8_t ALT_P[2][128][5];   /* 単位パルスの区分の表: 位置と符号 */
static float ALT_S[2][128][5];   /* (整数のラグ, 小数 + 1) → 絶対ピッチの符号(無ければ −1) */
static int tables_ready;

static void init_tables(void) {
    int i;
    if (tables_ready) return;
    for (i = 0; i < 240; i++) HAMMING[i] = 0.54f - 0.46f * cosf(2 * PI_F * i / 239);
    for (i = 0; i < 11; i++) {
        float w = 2 * PI_F * 60 * i / 8000;
        LAGWIN[i] = expf(-0.5f * w * w);
    }
    for (i = 0; i < 128; i++) {
        PITCH_GAINS[i] = mrl_quantize(MRL_PITCH_GAIN[i]) / 16384.0f;
        EXP_LG[i] = expf(MRL_LOG_GAIN[i]);
    }
    for (i = 0; i <= 128; i++) COS_GRID[i] = cosf(PI_F * i / 128);
    for (i = 0; i < 240; i++) SHAPES[i] = MRL_FCB_SHAPES[i];
    for (i = 0; i < 31; i++) INTERP[i] = MRL_PITCH_INTERP[i] * (1.0f / 32768);
    for (i = 0; i < 1280; i++) CB1[i] = MRL_LSF_CB1[i];
    {
        int ph, j;
        for (ph = 0; ph < 3; ph++)
            for (j = 0; j < 10; j++) {
                FWD[ph][j] = INTERP[ph + 3 * (9 - j)];   /* x[c − 9 + j] に掛ける */
                BWD[ph][j] = INTERP[3 - ph + 3 * j];     /* x[c + 1 + j] に掛ける */
            }
        for (j = 0; j < 12; j++)
            for (i = 0; i < 20; i++) SHAPES_R[20 * j + i] = SHAPES[20 * j + 19 - i];
    }
    {
        int t, r, a;
        for (t = 0; t < 2; t++)
            for (r = 0; r < 128; r++)
                for (a = 0; a < 5; a++) {
                    int e = (t ? MRL_FCB_ALT_B : MRL_FCB_ALT_A)[5 * r + a];
                    ALT_P[t][r][a] = (uint8_t)((e < 0 ? -e : e) - 1);
                    ALT_S[t][r][a] = e > 0 ? 1.0f : -1.0f;
                }
    }
    memset(ABS_CODE, 0xFF, sizeof ABS_CODE);
    for (i = 255; i >= 0; i--) {
        mrl_lag l = ABS_LAG[i] = mrl_decode_absolute(i);
        if (l.integer >= 20 && l.integer < 144) ABS_CODE[l.integer][l.frac + 1] = (int16_t)i;
    }
    tables_ready = 1;
}

/* 差分のピッチの符号 → ラグ(mrl_decode_relative と同じ: 範囲の下端 lo、商 q = (c + 2) / 3 − 1) */
static inline mrl_lag rel_lag(int c, int prev) {
    mrl_lag l;
    int lo = prev - 5 > 20 ? prev - 5 : 20, q = (c + 2) / 3 - 1;
    if (lo + 9 > 143) lo = 134;
    l.integer = q + lo;
    l.frac = c - 2 - 3 * q;
    return l;
}

static inline mrl_lag code_lag(int sub, int c, int prev) { return sub == 0 ? ABS_LAG[c] : rel_lag(c, prev); }

/* 1/x の近似(ビットの初期値 + Newton 2 回、相対誤差 1e-6 程度)。浮動小数の割り算がソフトウェアになるマイコン向け */
static inline float rcp(float x) {
    union { float f; uint32_t u; } v;
    float y;
    v.f = x;
    v.u = 0x7EF311C7u - v.u;
    y = v.f;
    y = y * (2.0f - x * y);
    y = y * (2.0f - x * y);
    return y * (2.0f - x * y);
}

void mrl_enc_default_config(mrl_enc_config *c) {
    /* dev(LibriSpeech dev-clean など 130 発話)で、品質をほとんど落とさずに探索を絞った値(2026-10) */
    c->gamma1 = 0.94f;
    c->gamma2 = 0.6f;
    c->lsf_stage1 = 3;
    c->lsf_stage2 = 2;
    c->sub0_weight = 0.5f;
    c->m3 = 3;
    c->m2 = 5;
    c->malt = 5;
    c->keep = 3;
    c->pitch_w0 = 2;
    c->pitch_w1 = -1;   /* 開ループの 2 つ目の候補は調べない(倍ピッチの誤りが増えて、かえって悪い) */
    c->pitch_w2 = 3;
    c->pitch_fine = 0;
}

size_t mrl_encoder_size(void) { return sizeof(struct mrl_encoder); }

mrl_encoder *mrl_encoder_init(void *mem, const mrl_enc_config *cfg) {
    mrl_encoder *e = (mrl_encoder *)mem;
    int k;
    init_tables();
    memset(e, 0, sizeof *e);
    if (cfg)
        e->cfg = *cfg;
    else
        mrl_enc_default_config(&e->cfg);
    mrl_state_init(&e->s);
    for (k = 0; k < 10; k++) {
        e->prev_q[k] = MRL_LSF_MEAN[k];
        e->prev_lsf_rad[k] = MRL_LSF_MEAN[k] / Q13_PER_RAD;
    }
    return e;
}

/* ---------------------------------------------------------------- 線形予測と LSF */

static void lpc_analysis(const float *x, float a[11]) {
    float w[240], r[11], err, k;
    int i, j;
    for (i = 0; i < 240; i++) w[i] = x[i] * HAMMING[i];
    for (i = 0; i <= 10; i++) r[i] = dot(w + i, w, 240 - i) * LAGWIN[i];
    memset(a, 0, sizeof(float) * 11);
    a[0] = 1;
    if (r[0] <= 1e-6f) return;
    r[0] *= 1.0001f;
    err = r[0];
    for (i = 1; i <= 10; i++) {
        float tmp[11], s = r[i];
        for (j = 1; j < i; j++) s += a[j] * r[i - j];
        k = -s * rcp(err);
        memcpy(tmp, a, sizeof tmp);
        for (j = 1; j < i; j++) a[j] = tmp[j] + k * tmp[i - j];
        a[i] = k;
        err *= 1 - k * k;
        if (err <= 0) break;
    }
}

/* acos の近似(Abramowitz–Stegun 4.4.46、7 次、誤差 2e-8 rad 程度) */
static float fast_acos(float x) {
    float a = x < 0 ? -x : x, r;
    r = ((((((-0.0012624911f * a + 0.0066700901f) * a - 0.0170881256f) * a + 0.0308918810f) * a - 0.0501743046f) * a +
          0.0889789874f) * a - 0.2145988016f) * a + 1.5707963050f;
    r *= sqrtf(1 - a);
    return x < 0 ? PI_F - r : r;
}

static float cheb(const float f[6], float x) {   /* Σ f[i] T_{5−i}(x)(f[5] は 1/2) */
    float b0 = 0, b1 = 0, b2;
    int i;
    for (i = 0; i < 5; i++) {
        b2 = b1;
        b1 = b0;
        b0 = 2 * x * b1 - b2 + f[i];
    }
    return x * b0 - b1 + 0.5f * f[5];
}

/* LPC → LSF [rad]。10 個求まらなければ 0 */
static int lpc_to_lsf(const float a[11], float lsf[10]) {
    float f1[6], f2[6], xlow, ylow, xhigh, yhigh;
    int i, n = 0, which = 0, k;
    const int GRID = 128;
    f1[0] = f2[0] = 1;
    for (i = 1; i <= 5; i++) {
        f1[i] = a[i] + a[11 - i] - f1[i - 1];
        f2[i] = a[i] - a[11 - i] + f2[i - 1];
    }
    xlow = 1.0f;
    ylow = cheb(f1, xlow);
    for (k = 1; k <= GRID && n < 10; k++) {
        const float *f = which ? f2 : f1;
        xhigh = xlow;
        yhigh = ylow;
        xlow = COS_GRID[k];
        ylow = cheb(f, xlow);
        if (ylow * yhigh <= 0) {
            int it;
            for (it = 0; it < 6; it++) {   /* 二分法 */
                float xm = 0.5f * (xlow + xhigh), ym = cheb(f, xm);
                if (ym * ylow <= 0) {
                    xhigh = xm;
                    yhigh = ym;
                } else {
                    xlow = xm;
                    ylow = ym;
                }
            }
            {
                float x = (ylow == yhigh) ? xlow : xlow - ylow * (xhigh - xlow) / (yhigh - ylow);
                lsf[n++] = fast_acos(x < -1 ? -1 : (x > 1 ? 1 : x));
                xlow = x;
            }
            which ^= 1;
            ylow = cheb(which ? f2 : f1, xlow);
        }
    }
    return n == 10;
}

static void lsf_to_lpc(const float lsf[10], float a[11]) {
    float p[12] = {1}, q[12] = {1};
    int i, j, n = 1;
    for (i = 0; i < 5; i++) {   /* Π (1 − 2 cos ω z^-1 + z^-2) */
        float cp = -2 * cosf(lsf[2 * i]), cq = -2 * cosf(lsf[2 * i + 1]);
        for (j = n + 1; j >= 0; j--) {
            float pv = p[j] + (j >= 1 ? cp * p[j - 1] : 0) + (j >= 2 ? p[j - 2] : 0);
            float qv = q[j] + (j >= 1 ? cq * q[j - 1] : 0) + (j >= 2 ? q[j - 2] : 0);
            p[j] = pv;
            q[j] = qv;
        }
        n += 2;
    }
    for (i = 0; i <= 10; i++) {
        float pp = p[i] + (i ? p[i - 1] : 0), qq = q[i] - (i ? q[i - 1] : 0);
        a[i] = 0.5f * (pp + qq);
    }
}

/* ---------------------------------------------------------------- LSF の量子化 */

static void lsf_weights(const float lsf[10], float w[10]) {
    int k;
    for (k = 0; k < 10; k++) {
        float lo = k ? lsf[k] - lsf[k - 1] : lsf[0], hi = (k < 9 ? lsf[k + 1] : 25736.0f) - lsf[k];
        w[k] = 1.0f / (lo > 1 ? lo : 1) + 1.0f / (hi > 1 ? hi : 1);
    }
}

/* corr[i]^2 / en[i] の大きい順に keep 個(割り算をせず、たすき掛けで比べる)。corr ≥ 0 */
static void top_ratio(const float *corr, const float *en, int n, int keep, int *idx) {
    int i, j, filled = 0;
    for (i = 0; i < n; i++) {
        float c2 = corr[i] * corr[i];
        if (filled == keep) {
            int w = idx[keep - 1];
            if (c2 * en[w] <= corr[w] * corr[w] * en[i]) continue;
            j = keep - 1;
        } else {
            j = filled++;
        }
        while (j > 0 && c2 * en[idx[j - 1]] > corr[idx[j - 1]] * corr[idx[j - 1]] * en[i]) {
            idx[j] = idx[j - 1];
            j--;
        }
        idx[j] = i;
    }
}

/* 正の corr[i] だけを、corr の大きい順に keep 個まで。*count に数 */
static void top_ratio_pos(const float *corr, const float *unused, int n, int keep, int *idx, int *count) {
    int i, j, filled = 0;
    (void)unused;
    for (i = 0; i < n; i++) {
        float c = corr[i];
        if (c <= 0) continue;
        if (filled == keep) {
            if (c <= corr[idx[keep - 1]]) continue;
            j = keep - 1;
        } else {
            j = filled++;
        }
        while (j > 0 && c > corr[idx[j - 1]]) {
            idx[j] = idx[j - 1];
            j--;
        }
        idx[j] = i;
    }
    *count = filled;
}

/* 2 段の表(64 × 5)から、重みつき距離の小さい keep 個(途中の和が keep 番目を超えたら打ち切る) */
static void stage2_top(const float rem[5], const float w[5], const float *tab, int keep, float *err, int *idx) {
    int i, j, k, filled = 0;
    for (i = 0; i < 64; i++) {
        const float *t = tab + 5 * i;
        float lim = filled == keep ? err[idx[keep - 1]] : 1e30f, sv = 0;
        for (k = 0; k < 5; k++) {
            float d = rem[k] - t[k];
            sv += d * d * w[k];
            if (sv >= lim) break;
        }
        if (sv >= lim) continue;
        err[i] = sv;
        j = filled < keep ? filled++ : keep - 1;
        while (j > 0 && err[idx[j - 1]] > sv) {
            idx[j] = idx[j - 1];
            j--;
        }
        idx[j] = i;
    }
}

static void quantize_lsf(mrl_encoder *e, const float t1[10], const float t0[10], mrl_half *out) {
    float w1[10], w0[10], best = 1e30f;
    int mode, k;
    int S1 = e->cfg.lsf_stage1 > MAXC ? MAXC : e->cfg.lsf_stage1, S2 = e->cfg.lsf_stage2 > MAXC ? MAXC : e->cfg.lsf_stage2;
    lsf_weights(t1, w1);
    lsf_weights(t0, w0);
    for (mode = 0; mode < 2; mode++) {
        const float *m = MRL_MA + 40 * mode;
        float base[10], rt[10], wr[10], d1[128], el[64], eh[64];
        int s1i[MAXC], li[MAXC], hi[MAXC], a, b, c, i;
        for (k = 0; k < 10; k++) {
            base[k] = e->s.spectral_history[0][k] * m[10 + k] + e->s.spectral_history[1][k] * m[20 + k] +
                      e->s.spectral_history[2][k] * m[30 + k];
            rt[k] = (t1[k] - base[k]) / m[k];
            wr[k] = w1[k] * m[k] * m[k];
        }
        {   /* 上位 S1 個を保ちながら、途中の和が S1 番目を超えたら打ち切る(部分距離探索、結果は同じ) */
            int filled = 0;
            for (i = 0; i < 128; i++) {
                const float *cb = CB1 + 10 * i;
                float sv = 0, lim = filled == S1 ? d1[s1i[S1 - 1]] : 1e30f;
                int j;
                for (k = 0; k < 10; k += 2) {
                    float da = rt[k] - cb[k], db = rt[k + 1] - cb[k + 1];
                    sv += da * da * wr[k] + db * db * wr[k + 1];
                    if (sv >= lim) break;
                }
                if (sv >= lim) continue;
                d1[i] = sv;
                j = filled < S1 ? filled++ : S1 - 1;
                while (j > 0 && d1[s1i[j - 1]] > sv) {
                    s1i[j] = s1i[j - 1];
                    j--;
                }
                s1i[j] = i;
            }
        }
        PROF(P_LSF_S1);
        for (a = 0; a < S1; a++) {
            float rem[10];
            for (k = 0; k < 10; k++) rem[k] = rt[k] - CB1[10 * s1i[a] + k];
            stage2_top(rem, wr, MRL_STAGE2_LOW, S2, el, li);
            stage2_top(rem + 5, wr + 5, MRL_STAGE2_HIGH, S2, eh, hi);
            PROF(P_LSF_S2);
            for (b = 0; b < S2; b++)
                for (c = 0; c < S2; c++) {
                    float q[10], err = 0;
                    int j;
                    for (k = 0; k < 10; k++)   /* デコーダの再構成の近似(整列・間隔・範囲、丸めなし) */
                        q[k] = (CB1[10 * s1i[a] + k] +
                                (k < 5 ? MRL_STAGE2_LOW[5 * li[b] + k] : MRL_STAGE2_HIGH[5 * hi[c] + k - 5])) * m[k] + base[k];
                    for (k = 1; k < 10; k++)
                        for (j = k; j > 0 && q[j - 1] > q[j]; j--) {
                            float t = q[j];
                            q[j] = q[j - 1];
                            q[j - 1] = t;
                        }
                    if (q[0] < 41.0f) q[0] = 41.0f;
                    for (k = 1; k < 10; k++)
                        if (q[k] < q[k - 1] + 321.0f) q[k] = q[k - 1] + 321.0f;
                    if (q[9] > 25682.0f) q[9] = 25682.0f;
                    for (k = 0; k < 10; k++) {
                        float d = q[k] - t1[k], dm = 0.5f * (q[k] + e->prev_q[k]) - t0[k];
                        err += d * d * w1[k] + e->cfg.sub0_weight * dm * dm * w0[k];
                    }
                    if (err < best) {
                        best = err;
                        out->mode = mode;
                        out->stage1 = s1i[a];
                        out->stage2_low = li[b];
                        out->stage2_high = hi[c];
                    }
                }
        }
    }
}

/* ---------------------------------------------------------------- フィルタの補助 */

/* y = B(z)/A(z) x(10 次)。xm / ym は直前 10 sample(古い順)、書き換える */

static void pole_zero(const float b[11], int nb, const float a[11], const float *x, float *y, int n, float xm[10], float ym[10]) {
    float xb[10 + 80], yb[10 + 80], br[11], ar[10];
    int i, k;
    for (k = 0; k < 11; k++) br[k] = k < 11 - nb ? 0.0f : b[10 - k];   /* 逆順(内積の形) */
    for (k = 0; k < 10; k++) ar[k] = a[10 - k];
    memcpy(xb, xm, sizeof(float) * 10);
    memcpy(yb, ym, sizeof(float) * 10);
    memcpy(xb + 10, x, sizeof(float) * n);
    if (nb == 1)
        for (i = 0; i < n; i++) yb[10 + i] = y[i] = b[0] * xb[10 + i] - dot(yb + i, ar, 10);
    else
        for (i = 0; i < n; i++) yb[10 + i] = y[i] = dot(xb + i, br, 11) - dot(yb + i, ar, 10);
    memcpy(xm, xb + n, sizeof(float) * 10);
    memcpy(ym, yb + n, sizeof(float) * 10);
}


static void convolve_dense(const float h[80], const float *v, float *y) {   /* y = H v(零状態、80 sample、密な v) */
    float hr[80];
    int n;
    for (n = 0; n < 80; n++) hr[n] = h[79 - n];
    for (n = 0; n < 80; n++) y[n] = dot(v, hr + 79 - n, n + 1);
}

static void convolve(const float h[80], const float *v, float *y) {   /* y = H v(零状態、80 sample)。v の 0 の要素は飛ばす */
    int k, n;
    memset(y, 0, sizeof(float) * 80);
    for (k = 0; k < 80; k++) {
        float a = v[k], *yk = y + k;
        if (a == 0) continue;
        for (n = 0; n < 80 - k; n++) yk[n] += a * h[n];
    }
}

static void backward(const float h[80], const float x[80], float d[80]) {   /* d = H^T x: d[j] = Σ_{n ≥ j} x[n] h[n − j] */
    int j;
    for (j = 0; j < 80; j++) d[j] = dot(x + j, h, 80 - j);
}

/* ---------------------------------------------------------------- 開ループのピッチ */

static void open_loop(const float *sw_end, int out[2]) {   /* sw_end: 半フレームの終わり(sw_end[-1] が最後) */
    const float *cur = sw_end - 160;
    float score[145];
    int lag, r, best[3];
    const int lo[3] = {20, 40, 80}, hi[3] = {40, 80, 144};
    for (lag = 20; lag < 145; lag++) score[lag] = -1e30f;
    for (lag = 20; lag < 144; lag += 2) {   /* 偶数のラグ、1 つおきの標本で粗く → 各範囲の最大の前後を全部の標本で */
        const float *past = cur - lag;
        float en = 0, c = 0;
        int n;
        for (n = 0; n < 160; n += 2) {
            en += past[n] * past[n];
            c += cur[n] * past[n];
        }
        score[lag] = en > 0 ? c * rcp(sqrtf(en)) : 0;
    }
    for (r = 0; r < 3; r++) {
        int b = lo[r], k;
        for (lag = lo[r]; lag < hi[r]; lag += 2)
            if (score[lag] > score[b]) b = lag;
        for (k = b - 1; k <= b + 1; k++)
            if (k >= lo[r] && k < hi[r]) {
                const float *past = cur - k;
                float en = dot(past, past, 160), c = dot(cur, past, 160);
                score[k] = en > 0 ? c * rcp(sqrtf(en)) : 0;
                if (score[k] > score[b]) b = k;
            }
        best[r] = b;
    }
    {   /* 一番強い候補と、短いラグで十分強い候補 */
        int top = best[0], second = -1;
        for (r = 1; r < 3; r++)
            if (score[best[r]] > score[top]) top = best[r];
        for (r = 0; r < 3; r++)
            if (best[r] < top && score[best[r]] > 0.85f * score[top]) {
                second = top;
                top = best[r];
                break;
            }
        if (second < 0)
            for (r = 0; r < 3; r++)
                if (best[r] != top && (second < 0 || score[best[r]] > score[second])) second = best[r];
        out[0] = top;
        out[1] = second;
    }
}

/* ---------------------------------------------------------------- 固定符号帳 */

typedef struct {
    float score;
    int index;
} cand;

static void push_cand(cand *list, int *n, int keep, float score, int index) {
    int j;
    if (keep < 1 || keep > MAXC) return;
    if (*n == keep && score <= list[keep - 1].score) return;
    j = *n < keep ? (*n)++ : keep - 1;
    while (j > 0 && list[j - 1].score < score) {
        list[j] = list[j - 1];
        j--;
    }
    list[j].score = score;
    list[j].index = index;
}

/* num^2 / den が上位 keep 個に入るなら加える(入らないものは割り算をしない) */
static void offer(cand *list, int *n, int keep, float num, float den, int index) {
    if (den <= 1e-9f || keep < 1 || keep > MAXC) return;
    if (*n == keep && num * num <= list[keep - 1].score * den) return;
    push_cand(list, n, keep, num * num * rcp(den), index);
}

static void sharpened_h(const float h[80], mrl_lag lag, int gain, float out[80]) {
    int n, t = lag.integer;
    memcpy(out, h, sizeof(float) * 80);
    if (t >= 80) return;
    for (n = t; n < 80; n++) out[n] += gain / 16384.0f * h[n - t];
}

/* 区分 0..4: パルス k の形をフィルタに通した g_k、位置 p の原子は g_k[n − p](n ≥ p)。
 * 相関は d = H^T x と形の内積(20 タップ)、エネルギーは g_k の 2 乗の累積和、原子どうしの相関は重なる部分の内積 */
static void search_shapes(const mrl_enc_config *cfg, int cls, const float h[80], const float d[80], float g[3][80], cand *list,
                          int *nlist, int keep) {
    int pulses = MRL_FCB_PULSES[cls], k, i, j, M = pulses == 3 ? cfg->m3 : cfg->m2;
    float cum[3][81], corr[3][40], en[3][40];
    int sel[3][MAXC], nsel[3] = {0, 0, 0}, sign[3][40], pos[3][MAXC];
    float cross[3][MAXC][MAXC];   /* cross[pair][i][j]: pair 0 = (0,1), 1 = (0,2), 2 = (1,2) */
    if (M > MAXC) M = MAXC;
    PROF(P_SHAPES);
    for (k = 0; k < pulses; k++) {
        const float *shape = SHAPES + 20 * (MRL_FCB_SHAPE_SEL[3 * cls] + k);
        int radix = MRL_FCB_RADIX[3 * cls + k], n;
        {   /* g[n] = Σ_{m ≤ min(n, 19)} shape[m] h[n − m] */
            const float *sr = SHAPES_R + 20 * (MRL_FCB_SHAPE_SEL[3 * cls] + k);
            for (n = 0; n < 19; n++) g[k][n] = dot(sr + 19 - n, h, n + 1);
            for (; n < 80; n++) g[k][n] = dot(sr, h + n - 19, 20);
        }
        PROF(P_SH_G);
        cum[k][0] = 0;   /* cum[k][L] = Σ_{n<L} g_k[n]^2 */
        for (n = 0; n < 80; n++) cum[k][n + 1] = cum[k][n] + g[k][n] * g[k][n];
        for (i = 0; i < radix; i++) {
            int p = MRL_FCB_POSITIONS[40 * (3 * cls + k) + i];
            float c = dot(shape, d + p, p + 20 <= 80 ? 20 : 80 - p);
            corr[k][i] = c < 0 ? -c : c;
            sign[k][i] = c >= 0;
            en[k][i] = cum[k][80 - p] > 1e-9f ? cum[k][80 - p] : 1e-9f;
        }
        nsel[k] = radix < M ? radix : M;
        top_ratio(corr[k], en[k], radix, nsel[k], sel[k]);
        for (i = 0; i < nsel[k]; i++) pos[k][i] = MRL_FCB_POSITIONS[40 * (3 * cls + k) + sel[k][i]];
        PROF(P_SH_CORR);
    }
    for (k = 0; k < pulses; k++)
        for (j = k + 1; j < pulses; j++) {
            int pair = k + j - 1, a, b;
            for (a = 0; a < nsel[k]; a++)
                for (b = 0; b < nsel[j]; b++) {
                    int p = pos[k][a], q = pos[j][b];
                    float sv = p >= q ? dot(g[k], g[j] + (p - q), 80 - p) : dot(g[k] + (q - p), g[j], 80 - q);
                    if (!sign[k][sel[k][a]] != !sign[j][sel[j][b]]) sv = -sv;   /* 符号を合わせた原子どうし */
                    cross[pair][a][b] = sv;
                }
        }
    PROF(P_SH_CROSS);
    if (pulses == 2) {
        int a, b;
        for (a = 0; a < nsel[0]; a++)
            for (b = 0; b < nsel[1]; b++) {
                int ia = sel[0][a], ib = sel[1][b], idx = ia * MRL_FCB_RADIX[3 * cls + 1] + ib;
                offer(list, nlist, keep, corr[0][ia] + corr[1][ib], en[0][ia] + en[1][ib] + 2 * cross[0][a][b],
                      MRL_FCB_CLASS_BASE[cls] + ((idx << 2) | (sign[1][ib] << 1) | sign[0][ia]));
            }
    } else {
        int a, b, c;
        for (a = 0; a < nsel[0]; a++)
            for (b = 0; b < nsel[1]; b++)
                for (c = 0; c < nsel[2]; c++) {
                    int ia = sel[0][a], ib = sel[1][b], ic = sel[2][c];
                    int idx = (ia * MRL_FCB_RADIX[3 * cls + 1] + ib) * MRL_FCB_RADIX[3 * cls + 2] + ic;
                    offer(list, nlist, keep, corr[0][ia] + corr[1][ib] + corr[2][ic],
                          en[0][ia] + en[1][ib] + en[2][ic] + 2 * (cross[0][a][b] + cross[1][a][c] + cross[2][b][c]),
                          MRL_FCB_CLASS_BASE[cls] + ((idx << 3) | (sign[2][ic] << 2) | (sign[1][ib] << 1) | sign[0][ia]));
                }
    }
    PROF(P_SH_COMB);
}

/* 区分 5: 表 A の行 u と表 B の行 l(各 5 個の単位パルス、符号は表で決まる)。phi は H^T H(下三角だけ)、d は H^T x。
 * 行は相関で 2 malt 個に絞ってからエネルギーを計算し、正規化した値で malt 個に絞って組み合わせる */
#define PHI(a, b) ((a) >= (b) ? phi[a][b] : phi[b][a])
static void search_alt(const mrl_enc_config *cfg, const float h[80], const float d[80], cand *list, int *nlist, int keep) {
    static float phi[80][80];   /* 大きいので static(エンコーダは 1 つずつ動く前提) */
    float rc[2][128], re[2][128];
    int sel[2][MAXC], i, j, t, M = cfg->malt > MAXC ? MAXC : cfg->malt, M1 = 2 * M > 2 * MAXC ? 2 * MAXC : 2 * M;
    for (i = 0; i < 80; i++) {   /* phi[p][p−i] = Σ_{j ≤ 79−p} h[j] h[j+i]、p = 79 − n(対角を後ろから) */
        float sv = 0, *q = &phi[79][79 - i];
        const float *ha = h, *hb = h + i;
        int n;
        for (n = 0; n < 80 - i; n++, q -= 81) {
            sv += ha[n] * hb[n];
            *q = sv;
        }
    }
    PROF(P_ALT_PHI);
    for (t = 0; t < 2; t++) {
        int pre[2 * MAXC], np = 0, a, b, q;
        float one[128];
        for (i = 0; i < 128; i++) {   /* 相関(5 項) */
            const uint8_t *p = ALT_P[t][i];
            const float *sg = ALT_S[t][i];
            float c = sg[0] * d[p[0]] + sg[1] * d[p[1]] + sg[2] * d[p[2]] + sg[3] * d[p[3]] + sg[4] * d[p[4]];
            rc[t][i] = c;
            one[i] = 1;
        }
        top_ratio_pos(rc[t], one, 128, M1, pre, &np);
        for (q = 0; q < np; q++) {   /* エネルギー(15 項) */
            const uint8_t *p = ALT_P[t][pre[q]];
            const float *sg = ALT_S[t][pre[q]];
            float e = 0;
            for (a = 0; a < 5; a++) {
                e += PHI(p[a], p[a]);
                for (b = 0; b < a; b++) e += 2 * sg[a] * sg[b] * PHI(p[a], p[b]);
            }
            re[t][pre[q]] = e > 1e-9f ? e : 1e-9f;
        }
        {
            float c2[2 * MAXC], e2[2 * MAXC];
            int s2[MAXC], m2 = np < M ? np : M;
            for (q = 0; q < np; q++) {
                c2[q] = rc[t][pre[q]];
                e2[q] = re[t][pre[q]];
            }
            top_ratio(c2, e2, np, m2, s2);
            for (q = 0; q < m2; q++) sel[t][q] = pre[s2[q]];
            for (; q < M; q++) sel[t][q] = -1;
        }
    }
    PROF(P_ALT_ROWS);
    for (i = 0; i < M; i++)
        for (j = 0; j < M; j++) {
            int u = sel[0][i], l = sel[1][j], a, b;
            float num, cr = 0;
            if (u < 0 || l < 0) continue;
            num = rc[0][u] + rc[1][l];
            if (num <= 0) continue;
            for (a = 0; a < 5; a++) {
                int pa = ALT_P[0][u][a];
                float sa = ALT_S[0][u][a];
                for (b = 0; b < 5; b++) cr += sa * ALT_S[1][l][b] * PHI(pa, ALT_P[1][l][b]);
            }
            offer(list, nlist, keep, num, re[0][u] + re[1][l] + 2 * cr, MRL_FCB_CLASS_BASE[5] + u * 128 + l);
        }
}

/* 符号帳の index → フィルタを通したベクトルの近似 yc と、フィルタの前のベクトル cf(エネルギー用、周期強調なし) */
static void approx_code(int index, float gk[5][3][80], const float hv[80], float yc[80], float cf[80]) {
    int cls = 0, k, n;
    memset(yc, 0, sizeof(float) * 80);
    memset(cf, 0, sizeof(float) * 80);
    for (k = 5; k >= 0; k--)
        if (index >= MRL_FCB_CLASS_BASE[k]) {
            cls = k;
            break;
        }
    if (cls == 5) {
        int r[2], t;
        r[0] = (index - MRL_FCB_CLASS_BASE[5]) >> 7;
        r[1] = (index - MRL_FCB_CLASS_BASE[5]) & 127;
        for (t = 0; t < 2; t++)
            for (k = 0; k < 5; k++) {
                int p = ALT_P[t][r[t]][k];
                float a = 8192 * ALT_S[t][r[t]][k];
                cf[p] += a;
                for (n = p; n < 80; n++) yc[n] += a * hv[n - p];
            }
        return;
    }
    {
        int pulses = MRL_FCB_PULSES[cls], rest = index - MRL_FCB_CLASS_BASE[cls], slot[3], sg[3];
        for (k = 0; k < pulses; k++) sg[k] = (rest >> k) & 1;
        rest >>= pulses;
        for (k = pulses - 1; k >= 0; k--) {
            int radix = MRL_FCB_RADIX[3 * cls + k];
            slot[k] = rest % radix;
            rest /= radix;
        }
        for (k = 0; k < pulses; k++) {
            int p = MRL_FCB_POSITIONS[40 * (3 * cls + k) + slot[k]];
            const float *shape = SHAPES + 20 * (MRL_FCB_SHAPE_SEL[3 * cls] + k), *g = gk[cls][k];
            float a = sg[k] ? 1.0f : -1.0f;
            for (n = p; n < 80; n++) yc[n] += a * g[n - p];
            for (n = 0; n < 20 && p + n < 80; n++) cf[p + n] += a * shape[n];
        }
    }
}

/* ---------------------------------------------------------------- サブフレーム */

/* 適応符号帳の浮動小数の近似(デコーダの mrl_predict と同じ補間、丸めなし)。ext: 励振(ext[at] がサブフレームの先頭) */
static void adaptive_float(const float *ext_in, int at, mrl_lag lag, float v[80]) {
    float ext[314];
    int phase = -lag.frac, delay = lag.integer, n, k;
    memcpy(ext, ext_in, sizeof(float) * at);
    if (phase < 0) {
        phase += 3;
        delay++;
    }
    for (n = 0; n < 80; n++) {
        int c = at + n - delay;
        ext[at + n] = v[n] = dot(ext + c - 9, FWD[phase], 10) + dot(ext + c + 1, BWD[phase], 10);
    }
    (void)k;
}

/* 候補の符号の正規化相関の 2 乗(浮動小数の適応符号帳をフィルタに通して)。比べるだけに使う */
static float frac_score(const mrl_state *s, int sub, const float *ext, const float h[80], const float x[80], int code) {
    mrl_lag l = code_lag(sub, code, s->previous_lag);
    float v[80], y[80], num, den;
    adaptive_float(ext, 154 + 80 * sub, l, v);
    convolve_dense(h, v, y);
    num = dot(y, x, 80);
    den = dot(y, y, 80);
    return num > 0 && den > 0 ? num * num * rcp(den) : 0;
}

/* ピッチ: 整数のラグ [lo, hi] を調べる。分母 |H v_k|^2 は、補間フィルタ(位相 0)を通した過去の励振 z を 1 sample ずつずらす
 * 再帰(G.729 と同じ。ラグがサブフレームより短いときは範囲の最小のラグで延ばす近似)、分子 v_k · H^T x は正確な適応符号帳で。
 * 一番良い整数のラグに一番近い符号と、その前後の符号(±1/3 sample)を、浮動小数の適応符号帳の正規化相関で比べる。
 * 戻り値: 符号。score: 小数まで詰めた後の正規化相関の 2 乗(範囲どうしを比べる) */
static int pitch_range(const mrl_state *s, int sub, const float *ext, const float h[80], const float x[80], const float d[80],
                       int lo, int hi, int fine, float *score) {
    int at = 154 + 80 * sub, k, n, best_int = -1, best = 0, c, c0 = 0, nc = sub == 0 ? 256 : 32, dist = 1 << 30;
    float buf[314], y[80], bs = -1, bf = -1, bden = 1, bnum = 0, sc_int[144];
    int side = 0;
    *score = -1;
    if (lo < 20) lo = 20;
    if (hi > 143) hi = 143;
    if (lo > hi) return -1;
    /* w: 過去の励振 + 最小のラグ lo の適応符号帳で延ばした分。z = w に補間フィルタ(位相 0)をかけたもの:
     * 小数 0 のラグ k の適応符号帳は v_k[n] ≈ z[at + n − k](k が 90 以上なら正確) */
    {
        float w[314];
        mrl_lag l0 = {lo, 0};
        int m, base = at - hi - 1;
        adaptive_float(ext, at, l0, w + at);
        memcpy(w, ext, sizeof(float) * at);
        for (m = base; m < at + 80 - lo; m++) buf[m] = dot(w + m - 9, FWD[0], 10) + dot(w + m + 1, BWD[0], 10);
        convolve_dense(h, buf + at - lo, y);
    }
    for (k = lo;; k++) {
        /* 分母 |H v_k|^2 は再帰の近似で、分子 v_k · d は正確に(順位は分子で大きく変わる) */
        float num, den = dot(y, y, 80);
        if (k >= 90) {
            num = dot(buf + at - k, d, 80);   /* 補間の窓が全部過去に入るので z のままで正確 */
        } else {   /* n ≤ k − 11 は z のまま正確、残りはデコーダと同じ再帰(自分の前の値を使う) */
            float wv[314], *v = wv + at;
            int cut = k - 10;
            memcpy(wv + at - k - 10, ext + at - k - 10, sizeof(float) * (k + 10));
            memcpy(v, buf + at - k, sizeof(float) * cut);
            for (n = cut; n < 80; n++) {
                int c = at + n - k;
                v[n] = dot(wv + c - 9, FWD[0], 10) + dot(wv + c + 1, BWD[0], 10);
            }
            num = dot(v, d, 80);
        }
        sc_int[k - lo] = num > 0 && den > 0 ? num * num * rcp(den) : 0;
        if (num > 0 && den > 0 && num * num > bs * den) {
            bs = num * num * rcp(den);
            bden = den;
            bnum = num;
            best_int = k;
        }
        if (k == hi) break;
        {
            float u = buf[at - k - 1];
            for (n = 79; n > 0; n--) y[n] = y[n - 1] + u * h[n];
            y[0] = u * h[0];
        }
    }
    if (best_int < 0) best_int = lo;
    PROF(P_P_INT);
    if (!fine && bs < 0.1f * dot(x, x, 80)) fine = -1;   /* ピッチの寄与が小さい(無声)なら小数は調べない */
    for (c = 0; c < nc; c++) {   /* 整数のラグに一番近い符号(範囲の端では小数 0 の符号が無いことがある) */
        mrl_lag l = code_lag(sub, c, s->previous_lag);
        int dd = 3 * l.integer + l.frac - 3 * best_int;
        if (dd < 0) dd = -dd;
        if (dd < dist) {
            dist = dd;
            c0 = c;
        }
    }
    {   /* 整数のラグの前後の得点(正規化相関の 2 乗)の高い側へ、小数を動かす候補だけ調べる */
        float lo_s = best_int > lo ? sc_int[best_int - 1 - lo] : -1, hi_s = best_int < hi ? sc_int[best_int + 1 - lo] : -1;
        side = 0;
        if ((lo_s >= 0 || hi_s >= 0) && c0 + 1 < nc) {
            mrl_lag lp = code_lag(sub, c0 + 1, s->previous_lag);
            int up = 3 * lp.integer + lp.frac > 3 * best_int;   /* c0 + 1 が長いラグ側か */
            int longer = hi_s > lo_s;                            /* 長いラグ側の得点が高い */
            side = (longer == up) ? 1 : -1;
        }
    }
    for (c = c0 - 1; c <= c0 + 1; c++) {
        float sc;
        if (c < 0 || c >= nc || (fine < 0 && c != c0)) continue;
        if (fine == 0 && c != c0 && side != 0 && (c - c0) != side) continue;   /* 傾きの高い側だけ */
        if (fine > 0) {
            sc = frac_score(s, sub, ext, h, x, c);
        } else {   /* 分母は一番良い整数のラグのもの(近似)。分子 v · H^T x だけ正確に */
            mrl_lag l = code_lag(sub, c, s->previous_lag);
            float num;
            if (l.integer == best_int && l.frac == 0) {
                num = bnum;   /* 整数の段で計算済み */
            } else {
                float v[80];
                adaptive_float(ext, at, l, v);
                num = dot(v, d, 80);
            }
            sc = num > 0 ? num * num * rcp(bden) : 0;
        }
        if (sc > bf) {
            bf = sc;
            best = c;
        }
    }
    PROF(P_P_FRAC);
    *score = bf;
    return best;
}

static void encode_subframe(mrl_encoder *e, int sub, const float target[80], const float a[11], const int16_t lpcq[11],
                            const int ol[2], mrl_half *half) {
    const mrl_enc_config *cfg = &e->cfg;
    float aw1[11], aw2[11], aq[11], zero[80], zm[10], tmp[80], x[80], h[80], yv[80], gp, g1 = 1, g2 = 1, dummy[10];
    float hv[80], hf[80], x2[80], gk[5][3][80];
    int i, pcode, nlist = 0, keep = cfg->keep > MAXC ? MAXC : (cfg->keep < 1 ? 1 : cfg->keep);
    mrl_lag lag;
    cand list[MAXC];
    float best_err = 1e30f;
    int best_index = 0, best_gain = 0;
    int16_t best_code[80];
    mrl_state trial;
    for (i = 0; i < 11; i++) {
        aw1[i] = a[i] * g1;
        aw2[i] = a[i] * g2;
        aq[i] = lpcq[i] / 4096.0f;
        g1 *= cfg->gamma1;
        g2 *= cfg->gamma2;
    }
    /* 目標: 零入力応答を引いて重み付け */
    for (i = 0; i < 10; i++) zm[i] = e->s.synthesis_memory[i];
    memset(tmp, 0, sizeof tmp);
    memset(dummy, 0, sizeof dummy);
    pole_zero(aq, 1, aq, tmp, zero, 80, dummy, zm);
    for (i = 0; i < 80; i++) tmp[i] = target[i] - zero[i];
    {
        float wi[10], wo[10];
        memcpy(wi, e->w_in, sizeof wi);
        memcpy(wo, e->w_out, sizeof wo);
        pole_zero(aw1, 11, aw2, tmp, x, 80, wi, wo);
    }
    /* インパルス応答 W(z)/Aq(z): A(z/γ1) を 1/A(z/γ2) と 1/Aq(z) に順に通す(前に 0 を置いて内積の形) */
    {
        float t1[90], t2[90], r2[10], rq[10];
        memset(t1, 0, sizeof(float) * 10);
        memset(t2, 0, sizeof(float) * 10);
        for (i = 0; i < 10; i++) {
            r2[i] = aw2[10 - i];
            rq[i] = aq[10 - i];
        }
        for (i = 0; i < 80; i++) t1[10 + i] = (i < 11 ? aw1[i] : 0.0f) - dot(t1 + i, r2, 10);
        for (i = 0; i < 80; i++) t2[10 + i] = t1[10 + i] - dot(t2 + i, rq, 10);
        memcpy(h, t2 + 10, sizeof(float) * 80);
    }
    PROF(P_TARGET);
    /* ピッチ */
    {
        float ext[234], d[80], s1, s2 = -1;
        int c1, c2 = -1, at = 154 + 80 * sub;
        for (i = 0; i < at; i++) ext[i] = e->s.excitation[i];
        backward(h, x, d);
        PROF(P_P_BACK);
        if (sub == 0) {
            c1 = pitch_range(&e->s, 0, ext, h, x, d, ol[0] - cfg->pitch_w0, ol[0] + cfg->pitch_w0, cfg->pitch_fine, &s1);
            if (cfg->pitch_w1 >= 0 && ol[1] > 0 && (ol[1] < ol[0] - cfg->pitch_w0 || ol[1] > ol[0] + cfg->pitch_w0))
                c2 = pitch_range(&e->s, 0, ext, h, x, d, ol[1] - cfg->pitch_w1, ol[1] + cfg->pitch_w1, cfg->pitch_fine, &s2);
            pcode = (c2 >= 0 && s2 > s1) ? c2 : c1;
            if (pcode < 0) pcode = 0;
            half->absolute_pitch = pcode;
            lag = ABS_LAG[pcode];
        } else {
            int lo_r = e->s.previous_lag - 5 > 20 ? e->s.previous_lag - 5 : 20;
            if (lo_r + 9 > 143) lo_r = 134;
            {
                int c = e->s.previous_lag, lo1 = c - cfg->pitch_w2, hi1 = c + cfg->pitch_w2;   /* 差分の範囲 [lo_r, lo_r + 10] の中で */
                if (lo1 < lo_r) lo1 = lo_r;
                if (hi1 > lo_r + 10) hi1 = lo_r + 10;
                pcode = pitch_range(&e->s, 1, ext, h, x, d, lo1, hi1, cfg->pitch_fine, &s1);
            }
            if (pcode < 0) pcode = 0;
            half->relative_pitch = pcode;
            lag = rel_lag(pcode, e->s.previous_lag);
        }
    }
    PROF(P_PITCH);
    /* 正確な適応符号帳のベクトル(デコーダの状態を進める)と、そのピッチ利得 */
    trial = e->s;
    mrl_prepare(&trial, sub, lpcq, lag);
    {
        float vf[80], num, den;
        for (i = 0; i < 80; i++) vf[i] = trial.excitation[154 + 80 * sub + i];
        convolve_dense(h, vf, yv);
        num = dot(yv, x, 80);
        den = dot(yv, yv, 80);
        gp = den > 0 && num > 0 ? num * rcp(den) : 0;
        if (gp > 1.2f) gp = 1.2f;
    }
    /* 固定符号帳(周期強調の近似つきのインパルス応答) */
    sharpened_h(h, lag, trial.sharpen_fixed, hf);
    sharpened_h(h, lag, trial.sharpen_voiced, hv);
    for (i = 0; i < 80; i++) x2[i] = x[i] - gp * yv[i];
    {
        int cls;
        float df[80], dv[80];
        backward(hf, x2, df);
        if (lag.integer < 80)
            backward(hv, x2, dv);
        else
            memcpy(dv, df, sizeof dv);   /* 周期強調が無ければ同じ */
        for (cls = 0; cls < 5; cls++)
            search_shapes(cfg, cls, (cls >= 1 && cls < 3) ? hv : hf, (cls >= 1 && cls < 3) ? dv : df, gk[cls], list, &nlist, keep);
        PROF(P_SHAPES);
        search_alt(cfg, hv, dv, list, &nlist, keep);
        PROF(P_ALT);
    }
    if (nlist == 0) push_cand(list, &nlist, keep, 0, 0);   /* 無音などで何も残らないとき */
    /* 上位の候補ごとに 128 通りのゲインを比べる。フィルタを通した符号ベクトルは、探索で作った g_k(パルスの形)や
     * h(単位パルス)を足し合わせた近似で作る(正確な周期強調は選んだ 1 つだけ、デコーダの状態を進めるときに) */
    {
        float ex = dot(x, x, 80), vv = dot(yv, yv, 80), xv = dot(x, yv, 80), P1[128];
        int c;
        for (i = 0; i < 128; i++) P1[i] = ex + PITCH_GAINS[i] * (PITCH_GAINS[i] * vv - 2 * xv);
        for (c = 0; c < nlist; c++) {
            float yc[80], cf[80], cc, vc2, xc2, base, energy = 0;
            int gi;
            approx_code(list[c].index, gk, hv, yc, cf);
            for (i = 0; i < 80; i++) energy += cf[i] * cf[i];
            cc = dot(yc, yc, 80);
            vc2 = 2 * dot(yv, yc, 80);
            xc2 = 2 * dot(x, yc, 80);
            base = expf(mrl_gain_prediction(trial.gain_history, energy * (1.0f / 80))) * (1.0f / 8192);
            for (gi = 0; gi < 128; gi++) {   /* |x − gp yv − gc yc|^2(gc の丸めは省く) */
                float gc = base * EXP_LG[gi], err;
                if (gc > 4.0f) gc = 4.0f;
                err = P1[gi] + gc * (gc * cc - xc2 + PITCH_GAINS[gi] * vc2);
                if (err < best_err) {
                    best_err = err;
                    best_index = list[c].index;
                    best_gain = gi;
                }
            }
        }
        /* 選んだ符号ベクトルは正確に作り(周期強調つき)、128 通りのゲインを正確な値(丸めも)で選び直す */
        mrl_innovation(&trial, best_index, lag, best_code);
        {
            float cf[80], yc[80], cc, vc2, xc2, base;
            int gi;
            for (i = 0; i < 80; i++) cf[i] = best_code[i];
            convolve(h, cf, yc);
            cc = dot(yc, yc, 80);
            vc2 = 2 * dot(yv, yc, 80);
            xc2 = 2 * dot(x, yc, 80);
            base = expf(mrl_gain_prediction(trial.gain_history, mrl_code_energy(best_code)));
            best_err = 1e30f;
            for (gi = 0; gi < 128; gi++) {
                float gc = (float)(int)(base * EXP_LG[gi] + 0.5f), err;
                if (gc > 32767) gc = 32767;
                gc *= 1.0f / 8192;
                err = P1[gi] + gc * (gc * cc - xc2 + PITCH_GAINS[gi] * vc2);
                if (err < best_err) {
                    best_err = err;
                    best_gain = gi;
                }
            }
        }
    }
    PROF(P_GAINS);
    /* デコーダの状態を正確に進める */
    {
        int16_t synth[80];
        float d[80], wd[80];
        int pg, cg;
        e->s = trial;
        mrl_gain_decode(e->s.gain_history, best_gain, best_code, &pg, &cg);
        mrl_finish(&e->s, sub, lpcq, best_code, pg, cg, synth);
        for (i = 0; i < 80; i++) {
            d[i] = target[i] - synth[i];
            e->target_energy += target[i] * target[i];
            e->error_energy += d[i] * d[i];
        }
        pole_zero(aw1, 11, aw2, d, wd, 80, e->w_in, e->w_out);
        memcpy(e->last_synth + 80 * sub, synth, sizeof synth);
    }
    half->code[sub] = best_index;
    half->gain[sub] = best_gain;
}

/* ---------------------------------------------------------------- フレーム */

/* sp の中の位置: 絶対位置 a は sp[a − (320F − 640)](F = これまでに push したフレーム数、符号化中のフレームは F − 1) */
static void encode_frame(mrl_encoder *e, mrl_half halves[2]) {
    int j, h;
    const int FS = 320;   /* 符号化するフレームの先頭の sp の中の位置 */
    PROF(P_STATE);
    for (j = 0; j < 4; j++) {   /* 4 サブフレームの線形予測(窓の中心 = サブフレームの中心) */
        int c = FS + 80 * j + 40;
        float a[11], lsf[10];
        lpc_analysis(e->sp + c - 120, a);
        if (!lpc_to_lsf(a, lsf)) {
            memcpy(lsf, e->prev_lsf_rad, sizeof lsf);
            lsf_to_lpc(lsf, a);
        }
        memcpy(e->prev_lsf_rad, lsf, sizeof lsf);
        memcpy(e->lpc[j], a, sizeof a);
        for (h = 0; h < 10; h++) e->lsf[j][h] = lsf[h] * Q13_PER_RAD;
    }
    for (h = 0; h < 2; h++) {
        int t = FS + 160 * h, sub, ol[2], k;
        int16_t lsf[10], lsp[10], lpcq[2][11];
        float res[10];
        /* 重みつきの音声(この半フレームの 2 サブフレーム)と開ループのピッチ */
        for (sub = 0; sub < 2; sub++) {
            float aw1[11], aw2[11], g1 = 1, g2 = 1;
            for (k = 0; k < 11; k++) {
                aw1[k] = e->lpc[2 * h + sub][k] * g1;
                aw2[k] = e->lpc[2 * h + sub][k] * g2;
                g1 *= e->cfg.gamma1;
                g2 *= e->cfg.gamma2;
            }
            pole_zero(aw1, 11, aw2, e->sp + t + 80 * sub, e->sw + t + 80 * sub, 80, e->sw_x, e->sw_y);
        }
        PROF(P_LPC);
        open_loop(e->sw + t + 160, ol);
        PROF(P_OL);
        memset(&halves[h], 0, sizeof halves[h]);
        quantize_lsf(e, e->lsf[2 * h + 1], e->lsf[2 * h], &halves[h]);
        mrl_lsf_decode((const float(*)[10])e->s.spectral_history, &halves[h], lsf, res);
        mrl_spectral_push(e->s.spectral_history, res);
        for (k = 0; k < 10; k++) e->prev_q[k] = lsf[k];
        mrl_lsf_to_lsp(lsf, lsp);
        mrl_begin_half(&e->s, lsp, lpcq[0], lpcq[1]);
        PROF(P_LSF);
        for (sub = 0; sub < 2; sub++) encode_subframe(e, sub, e->sp + t + 80 * sub, e->lpc[2 * h + sub], lpcq[sub], ol, &halves[h]);
        /* 半フレームの終わり(デコーダの mrl_finish_half から出力フィルタを除いたもの: 出力フィルタの状態は符号化に使わない) */
        memmove(e->s.excitation, e->s.excitation + 160, sizeof(int16_t) * 154);
        e->s.last_half_lag = e->s.previous_lag;
        PROF(P_STATE);
    }
}

int mrl_encode_push(mrl_encoder *e, const int16_t pcm[320], uint8_t payload[20]) {
    int i;
    memmove(e->sp, e->sp + 320, sizeof(float) * (BUF - 320));
    memmove(e->sw, e->sw + 320, sizeof(float) * (BUF - 320));
    for (i = 0; i < 320; i++) {   /* 2 次の Butterworth 高域通過(80 Hz)、出力フィルタの利得で割る */
        float x = pcm ? pcm[i] : 0.0f;
        float y = 0.95654f * x - 1.91309f * e->hp_x[0] + 0.95654f * e->hp_x[1] + 1.91119f * e->hp_y[0] - 0.91497f * e->hp_y[1];
        e->hp_x[1] = e->hp_x[0];
        e->hp_x[0] = x;
        e->hp_y[1] = e->hp_y[0];
        e->hp_y[0] = y;
        e->sp[BUF - 320 + i] = y * (1.0f / 1.1999658f);
    }
    e->frames++;
    if (e->frames < 2) return 0;   /* 先読み(次のフレームの 10 ms)が要る */
    {
        mrl_half halves[2];
        encode_frame(e, halves);
        mrl_pack(halves, payload);
    }
    return 1;
}

int mrl_encode_flush(mrl_encoder *e, uint8_t payload[20]) {
    if (e->frames < 1) return 0;
    return mrl_encode_push(e, NULL, payload);
}

void mrl_encoder_stats(const mrl_encoder *e, float *target_energy, float *error_energy) {
    *target_energy = e->target_energy;
    *error_energy = e->error_energy;
}
