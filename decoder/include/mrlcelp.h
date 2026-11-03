/* 改良型 RL-CELP(Modified RL-CELP)のデコーダ(C99、malloc なし、libm は logf / expf だけ)。
 * 1 フレーム = 40 ms = 20 byte(158 bit、LSB first)⇔ 8 kHz・16 bit の PCM 320 sample。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#ifndef MRLCELP_H
#define MRLCELP_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MRL_FRAME_SAMPLES 320
#define MRL_FRAME_BYTES 20

/* デコーダ: 状態は mrl_decoder_size() byte(呼ぶ側が用意する、8 byte 境界)。原本(jr-voice)と PCM がビット一致 */
typedef struct mrl_decoder mrl_decoder;
size_t mrl_decoder_size(void);
mrl_decoder *mrl_decoder_init(void *mem);
/* 戻り値: 0 = 成功、-1 = 上位 2 bit が 0 でない(不正なペイロード) */
int mrl_decode(mrl_decoder *d, const uint8_t payload[MRL_FRAME_BYTES], int16_t pcm[MRL_FRAME_SAMPLES]);
/* 出力フィルタの前の合成も欲しいとき(評価用、NULL 可) */
int mrl_decode_ex(mrl_decoder *d, const uint8_t payload[MRL_FRAME_BYTES], int16_t pcm[MRL_FRAME_SAMPLES],
                  int16_t pre_filter[MRL_FRAME_SAMPLES]);

/* 三菱型の復号(ビット一致の対象なし): 本体の合成のあとに、三菱 CELP(mcelp-codec)の適応ポストフィルタを掛けてから
 * 出力フィルタに通す。MRL-CELP の出力フィルタは三菱 CELP と係数まで同一で、三菱 CELP ではその直前にこのポストフィルタがある */
typedef struct mrl_mpf_decoder mrl_mpf_decoder;
size_t mrl_mpf_decoder_size(void);
mrl_mpf_decoder *mrl_mpf_decoder_init(void *mem);
int mrl_decode_mpf(mrl_mpf_decoder *d, const uint8_t payload[MRL_FRAME_BYTES], int16_t pcm[MRL_FRAME_SAMPLES]);

/* 聞きやすさ優先の復号(ビット一致ではない、浮動小数)。パラメータの復号は上と同じで、励振・合成をやり直して後処理を掛ける。
 * 状態は mrl_enhanced_size() byte(8 byte 境界)。cfg = NULL で既定の設定 */
typedef struct {
    int float_synthesis;   /* 1: 浮動小数の励振・合成(飽和しない) */
    int anti_sparse;       /* 固定符号帳の位相分散(float_synthesis のときだけ): 0 = なし、1 = ピッチ利得だけで判断、2 = 疎なベクトルだけ */
    int postfilter;        /* 適応ポストフィルタ: 0 = なし、1 = G.729 型(浮動小数)、2 = 三菱 CELP 型(固定小数点、mcelp-codec の移植) */
    int limiter;           /* 1: ソフトリミッタ */
    float gamma_n, gamma_d, gamma_p, gamma_t;   /* ポストフィルタ: 短期の分子・分母、長期、傾き(G.729 と同じ 0.55 / 0.7 / 0.5 / 0.8) */
    float gain;            /* 出力の倍率(既定 0.5) */
    float low_cut_hz;      /* 0 より大きければ、その周波数の 2 次のハイパスを足す(低域の膨らみを抑える EQ) */
    float level_rms;       /* 0 より大きければ、声のある所の rms をこの値にそろえる(ゆっくりした AGC、−6〜+12 dB) */
    int pitch_enhance;     /* 1: AMR-WB のピッチ強調 */
    int noise_enhance;     /* 1: AMR-WB のノイズ強調 */
    int bass_postfilter;   /* 1: G.718 型のバスポストフィルタ */
    float bass_strength;   /* バスポストフィルタの α の上限(既定 0.25。大きいほど低域の倍音の間を消す。強すぎるとブザーのような声) */
} mrl_enh_config;
typedef struct mrl_enhanced mrl_enhanced;
void mrl_enh_default_config(mrl_enh_config *c);
size_t mrl_enhanced_size(void);
mrl_enhanced *mrl_enhanced_init(void *mem, const mrl_enh_config *cfg);
/* 出力は 160 sample(20 ms)遅れる(バスポストフィルタの先読み)。最後に mrl_enhanced_flush で残りの 160 sample を出す */
int mrl_decode_enhanced(mrl_enhanced *e, const uint8_t payload[MRL_FRAME_BYTES], int16_t pcm[MRL_FRAME_SAMPLES]);
int mrl_enhanced_flush(mrl_enhanced *e, int16_t pcm[MRL_FRAME_SAMPLES / 2]);
long mrl_enhanced_limited(const mrl_enhanced *e);   /* リミッタが働いた sample 数 */

/* 帯域拡張(8 kHz → 16 kHz)。どの復号の出力にも付けられる後処理。4 kHz より上は、声としてもっともらしく作った音(本物ではない)。
 * 80 sample(10 ms)ずつ 160 sample にする。出力は mrl_widen_delay() sample(16 kHz)遅れる */
typedef struct {
    float gain;            /* 高域の大きさ(1 = 3〜3.8 kHz と同じ密度 × 傾きの係数。既定 1.5) */
    float low_hz, high_hz; /* 足す帯域(既定 3900〜6000 Hz) */
    float env_hz;          /* 高域の音源の包絡の平滑(既定 150 Hz。大きいほどピッチの揺れが残り、ざらつきやすい) */
    float fac_power;       /* 傾きの係数 1 − tilt のべき(既定 2。大きいほど母音に足さない) */
    float slope_hz;        /* 高域の傾き(1 次の低域通過の周波数、既定 4500 Hz。0 で平ら) */
    int source;            /* 高域の音源: 0 = 雑音 × 包絡、1 = 1.2〜3.8 kHz の全波整流(有声音では倍音が続く、既定)、2 = 有声の度合いで混ぜる */
    float aperiodic;       /* source = 1 のとき、有声音にも混ぜる雑音の割合(パワー、0〜1。既定 0.1) */
    int square;            /* 1: 全波整流 |x| の代わりに 2 乗 x²(2 次の和と差の周波数だけを作る。既定 1) */
} mrl_widen_config;
typedef struct mrl_widen mrl_widen;
void mrl_widen_default_config(mrl_widen_config *c);
size_t mrl_widen_size(void);
mrl_widen *mrl_widen_init(void *mem, const mrl_widen_config *cfg);
void mrl_widen_process(mrl_widen *w, const int16_t in[80], int16_t out[160]);
int mrl_widen_delay(void);

#ifdef __cplusplus
}
#endif
#endif
