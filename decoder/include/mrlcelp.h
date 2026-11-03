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
} mrl_enh_config;
typedef struct mrl_enhanced mrl_enhanced;
void mrl_enh_default_config(mrl_enh_config *c);
size_t mrl_enhanced_size(void);
mrl_enhanced *mrl_enhanced_init(void *mem, const mrl_enh_config *cfg);
/* 出力は 160 sample(20 ms)遅れる(バスポストフィルタの先読み)。最後に mrl_enhanced_flush で残りの 160 sample を出す */
int mrl_decode_enhanced(mrl_enhanced *e, const uint8_t payload[MRL_FRAME_BYTES], int16_t pcm[MRL_FRAME_SAMPLES]);
int mrl_enhanced_flush(mrl_enhanced *e, int16_t pcm[MRL_FRAME_SAMPLES / 2]);
long mrl_enhanced_limited(const mrl_enhanced *e);   /* リミッタが働いた sample 数 */

#ifdef __cplusplus
}
#endif
#endif
