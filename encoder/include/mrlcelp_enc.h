/* 改良型 RL-CELP(Modified RL-CELP)のエンコーダ(C99、malloc なし)。デコーダ(../decoder)の状態の処理を使う。
 * 1 フレーム = 40 ms:8 kHz・16 bit の PCM 320 sample → 20 byte(158 bit、LSB first)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#ifndef MRLCELP_ENC_H
#define MRLCELP_ENC_H
#include "mrlcelp.h"
#ifdef __cplusplus
extern "C" {
#endif

/* エンコーダ: 状態は mrl_encoder_size() byte(呼ぶ側が用意する、8 byte 境界)。
 * mrl_encode_push に 320 sample(40 ms)ずつ渡すと、1 つ前のフレームのペイロードを返す(戻り値 1)。最初の 1 回は 0
 * (先読み 10 ms のため 1 フレーム遅れる)。最後に mrl_encode_flush で残りの 1 フレームを出す。
 * 最後のフレームが半端なら 0 で埋めて渡す。 */
typedef struct {
    float gamma1, gamma2;   /* 聴感の重み付け W(z) = A(z/γ1)/A(z/γ2) */
    int lsf_stage1, lsf_stage2;   /* LSF の 1 段・2 段の候補の数(≤ 16) */
    float sub0_weight;      /* LSF の誤差に前半のサブフレーム(補間)の分を足す重み */
    int m3, m2, malt;       /* 固定符号帳: 3 パルス・2 パルスの区分のパルスごと、単位パルスの区分の表ごとに残す位置の数(≤ 16) */
    int keep;               /* 正確なベクトルと 128 通りのゲインで比べる符号帳の候補の数(≤ 16) */
    int pitch_w0, pitch_w1; /* 前半のサブフレームのピッチ: 開ループの 1 つ目 ± w0、2 つ目 ± w1(負なら調べない)の整数のラグ */
    int pitch_w2;           /* 後半のサブフレーム: 前のラグ ± w2(差分で表せる範囲の中) */
    int pitch_fine;         /* 1 なら小数の候補も畳み込みで正確に比べる(重い) */
} mrl_enc_config;

typedef struct mrl_encoder mrl_encoder;
void mrl_enc_default_config(mrl_enc_config *c);
size_t mrl_encoder_size(void);
mrl_encoder *mrl_encoder_init(void *mem, const mrl_enc_config *cfg);   /* cfg = NULL で既定 */
int mrl_encode_push(mrl_encoder *e, const int16_t pcm[MRL_FRAME_SAMPLES], uint8_t payload[MRL_FRAME_BYTES]);
int mrl_encode_flush(mrl_encoder *e, uint8_t payload[MRL_FRAME_BYTES]);
/* 合成の誤差の統計(出力フィルタの前、評価用) */
void mrl_encoder_stats(const mrl_encoder *e, float *target_energy, float *error_energy);

#ifdef __cplusplus
}
#endif
#endif
