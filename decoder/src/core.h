/* デコーダの状態と処理(エンコーダも同じものを使う)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#ifndef MRL_CORE_H
#define MRL_CORE_H
#include <stdint.h>

typedef struct { int integer, frac; } mrl_lag;

typedef struct {
    int16_t previous_lsp[10];
    int16_t excitation[314];
    int16_t synthesis_memory[10];
    int sharpen_fixed, sharpen_voiced, last_half_lag, previous_lag;
    int32_t fir[6], iir[6];
    float gain_history[4];
    float spectral_history[3][10];
} mrl_state;

typedef struct {
    int mode, stage1, stage2_low, stage2_high, absolute_pitch, relative_pitch;
    int code[2], gain[2];
} mrl_half;

void mrl_state_init(mrl_state *s);
void mrl_unpack(const uint8_t payload[20], mrl_half halves[2]);
void mrl_pack(const mrl_half halves[2], uint8_t payload[20]);

/* LSF の量子化の値 → LSF(Q13、デコーダの整列・制限・丸めのあと)と、移動平均予測の残差。履歴は進めない */
void mrl_lsf_decode(const float history[3][10], const mrl_half *h, int16_t lsf[10], float residual[10]);
void mrl_spectral_push(float history[3][10], const float residual[10]);
void mrl_lsf_to_lsp(const int16_t lsf[10], int16_t lsp[10]);
void mrl_lsp_to_lpc(const int16_t lsp[10], int16_t a[11]);
void mrl_midpoint(const int16_t a[10], const int16_t b[10], int16_t out[10]);
int mrl_reflection_first(const int16_t lpc[11]);
mrl_lag mrl_decode_absolute(int code);
mrl_lag mrl_decode_relative(int code, int prev);
void mrl_predict(int16_t *exc, int at, mrl_lag lag);
void mrl_sharpen(int16_t code[80], mrl_lag lag, int gain);
int mrl_codebook_vector(int index, int16_t out[80]);   /* 区分を返す */
void mrl_interpolation_gains(int reflection, int previous_gain, int lag, int previous_lag, int *first, int *second);
void mrl_synthesis(const int16_t a[11], const int16_t *in, int16_t mem[10], int16_t *out, int n);
void mrl_mix(int16_t *exc, int at, const int16_t code[80], int pitch_gain, int code_gain);
void mrl_output_filter(int32_t fir[6], int32_t iir[6], int16_t *block, int n);

/* ゲイン: 予測の値(f32)と、index から (ピッチ利得 Q14, 符号帳利得) */
float mrl_gain_prediction(const float history[4], float energy);
float mrl_code_energy(const int16_t code[80]);
void mrl_gain_decode(float history[4], int index, const int16_t code[80], int *pitch_gain, int *code_gain);
int mrl_quantize(float x);

/* サブフレームの段階(エンコーダが候補ごとに使う)。prepare: 適応符号帳と周期強調の利得 → innovation → finish */
void mrl_prepare(mrl_state *s, int sub, const int16_t lpc[11], mrl_lag lag);
void mrl_innovation(const mrl_state *s, int code_index, mrl_lag lag, int16_t code[80]);
void mrl_finish(mrl_state *s, int sub, const int16_t lpc[11], const int16_t code[80], int pitch_gain, int code_gain,
                int16_t synth[80]);
void mrl_begin_half(mrl_state *s, const int16_t lsp[10], int16_t lpc0[11], int16_t lpc1[11]);
void mrl_finish_half(mrl_state *s, const int16_t synth[160], int16_t pcm[160]);

#endif
