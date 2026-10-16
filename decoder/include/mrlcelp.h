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

#ifdef __cplusplus
}
#endif
#endif
