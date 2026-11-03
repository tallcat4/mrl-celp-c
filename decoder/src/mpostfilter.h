/* 三菱 CELP の適応ポストフィルタ(固定小数点、mcelp-codec の移植)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#ifndef MRL_MPOSTFILTER_H
#define MRL_MPOSTFILTER_H
#include <stdint.h>

#define MRL_MPF_RES_HISTORY 152
#define MRL_MPF_PROBE 20

typedef struct {
    int16_t residual[MRL_MPF_RES_HISTORY + 80];
    int16_t synthesis[10];
    int16_t previous, agc_gain;
} mrl_mpf;

void mrl_mpf_init(mrl_mpf *p);
/* 1 サブフレーム。lsp はそのサブフレームの LSP、lag0 は半フレームの 1 つ目のサブフレームの整数ラグ、
 * speech は合成音声の 10 sample の過去 + 80 sample。長期ポストフィルタが決めたラグ(0 = 切)を返す */
int mrl_mpf_subframe(mrl_mpf *p, const int16_t lsp[10], int lag0, const int16_t speech[10 + 80], int16_t out[80]);

#endif
