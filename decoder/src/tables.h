/* 自動生成(mrl-celp の tools/gen_c_tables.py)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai */
#ifndef MRL_TABLES_H
#define MRL_TABLES_H
#include <stdint.h>

extern const int16_t MRL_COS[64];
extern const int16_t MRL_COS_SLOPE[64];
extern const int16_t MRL_FCB_ALT_A[640];
extern const int16_t MRL_FCB_ALT_B[640];
extern const uint16_t MRL_FCB_CLASS_BASE[6];
extern const uint16_t MRL_FCB_INDEX_LIMIT[1];
extern const int16_t MRL_FCB_POSITIONS[600];
extern const int16_t MRL_FCB_PULSES[5];
extern const int16_t MRL_FCB_RADIX[15];
extern const int16_t MRL_FCB_RADIX_INV[15];
extern const int16_t MRL_FCB_SHAPES[240];
extern const int16_t MRL_FCB_SHAPE_SEL[15];
extern const int16_t MRL_LSF_CB1[1280];
extern const int16_t MRL_LSF_MEAN[10];
extern const int16_t MRL_LSP_MEAN[10];
extern const int16_t MRL_OUT_FIR[4];
extern const int16_t MRL_OUT_IIR[3];
extern const int16_t MRL_PITCH_INTERP[31];
extern const float MRL_PITCH_GAIN[128];
extern const float MRL_LOG_GAIN[128];
extern const float MRL_GAIN_BIAS1[1];
extern const float MRL_STAGE2_LOW[320];
extern const float MRL_STAGE2_HIGH[320];
extern const float MRL_MA[80];

#endif
