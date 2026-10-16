/* 固定小数点の基本演算(原本 jr-voice の fixed_core/fixed.rs と同じ)。16 bit 固定小数点 DSP の 40 bit 累算器を int64_t で再現する。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai */
#ifndef MRL_FIXED_H
#define MRL_FIXED_H
#include <stdint.h>

#define M40 ((int64_t)0xFFFFFFFFFF)

static inline int64_t wrap16(int64_t v) { return (int64_t)(int16_t)(uint16_t)(v & 0xFFFF); }
static inline int64_t acc(int64_t v) {   /* 40 bit に巻き戻す(2 の補数) */
    v &= M40;
    return (v >> 39) ? v - ((int64_t)1 << 40) : v;
}
static inline int64_t sat(int64_t v) { return v < -0x80000000LL ? -0x80000000LL : (v > 0x7FFFFFFFLL ? 0x7FFFFFFFLL : v); }
static inline int64_t hi(int64_t v) { return wrap16(v >> 16); }
/* 32 x 16 の小数乗算。下位 16 bit(符号なし)× y は int32 に収まり、和も |·| < 2^33 なので 40 bit の巻き戻しは起きない
 * (原本の式 acc(acc(lo·y·2) >> 16 + hi(x)·y·2) と同じ値) */
static inline int64_t mul32x16(int64_t x, int64_t y) {
    int32_t partial = (int32_t)((uint32_t)(x & 0xFFFF) * (int32_t)y) >> 15;
    return (int64_t)partial + (int64_t)((int32_t)hi(x) * (int32_t)y) * 2;   /* 積は 32 bit に収まる(64 bit の掛け算を避ける) */
}
static inline int64_t trunc32(int64_t v) { return (int64_t)(int32_t)(uint32_t)(v & 0xFFFFFFFF); }
/* 左シフトは符号なしで(負の値の左シフトは C では未定義) */
static inline int64_t shl(int64_t v, int s) { return (int64_t)((uint64_t)v << s); }
static inline int64_t shift(int64_t v, int s) { return s >= 0 ? acc(shl(v, s)) : acc(v >> (-s)); }

#endif
