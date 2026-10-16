/* 158 bit のペイロード(20 byte、LSB first)の読み書き。原本 jr-voice の bits.rs と同じ並び(mrl_pack はその逆)。
 * 原本の注記どおり、後半 2 つ目のゲインの下位 4 bit の位置と前半の絶対ピッチの ^3 は経験的に確かめた仮定。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#include <string.h>

#include "core.h"

/* フィールドごとの 158 bit 列の中の位置(上位ビットから)。q(k) = 84 + 39h + k、a(k) = 20h + k、b(k) = 40 + 20h + k */
enum { F_MODE, F_STAGE1, F_S2L, F_S2H, F_ABS, F_REL, F_CODE0, F_CODE1, F_GH0, F_GL0, F_GH1, F_GL1, F_N };
static const int WIDTH[F_N] = {1, 7, 6, 6, 8, 5, 16, 16, 3, 4, 3, 4};

static int positions(int h, int field, int pos[16]) {
#define Q(k) (84 + 39 * h + (k))
#define A(k) (20 * h + (k))
#define B(k) (40 + 20 * h + (k))
    int k, n = WIDTH[field];
    switch (field) {
    case F_MODE: pos[0] = Q(0); break;
    case F_STAGE1: for (k = 0; k < 7; k++) pos[k] = Q(1 + k); break;
    case F_S2L: { int p[6] = {Q(8), Q(9), Q(10), Q(11), A(3), A(1)}; memcpy(pos, p, sizeof p); } break;
    case F_S2H: { int p[6] = {B(3), B(1), A(2), A(0), B(2), B(0)}; memcpy(pos, p, sizeof p); } break;
    case F_ABS: { int p[8] = {Q(12), Q(13), Q(14), Q(15), Q(16), Q(17), A(7), A(5)}; memcpy(pos, p, sizeof p); } break;
    case F_REL: { int p[5] = {Q(27), Q(28), Q(29), A(15), A(13)}; memcpy(pos, p, sizeof p); } break;
    case F_CODE0: case F_CODE1: {
        int c = field == F_CODE0 ? 0 : 8;
        int p[16] = {Q(c == 0 ? 18 : 30), Q(c == 0 ? 19 : 31), B(7 + c), B(5 + c), A(6 + c), A(4 + c), B(6 + c), B(4 + c),
                     A(11 + c), A(9 + c), B(11 + c), B(9 + c), A(10 + c), A(8 + c), B(10 + c), B(8 + c)};
        memcpy(pos, p, sizeof p);
    } break;
    case F_GH0: for (k = 0; k < 3; k++) pos[k] = Q(20 + k); break;
    case F_GL0: for (k = 0; k < 4; k++) pos[k] = Q(23 + k); break;
    case F_GH1: for (k = 0; k < 3; k++) pos[k] = Q(32 + k); break;
    case F_GL1: for (k = 0; k < 4; k++) pos[k] = Q(35 + k); break;
    }
    return n;
#undef Q
#undef A
#undef B
}

static int get_bit(const uint8_t *p, int i) { return (p[i >> 3] >> (i & 7)) & 1; }

void mrl_unpack(const uint8_t payload[20], mrl_half halves[2]) {
    uint8_t bits[158];
    int i, h, f, pos[16];
    for (i = 0; i < 80; i++) bits[i] = (uint8_t)get_bit(payload, 78 + i);
    bits[1] ^= 1;
    bits[5] ^= 1;
    bits[7] ^= 1;
    for (i = 0; i < 74; i++) bits[84 + i] = (uint8_t)get_bit(payload, i);
    for (h = 0; h < 2; h++) {
        int v[F_N] = {0};
        for (f = 0; f < F_N; f++) {
            int n = positions(h, f, pos);
            if (h == 1 && f == F_GL1) {   /* 後半の 2 つ目のゲインの下位 4 bit はペイロードの bit 74..77 */
                for (i = 74; i < 78; i++) v[f] = 2 * v[f] + get_bit(payload, i);
                continue;
            }
            for (i = 0; i < n; i++) v[f] = 2 * v[f] + bits[pos[i]];
        }
        halves[h].mode = v[F_MODE];
        halves[h].stage1 = v[F_STAGE1];
        halves[h].stage2_low = v[F_S2L];
        halves[h].stage2_high = v[F_S2H];
        halves[h].absolute_pitch = h == 0 ? v[F_ABS] ^ 3 : v[F_ABS];
        halves[h].relative_pitch = v[F_REL];
        halves[h].code[0] = v[F_CODE0];
        halves[h].code[1] = v[F_CODE1];
        halves[h].gain[0] = (v[F_GH0] & 7) * 16 + (v[F_GL0] & 15);
        halves[h].gain[1] = (v[F_GH1] & 7) * 16 + (v[F_GL1] & 15);
    }
}

static void put_bit(uint8_t *p, int i, int v) { p[i >> 3] |= (uint8_t)((v & 1) << (i & 7)); }

void mrl_pack(const mrl_half halves[2], uint8_t payload[20]) {
    uint8_t bits[158];
    int i, h, f, pos[16], late_low = 0;
    memset(bits, 0, sizeof bits);
    memset(payload, 0, 20);
    for (h = 0; h < 2; h++) {
        const mrl_half *x = &halves[h];
        int v[F_N] = {x->mode, x->stage1, x->stage2_low, x->stage2_high, h == 0 ? x->absolute_pitch ^ 3 : x->absolute_pitch,
                      x->relative_pitch, x->code[0], x->code[1], x->gain[0] >> 4, x->gain[0] & 15, x->gain[1] >> 4,
                      x->gain[1] & 15};
        for (f = 0; f < F_N; f++) {
            int n = positions(h, f, pos);
            if (h == 1 && f == F_GL1) {
                late_low = v[f];
                continue;
            }
            for (i = 0; i < n; i++) bits[pos[i]] = (uint8_t)((v[f] >> (n - 1 - i)) & 1);
        }
    }
    for (i = 0; i < 80; i++) put_bit(payload, 78 + i, bits[i] ^ (i == 1 || i == 5 || i == 7));
    for (i = 0; i < 74; i++) put_bit(payload, i, bits[84 + i]);
    for (i = 0; i < 4; i++) put_bit(payload, 74 + i, (late_low >> (3 - i)) & 1);
}
