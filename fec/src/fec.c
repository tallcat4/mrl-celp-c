/* 改良型 RL-CELP の音声フレームの誤り訂正。
 * jr-conventional-line-radio-receiver の crates/receiver-core/src/{voice_frame.rs,coding.rs}(commit 9945870)の C99 移植。
 * 分岐と受理の条件は原本のまま(ビット一致)。Vec は固定長の配列に置き換えた。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#include "mrlfec.h"
#include <string.h>

#define WORD_BYTES 42
#define CODED_BITS 176 /* 畳み込み符号の出力(パンクチャ後) */
#define SOURCE_BITS 94 /* CRC-8 4 + ペイロード 78 + CRC-8 4 + 終端 8 */
#define TAIL 28        /* 語の 28〜41 byte が RS の符号語(14 byte、パリティ 6 byte) */
#define TAIL_BYTES 14
#define ROOTS 6
#define PUNCTURED 255
#define SOFT_DISTANCE 24 /* 軟判定のあとに許すハミング距離(受理には CRC-8 と末尾 64 bit の一致も要る) */

typedef struct {
    uint8_t payload[MRL_FRAME_BYTES];
    int bits;
} speech;

/* 畳み込み符号の出力 2 bit の、176 bit の中での位置(t ごとに 1 本目・2 本目、255 = パンクチャ) */
static const uint8_t COORDINATES[2 * SOURCE_BITS] = {
    2,   6,   3,   7,   255, 4,   5,   0,   14,  1,   15,  10,  12,  11,  13,  8,   22,  9,   23,  18,  255, 19,
    16,  20,  17,  21,  26,  30,  27,  31,  24,  28,  25,  29,  34,  38,  255, 39,  36,  35,  37,  32,  46,  33,
    47,  42,  44,  43,  45,  40,  54,  41,  255, 50,  51,  55,  48,  52,  49,  53,  58,  62,  59,  63,  56,  60,
    57,  61,  255, 70,  71,  66,  68,  67,  69,  64,  78,  65,  79,  74,  76,  75,  77,  72,  255, 73,  82,  86,
    83,  87,  80,  84,  81,  85,  90,  94,  91,  95,  88,  92,  255, 93,  102, 89,  103, 98,  100, 99,  101, 96,
    110, 97,  111, 106, 108, 107, 255, 104, 105, 109, 114, 118, 115, 119, 112, 116, 113, 117, 122, 126, 123, 127,
    255, 124, 125, 120, 134, 121, 135, 130, 132, 131, 133, 128, 142, 129, 143, 138, 255, 139, 136, 140, 137, 141,
    146, 150, 147, 151, 144, 148, 145, 149, 154, 158, 255, 159, 156, 155, 157, 152, 166, 153, 167, 162, 164, 163,
    165, 160, 174, 161, 255, 170, 171, 175, 168, 172, 169, 173};
static const uint8_t CRC_POS[8] = {0, 1, 2, 3, 82, 83, 84, 85};

/* GF(256)、原始多項式 0x11d */
static const uint8_t gf_exp[255] = {
    1,   2,   4,   8,   16,  32,  64,  128, 29,  58,  116, 232, 205, 135, 19,  38,  76,  152, 45,  90,  180, 117,
    234, 201, 143, 3,   6,   12,  24,  48,  96,  192, 157, 39,  78,  156, 37,  74,  148, 53,  106, 212, 181, 119,
    238, 193, 159, 35,  70,  140, 5,   10,  20,  40,  80,  160, 93,  186, 105, 210, 185, 111, 222, 161, 95,  190,
    97,  194, 153, 47,  94,  188, 101, 202, 137, 15,  30,  60,  120, 240, 253, 231, 211, 187, 107, 214, 177, 127,
    254, 225, 223, 163, 91,  182, 113, 226, 217, 175, 67,  134, 17,  34,  68,  136, 13,  26,  52,  104, 208, 189,
    103, 206, 129, 31,  62,  124, 248, 237, 199, 147, 59,  118, 236, 197, 151, 51,  102, 204, 133, 23,  46,  92,
    184, 109, 218, 169, 79,  158, 33,  66,  132, 21,  42,  84,  168, 77,  154, 41,  82,  164, 85,  170, 73,  146,
    57,  114, 228, 213, 183, 115, 230, 209, 191, 99,  198, 145, 63,  126, 252, 229, 215, 179, 123, 246, 241, 255,
    227, 219, 171, 75,  150, 49,  98,  196, 149, 55,  110, 220, 165, 87,  174, 65,  130, 25,  50,  100, 200, 141,
    7,   14,  28,  56,  112, 224, 221, 167, 83,  166, 81,  162, 89,  178, 121, 242, 249, 239, 195, 155, 43,  86,
    172, 69,  138, 9,   18,  36,  72,  144, 61,  122, 244, 245, 247, 243, 251, 235, 203, 139, 11,  22,  44,  88,
    176, 125, 250, 233, 207, 131, 27,  54,  108, 216, 173, 71,  142};
static const uint8_t gf_log[256] = {
    0,   0,   1,   25,  2,   50,  26,  198, 3,   223, 51,  238, 27,  104, 199, 75,  4,   100, 224, 14,  52,  141,
    239, 129, 28,  193, 105, 248, 200, 8,   76,  113, 5,   138, 101, 47,  225, 36,  15,  33,  53,  147, 142, 218,
    240, 18,  130, 69,  29,  181, 194, 125, 106, 39,  249, 185, 201, 154, 9,   120, 77,  228, 114, 166, 6,   191,
    139, 98,  102, 221, 48,  253, 226, 152, 37,  179, 16,  145, 34,  136, 54,  208, 148, 206, 143, 150, 219, 189,
    241, 210, 19,  92,  131, 56,  70,  64,  30,  66,  182, 163, 195, 72,  126, 110, 107, 58,  40,  84,  250, 133,
    186, 61,  202, 94,  155, 159, 10,  21,  121, 43,  78,  212, 229, 172, 115, 243, 167, 87,  7,   112, 192, 247,
    140, 128, 99,  13,  103, 74,  222, 237, 49,  197, 254, 24,  227, 165, 153, 119, 38,  184, 180, 124, 17,  68,
    146, 217, 35,  32,  137, 46,  55,  63,  209, 91,  149, 188, 207, 205, 144, 135, 151, 178, 220, 252, 190, 97,
    242, 86,  211, 171, 20,  42,  93,  158, 132, 60,  57,  83,  71,  109, 65,  162, 31,  45,  67,  216, 183, 123,
    164, 118, 196, 23,  73,  236, 127, 12,  111, 246, 108, 161, 59,  82,  41,  157, 85,  170, 251, 96,  134, 177,
    187, 204, 62,  90,  203, 89,  95,  176, 156, 169, 160, 81,  11,  245, 22,  235, 122, 117, 44,  215, 79,  174,
    213, 233, 230, 231, 173, 232, 116, 214, 244, 234, 168, 80,  88,  175};

static uint8_t gf_mul(uint8_t a, uint8_t b) {
    return (a == 0 || b == 0) ? 0 : gf_exp[(gf_log[a] + gf_log[b]) % 255];
}
static uint8_t gf_div(uint8_t a, uint8_t b) { /* b != 0 */
    return a == 0 ? 0 : gf_exp[(gf_log[a] + 255 - gf_log[b]) % 255];
}
static int popcount8(uint8_t v) {
    int n = 0;
    for (; v; v &= (uint8_t)(v - 1)) n++;
    return n;
}

/* ---- ビットの扱い(語とペイロードは LSB first) ---- */
static uint8_t bit(const uint8_t *data, int i) { return (uint8_t)((data[i / 8] >> (i % 8)) & 1); }
static void set(uint8_t *data, int i, uint8_t v) { data[i / 8] |= (uint8_t)(v << (i % 8)); }
static uint8_t parity(unsigned v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1);
}
/* 語の i bit 目の、384 bit の中での位置(MSB first)。各 128 bit の先頭 16 bit は同期の符号 */
static int envelope_pos(int i) { return 128 * (i / 112) + 16 + i % 112; }

/* ---- スクランブル(PN 9 段、x^9 + x^5 + 1、初期値 全 1)と CRC ---- */
static void descramble(const uint8_t env[MRL_ENVELOPE_BYTES], uint8_t word[WORD_BYTES]) {
    uint8_t pn[336];
    memset(word, 0, WORD_BYTES);
    for (int i = 0; i < 336; i++) {
        pn[i] = i >= 9 ? (uint8_t)(pn[i - 9] ^ pn[i - 5]) : 1;
        int pos = envelope_pos(i);
        word[i / 8] |= (uint8_t)((((env[pos / 8] >> (7 - pos % 8)) & 1) ^ pn[i]) << (i % 8));
    }
}
static void scramble(const uint8_t word[WORD_BYTES], uint8_t env[MRL_ENVELOPE_BYTES]) {
    static const uint16_t markers[3] = {0x3d22, 0xe867, 0xe867};
    uint8_t pn[336];
    memset(env, 0, MRL_ENVELOPE_BYTES);
    for (int m = 0; m < 3; m++) {
        env[16 * m] = (uint8_t)(markers[m] >> 8);
        env[16 * m + 1] = (uint8_t)markers[m];
    }
    for (int i = 0; i < 336; i++) {
        pn[i] = i >= 9 ? (uint8_t)(pn[i - 9] ^ pn[i - 5]) : 1;
        int pos = envelope_pos(i);
        env[pos / 8] |= (uint8_t)((((word[i / 8] >> (i % 8)) & 1) ^ pn[i]) << (7 - pos % 8));
    }
}
static uint16_t crc16(const uint8_t *bytes, int n) { /* CRC-16/CCITT-FALSE */
    uint16_t crc = 0xffff;
    for (int k = 0; k < n; k++) {
        crc ^= (uint16_t)(bytes[k] << 8);
        for (int j = 0; j < 8; j++) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}
static int tail_crc_ok(const uint8_t word[WORD_BYTES]) {
    return crc16(word + TAIL, 6) == (uint16_t)((word[34] << 8) | word[35]);
}
/* 1 bit ずつの 78 bit(data[0..78])の CRC-8(多項式 0x19b、初期値 0xff) */
static uint8_t crc8(const uint8_t *data) {
    unsigned state = 255;
    for (int i = 0; i < 86; i++) {
        state = (state << 1) | (i < 78 ? data[i] : 0);
        if (state & 256) state ^= 0x19b;
    }
    return (uint8_t)state;
}

/* ---- 畳み込み符号(拘束長 9、生成多項式 0x1af と 0x11d) ---- */
static void convolution(const uint8_t input[SOURCE_BITS], uint8_t result[CODED_BITS]) {
    unsigned state = 0;
    memset(result, 0, CODED_BITS);
    for (int i = 0; i < SOURCE_BITS; i++) {
        state = ((state << 1) | input[i]) & 511;
        if (COORDINATES[2 * i] != PUNCTURED) result[COORDINATES[2 * i]] = parity(state & 0x1af);
        result[COORDINATES[2 * i + 1]] = parity(state & 0x11d);
    }
}
/* 語の 5〜15 byte と 21〜31 byte に交互に入っている 176 bit */
static void protected_bits(const uint8_t word[WORD_BYTES], uint8_t r[CODED_BITS]) {
    for (int i = 0; i < 88; i++) {
        r[2 * i] = bit(word + 5, i);
        r[2 * i + 1] = bit(word + 21, i);
    }
}
static void write_protected(uint8_t word[WORD_BYTES], const uint8_t data[CODED_BITS]) {
    memset(word + 5, 0, 11);
    memset(word + 21, 0, 11);
    for (int i = 0; i < 88; i++) {
        set(word + 5, i, data[2 * i]);
        set(word + 21, i, data[2 * i + 1]);
    }
}
/* 2 本目(0x11d、x^0 の項がある)だけから入力を逆算する(誤りがなければ正しい) */
static void inverse(const uint8_t received[CODED_BITS], uint8_t r[SOURCE_BITS]) {
    unsigned state = 0;
    for (int t = 0; t < SOURCE_BITS; t++) {
        r[t] = received[COORDINATES[2 * t + 1]] ^ parity(state & (0x11d >> 1));
        state = ((state << 1) | r[t]) & 255;
    }
}
static void traceback(uint8_t survivors[SOURCE_BITS][32], uint8_t r[SOURCE_BITS]) {
    unsigned state = 0;
    for (int t = SOURCE_BITS - 1; t >= 0; t--) {
        r[t] = (uint8_t)(state & 1);
        state = (state >> 1) | ((unsigned)bit(survivors[t], (int)state) << 7);
    }
}
static void viterbi(const uint8_t received[CODED_BITS], uint8_t r[SOURCE_BITS]) {
    uint16_t costs[256], next[256];
    uint8_t survivors[SOURCE_BITS][32];
    for (int s = 0; s < 256; s++) costs[s] = 255;
    costs[0] = 0;
    memset(survivors, 0, sizeof survivors);
    for (int t = 0; t < SOURCE_BITS; t++) {
        int punctured = COORDINATES[2 * t] == PUNCTURED;
        unsigned observed = punctured ? 1 : 2;
        uint8_t a = received[COORDINATES[2 * t + 1]];
        uint8_t b = punctured ? 0 : received[COORDINATES[2 * t]];
        for (unsigned prior = 0; prior < 128; prior++) {
            unsigned symbol = prior * 2;
            unsigned distance = (unsigned)(parity(symbol & 0x11d) ^ a) + (punctured ? 0u : (unsigned)(parity(symbol & 0x1af) ^ b));
            unsigned complement = observed - distance;
            unsigned left = costs[prior], right = costs[prior + 128];
            unsigned x = left + distance < right + complement ? left + distance : right + complement;
            unsigned y = left + complement < right + distance ? left + complement : right + distance;
            next[2 * prior] = (uint16_t)(x < 255 ? x : 255);
            next[2 * prior + 1] = (uint16_t)(y < 255 ? y : 255);
            set(survivors[t], (int)(2 * prior), right + complement < left + distance);
            set(survivors[t], (int)(2 * prior + 1), right + distance < left + complement);
        }
        memcpy(costs, next, sizeof costs);
    }
    traceback(survivors, r);
}
/* ビットごとの重みつき:食い違う符号ビットは信頼度の分だけ費用がかかる(重み 0 の消失ビットは証拠にならない) */
static void soft_viterbi(const uint8_t received[CODED_BITS], const uint8_t weights[CODED_BITS], uint8_t r[SOURCE_BITS]) {
    const uint32_t UNREACHED = UINT32_MAX / 4;
    uint32_t costs[256], next[256];
    uint8_t survivors[SOURCE_BITS][32];
    for (int s = 0; s < 256; s++) costs[s] = UNREACHED;
    costs[0] = 0;
    memset(survivors, 0, sizeof survivors);
    for (int t = 0; t < SOURCE_BITS; t++) {
        int punctured = COORDINATES[2 * t] == PUNCTURED;
        uint8_t a = received[COORDINATES[2 * t + 1]];
        uint32_t wa = weights[COORDINATES[2 * t + 1]];
        uint8_t b = punctured ? 0 : received[COORDINATES[2 * t]];
        uint32_t wb = punctured ? 0 : weights[COORDINATES[2 * t]];
        for (unsigned prior = 0; prior < 128; prior++) {
            unsigned symbol = prior * 2;
            uint32_t distance = (uint32_t)(parity(symbol & 0x11d) ^ a) * wa + (uint32_t)(parity(symbol & 0x1af) ^ b) * wb;
            uint32_t complement = wa + wb - distance;
            uint32_t left = costs[prior], right = costs[prior + 128];
            next[2 * prior] = left + distance < right + complement ? left + distance : right + complement;
            next[2 * prior + 1] = left + complement < right + distance ? left + complement : right + distance;
            set(survivors[t], (int)(2 * prior), right + complement < left + distance);
            set(survivors[t], (int)(2 * prior + 1), right + distance < left + complement);
        }
        uint32_t floor = next[0];
        for (int s = 1; s < 256; s++)
            if (next[s] < floor) floor = next[s];
        for (int s = 0; s < 256; s++) {
            uint32_t c = next[s] - floor;
            costs[s] = c < UNREACHED ? c : UNREACHED;
        }
    }
    traceback(survivors, r);
}

/* ---- Reed-Solomon(14 byte、根 α^0..α^5、先頭の byte が最高次) ---- */
#define RS_MAX 16
static void syndromes(const uint8_t *word, int n, int roots, uint8_t *syn) {
    for (int r = 0; r < roots; r++) {
        uint8_t v = 0;
        for (int k = 0; k < n; k++) v = (uint8_t)(gf_mul(v, gf_exp[r]) ^ word[k]);
        syn[r] = v;
    }
}
static int syndromes_zero(const uint8_t *word, int n, int roots) {
    uint8_t syn[RS_MAX];
    syndromes(word, n, roots, syn);
    for (int r = 0; r < roots; r++)
        if (syn[r]) return 0;
    return 1;
}
/* 位置多項式(低次が先)の根を Chien 探索で数える。根の数を返す */
static int chien(const uint8_t *loc, int degree, int n, int *positions) {
    int count = 0;
    for (int p = 0; p < n; p++) {
        uint8_t inv = gf_exp[(p + 255 - (n - 1)) % 255];
        uint8_t v = 0;
        for (int i = degree; i >= 0; i--) v = (uint8_t)(gf_mul(v, inv) ^ loc[i]);
        if (v == 0) positions[count++] = p;
    }
    return count;
}
/* Vandermonde の連立方程式で誤りの大きさを求めて直す。最後に全シンドロームが 0 か確かめる */
static int solve_and_fix(uint8_t *word, int n, int roots, const uint8_t *syn, const int *positions, int count) {
    uint8_t mat[RS_MAX][RS_MAX + 1];
    for (int row = 0; row < count; row++) {
        for (int col = 0; col < count; col++) mat[row][col] = gf_exp[(row * (n - 1 - positions[col])) % 255];
        mat[row][count] = syn[row];
    }
    for (int col = 0; col < count; col++) {
        int pivot = col;
        while (pivot < count && mat[pivot][col] == 0) pivot++;
        if (pivot == count) return -1;
        if (pivot != col) {
            uint8_t tmp[RS_MAX + 1];
            memcpy(tmp, mat[pivot], sizeof tmp);
            memcpy(mat[pivot], mat[col], sizeof tmp);
            memcpy(mat[col], tmp, sizeof tmp);
        }
        uint8_t scale = mat[col][col];
        for (int i = col; i <= count; i++) mat[col][i] = gf_div(mat[col][i], scale);
        for (int row = 0; row < count; row++) {
            if (row == col) continue;
            uint8_t factor = mat[row][col];
            for (int i = col; i <= count; i++) mat[row][i] ^= gf_mul(factor, mat[col][i]);
        }
    }
    for (int i = 0; i < count; i++) word[positions[i]] ^= mat[i][count];
    return syndromes_zero(word, n, roots) ? 0 : -1;
}
/* 誤りだけの訂正(Berlekamp-Massey、Chien 探索)。0 = 成功、-1 = 失敗 */
static int rs_correct(uint8_t *word, int n, int roots) {
    uint8_t syn[RS_MAX], loc[RS_MAX + 1] = {0}, prev[RS_MAX + 1] = {0}, saved[RS_MAX + 1];
    int positions[RS_MAX + 1];
    syndromes(word, n, roots, syn);
    if (syndromes_zero(word, n, roots)) return 0;
    loc[0] = prev[0] = 1;
    int degree = 0, gap = 1;
    uint8_t last = 1;
    for (int step = 0; step < roots; step++) {
        uint8_t d = syn[step];
        for (int i = 1; i <= degree; i++) d ^= gf_mul(loc[i], syn[step - i]);
        if (d == 0) {
            gap++;
            continue;
        }
        memcpy(saved, loc, sizeof saved);
        uint8_t scale = gf_div(d, last);
        for (int i = 0; i < roots + 1 - gap; i++) loc[i + gap] ^= gf_mul(scale, prev[i]);
        if (2 * degree <= step) {
            degree = step + 1 - degree;
            memcpy(prev, saved, sizeof prev);
            last = d;
            gap = 1;
        } else {
            gap++;
        }
    }
    if (degree == 0 || degree > roots / 2) return -1;
    if (chien(loc, degree, n, positions) != degree) return -1;
    return solve_and_fix(word, n, roots, syn, positions, degree);
}
/* Berlekamp-Massey(位置多項式は低次が先)。次数を返す、2 × 次数 > len なら -1 */
static int berlekamp_massey(const uint8_t *sequence, int len, uint8_t *loc) {
    uint8_t prev[RS_MAX + 1] = {0}, saved[RS_MAX + 1];
    memset(loc, 0, RS_MAX + 1);
    loc[0] = prev[0] = 1;
    int degree = 0, gap = 1;
    uint8_t last = 1;
    for (int step = 0; step < len; step++) {
        uint8_t d = sequence[step];
        for (int i = 1; i <= (degree < step ? degree : step); i++) d ^= gf_mul(loc[i], sequence[step - i]);
        if (d == 0) {
            gap++;
            continue;
        }
        memcpy(saved, loc, sizeof saved);
        uint8_t scale = gf_div(d, last);
        for (int i = 0; i < len + 1 - gap; i++) loc[i + gap] ^= gf_mul(scale, prev[i]);
        if (2 * degree <= step) {
            degree = step + 1 - degree;
            memcpy(prev, saved, sizeof prev);
            last = d;
            gap = 1;
        } else {
            gap++;
        }
    }
    return 2 * degree <= len ? degree : -1;
}
/* 誤りと消失の訂正。f 個の消失と e 個の誤りは 2e + f <= roots なら直る。0 = 成功、-1 = 失敗 */
static int rs_correct_with_erasures(uint8_t *word, int n, int roots, const int *erasures, int f) {
    uint8_t syn[RS_MAX], gamma[RS_MAX + 1] = {0}, forney[RS_MAX] = {0}, lambda[RS_MAX + 1];
    int positions[RS_MAX + RS_MAX + 1];
    if (f > roots) return -1;
    for (int k = 0; k < f; k++)
        if (erasures[k] >= n) return -1;
    syndromes(word, n, roots, syn);
    if (syndromes_zero(word, n, roots)) return 0;
    /* 消失の位置多項式 Γ(x) = Π (1 + X_j x)、低次が先 */
    gamma[0] = 1;
    for (int k = 0; k < f; k++) {
        uint8_t x = gf_exp[(n - 1 - erasures[k]) % 255];
        for (int i = k + 1; i >= 1; i--) gamma[i] ^= gf_mul(gamma[i - 1], x);
    }
    /* Forney のシンドローム T(x) = S(x) Γ(x) mod x^roots の T_f..T_(roots-1) */
    for (int i = 0; i < roots; i++)
        for (int j = 0; j <= f; j++)
            if (i + j < roots) forney[i + j] ^= gf_mul(syn[i], gamma[j]);
    int degree = berlekamp_massey(forney + f, roots - f, lambda);
    if (degree < 0) return -1;
    int count = chien(lambda, degree, n, positions);
    if (count != degree) return -1;
    for (int k = 0; k < f; k++) {
        int found = 0;
        for (int i = 0; i < count; i++) found |= positions[i] == erasures[k];
        if (!found) positions[count++] = erasures[k];
    }
    if (count > roots) return -1;
    return solve_and_fix(word, n, roots, syn, positions, count);
}

/* ---- 音声フレーム ---- */
/* CRC-16 と RS のパリティを 28〜33 byte から作り直す */
static void tail(uint8_t word[WORD_BYTES]) {
    static const uint8_t g[7] = {1, 63, 1, 218, 32, 227, 38};
    uint8_t work[TAIL_BYTES] = {0};
    uint16_t crc = crc16(word + TAIL, 6);
    word[34] = (uint8_t)(crc >> 8);
    word[35] = (uint8_t)crc;
    memcpy(work, word + TAIL, 8);
    for (int i = 0; i < 8; i++) {
        uint8_t c = work[i];
        for (int j = 0; j < 7; j++) work[i + j] ^= gf_mul(c, g[j]);
    }
    memcpy(word + 36, work + 8, 6);
}
static int mode_ok(const uint8_t word[WORD_BYTES]) { return word[32] == 0x80 && word[33] <= 1; }

/* 入力の候補を確かめる:終端が 0、CRC-8 が合う、符号化し直したものと受信の距離が max 以下、末尾が一致 */
static int checked(const uint8_t word[WORD_BYTES], const uint8_t received[CODED_BITS], const uint8_t input[SOURCE_BITS],
                   int max, speech *out) {
    uint8_t encoded[CODED_BITS], corrected[WORD_BYTES];
    for (int i = 86; i < SOURCE_BITS; i++)
        if (input[i]) return -1;
    uint8_t checksum = 0;
    for (int i = 0; i < 8; i++) checksum |= (uint8_t)(input[CRC_POS[i]] << i);
    if (checksum != crc8(input + 4)) return -1;
    convolution(input, encoded);
    int distance = 0;
    for (int i = 0; i < CODED_BITS; i++) distance += encoded[i] ^ received[i];
    if (distance > max) return -1;
    memcpy(corrected, word, WORD_BYTES);
    write_protected(corrected, encoded);
    tail(corrected);
    if (memcmp(corrected + 34, word + 34, WORD_BYTES - 34) != 0) return -1;
    memset(out->payload, 0, MRL_FRAME_BYTES);
    for (int i = 0; i < 78; i++) set(out->payload, i, input[4 + i]);
    for (int i = 0; i < 40; i++) {
        set(out->payload, 78 + i, bit(word, i));
        set(out->payload, 118 + i, bit(word + 16, i));
    }
    out->bits = distance;
    return 0;
}
/* 畳み込み符号の側の候補から末尾を作り直し、受信の末尾との違いが小さければ受理する */
static int joint(const uint8_t word[WORD_BYTES], const uint8_t received[CODED_BITS], const uint8_t source[SOURCE_BITS],
                 speech *out) {
    uint8_t c[WORD_BYTES], encoded[CODED_BITS];
    memcpy(c, word, WORD_BYTES);
    convolution(source, encoded);
    write_protected(c, encoded);
    c[32] = 0x80;
    for (int alternate = 0; alternate < 2; alternate++) {
        c[33] = (uint8_t)((word[33] & 1) ^ alternate);
        if (popcount8(word[32] ^ c[32]) + popcount8(word[33] ^ c[33]) > 2) continue;
        tail(c);
        int outer_bits = 0, outer_symbols = 0, extra = 0;
        for (int i = TAIL; i < WORD_BYTES; i++) {
            int bits = popcount8(c[i] ^ word[i]);
            outer_bits += bits;
            outer_symbols += bits != 0;
            if (i >= 32) extra += bits;
        }
        if (outer_bits > 6 || outer_symbols > 4) continue;
        if (checked(c, received, source, 8, out) == 0 && out->bits + extra <= 12) {
            out->bits += extra;
            return 0;
        }
    }
    return -1;
}
static int decode_hard(const uint8_t env[MRL_ENVELOPE_BYTES], speech *out) {
    uint8_t word[WORD_BYTES], repaired[WORD_BYTES];
    uint8_t received[CODED_BITS], outer[CODED_BITS];
    uint8_t inverted[SOURCE_BITS], decoded[SOURCE_BITS], source[SOURCE_BITS];
    descramble(env, word);
    protected_bits(word, received);
    inverse(received, inverted);
    int is_mode = mode_ok(word);
    if (is_mode && checked(word, received, inverted, 8, out) == 0) return 0;
    memcpy(repaired, word, WORD_BYTES);
    if (rs_correct(repaired + TAIL, TAIL_BYTES, ROOTS) == 0 && mode_ok(repaired) && tail_crc_ok(repaired) &&
        memcmp(repaired, word, WORD_BYTES) != 0) {
        protected_bits(repaired, outer);
        inverse(outer, source);
        int ok = checked(repaired, received, source, 8, out) == 0;
        if (!ok) {
            viterbi(outer, source);
            ok = checked(repaired, received, source, 8, out) == 0;
        }
        if (ok) {
            for (int i = 32; i < WORD_BYTES; i++) out->bits += popcount8(word[i] ^ repaired[i]);
            return 0;
        }
    }
    if (is_mode) {
        viterbi(received, decoded);
        if (checked(word, received, decoded, 8, out) == 0) return 0;
    }
    if (popcount8(word[32] ^ 0x80) + popcount8(word[33] & 0xfe) > 2) return -1;
    if (joint(word, received, inverted, out) == 0) return 0;
    if (!is_mode) viterbi(received, decoded);
    return joint(word, received, decoded, out);
}
static int decode_soft(const uint8_t env[MRL_ENVELOPE_BYTES], const uint8_t confidence[MRL_ENVELOPE_BITS], speech *out) {
    uint8_t word[WORD_BYTES], reliability[336], received[CODED_BITS], weights[CODED_BITS], decoded[SOURCE_BITS];
    uint8_t candidates[5][WORD_BYTES];
    int order[TAIL_BYTES], key[TAIL_BYTES], ncand = 1;
    static const int ERASED[4] = {0, 2, 4, 6};
    if (decode_hard(env, out) == 0) return 0;
    if (!confidence) return -1;
    descramble(env, word);
    for (int i = 0; i < 336; i++) reliability[i] = confidence[envelope_pos(i)];
    protected_bits(word, received);
    for (int i = 0; i < 88; i++) {
        weights[2 * i] = reliability[40 + i];
        weights[2 * i + 1] = reliability[168 + i];
    }
    soft_viterbi(received, weights, decoded);
    /* 末尾の 14 byte を、byte の中で一番低い信頼度の小さい順に(同じなら位置の順、安定な整列) */
    for (int b = 0; b < TAIL_BYTES; b++) {
        int m = 255;
        for (int k = 0; k < 8; k++)
            if (reliability[8 * (TAIL + b) + k] < m) m = reliability[8 * (TAIL + b) + k];
        key[b] = m;
        int j = b;
        while (j > 0 && key[order[j - 1]] > m) {
            order[j] = order[j - 1];
            j--;
        }
        order[j] = b;
    }
    memcpy(candidates[0], word, WORD_BYTES);
    for (int e = 0; e < 4; e++) {
        int erased = ERASED[e], positions[ROOTS];
        uint8_t repaired[WORD_BYTES];
        for (int k = 0; k < erased; k++) { /* 位置を小さい順に */
            int p = order[k], j = k;
            while (j > 0 && positions[j - 1] > p) {
                positions[j] = positions[j - 1];
                j--;
            }
            positions[j] = p;
        }
        memcpy(repaired, word, WORD_BYTES);
        if (rs_correct_with_erasures(repaired + TAIL, TAIL_BYTES, ROOTS, positions, erased) != 0) continue;
        int seen = 0;
        for (int c = 0; c < ncand; c++) seen |= memcmp(candidates[c], repaired, WORD_BYTES) == 0;
        if (!seen) memcpy(candidates[ncand++], repaired, WORD_BYTES);
    }
    for (int c = 0; c < ncand; c++) {
        if (!mode_ok(candidates[c]) || !tail_crc_ok(candidates[c])) continue;
        if (checked(candidates[c], received, decoded, SOFT_DISTANCE, out) == 0) return 0;
    }
    return -1;
}

/* ---- 公開の関数 ---- */
static int finish(int r, const speech *s, uint8_t payload[MRL_FRAME_BYTES], int *corrected_bits) {
    if (r != 0) return -1;
    memcpy(payload, s->payload, MRL_FRAME_BYTES);
    if (corrected_bits) *corrected_bits = s->bits;
    return 0;
}
int mrl_fec_decode(const uint8_t envelope[MRL_ENVELOPE_BYTES], uint8_t payload[MRL_FRAME_BYTES], int *corrected_bits) {
    speech s;
    return finish(decode_hard(envelope, &s), &s, payload, corrected_bits);
}
int mrl_fec_decode_soft(const uint8_t envelope[MRL_ENVELOPE_BYTES], const uint8_t confidence[MRL_ENVELOPE_BITS],
                        uint8_t payload[MRL_FRAME_BYTES], int *corrected_bits) {
    speech s;
    return finish(decode_soft(envelope, confidence, &s), &s, payload, corrected_bits);
}
void mrl_fec_encode(const uint8_t payload[MRL_FRAME_BYTES], int alternate, uint8_t envelope[MRL_ENVELOPE_BYTES]) {
    uint8_t input[SOURCE_BITS] = {0}, word[WORD_BYTES] = {0}, encoded[CODED_BITS];
    for (int i = 0; i < 78; i++) input[4 + i] = bit(payload, i);
    uint8_t checksum = crc8(input + 4);
    for (int i = 0; i < 8; i++) input[CRC_POS[i]] = (uint8_t)((checksum >> i) & 1);
    for (int i = 0; i < 40; i++) {
        set(word, i, bit(payload, 78 + i));
        set(word + 16, i, bit(payload, 118 + i));
    }
    convolution(input, encoded);
    write_protected(word, encoded);
    word[32] = 0x80;
    word[33] = alternate ? 1 : 0;
    tail(word);
    scramble(word, envelope);
}
