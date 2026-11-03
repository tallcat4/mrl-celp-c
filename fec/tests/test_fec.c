/* 誤り訂正のテスト:原本(Rust)と同じ乱数で同じフレームと誤りを作り、復号の結果のハッシュを参照と比べる。
 *   ./test_fec tests/vectors/fec_reference.txt        比べる
 *   ./test_fec -v                                      ケースごとの結果を出す(違ったときの調べ用)
 * 参照は tests/vectors/fec_reference.txt(作り方はそのファイルの先頭)。
 * SPDX-License-Identifier: MIT */
#include "mrlfec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t rng = 0x9e3779b97f4a7c15ull;
static uint64_t next(void) {
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    return rng;
}
static uint64_t below(uint64_t n) { return next() % n; }

static uint64_t hash;
static void hbyte(uint8_t b) {
    hash ^= b;
    hash *= 0x100000001b3ull;
}
static void hbytes(const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) hbyte(b[i]);
}
static void flip(uint8_t *env, int pos) { env[pos / 8] ^= (uint8_t)(1 << (7 - pos % 8)); }
static int data_pos(int i) { return 128 * (i / 112) + 16 + i % 112; }

static void record(int r, const uint8_t *payload, int bits, unsigned *accepted) {
    if (r != 0) {
        hbyte(0);
        return;
    }
    ++*accepted;
    hbyte(1);
    hbytes(payload, MRL_FRAME_BYTES);
    hbyte((uint8_t)bits);
    hbyte((uint8_t)(bits >> 8));
}
static void show(const char *tag, int r, const uint8_t *payload, int bits) {
    printf(" %s ", tag);
    if (r != 0) {
        printf("-");
        return;
    }
    for (int i = 0; i < MRL_FRAME_BYTES; i++) printf("%02x", payload[i]);
    printf(" %d", bits);
}

int main(int argc, char **argv) {
    static const struct {
        const char *name;
        int count;
    } cats[5] = {{"clean", 500}, {"scattered", 3000}, {"uninformed", 1000}, {"burst", 1500}, {"random", 2000}};
    int verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    char lines[5][128];
    int failures = 0;

    for (int cat = 0; cat < 5; cat++) {
        unsigned nh = 0, ns = 0;
        hash = 0xcbf29ce484222325ull;
        for (int n = 0; n < cats[cat].count; n++) {
            uint8_t env[MRL_ENVELOPE_BYTES], conf[MRL_ENVELOPE_BITS], payload[MRL_FRAME_BYTES] = {0};
            if (cat == 4) {
                for (int i = 0; i < MRL_ENVELOPE_BYTES; i++) env[i] = (uint8_t)next();
                for (int i = 0; i < MRL_ENVELOPE_BITS; i++) conf[i] = (uint8_t)next();
            } else {
                for (int i = 0; i < MRL_FRAME_BYTES; i++) payload[i] = (uint8_t)next();
                payload[19] &= 0x3f;
                int alternate = below(2) == 1;
                mrl_fec_encode(payload, alternate, env);
                if (cat == 0) hbytes(env, MRL_ENVELOPE_BYTES);
                for (int i = 0; i < MRL_ENVELOPE_BITS; i++) conf[i] = (uint8_t)(60 + below(196));
                if (cat == 1 || cat == 2) {
                    int k = 1 + (int)below(30);
                    for (int j = 0; j < k; j++) {
                        int p = data_pos((int)below(336));
                        flip(env, p);
                        conf[p] = (uint8_t)below(41);
                    }
                    for (int i = 0; i < MRL_ENVELOPE_BITS; i++)
                        if (below(10) == 0) conf[i] = (uint8_t)below(60);
                    if (cat == 2)
                        for (int i = 0; i < MRL_ENVELOPE_BITS; i++) conf[i] = (uint8_t)next();
                } else if (cat == 3) {
                    int len = 1 + (int)below(48);
                    int start = (int)below((uint64_t)(MRL_ENVELOPE_BITS - len + 1));
                    for (int p = start; p < start + len; p++) {
                        flip(env, p);
                        conf[p] = (uint8_t)below(30);
                    }
                }
            }
            uint8_t hp[MRL_FRAME_BYTES], sp[MRL_FRAME_BYTES], np[MRL_FRAME_BYTES];
            int hb = -1, sb = -1, nb = -1;
            int hr = mrl_fec_decode(env, hp, &hb);
            int sr = mrl_fec_decode_soft(env, conf, sp, &sb);
            int nr = mrl_fec_decode_soft(env, NULL, np, &nb);
            /* 信頼度なしの軟判定は硬判定と同じ */
            if (nr != hr || (hr == 0 && (nb != hb || memcmp(np, hp, MRL_FRAME_BYTES) != 0))) {
                printf("%s %d: decode_soft(NULL) differs from decode\n", cats[cat].name, n);
                failures++;
            }
            /* 誤りのないフレームは訂正 0 でそのまま戻る */
            if (cat == 0 && (hr != 0 || hb != 0 || memcmp(hp, payload, MRL_FRAME_BYTES) != 0)) {
                printf("clean %d: round trip failed\n", n);
                failures++;
            }
            if (verbose) {
                printf("%s %d", cats[cat].name, n);
                show("H", hr, hp, hb);
                show("S", sr, sp, sb);
                printf("\n");
            }
            record(hr, hp, hb, &nh);
            record(sr, sp, sb, &ns);
        }
        snprintf(lines[cat], sizeof lines[cat], "%s %d %u %u %016llx", cats[cat].name, cats[cat].count, nh, ns,
                 (unsigned long long)hash);
    }
    if (verbose) return failures != 0;

    FILE *f = argc > 1 ? fopen(argv[1], "r") : NULL;
    if (!f) {
        for (int cat = 0; cat < 5; cat++) printf("%s\n", lines[cat]);
        fprintf(stderr, "usage: %s tests/vectors/fec_reference.txt\n", argv[0]);
        return 2;
    }
    char buf[256];
    int cat = 0;
    while (fgets(buf, sizeof buf, f)) {
        buf[strcspn(buf, "\r\n")] = 0;
        if (buf[0] == '#' || buf[0] == 0) continue;
        if (cat >= 5) break;
        int same = strcmp(buf, lines[cat]) == 0;
        printf("%-10s %s  (%s)\n", cats[cat].name, same ? "match" : "MISMATCH", lines[cat]);
        if (!same) {
            printf("  expected: %s\n", buf);
            failures++;
        }
        cat++;
    }
    fclose(f);
    if (cat != 5) {
        printf("reference has %d of 5 lines\n", cat);
        failures++;
    }
    printf(failures ? "FAILED\n" : "all match the reference (jr-conventional-line-radio-receiver voice_frame.rs)\n");
    return failures != 0;
}
