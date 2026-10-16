/* デコーダの公開 API。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#include <string.h>

#include "core.h"
#include "mrlcelp.h"

struct mrl_decoder {
    mrl_state s;
};

size_t mrl_decoder_size(void) { return sizeof(struct mrl_decoder); }

mrl_decoder *mrl_decoder_init(void *mem) {
    mrl_decoder *d = (mrl_decoder *)mem;
    mrl_state_init(&d->s);
    return d;
}

int mrl_decode_ex(mrl_decoder *d, const uint8_t payload[20], int16_t pcm[320], int16_t pre_filter[320]) {
    mrl_half halves[2];
    int h, sub;
    if (payload[19] & 0xC0) return -1;
    mrl_unpack(payload, halves);
    for (h = 0; h < 2; h++) {
        int16_t lsf[10], lsp[10], lpc[2][11], synth[160], code[80];
        float residual[10];
        mrl_lsf_decode((const float(*)[10])d->s.spectral_history, &halves[h], lsf, residual);
        mrl_spectral_push(d->s.spectral_history, residual);
        mrl_lsf_to_lsp(lsf, lsp);
        mrl_begin_half(&d->s, lsp, lpc[0], lpc[1]);
        for (sub = 0; sub < 2; sub++) {
            mrl_lag lag = sub == 0 ? mrl_decode_absolute(halves[h].absolute_pitch)
                                   : mrl_decode_relative(halves[h].relative_pitch, d->s.previous_lag);
            int pg, cg;
            mrl_prepare(&d->s, sub, lpc[sub], lag);
            mrl_innovation(&d->s, halves[h].code[sub], lag, code);
            mrl_gain_decode(d->s.gain_history, halves[h].gain[sub], code, &pg, &cg);
            mrl_finish(&d->s, sub, lpc[sub], code, pg, cg, synth + 80 * sub);
        }
        if (pre_filter) memcpy(pre_filter + 160 * h, synth, sizeof synth);
        mrl_finish_half(&d->s, synth, pcm + 160 * h);
    }
    return 0;
}

int mrl_decode(mrl_decoder *d, const uint8_t payload[20], int16_t pcm[320]) { return mrl_decode_ex(d, payload, pcm, NULL); }
