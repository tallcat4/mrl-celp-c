/* 改良型 RL-CELP の音声フレームの誤り訂正(C99、malloc なし、libm なし)。
 * 無線の 1 フレーム = 384 bit(48 byte、MSB first)⇔ 音声のペイロード 20 byte(158 bit、LSB first)。
 * 384 bit は 128 bit × 3 の区間で、各区間の先頭 16 bit が同期の符号、残り 112 bit × 3 = 336 bit がデータ。
 * データはスクランブル(PN 9 段)をかけた 42 byte の語で、その中身は
 *   ペイロードの 78 bit + CRC-8 → 畳み込み符号(拘束長 9、パンクチャ)176 bit
 *   ペイロードの残り 80 bit は符号なし
 *   語の 28〜41 byte: モード(0x80、0/1)と CRC-16 と Reed-Solomon のパリティ 6 byte
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 KIRISHIKI Yudai, tallcat4 */
#ifndef MRLFEC_H
#define MRLFEC_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MRL_ENVELOPE_BYTES 48
#define MRL_ENVELOPE_BITS 384
#ifndef MRL_FRAME_BYTES
#define MRL_FRAME_BYTES 20
#endif

/* 硬判定の復号。原本(jr-conventional-line-radio-receiver の voice_frame::decode)とビット一致。
 * 戻り値: 0 = 受理(payload に 20 byte、*corrected_bits に訂正したビット数、NULL 可)、-1 = 棄却 */
int mrl_fec_decode(const uint8_t envelope[MRL_ENVELOPE_BYTES], uint8_t payload[MRL_FRAME_BYTES],
                   int *corrected_bits);

/* 軟判定つきの復号(原本の voice_frame::decode_soft)。まず硬判定を試し、だめなら
 * ビットごとの信頼度で重みをつけた Viterbi と、信頼度の低い byte を消失として扱う RS の訂正を試す。
 * confidence: 384 bit それぞれの信頼度(0 = まったく当てにならない、大きいほど確か、envelope と同じ MSB first の並び)。
 * NULL なら硬判定だけ。スタックは硬判定で約 5 KB、軟判定で約 6.5 KB 使う */
int mrl_fec_decode_soft(const uint8_t envelope[MRL_ENVELOPE_BYTES], const uint8_t confidence[MRL_ENVELOPE_BITS],
                        uint8_t payload[MRL_FRAME_BYTES], int *corrected_bits);

/* 符号化(原本の voice_frame::encode と coding::scramble)。同期の符号も入れた 384 bit を作る。
 * payload の上位 2 bit は無視する。alternate: 語の 33 byte 目(0 か 1) */
void mrl_fec_encode(const uint8_t payload[MRL_FRAME_BYTES], int alternate, uint8_t envelope[MRL_ENVELOPE_BYTES]);

#ifdef __cplusplus
}
#endif
#endif
