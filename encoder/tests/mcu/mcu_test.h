/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#ifndef MRL_MCU_TEST_H
#define MRL_MCU_TEST_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void mrl_mcu_test(const uint8_t *pcm, long bytes, const uint8_t *expect, long expect_bytes, uint32_t expect_hash,
                  uint32_t (*read_ticks)(void));
#ifdef __cplusplus
}
#endif
#endif
