/* ESP-IDF の入口(tests/mcu/mcu_test.c、データは EMBED_FILES)。
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 tallcat4 */
#include "esp_cpu.h"
#include "mcu_test.h"
#include "expect_hash.h"

extern const uint8_t pcm_start[] asm("_binary_speech_pcm_start");
extern const uint8_t pcm_end[] asm("_binary_speech_pcm_end");
extern const uint8_t exp_start[] asm("_binary_speech_expected_mrl_start");
extern const uint8_t exp_end[] asm("_binary_speech_expected_mrl_end");

static uint32_t ticks(void) { return esp_cpu_get_cycle_count(); }

void app_main(void) {
    mrl_mcu_test(pcm_start, (long)(pcm_end - pcm_start), exp_start, (long)(exp_end - exp_start), MRL_EXPECT_HASH, ticks);
}
