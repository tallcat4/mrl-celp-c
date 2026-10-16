// Portenta H7(Cortex-M7)での計測: tests/mcu/mcu_test.c を DWT のサイクルカウンタで、結果は USB シリアルへ。
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 tallcat4
// run_portenta.sh が組み立てる(ソースとテストデータを足す)。
#include "mcu_test.h"
#include "expect_hash.h"

extern "C" const unsigned char speech_pcm[], speech_expected_mrl[];
extern "C" const unsigned int speech_pcm_len, speech_expected_mrl_len;
extern "C" int mcu_printf(const char *f, ...);
extern "C" uint32_t mcu_ticks(void);

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    delay(500);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;   // 書き込みの解錠(Cortex-M7)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    mcu_printf("core clock %lu Hz, I-cache %s, D-cache %s\n", (unsigned long)SystemCoreClock,
               (SCB->CCR & SCB_CCR_IC_Msk) ? "on" : "off", (SCB->CCR & SCB_CCR_DC_Msk) ? "on" : "off");
    mrl_mcu_test(speech_pcm, (long)speech_pcm_len, speech_expected_mrl, (long)speech_expected_mrl_len, MRL_EXPECT_HASH, mcu_ticks);
}

void loop() {}
