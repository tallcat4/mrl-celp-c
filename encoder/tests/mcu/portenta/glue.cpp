// mcu_test.c から呼ぶ部品: USB シリアルへの printf と DWT のサイクルカウンタ。
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 tallcat4
#include <Arduino.h>
#include <stdarg.h>

extern "C" int mcu_printf(const char *f, ...) {
    char b[256];
    va_list ap;
    va_start(ap, f);
    int n = vsnprintf(b, sizeof(b), f, ap);
    va_end(ap);
    for (char *p = b; *p; p++) {
        if (*p == '\n') Serial.write('\r');
        Serial.write(*p);
    }
    return n;
}

extern "C" uint32_t mcu_ticks(void) { return DWT->CYCCNT; }
