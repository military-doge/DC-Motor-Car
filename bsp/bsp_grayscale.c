/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "bsp_grayscale.h"
#include "ti_msp_dl_config.h"

static void grayscale_select_channel(uint8_t channel)
{
    if (channel & 1) {
        DL_GPIO_setPins(GS_AD_PORT, GS_AD_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GS_AD_PORT, GS_AD_AD0_PIN);
    }

    if (channel & 2) {
        DL_GPIO_setPins(GS_AD_PORT, GS_AD_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GS_AD_PORT, GS_AD_AD1_PIN);
    }

    if (channel & 4) {
        DL_GPIO_setPins(GS_AD_PORT, GS_AD_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GS_AD_PORT, GS_AD_AD2_PIN);
    }
}

static uint16_t grayscale_read_out(void)
{
    return !!(DL_GPIO_readPins(GS_OUT_PORT, GS_OUT_OUT_PIN));
}

static void grayscale_delay_us(uint32_t us)
{
    uint32_t target = us * (CPUCLK_FREQ / 1000000UL);
    uint32_t start  = DL_SYSTICK_getValue();

    while (1) {
        uint32_t now     = DL_SYSTICK_getValue();
        uint32_t elapsed = (start >= now)
            ? (start - now)
            : (start + DL_SYSTICK_getPeriod() + 1U - now);
        if (elapsed >= target) break;
    }
}

bool BSP_Grayscale_Init(void)
{
    /* Clear all address lines initially (select channel 0) */
    DL_GPIO_clearPins(GS_AD_PORT,
        GS_AD_AD0_PIN | GS_AD_AD1_PIN | GS_AD_AD2_PIN);

    return true;
}

void BSP_Grayscale_ReadAll(uint16_t *out_values)
{
    uint8_t i;

    for (i = 0; i < BSP_GRAYSCALE_CHANNELS; i++) {
        grayscale_select_channel(i);
        grayscale_delay_us(50);
        out_values[i] = grayscale_read_out();
    }
}
