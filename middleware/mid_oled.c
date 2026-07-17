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

#include "mid_oled.h"
#include "mid_oledfont.h"
#include "bsp_delay.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdint.h>

/* ---- Static framebuffer ---- */
static uint8_t s_gram[MID_OLED_WIDTH][MID_OLED_PAGES];

/* GPIO pin macros come from SysConfig-generated ti_msp_dl_config.h.
 * Configure 4 GPIO instances in .syscfg with the following naming:
 *   Instance $name  |  Pin $name   |  Generated macros
 *   ----------------|--------------|-----------------------
 *   OLED_RST        |  PIN_RST     |  OLED_RST_PORT, OLED_RST_PIN_RST_PIN
 *   OLED_DC         |  PIN_DC      |  OLED_DC_PORT,  OLED_DC_PIN_DC_PIN
 *   OLED_SCL        |  PIN_SCL     |  OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN
 *   OLED_SDA        |  PIN_SDA     |  OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN
 */

/* ---- Low-level GPIO helpers ---- */
static void oled_rst_clr(void)
{
    DL_GPIO_clearPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
}
static void oled_rst_set(void)
{
    DL_GPIO_setPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
}
static void oled_dc_clr(void)
{
    DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
}
static void oled_dc_set(void)
{
    DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
}
static void oled_sclk_clr(void)
{
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
}
static void oled_sclk_set(void)
{
    DL_GPIO_setPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
}
static void oled_sdin_clr(void)
{
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
}
static void oled_sdin_set(void)
{
    DL_GPIO_setPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
}

/* ---- Write one byte via software SPI ---- */
static void oled_wr_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd)
        oled_dc_set();
    else
        oled_dc_clr();
    delay_cycles(10);

    for (uint8_t i = 0; i < 8; i++) {
        oled_sclk_clr();
        if (dat & 0x80)
            oled_sdin_set();
        else
            oled_sdin_clr();
        delay_cycles(20);
        oled_sclk_set();
        delay_cycles(20);
        dat <<= 1;
    }
    oled_dc_set();
}

/* ---- Set column/page address ---- */
static void oled_set_pos(uint8_t x, uint8_t y)
{
    oled_wr_byte(0xB0 + y, OLED_CMD);
    oled_wr_byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);
    oled_wr_byte(x & 0x0F, OLED_CMD);
}

/* ---- Public API ---- */

bool MID_OLED_Init(void)
{
    /* GPIO pins already configured by SYSCFG_DL_GPIO_init().
     * Directly apply hardware reset (active low) */
    oled_rst_clr();
    BSP_Delay_ms(120);
    oled_rst_set();

    /* SSD1306 init sequence (matching WHEELTEC C07A reference) */
    oled_wr_byte(0xAE, OLED_CMD); /* Display OFF */
    oled_wr_byte(0xD5, OLED_CMD); /* Clock divide ratio */
    oled_wr_byte(0x50, OLED_CMD);
    oled_wr_byte(0xA8, OLED_CMD); /* Multiplex ratio */
    oled_wr_byte(0x3F, OLED_CMD); /* 64 rows */
    oled_wr_byte(0xD3, OLED_CMD); /* Display offset */
    oled_wr_byte(0x00, OLED_CMD);
    oled_wr_byte(0x40, OLED_CMD); /* Start line = 0 */
    oled_wr_byte(0x8D, OLED_CMD); /* Charge pump */
    oled_wr_byte(0x14, OLED_CMD); /* Enable */
    oled_wr_byte(0x20, OLED_CMD); /* Memory mode */
    oled_wr_byte(0x02, OLED_CMD); /* Page addressing */
    oled_wr_byte(0xA1, OLED_CMD); /* Segment remap (column 127 = SEG0) */
    oled_wr_byte(0xC0, OLED_CMD); /* COM scan direction (normal) */
    oled_wr_byte(0xDA, OLED_CMD); /* COM pins config */
    oled_wr_byte(0x12, OLED_CMD); /* Alternative pin config */
    oled_wr_byte(0x81, OLED_CMD); /* Contrast */
    oled_wr_byte(0xEF, OLED_CMD);
    oled_wr_byte(0xD9, OLED_CMD); /* Pre-charge period */
    oled_wr_byte(0xF1, OLED_CMD); /* Phase1=1, Phase2=15 */
    oled_wr_byte(0xDB, OLED_CMD); /* VCOMH deselect */
    oled_wr_byte(0x30, OLED_CMD); /* ~0.65 x VCC */
    oled_wr_byte(0xA4, OLED_CMD); /* Resume to RAM content */
    oled_wr_byte(0xA6, OLED_CMD); /* Normal display (not inverted) */
    oled_wr_byte(0x2E, OLED_CMD); /* Deactivate scroll */
    oled_wr_byte(0xAF, OLED_CMD); /* Display ON */

    MID_OLED_Clear();
    return true;
}

void MID_OLED_DisplayOn(void)
{
    oled_wr_byte(0x8D, OLED_CMD);
    oled_wr_byte(0x14, OLED_CMD);
    oled_wr_byte(0xAF, OLED_CMD);
}

void MID_OLED_DisplayOff(void)
{
    oled_wr_byte(0x8D, OLED_CMD);
    oled_wr_byte(0x10, OLED_CMD);
    oled_wr_byte(0xAE, OLED_CMD);
}

void MID_OLED_Clear(void)
{
    memset(s_gram, 0x00, sizeof(s_gram));
    MID_OLED_RefreshGram();
}

void MID_OLED_RefreshGram(void)
{
    for (uint8_t page = 0; page < MID_OLED_PAGES; page++) {
        oled_set_pos(0, page);
        for (uint8_t col = 0; col < MID_OLED_WIDTH; col++) {
            oled_wr_byte(s_gram[col][page], OLED_DATA);
        }
    }
}

void MID_OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= MID_OLED_WIDTH || y >= MID_OLED_HEIGHT) return;

    uint8_t pos = 7 - y / 8;
    uint8_t bx  = y % 8;
    uint8_t mask = 1 << (7 - bx);

    if (color)
        s_gram[x][pos] |= mask;
    else
        s_gram[x][pos] &= ~mask;
}

void MID_OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size)
{
    if (size != 12 && size != 16) return;
    if (ch < ' ' || ch > '~') return;
    uint8_t y0 = y;
    ch -= ' ';

    for (uint8_t t = 0; t < size; t++) {
        uint8_t temp;
        if (size == 12)
            temp = asc2_1206[(uint8_t)ch][t];
        else
            temp = asc2_1608[(uint8_t)ch][t];

        for (uint8_t t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                MID_OLED_DrawPoint(x, y, 1);
            else
                MID_OLED_DrawPoint(x, y, 0);
            temp <<= 1;
            y++;
            if ((y - y0) == size) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/* Power helper for digit extraction */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void MID_OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t enshow = 0;
    for (uint8_t t = 0; t < len; t++) {
        uint8_t temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                MID_OLED_ShowChar(x + (size / 2) * t, y, ' ', size);
                continue;
            } else {
                enshow = 1;
            }
        }
        MID_OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}

void MID_OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    uint8_t char_width = size / 2;

    while (*str != '\0') {
        if (x > (MID_OLED_WIDTH - char_width)) { x = 0; y += size; }
        if (y > (MID_OLED_HEIGHT - size)) { y = 0; x = 0; MID_OLED_Clear(); }
        MID_OLED_ShowChar(x, y, *str, size);
        x += char_width;
        str++;
    }
}

