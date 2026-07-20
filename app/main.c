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

#include "ti_msp_dl_config.h"
#include "bsp_delay.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_grayscale.h"
#include "bsp_key.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_dma_rx.h"
#include "mid_oled.h"
#include "mid_ble.h"
#include "mid_line_track.h"
#include "mid_jy62.h"
#include "mid_gyro_hold.h"
#include "app_control.h"
#include "app_gyro_task.h"
/* ---- Callback glue (lives in main.c, delegates to app layer) ---- */

static void on_timer_10ms(void)
{
    BSP_DMA_RX_Process();
    APP_Control_TimerTick();
}

static void on_key_click(void)
{
    APP_GyroTask_Start();
}

/* ---- Main ---- */

int main(void)
{
    /* [1] Core: SysConfig-generated init (clock, GPIO, peripherals) */
    SYSCFG_DL_init();

    /* [2] BSP layer init */
    BSP_Delay_Init();
    BSP_LED_Init();
    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_Grayscale_Init();
    BSP_Key_Init();
    BSP_Timer_Init();
    BSP_UART_Init();

    /* [3] Middleware layer init */
    MID_OLED_Init();
    MID_BLE_Init();
    MID_LineTrack_Init();

    /* [3.5] Gyro subsystem init */
    BSP_DMA_RX_Init();
    BSP_DMA_RX_RegisterCallback(MID_JY62_UartRxIsr);
    MID_JY62_Init();
    MID_GyroHold_Init();

    /* [4] App layer init */
    APP_Control_Init();
    APP_GyroTask_Init();

    /* [5] Register cross-layer callbacks */
    BSP_Timer_RegisterCallback(on_timer_10ms);
    BSP_Key_RegisterClickCallback(on_key_click);

    /* [6] Boot screen */
    MID_OLED_ShowString(0, 0, "Init OK", 16);
    MID_OLED_RefreshGram();
    BSP_Delay_ms(500);
    MID_OLED_Clear();

    /* [7] Main loop: app dispatch only, no business logic here */
    while (1) {
        MID_BLE_Poll();
        APP_Control_Run();
    }
}
