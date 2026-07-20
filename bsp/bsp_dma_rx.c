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

#include "bsp_dma_rx.h"
#include "ti_msp_dl_config.h"

#define BSP_DMA_RX_BUF_SIZE  256

/* DMA buffer must be aligned for DMA access */
static uint8_t  s_dma_buf[BSP_DMA_RX_BUF_SIZE] __attribute__((aligned(32)));
static uint32_t s_read_idx;
static uint32_t s_last_remaining;
static uint32_t s_byte_count;
static bsp_dma_rx_callback_t s_rx_callback = NULL;

bool BSP_DMA_RX_Init(void)
{
    DL_DMA_Config dma_cfg = {
        .trigger       = UART_2_INST_DMA_TRIGGER,
        .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL,
        .transferMode  = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode  = 0,
        .srcWidth      = DL_DMA_WIDTH_BYTE,
        .destWidth     = DL_DMA_WIDTH_BYTE,
        .srcIncrement  = 0,   /* Fixed source: UART RXDATA register */
        .destIncrement = DL_DMA_ADDR_INCREMENT,
    };

    /* Ensure UART2 is active and loopback disabled before DMA setup */
    DL_UART_Main_disable(UART_2_INST);
    DL_UART_Main_disableLoopbackMode(UART_2_INST);
    DL_UART_Main_enable(UART_2_INST);
    while (!DL_UART_isRXFIFOEmpty(UART_2_INST)) {
        DL_UART_receiveData(UART_2_INST);
    }

    /* Disable channel first in case SysConfig already configured it */
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);

    /* Init DMA channel 0 for UART2 RX */
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID, &dma_cfg);

    /* Source = UART2 RX data register (fixed), Dest = buffer */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)&(UART_2_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)s_dma_buf);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BSP_DMA_RX_BUF_SIZE);

    /* Enable UART2 -> DMA trigger */
    DL_UART_enableDMAReceiveEvent(UART_2_INST, DL_UART_DMA_INTERRUPT_RX);

    /* Start DMA */
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    s_read_idx      = 0;
    s_last_remaining = BSP_DMA_RX_BUF_SIZE;
    s_byte_count    = 0;

    return true;
}

void BSP_DMA_RX_RegisterCallback(bsp_dma_rx_callback_t cb)
{
    s_rx_callback = cb;
}

void BSP_DMA_RX_Process(void)
{
    uint32_t remaining;
    uint32_t write_idx;

    remaining = DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);

    /*
     * DMA fills buffer from index 0 upward.  After transferring N bytes,
     * `remaining` = BSP_DMA_RX_BUF_SIZE - N, so the write position is:
     *   write_idx = BSP_DMA_RX_BUF_SIZE - remaining
     */
    if (remaining == s_last_remaining)
        return;  /* No new data */

    /* Handle wrap-around: DMA finished and was restarted */
    if (remaining > s_last_remaining) {
        /* Process tail of buffer first */
        while (s_read_idx < BSP_DMA_RX_BUF_SIZE) {
            s_rx_callback(s_dma_buf[s_read_idx++]);
            s_byte_count++;
        }
        s_read_idx = 0;
    }

    s_last_remaining = remaining;
    write_idx = BSP_DMA_RX_BUF_SIZE - remaining;

    /* Process new bytes */
    while (s_read_idx < write_idx) {
        s_rx_callback(s_dma_buf[s_read_idx++]);
        s_byte_count++;
    }

    /* If buffer nearing full, restart DMA */
    if (remaining < 32) {
        DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);

        /* Flush any last bytes that arrived during disable */
        write_idx = BSP_DMA_RX_BUF_SIZE -
            DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
        while (s_read_idx < write_idx) {
            s_rx_callback(s_dma_buf[s_read_idx++]);
            s_byte_count++;
        }

        /* Reset and restart */
        DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)s_dma_buf);
        DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BSP_DMA_RX_BUF_SIZE);
        DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

        s_read_idx      = 0;
        s_last_remaining = BSP_DMA_RX_BUF_SIZE;
    }
}

uint32_t BSP_DMA_RX_GetByteCount(void)
{
    return s_byte_count;
}
