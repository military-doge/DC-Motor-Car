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

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

/* ---- Constants ---- */

#define RX_BUF_SIZE 256

/* ---- Ring buffer (ISR writes, poll reads) ---- */

static volatile uint8_t  s_rx_ring[RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

/* ---- Public API ---- */

bool BSP_UART_Init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;

    /* UART config (baud rate, etc.) handled by SysConfig */

    /* FIFO */
    DL_UART_Main_enableFIFOs(UART_1_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_1_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Interrupt */
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_SetPriority(UART_1_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    return true;
}

void BSP_UART_SendByte(uint8_t byte)
{
    while (DL_UART_isBusy(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, byte);
}

void BSP_UART_SendBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        BSP_UART_SendByte(data[i]);
    }
}

void BSP_UART_SendString(const char *str)
{
    while (*str) {
        BSP_UART_SendByte((uint8_t)*str++);
    }
}

uint16_t BSP_UART_Available(void)
{
    if (s_rx_head == s_rx_tail) return 0;
    if (s_rx_head > s_rx_tail) {
        return s_rx_head - s_rx_tail;
    }
    return RX_BUF_SIZE - s_rx_tail + s_rx_head;
}

uint8_t BSP_UART_ReadByte(void)
{
    uint8_t byte = 0;
    if (s_rx_head == s_rx_tail) return 0;
    byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % RX_BUF_SIZE;
    return byte;
}

/* ---- ISR ---- */

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        /* Drain RX FIFO into ring buffer */
        while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
            uint8_t  byte = DL_UART_Main_receiveData(UART_1_INST);
            uint16_t next = (s_rx_head + 1) % RX_BUF_SIZE;
            if (next != s_rx_tail) {
                s_rx_ring[s_rx_head] = byte;
                s_rx_head = next;
            }
        }
        break;
    default:
        break;
    }
}
