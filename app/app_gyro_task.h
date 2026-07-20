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

#ifndef APP_GYRO_TASK_H
#define APP_GYRO_TASK_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    APP_GYRO_TASK_IDLE = 0,
    APP_GYRO_TASK_TURN,
    APP_GYRO_TASK_DRIVE,
    APP_GYRO_TASK_DONE
} app_gyro_task_state_t;

void                  APP_GyroTask_Init(void);
void                  APP_GyroTask_Start(void);
app_gyro_task_state_t APP_GyroTask_GetState(void);

/*
 * Called from APP_Control_TimerTick at 100 Hz.
 * Writes target speeds to *left_tgt and *right_tgt.
 * Returns true when task has finished (DONE state reached this tick).
 */
bool APP_GyroTask_Update(float drive_speed,
    int32_t encoder_a, int32_t encoder_b,
    float *left_tgt, float *right_tgt);

#endif /* APP_GYRO_TASK_H */
