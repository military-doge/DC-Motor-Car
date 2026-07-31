#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

void APP_Control_Init(void);
void APP_Control_TimerTick(void);
void APP_Control_Run(void);
void APP_Control_Start(void);
void APP_Control_Stop(void);
void APP_Control_ToggleStartStop(void);
bool APP_Control_IsRunning(void);
void APP_Control_StartDirect(float speed_mps);
void APP_Control_StopDirect(void);
void APP_Control_StartSweep(float kp, float ki);
bool APP_Control_IsSweepDone(void);
uint16_t APP_Control_GetSweepCount(void);
int16_t *APP_Control_GetSweepTarget(void);
int16_t *APP_Control_GetSweepActualL(void);
int16_t *APP_Control_GetSweepActualR(void);
void APP_Control_SetPID(float kp, float ki);
void APP_Control_GetPID(float *kp, float *ki);
float APP_Control_GetSpeedA(void);
float APP_Control_GetSpeedB(void);

#endif /* APP_CONTROL_H */
