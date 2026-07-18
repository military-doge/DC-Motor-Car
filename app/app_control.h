#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

void APP_Control_Init(void);
void APP_Control_TimerTick(void);
void APP_Control_Run(void);
void APP_Control_ToggleStartStop(void);
bool APP_Control_IsRunning(void);
float APP_Control_GetSpeedA(void);
float APP_Control_GetSpeedB(void);

#endif /* APP_CONTROL_H */
