#ifndef APP_LINE_TRACK_Q6_HOLD_POS_H
#define APP_LINE_TRACK_Q6_HOLD_POS_H

#include <stdint.h>
#include <stdbool.h>

void APP_Q6_HoldPos_Init(void);
void APP_Q6_HoldPos_TimerTick(void);
void APP_Q6_HoldPos_Run(void);
void APP_Q6_HoldPos_Start(void);
void APP_Q6_HoldPos_Stop(void);
bool APP_Q6_HoldPos_IsRunning(void);

#endif /* APP_LINE_TRACK_Q6_HOLD_POS_H */
