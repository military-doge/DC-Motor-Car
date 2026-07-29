#ifndef APP_LINE_TRACK_LOW_SPEED_CIRCLE_H
#define APP_LINE_TRACK_LOW_SPEED_CIRCLE_H

#include <stdint.h>
#include <stdbool.h>

void APP_LineTrack_LowSpeedCircle_Init(void);
void APP_LineTrack_LowSpeedCircle_TimerTick(void);
void APP_LineTrack_LowSpeedCircle_Run(void);
void APP_LineTrack_LowSpeedCircle_Start(void);
void APP_LineTrack_LowSpeedCircle_Stop(void);
bool APP_LineTrack_LowSpeedCircle_IsRunning(void);

#endif /* APP_LINE_TRACK_LOW_SPEED_CIRCLE_H */
