#ifndef APP_LINE_TRACK_LOW_SPEED_STRAIGHT_H
#define APP_LINE_TRACK_LOW_SPEED_STRAIGHT_H

#include <stdint.h>
#include <stdbool.h>

void APP_LineTrack_LowSpeedStraight_Init(void);
void APP_LineTrack_LowSpeedStraight_TimerTick(void);
void APP_LineTrack_LowSpeedStraight_Run(void);
void APP_LineTrack_LowSpeedStraight_Start(void);
void APP_LineTrack_LowSpeedStraight_Stop(void);
bool APP_LineTrack_LowSpeedStraight_IsRunning(void);

#endif /* APP_LINE_TRACK_LOW_SPEED_STRAIGHT_H */
