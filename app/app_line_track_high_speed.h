#ifndef APP_LINE_TRACK_HIGH_SPEED_H
#define APP_LINE_TRACK_HIGH_SPEED_H

#include <stdint.h>
#include <stdbool.h>

void APP_LineTrack_Init(void);
void APP_LineTrack_TimerTick(void);
void APP_LineTrack_Run(void);
void APP_LineTrack_Start(void);
void APP_LineTrack_Stop(void);
bool APP_LineTrack_IsRunning(void);

#endif /* APP_LINE_TRACK_HIGH_SPEED_H */
