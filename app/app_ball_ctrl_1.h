#ifndef APP_BALL_CTRL_1_H
#define APP_BALL_CTRL_1_H

#include <stdbool.h>
#include <stdint.h>

void APP_BallCtrl1_Init(void);
void APP_BallCtrl1_TimerTick(void);
void APP_BallCtrl1_Run(void);
void APP_BallCtrl1_Start(void);
void APP_BallCtrl1_Stop(void);
bool APP_BallCtrl1_IsRunning(void);

#endif /* APP_BALL_CTRL_1_H */
