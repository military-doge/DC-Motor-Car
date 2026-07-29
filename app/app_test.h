#ifndef APP_TEST_H
#define APP_TEST_H

#include <stdbool.h>

void APP_Test_Init(void);
void APP_Test_TimerTick(void);
void APP_Test_Run(void);
void APP_Test_Start(void);
void APP_Test_Stop(void);
bool APP_Test_IsRunning(void);

#endif /* APP_TEST_H */
