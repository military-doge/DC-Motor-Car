#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*bsp_timer_callback_t)(void);

bool BSP_Timer_Init(void);
void BSP_Timer_RegisterCallback(bsp_timer_callback_t cb);

#endif /* BSP_TIMER_H */
