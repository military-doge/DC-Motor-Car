#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

bool BSP_Delay_Init(void);
void BSP_Delay_ms(uint32_t ms);
void BSP_Delay_us(uint32_t us);
uint32_t BSP_Delay_GetTick(void);

/* Override this macro to feed the watchdog during busy-wait delay loops.
 * Example when WWDT is enabled via SysConfig:
 *   #include <ti/driverlib/dl_wwdt.h>
 *   #define BSP_DELAY_WDT_FEED()  DL_WWDT_clearTimer(WWDT_0_INST)
 */
#ifndef BSP_DELAY_WDT_FEED
#define BSP_DELAY_WDT_FEED()  ((void)0)
#endif

#endif /* BSP_DELAY_H */
