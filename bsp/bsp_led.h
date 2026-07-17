#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

bool BSP_LED_Init(void);
void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);
void BSP_LED_Flash(uint16_t time);

#endif /* BSP_LED_H */
