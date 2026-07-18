#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*bsp_key_callback_t)(void);

bool BSP_Key_Init(void);
void BSP_Key_Scan(void);
void BSP_Key_RegisterClickCallback(bsp_key_callback_t cb);

#endif /* BSP_KEY_H */
