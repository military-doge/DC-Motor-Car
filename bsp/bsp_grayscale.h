#ifndef BSP_GRAYSCALE_H
#define BSP_GRAYSCALE_H

#include <stdint.h>
#include <stdbool.h>

#define BSP_GRAYSCALE_CHANNELS 8

bool BSP_Grayscale_Init(void);
void BSP_Grayscale_ReadAll(uint16_t *out_values);

#endif /* BSP_GRAYSCALE_H */
