#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

bool BSP_Encoder_Init(void);
int16_t BSP_Encoder_GetCountA(void);
int16_t BSP_Encoder_GetCountB(void);
void BSP_Encoder_ResetCounts(void);

#endif /* BSP_ENCODER_H */
