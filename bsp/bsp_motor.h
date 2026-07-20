#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

bool BSP_Motor_Init(void);
void BSP_Motor_SetPWM(int16_t pwm_a, int16_t pwm_b);
void BSP_Motor_Stop(void);

#endif /* BSP_MOTOR_H */
