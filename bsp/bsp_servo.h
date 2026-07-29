#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>
#include <stdbool.h>

bool BSP_Servo_Init(void);
void BSP_Servo_SetAngle(uint16_t angle_deg);
void BSP_Servo_SetPWM(uint16_t pulse_us);

#endif /* BSP_SERVO_H */
