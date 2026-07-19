#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stdbool.h>

bool     BSP_UART_Init(void);
void     BSP_UART_SendByte(uint8_t byte);
void     BSP_UART_SendBytes(const uint8_t *data, uint16_t len);
void     BSP_UART_SendString(const char *str);
uint16_t BSP_UART_Available(void);
uint8_t  BSP_UART_ReadByte(void);

#endif /* BSP_UART_H */
