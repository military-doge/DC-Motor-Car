#ifndef MID_K230_H
#define MID_K230_H

#include <stdint.h>
#include <stdbool.h>

bool     MID_K230_Init(void);
void     MID_K230_Poll(void);
bool     MID_K230_IsDetected(void);
float    MID_K230_GetPosition(void);
uint32_t MID_K230_GetTimestamp(void);   /* K230 帧时间戳 (ms) */
uint32_t MID_K230_GetLastUpdate(void);  /* 本地 tick, 用于超时检测 */

#endif /* MID_K230_H */
