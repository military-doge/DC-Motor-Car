#ifndef MID_LINE_TRACK_H
#define MID_LINE_TRACK_H

#include <stdint.h>
#include <stdbool.h>

bool MID_LineTrack_Init(void);
void MID_LineTrack_Update(const uint16_t sensor_data[8],
    float *out_left_speed, float *out_right_speed);
void MID_LineTrack_Reset(void);
int8_t  MID_LineTrack_GetError(void);
int8_t  MID_LineTrack_GetLastError(void);
bool    MID_LineTrack_IsLineLost(void);

#endif /* MID_LINE_TRACK_H */
