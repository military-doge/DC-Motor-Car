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

/* Parameter override: app layer can call these to override mid defaults.
 * If never called, the built-in default tables are used. */
void        MID_LineTrack_SetBaseSpeed(float speed);
float       MID_LineTrack_GetBaseSpeed(void);
void        MID_LineTrack_SetKpTable(const float kp[8]);
void        MID_LineTrack_SetKdTable(const float kd[8]);
const float *MID_LineTrack_GetKpTable(void);
const float *MID_LineTrack_GetKdTable(void);
void        MID_LineTrack_ResetParams(void);

#endif /* MID_LINE_TRACK_H */
