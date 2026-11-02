#ifndef SENSOR_SIGNAL_H
#define SENSOR_SIGNAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_ROWS              22u
#define SENSOR_COLS              32u
#define SENSOR_HALF_COLS         16u
#define SENSOR_FRAME_SIZE        (SENSOR_ROWS * SENSOR_COLS)
#define SENSOR_HALF_SIZE         (SENSOR_ROWS * SENSOR_HALF_COLS)
#define SENSOR_VOTE_WINDOW       5u

typedef enum
{
    SENSOR_STATUS_CALIBRATING = 0,
    SENSOR_STATUS_READY = 1,
    SENSOR_STATUS_BAD_ARGUMENT = 2
} SensorStatus;

typedef struct
{
    uint32_t pressure_sum;
    uint16_t average_pressure;
    uint16_t max_column_sum;
    uint8_t max_consecutive_rows;
    uint8_t top3_row_fraction_percent;
    uint8_t single_frame_vote;
    uint8_t occupied;
} SensorHalfResult;

typedef struct
{
    SensorHalfResult left;
    SensorHalfResult right;
    uint16_t motion_changed_points;
    uint16_t motion_average_difference;
    uint8_t in_motion;
    uint8_t person_count;
} SensorResult;

typedef struct
{
    uint8_t calibration_done;
    uint8_t calibration_count;
    uint32_t calibration_sum_left;
    uint32_t calibration_sum_right;
    uint16_t threshold_left;
    uint16_t threshold_right;

    uint8_t vote_left[SENSOR_VOTE_WINDOW];
    uint8_t vote_right[SENSOR_VOTE_WINDOW];
    uint8_t vote_index;
    uint8_t left_occupied;
    uint8_t right_occupied;
    uint8_t leave_count_left;
    uint8_t leave_count_right;

    uint8_t previous_frame[SENSOR_FRAME_SIZE];
    uint8_t previous_frame_valid;
    uint8_t in_motion;
    uint8_t motion_enter_count;
    uint8_t motion_stable_count;
} SensorProcessor;

void SensorProcessor_Init(SensorProcessor *processor);

SensorStatus SensorProcessor_Process(SensorProcessor *processor,
                                     const uint8_t frame[SENSOR_FRAME_SIZE],
                                     SensorResult *result);

#ifdef __cplusplus
}
#endif

#endif
