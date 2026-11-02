#include "sensor_signal.h"

#include <stddef.h>
#include <string.h>

/*
 * Interview baseline extracted from the product's 22 x 32 pressure-sensor
 * processing path.  The implementation is deliberately straightforward.
 * Candidates may change the implementation but should preserve the public API
 * and observable behaviour unless they document a justified behaviour change.
 */

#define CALIBRATION_FRAMES              20u
#define DETECT_MARGIN                   4u
#define ROW_ACTIVE_MIN                  3u
#define CONSECUTIVE_ROWS_MIN            13u
#define LONG_ROWS_MIN                   17u
#define TOP3_FRACTION_MAX               30u
#define THIN_TOP3_FRACTION_MAX          33u
#define VOTE_THRESHOLD                  4u
#define LEAVE_FRAMES                    50u
#define BOOT_PERSON_PRESSURE_SUM        5000u

#define MOTION_DIFF_AVERAGE_ENTER       2u
#define MOTION_DIFF_AVERAGE_EXIT        1u
#define MOTION_CHANGED_POINTS_ENTER     12u
#define MOTION_CHANGED_POINTS_EXIT      8u
#define MOTION_ENTER_FRAMES             2u
#define MOTION_STABLE_FRAMES            4u


static uint16_t sum_votes(const uint8_t votes[SENSOR_VOTE_WINDOW])
{
    uint16_t sum = 0u;
    uint8_t i;
    for (i = 0u; i < SENSOR_VOTE_WINDOW; ++i) {
        sum = (uint16_t)(sum + votes[i]);
    }
    return sum;
}

static uint8_t make_vote(const SensorHalfResult *half, uint16_t threshold)
{
    uint8_t fraction_limit =
        (half->max_consecutive_rows >= LONG_ROWS_MIN)
            ? THIN_TOP3_FRACTION_MAX
            : TOP3_FRACTION_MAX;

    return (half->average_pressure > threshold &&
            half->max_consecutive_rows >= CONSECUTIVE_ROWS_MIN &&
            half->top3_row_fraction_percent <= fraction_limit) ? 1u : 0u;
}

static void update_motion(SensorProcessor *processor,
                          const uint8_t *frame,
                          SensorResult *result)
{
    uint32_t difference_sum = 0u;
    uint16_t changed = 0u;
    uint32_t i;

    if (!processor->previous_frame_valid) {
        processor->previous_frame_valid = 1u;
        memcpy(processor->previous_frame, frame, SENSOR_FRAME_SIZE);
        return;
    }

    for (i = 0u; i < SENSOR_FRAME_SIZE; ++i) {
        uint8_t current = frame[i];
        uint8_t previous = processor->previous_frame[i];
        uint8_t difference = current > previous
            ? (uint8_t)(current - previous)
            : (uint8_t)(previous - current);
        difference_sum += difference;
        if (difference >= 2u) {
            ++changed;
        }
    }

    result->motion_changed_points = changed;
    result->motion_average_difference =
        (uint16_t)(difference_sum / SENSOR_FRAME_SIZE);

    if (!processor->in_motion) {
        processor->motion_stable_count = 0u;
        if (result->motion_average_difference >= MOTION_DIFF_AVERAGE_ENTER &&
            changed >= MOTION_CHANGED_POINTS_ENTER) {
            ++processor->motion_enter_count;
            if (processor->motion_enter_count >= MOTION_ENTER_FRAMES) {
                processor->in_motion = 1u;
                processor->motion_enter_count = 0u;
            }
        } else {
            processor->motion_enter_count = 0u;
        }
    } else {
        processor->motion_enter_count = 0u;
        if (result->motion_average_difference < MOTION_DIFF_AVERAGE_EXIT ||
            changed < MOTION_CHANGED_POINTS_EXIT) {
            ++processor->motion_stable_count;
            if (processor->motion_stable_count >= MOTION_STABLE_FRAMES) {
                processor->in_motion = 0u;
                processor->motion_stable_count = 0u;
            }
        } else {
            processor->motion_stable_count = 0u;
        }
    }

    result->in_motion = processor->in_motion;
    memcpy(processor->previous_frame, frame, SENSOR_FRAME_SIZE);
}

static void extract_half_features(const uint8_t *frame,
                                  uint8_t left,
                                  SensorHalfResult *result)
{
    uint32_t row_sum[SENSOR_ROWS] = {0u};
    uint32_t total = 0u;
    uint8_t current_active = 0u;
    uint8_t max_active = 0u;
    uint32_t row;
    uint32_t col;
    uint32_t begin = left ? 0u : SENSOR_HALF_COLS;
    uint32_t end = begin + SENSOR_HALF_COLS;

    /* 1次遍历搞定总压力、行和、连续有效行 */
    for (row = 0u; row < SENSOR_ROWS; ++row) {
        uint8_t active = 0u;
        for (col = begin; col < end; ++col) {
            uint8_t val = frame[row * SENSOR_COLS + col];
            row_sum[row] += val;
            if (val != 0u) {
                ++active;
            }
        }
        total += row_sum[row];
        
        if (active >= ROW_ACTIVE_MIN) {
            ++current_active;
            if (current_active > max_active) {
                max_active = current_active;
            }
        } else {
            current_active = 0u;
        }
    }

    /* 计算 Top-3 行占比 */
    uint32_t top[3] = {0u, 0u, 0u};
    if (total == 0u) {
        result->top3_row_fraction_percent = 100u;
    } else {
        for (row = 0u; row < SENSOR_ROWS; ++row) {
            uint32_t value = row_sum[row];
            if (value >= top[0]) {
                top[2] = top[1];
                top[1] = top[0];
                top[0] = value;
            } else if (value >= top[1]) {
                top[2] = top[1];
                top[1] = value;
            } else if (value > top[2]) {
                top[2] = value;
            }
        }
        result->top3_row_fraction_percent = 
            (uint8_t)(((top[0] + top[1] + top[2]) * 100u) / total);
    }

    /* 赋值结果 */
    result->pressure_sum = total;
    result->average_pressure = (uint16_t)(total / SENSOR_HALF_SIZE);
    result->max_consecutive_rows = max_active;
    /* max_column_sum 因为未在判定中使用，已被移除 */
}

void SensorProcessor_Init(SensorProcessor *processor)
{
    if (processor == NULL) {
        return;
    }
    memset(processor, 0, sizeof(*processor));
    processor->threshold_left = 10u;
    processor->threshold_right = 10u;
}

SensorStatus SensorProcessor_Process(SensorProcessor *processor,
                                     const uint8_t frame[SENSOR_FRAME_SIZE],
                                     SensorResult *result)
{
    uint32_t frame_sum;
    uint16_t left_votes;
    uint16_t right_votes;

    if (processor == NULL || frame == NULL || result == NULL) {
        return SENSOR_STATUS_BAD_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));
    update_motion(processor, frame, result);
    extract_half_features(frame, 1u, &result->left);
    extract_half_features(frame, 0u, &result->right);

        if (!processor->calibration_done) {
        frame_sum = result->left.pressure_sum + result->right.pressure_sum;
        
        /* 记录开机时是否有人，但不直接终止校准 */
        if (processor->calibration_count == 0u &&
            frame_sum > BOOT_PERSON_PRESSURE_SUM) {
            processor->threshold_left = 0u; /* 用 0 作为一个标记，表示开机有人 */
            processor->threshold_right = 0u;
        }

        processor->calibration_sum_left += result->left.average_pressure;
        processor->calibration_sum_right += result->right.average_pressure;
        ++processor->calibration_count;
        
        if (processor->calibration_count >= CALIBRATION_FRAMES) {
            /* 如果开机有人，把安全余量增加到 10，否则使用默认的 4 */
            uint16_t margin = (processor->threshold_left == 0u) ? 10u : DETECT_MARGIN;
            
            processor->threshold_left = (uint16_t)
                (processor->calibration_sum_left / CALIBRATION_FRAMES + margin);
            processor->threshold_right = (uint16_t)
                (processor->calibration_sum_right / CALIBRATION_FRAMES + margin);
            processor->calibration_done = 1u;
        }
        return SENSOR_STATUS_CALIBRATING;
    }

    result->left.single_frame_vote =
        make_vote(&result->left, processor->threshold_left);
    result->right.single_frame_vote =
        make_vote(&result->right, processor->threshold_right);

    processor->vote_left[processor->vote_index] = result->left.single_frame_vote;
    processor->vote_right[processor->vote_index] = result->right.single_frame_vote;
    processor->vote_index =
        (uint8_t)((processor->vote_index + 1u) % SENSOR_VOTE_WINDOW);

    left_votes = sum_votes(processor->vote_left);
    right_votes = sum_votes(processor->vote_right);

    if (!processor->left_occupied) {
        processor->leave_count_left = 0u;
        if (left_votes >= VOTE_THRESHOLD) {
            processor->left_occupied = 1u;
        }
    } else if (result->left.single_frame_vote) {
        processor->leave_count_left = 0u;
    } else if (++processor->leave_count_left >= LEAVE_FRAMES) {
        processor->left_occupied = 0u;
        processor->leave_count_left = 0u;
    }

    if (!processor->right_occupied) {
        processor->leave_count_right = 0u;
        if (right_votes >= VOTE_THRESHOLD) {
            processor->right_occupied = 1u;
        }
    } else if (result->right.single_frame_vote) {
        processor->leave_count_right = 0u;
    } else if (++processor->leave_count_right >= LEAVE_FRAMES) {
        processor->right_occupied = 0u;
        processor->leave_count_right = 0u;
    }

    result->left.occupied = processor->left_occupied;
    result->right.occupied = processor->right_occupied;
    result->person_count = (uint8_t)
        (processor->left_occupied + processor->right_occupied);
    result->in_motion = processor->in_motion;
    return SENSOR_STATUS_READY;
}
