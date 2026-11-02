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

static uint32_t sum_half(const uint8_t *frame, uint8_t left)
{
    uint32_t sum = 0u;
    uint32_t row;
    uint32_t col;
    uint32_t begin = left ? 0u : SENSOR_HALF_COLS;
    uint32_t end = begin + SENSOR_HALF_COLS;

    for (row = 0u; row < SENSOR_ROWS; ++row) {
        for (col = begin; col < end; ++col) {
            sum += frame[row * SENSOR_COLS + col];
        }
    }
    return sum;
}

static uint16_t max_column_sum(const uint8_t *frame, uint8_t left)
{
    uint32_t column_sum[SENSOR_HALF_COLS] = {0u};
    uint32_t maximum = 0u;
    uint32_t row;
    uint32_t col;
    uint32_t offset = left ? 0u : SENSOR_HALF_COLS;

    for (row = 0u; row < SENSOR_ROWS; ++row) {
        for (col = 0u; col < SENSOR_HALF_COLS; ++col) {
            column_sum[col] += frame[row * SENSOR_COLS + offset + col];
        }
    }
    for (col = 0u; col < SENSOR_HALF_COLS; ++col) {
        if (column_sum[col] > maximum) {
            maximum = column_sum[col];
        }
    }
    return (uint16_t)maximum;
}

static uint8_t max_consecutive_active_rows(const uint8_t *frame, uint8_t left)
{
    uint8_t current = 0u;
    uint8_t maximum = 0u;
    uint32_t row;
    uint32_t col;
    uint32_t begin = left ? 0u : SENSOR_HALF_COLS;
    uint32_t end = begin + SENSOR_HALF_COLS;

    for (row = 0u; row < SENSOR_ROWS; ++row) {
        uint8_t active = 0u;
        for (col = begin; col < end; ++col) {
            if (frame[row * SENSOR_COLS + col] != 0u) {
                ++active;
            }
        }
        if (active >= ROW_ACTIVE_MIN) {
            ++current;
            if (current > maximum) {
                maximum = current;
            }
        } else {
            current = 0u;
        }
    }
    return maximum;
}

static uint8_t top3_row_fraction_percent(const uint8_t *frame, uint8_t left)
{
    uint32_t row_sum[SENSOR_ROWS];
    uint32_t top[3] = {0u, 0u, 0u};
    uint32_t total = 0u;
    uint32_t row;
    uint32_t col;
    uint32_t begin = left ? 0u : SENSOR_HALF_COLS;
    uint32_t end = begin + SENSOR_HALF_COLS;

    for (row = 0u; row < SENSOR_ROWS; ++row) {
        row_sum[row] = 0u;
        for (col = begin; col < end; ++col) {
            row_sum[row] += frame[row * SENSOR_COLS + col];
        }
        total += row_sum[row];
    }
    if (total == 0u) {
        return 100u;
    }

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

    return (uint8_t)(((top[0] + top[1] + top[2]) * 100u) / total);
}

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
    result->pressure_sum = sum_half(frame, left);
    result->average_pressure =
        (uint16_t)(result->pressure_sum / SENSOR_HALF_SIZE);
    result->max_column_sum = max_column_sum(frame, left);
    result->max_consecutive_rows = max_consecutive_active_rows(frame, left);
    result->top3_row_fraction_percent =
        top3_row_fraction_percent(frame, left);
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
        if (processor->calibration_count == 0u &&
            frame_sum > BOOT_PERSON_PRESSURE_SUM) {
            processor->threshold_left = DETECT_MARGIN;
            processor->threshold_right = DETECT_MARGIN;
            processor->calibration_done = 1u;
            return SENSOR_STATUS_CALIBRATING;
        }

        processor->calibration_sum_left += result->left.average_pressure;
        processor->calibration_sum_right += result->right.average_pressure;
        ++processor->calibration_count;
        if (processor->calibration_count >= CALIBRATION_FRAMES) {
            processor->threshold_left = (uint16_t)
                (processor->calibration_sum_left / CALIBRATION_FRAMES + DETECT_MARGIN);
            processor->threshold_right = (uint16_t)
                (processor->calibration_sum_right / CALIBRATION_FRAMES + DETECT_MARGIN);
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
