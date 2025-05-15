/*
 * encoder.c
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */


#include "encoder.h"

void encoder_init(EncoderData* enc, TIM_HandleTypeDef* htim, float cpr,
                 float gear_ratio, float conversion_factor) {
    enc->htim = htim;
    enc->count = 0;
    enc->last_count = (int32_t)__HAL_TIM_GET_COUNTER(htim);
    enc->speed = 0;
    enc->position = 0.0f;
    enc->speed_real = 0.0f;
    enc->cpr = cpr;
    enc->gear_ratio = gear_ratio;
    enc->conversion_factor = conversion_factor;
    enc->last_speed_calc_time = HAL_GetTick();


    float Q = 1.0f;    // Process noise (smaller = trust model more)
    float R = 0.05f;   // Measurement noise (smaller = trust measurement more)
    kalman_velocity_estimator_init(&enc->kalman_vel, Q, R);

    enc->kalman_velocity = 0.0f;
    enc->kalman_position = 0.0f;
}

void encoder_read(EncoderData* enc) {
    uint16_t current_count = __HAL_TIM_GET_COUNTER(enc->htim);

    enc->last_position = enc->position;

    int16_t diff = (int16_t)(current_count - enc->last_count);

    enc->count += diff;
    enc->last_count = current_count;

    enc->position = (float)enc->count * enc->conversion_factor / (enc->cpr * enc->gear_ratio);
}



void encoder_calculate_speed(EncoderData* enc) {
    uint32_t current_time = HAL_GetTick();
    uint32_t time_diff = current_time - enc->last_speed_calc_time;

    if (time_diff >= 10) {  // Calculate speed every 10ms
        float dt = time_diff / 1000.0f;  // Convert to seconds

        float position_diff = enc->position - enc->last_position;

        float raw_speed = position_diff / dt;

        float alpha = 0.7f;
        enc->speed_real = alpha * raw_speed + (1.0f - alpha) * enc->speed_real;

        enc->speed = (int16_t)(enc->speed_real);

        enc->last_speed_calc_time = current_time;
    }
}
void encoder_reset(EncoderData* enc) {
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->count = 0;
    enc->last_count = 0;
    enc->speed = 0;
    enc->position = 0.0f;
    enc->last_position = 0.0f;  // Reset the new field
    enc->speed_real = 0.0f;
    enc->kalman_velocity = 0.0f;
    enc->kalman_position = 0.0f;
    enc->last_count_for_speed = 0;

    float Q = 1.0f;
    float R = 0.05f;
    kalman_velocity_estimator_init(&enc->kalman_vel, Q, R);
}
