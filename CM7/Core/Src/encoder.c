/*
 * encoder.c
 */

#include "main.h"
#include "encoder.h"

// Calculate the distance based on the encoder counts

float EncoderGetDistance(uint16_t encoder_counts)
{
    // Calculate distance by dividing encoder counts by CPR and multiplying by wheel circumference

    float distance = (float)encoder_counts / ENCODER_CPR * WHEEL_CIRCUMFERENCE;
    return distance;
}
