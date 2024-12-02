/*
 * complementaryFilter.c
 */

#include "main.h"
#include "complementaryFilter.h"

float fuseMeasurements(float measurement_1, uint16_t measurement_2)
{
    // Convert second measurement (GPS angle and distance) to float for consistency

    float measurement_2_float = (float)measurement_2;

    // Apply the complementary filter

    float fusedMeasurement = ALPHA * measurement_1 + (1 - ALPHA) * measurement_2_float;

    return fusedMeasurement;
}
