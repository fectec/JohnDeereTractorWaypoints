#ifndef MOVE_TYPES_H_
#define MOVE_TYPES_H_

#include <stdint.h>

typedef struct{
    uint16_t x;
    uint16_t y;
    uint16_t angle;
} Coordinates;

typedef struct {
    float x;
    float y;
    float angle;
} CorrectionParameters;

typedef struct {
    float x;
    float y;
    float angle;
    float confidence;
} FusedCoordinate;

#endif