#pragma once

typedef struct {
    float x;
    float y;
} angle;

angle normalize(angle a);

angle rotate90(angle a);

angle rotate90neg(angle a);

angle rotate120(angle a);