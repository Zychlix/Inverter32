#pragma once

typedef struct {
    union {
        float x;
        float d;
    };
    union {
        float y;
        float q;
    };
} vec_t;

typedef struct {
    float a;
    float b;
    float c;
} abc_t;

vec_t normalize(vec_t a);

vec_t limit_amplitude(vec_t a, float max_amplitude);

vec_t rotate90(vec_t a);

vec_t rotate90neg(vec_t a);

vec_t rotate120(vec_t a);

vec_t rotate180(vec_t a);

vec_t angle(float phi);

vec_t clarkeTransform(abc_t in);

abc_t inverseClarkeTransform(vec_t in);

vec_t parkTransform(vec_t in, vec_t phi);

vec_t inverseParkTransform(vec_t in, vec_t phi);