#include "transforms.h"
#include "math.h"

#define SQRT_3 (const int64_t)(sqrt(3.0) * FRAC)

void clarkeTransform(abc_t *in, clarke_t *out){
    out->alpha = (2*in->a - in->b - in->c) / 3;
    out->beta = SQRT_3 * (in->b - in->c) / 3 / FRAC;
};

void inverseClarkeTransform(clarke_t *in, abc_t *out){
    out->a = in->alpha;
    out->b = (-in->alpha + SQRT_3 * (in->beta) / FRAC) / 2;
    out->c = (-in->alpha - SQRT_3 * (in->beta) / FRAC) / 2;
};

void parkTransform(clarke_t *in, park_t *out, int phi){
    int32_t sin = my_sin(phi);
    int32_t cos = my_cos(phi);
    out->d = ( cos * in->alpha + sin * in->beta) / SINE_LUT_PEAK;
    out->q = (-sin * in->alpha + cos * in->beta) / SINE_LUT_PEAK ;
};

void inverseParkTransform(park_t *in, clarke_t *out, int phi){
    int32_t sin = my_sin(phi);
    int32_t cos = my_cos(phi);
    out->alpha = (cos * in->d - sin * in->q) / SINE_LUT_PEAK;
    out->beta  = (sin * in->d + cos * in->q) / SINE_LUT_PEAK ;
};