#pragma once

#define IIR_FILTER_LENGTH 3

typedef struct {
    float a[IIR_FILTER_LENGTH];
    float b[IIR_FILTER_LENGTH];
    float input[IIR_FILTER_LENGTH];
    float output[IIR_FILTER_LENGTH];
} iir_filter_t;

void iir_filter_init(iir_filter_t *filter);
float iir_filter_calculate(iir_filter_t *filter, float input);
