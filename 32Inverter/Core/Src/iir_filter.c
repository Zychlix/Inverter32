
#include "iir_filter.h"

void iir_filter_init(iir_filter_t *filter){
  for(int i = 0; i < IIR_FILTER_LENGTH; i++){
    filter->input[i] = 0;
    filter->output[i] = 0;
  }
}

float iir_filter_calculate(iir_filter_t *filter, float input){
  float output = 0;

  for(int i = IIR_FILTER_LENGTH - 2; i >= 0 ; i--){
    filter->input[i+1] = filter->input[i];
  }
  filter->input[0] = input;

  for(int i = 0; i < IIR_FILTER_LENGTH; i++){
    output += filter->input[i] * filter->b[i];
  }

  for(int i = 0; i < IIR_FILTER_LENGTH - 1; i++){
    output -= filter->output[i] * filter->a[i + 1];
  }

  for(int i = IIR_FILTER_LENGTH - 2; i >= 0; i--){
    filter->output[i+1] = filter->output[i];
  }
  filter->output[0] = output;

  return output;
}
