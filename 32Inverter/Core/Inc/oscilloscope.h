#pragma once

#include <stdint.h>


void oscilloscope_trig();

void oscilloscope_push(float val, float valB);

void oscilloscope_check_and_send();
