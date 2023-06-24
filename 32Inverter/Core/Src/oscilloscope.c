#include "oscilloscope.h"
#include "stdio.h"

#define OSC_BUF_LEN 512


volatile enum {
    OSC_WAITING,
    OSC_ACQUIRING,
    OSC_SENDING,
} state = OSC_WAITING;

volatile uint16_t osc_index = 0;

float bufferA[OSC_BUF_LEN];
float bufferB[OSC_BUF_LEN];

void oscilloscope_trig() {
    if (state != OSC_WAITING) return;
    osc_index = 0;
    state = OSC_ACQUIRING;
}

void oscilloscope_push(float val, float valB) {
    if (state != OSC_ACQUIRING) return;
    if (osc_index >= OSC_BUF_LEN) {
        state = OSC_SENDING;
        return;
    }

    bufferA[osc_index] = val;
    bufferB[osc_index] = valB;
    osc_index++;

}

void oscilloscope_check_and_send() {
    if (state != OSC_SENDING) return;
    printf("START\n");
    for (int i = 0; i < OSC_BUF_LEN; i++) {
        printf("%06.3f %06.3f\n", bufferA[i], bufferB[i]);
    }
    printf("\n");
    state = OSC_WAITING;
}
