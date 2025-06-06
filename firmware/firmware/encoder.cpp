#include "include/encoder.h"

// Encoder Pins
const int enc_a[NUM_ENCODERS] = {12, 27, 32, 25};
const int enc_b[NUM_ENCODERS] = {13, 14, 33, 26};

// Global encoder counts
volatile long encoder_counts[NUM_ENCODERS] = {0};

void IRAM_ATTR encoder_isr(int i)
{
    int a = digitalRead(enc_a[i]);
    int b = digitalRead(enc_b[i]);
    if (a == b)
        encoder_counts[i]++;
    else
        encoder_counts[i]--;
}

void IRAM_ATTR encoder_isr_0() { encoder_isr(0); }
void IRAM_ATTR encoder_isr_1() { encoder_isr(1); }
void IRAM_ATTR encoder_isr_2() { encoder_isr(2); }
void IRAM_ATTR encoder_isr_3() { encoder_isr(3); }

void setup_encoders()
{
    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        pinMode(enc_a[i], INPUT_PULLUP);
        pinMode(enc_b[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(enc_a[0]), encoder_isr_0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_a[1]), encoder_isr_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_a[2]), encoder_isr_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_a[3]), encoder_isr_3, CHANGE);
}