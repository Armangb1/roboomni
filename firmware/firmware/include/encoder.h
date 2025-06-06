#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Encoder configuration
#define NUM_ENCODERS 4

// Encoder pin definitions
extern const int enc_a[NUM_ENCODERS];
extern const int enc_b[NUM_ENCODERS];

// Global encoder counts
extern volatile long encoder_counts[NUM_ENCODERS];

// Function declarations
void setup_encoders(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H