#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

// === Configuration ===
#define WHEEL_RADIUS        0.033f      // meters
#define WHEEL_BASE          0.195f      // meters
#define TICKS_PER_REV       1250.0f     // Adjust to your encoder specs

// Pin definitions (adjust for your wiring)
#define ENCODER1_PIN_A 4
#define ENCODER1_PIN_B 5
#define ENCODER2_PIN_B 6
#define ENCODER2_PIN_A 7

// Call once during setup
void initOdometry();

// Compute new position using encoder data and fused heading
void computeOdometry();

// Accessors for current position and heading
float getOdomX();
float getOdomY();
float getOdomTheta();  // returns the fused heading (yaw)

float getActualV1();
float getActualV2();

#endif // ENCODER_HPP
