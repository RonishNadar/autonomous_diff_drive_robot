#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

// Pin definitions (adjust for your wiring)
#define LEFT_ENC_A 2
#define LEFT_ENC_B 3
#define RIGHT_ENC_A 4
#define RIGHT_ENC_B 5

// === Configuration ===
#define WHEEL_RADIUS        0.03f                       // meters
#define TICKS_PER_REV       1250.0f                     // Adjust to your encoder specs
#define WHEEL_CIRCUMFERENCE (2.0f * PI * WHEEL_RADIUS)
#define DIST_PER_TICK       (WHEEL_CIRCUMFERENCE / TICKS_PER_REV)
#define WHEEL_BASE          0.16f                       // meters

// Call once during setup
void initEncoders();

// These return cumulative distance traveled (in meters)
float getLeftDistance();
float getRightDistance();

// Optional: reset encoder tick counters
void resetEncoders();

#endif // ENCODER_HPP
