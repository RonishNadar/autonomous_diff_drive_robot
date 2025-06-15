#include "encoder.hpp"

static volatile long left_ticks = 0;
static volatile long right_ticks = 0;

// Interrupt routines
void IRAM_ATTR handleLeftEncoder() {
    bool b = digitalRead(LEFT_ENC_B);
    if (b) left_ticks++;
    else   left_ticks--;
}

void IRAM_ATTR handleRightEncoder() {
    bool b = digitalRead(RIGHT_ENC_B);
    if (b) right_ticks++;
    else   right_ticks--;
}

void initEncoders() {
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoder, RISING);
}

float getLeftDistance() {
    noInterrupts();
    long ticks = left_ticks;
    interrupts();
    return ticks * DIST_PER_TICK;
}

float getRightDistance() {
    noInterrupts();
    long ticks = right_ticks;
    interrupts();
    return ticks * DIST_PER_TICK;
}

void resetEncoders() {
    noInterrupts();
    left_ticks = 0;
    right_ticks = 0;
    interrupts();
}
