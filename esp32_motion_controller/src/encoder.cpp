#include "encoder.hpp"

volatile int32_t encoder1_count = 0;
volatile int32_t encoder2_count = 0;
static float x = 0.0, y = 0.0, theta = 0.0, actual_v1 = 0.0, actual_v2 = 0.0;
unsigned long time_last = 0;

// Interrupt routines
void IRAM_ATTR encoder1ISR() {
    bool A = digitalRead(ENCODER1_PIN_A);
    bool B = digitalRead(ENCODER1_PIN_B);
    encoder1_count += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
    bool A = digitalRead(ENCODER2_PIN_A);
    bool B = digitalRead(ENCODER2_PIN_B);
    encoder2_count += (A == B) ? 1 : -1;
}

void initOdometry() {
    x = 0.0f;
    y = 0.0f;

    pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);
    pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);
    
    time_last = millis();
}

void computeOdometry() {
    unsigned long now = millis();
    float dt = (now - time_last) / 1000.0f;
    if (dt < 0.01f) return;
    time_last = now;

    int32_t c1 = encoder1_count;
    int32_t c2 = encoder2_count;
    encoder1_count = 0;
    encoder2_count = 0;

    float rps1 = (c1 / (float)TICKS_PER_REV) / dt;
    float rps2 = (c2 / (float)TICKS_PER_REV) / dt;
    float v1_actual = rps1 * 2 * PI * WHEEL_RADIUS;
    float v2_actual = rps2 * 2 * PI * WHEEL_RADIUS;
    actual_v1 = v1_actual;
    actual_v2 = v2_actual;

    float v_avg = (v1_actual + v2_actual) / 2.0f;
    float w_avg = (v2_actual - v1_actual) / WHEEL_BASE;
    theta += w_avg * dt;
    x += v_avg * cos(theta) * dt;
    y += v_avg * sin(theta) * dt;
}

float getOdomX() {
    return x;
}

float getOdomY() {
    return y;
}

float getOdomTheta() {
    return theta;
}

float getActualV1() {
    return actual_v1;
}

float getActualV2() {
    return actual_v2;
}

