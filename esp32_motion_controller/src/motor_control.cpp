// motor_control.cpp
#include "motor_control.hpp"
#include "encoder.hpp"
#include "kalman_fusion.hpp"
#include "serial_comm.hpp"
#include <Arduino.h>

// === PID gains ===
float Kp = 400.0f, Ki = 15.0f, Kd = 20.0f;

float target_v = 0.0f, target_w = 0.0f;
float v1_target = 0.0f, v2_target = 0.0f, v1_actual = 0.0f, v2_actual = 0.0f;
float integral1 = 0, integral2 = 0, last_err1 = 0, last_err2 = 0;
unsigned long last_time = 0;

// === Feedforward Mapping - Forward ===
float pwmFromVelocity(float v_target) {
    float velocity[] = {
        0.026, 0.043, 0.060, 0.077, 0.092, 0.112, 0.130, 0.150, 0.170,
        0.188, 0.206, 0.226, 0.246, 0.265, 0.284, 0.303, 0.319, 0.338,
        0.356, 0.366, 0.371, 0.371
    };
    float pwm[] = {
        150, 155, 160, 165, 170, 175, 180, 185, 190,
        195, 200, 205, 210, 215, 220, 225, 230, 235,
        240, 245, 250, 255
    };
    int n = sizeof(velocity) / sizeof(float);
    if (v_target <= velocity[0]) return pwm[0];
    if (v_target >= velocity[n - 1]) return pwm[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (v_target >= velocity[i] && v_target <= velocity[i + 1]) {
            float t = (v_target - velocity[i]) / (velocity[i + 1] - velocity[i]);
            return pwm[i] + t * (pwm[i + 1] - pwm[i]);
        }
    }
    return 0;
}

// === Feedforward Mapping - Reverse ===
float pwmFromVelocityReverse(float v_target) {
    float velocity[] = {
        0.031, 0.049, 0.065, 0.082, 0.100, 0.117, 0.134, 0.153, 0.170,
        0.188, 0.205, 0.223, 0.240, 0.258, 0.276, 0.293, 0.311, 0.329,
        0.344, 0.353, 0.357, 0.357
    };
    float pwm[] = {
        150, 155, 160, 165, 170, 175, 180, 185, 190,
        195, 200, 205, 210, 215, 220, 225, 230, 235,
        240, 245, 250, 255
    };
    int n = sizeof(velocity) / sizeof(float);
    if (v_target <= velocity[0]) return pwm[0];
    if (v_target >= velocity[n - 1]) return pwm[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (v_target >= velocity[i] && v_target <= velocity[i + 1]) {
            float t = (v_target - velocity[i]) / (velocity[i + 1] - velocity[i]);
            return pwm[i] + t * (pwm[i + 1] - pwm[i]);
        }
    }
    return 0;
}

int limitPWM(float pwm) {
    pwm = fabs(pwm);
    if (pwm > MAX_PWM) return MAX_PWM;
    if (pwm < MIN_PWM) return (pwm > 1e-2 ? MIN_PWM : 0);
    return (int)pwm;
}

void applyMotorPWM(int en_pin, int in1, int in2, int pwm_channel, int pwm, int dir) {
    digitalWrite(in1, dir > 0);
    digitalWrite(in2, dir < 0);
    ledcWrite(pwm_channel, pwm);
}

float getV1Target() {
    return v1_target;
}

float getV2Target() {
    return v2_target;
}

float getV1Actual() {
    return v1_actual;
}

float getV2Actual() {
    return v2_actual;
}


void initMotors() {
    pinMode(MOTOR1_IN1, OUTPUT); pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT); pinMode(MOTOR2_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR1_EN, PWM_CHANNEL1);
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR2_EN, PWM_CHANNEL2);
    last_time = millis();
}

void updateMotorPWM() {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.01f) return;
    last_time = now;

    // Target veocity - Get this from serial input
    v1_target = getTargetLinearVelocity() - (getTargetAngularVelocity() * WHEEL_BASE / 2);
    v2_target = getTargetLinearVelocity() + (getTargetAngularVelocity() * WHEEL_BASE / 2);

    // Target veocity - Get this from fused sensor input
    v1_actual = 0.0f;
    v2_actual = 0.0f;
    // v1_actual = getLinearVelocity() - (getAngularVelocity() * WHEEL_BASE / 2);
    // v2_actual = getLinearVelocity() + (getAngularVelocity() * WHEEL_BASE / 2);
    v1_actual = getActualV1();
    v2_actual = getActualV2();

    float err1 = fabs(v1_target) - fabs(v1_actual);
    float err2 = fabs(v2_target) - fabs(v2_actual);
    integral1 += err1 * dt;
    integral2 += err2 * dt;
    float d1 = (err1 - last_err1) / dt;
    float d2 = (err2 - last_err2) / dt;
    float pid1 = Kp * err1 + Ki * integral1 + Kd * d1;
    float pid2 = Kp * err2 + Ki * integral2 + Kd * d2;
    last_err1 = err1;
    last_err2 = err2;

    int dir1 = (v1_target > 0) ? 1 : (v1_target < 0 ? -1 : 0);
    int dir2 = (v2_target > 0) ? 1 : (v2_target < 0 ? -1 : 0);

    float ff1 = (dir1 > 0) ? pwmFromVelocity(v1_target) : pwmFromVelocityReverse(-v1_target);
    float ff2 = (dir2 > 0) ? pwmFromVelocity(v2_target) : pwmFromVelocityReverse(-v2_target);
    int pwm1 = limitPWM(ff1 + pid1);
    int pwm2 = limitPWM(ff2 + pid2);

    applyMotorPWM(MOTOR1_EN, MOTOR1_IN1, MOTOR1_IN2, PWM_CHANNEL1, pwm1, dir1);
    applyMotorPWM(MOTOR2_EN, MOTOR2_IN1, MOTOR2_IN2, PWM_CHANNEL2, pwm2, dir2);
}
