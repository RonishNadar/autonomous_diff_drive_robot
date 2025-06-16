// serial_comm.cpp
#include "serial_comm.hpp"
#include "kalman_fusion.hpp"
#include "motor_control.hpp"
#include <Arduino.h>

extern float v1_target, v2_target, v1_actual, v2_actual;
extern int pwm1, pwm2, dir1, dir2;
extern float ff1, ff2;
extern float v1_actual, v2_actual;
static float tar_lin_vel = 0.0f, tar_ang_vel = 0.0f;

float getTargetLinearVelocity() {
    return tar_lin_vel;
}

float getTargetAngularVelocity() {
    return tar_ang_vel;
}

void handleSerialCommand(String input) {
    input.trim();
    if (input.startsWith("set ")) {
        input = input.substring(4);
        int space = input.indexOf(' ');
        if (space == -1) return;
        String key = input.substring(0, space);
        float val = input.substring(space + 1).toFloat();

        if (key == "v") {
            tar_lin_vel = constrain(val, -MAX_VELOCITY, MAX_VELOCITY);
            Serial.printf("Linear velocity set to %.3f\n", tar_lin_vel);
        } else if (key == "w") {
            tar_ang_vel = val;
            Serial.printf("Angular velocity set to %.3f\n", tar_ang_vel);
        }
    }
}

void initSerial() {
    Serial.begin(115200);
    while (!Serial);
}

void sendOdometrySerial() {
    float x = getKalmanX();
    float y = getKalmanY();
    float theta = getKalmanTheta();
    float lin_vel = getLinearVelocity();
    float ang_vel = getAngularVelocity();

    Serial.printf("%.2f,%.2f,%.2f\n", x, y, theta);
    Serial.printf("L: %.2f (%.2f) | R: %.2f (%.2f) | PWM L: %d | PWM R: %d | DIR L: %d | DIR R: %d | FF L: %.2f | FF R: %.2f\n",
                  v1_actual, v1_target, v2_actual, v1_target, pwm1, pwm2, dir1, dir2, v1_actual, v2_actual);
}

void processSerialInput() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleSerialCommand(cmd);
    }
}
