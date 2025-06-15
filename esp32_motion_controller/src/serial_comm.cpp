// serial_comm.cpp
#include "serial_comm.hpp"
#include "kalman_fusion.hpp"
#include "motor_control.hpp"
#include <Arduino.h>

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
    Serial.printf("L: %.2f (%.2f) | R: %.2f (%.2f)\n",
                  getV1Actual(), getV1Target(), getV2Actual(), getV2Target());
}

void processSerialInput() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleSerialCommand(cmd);
    }
}
