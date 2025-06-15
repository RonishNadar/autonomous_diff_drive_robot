// serial_comm.cpp
#include "serial_comm.hpp"
#include "kalman_fusion.hpp"
#include "motor_control.hpp"
#include <Arduino.h>

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
    Serial.printf("v: %.2f m/s, ω: %.2f rad/s\n", lin_vel, ang_vel);
}

void processSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        int comma1 = input.indexOf(',');
        int comma2 = input.lastIndexOf(',');

        if (comma1 > 0 && comma2 > comma1) {
            int left_pwm = input.substring(0, comma1).toInt();
            int right_pwm = input.substring(comma1 + 1, comma2).toInt();
            setMotorPWM(left_pwm, right_pwm);
        }
    }
}
