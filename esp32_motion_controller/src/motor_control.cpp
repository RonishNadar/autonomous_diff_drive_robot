// motor_control.cpp
#include "motor_control.hpp"
#include <Arduino.h>

int targetLeftPWM = 0;
int targetRightPWM = 0;

void initMotors() {
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

void setMotorPWM(int left_pwm, int right_pwm) {
    targetLeftPWM = constrain(left_pwm, -255, 255);
    targetRightPWM = constrain(right_pwm, -255, 255);
}

void updateMotorPWM() {
    // Left motor
    digitalWrite(LEFT_MOTOR_DIR, targetLeftPWM >= 0 ? HIGH : LOW);
    analogWrite(LEFT_MOTOR_PWM, abs(targetLeftPWM));

    // Right motor
    digitalWrite(RIGHT_MOTOR_DIR, targetRightPWM >= 0 ? HIGH : LOW);
    analogWrite(RIGHT_MOTOR_PWM, abs(targetRightPWM));
}
