// motor_control.hpp
#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

// === Motor & Encoder Pins ===
#define MOTOR1_EN   15
#define MOTOR1_IN1  17
#define MOTOR1_IN2  16
#define MOTOR2_IN2  10
#define MOTOR2_IN1  11
#define MOTOR2_EN   12

#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ     20000
#define PWM_RES      8
#define MAX_PWM      255
#define MIN_PWM      70
#define MAX_VELOCITY 0.35f

void initMotors();
void updateMotorPWM();

#endif // MOTOR_CONTROL_HPP
