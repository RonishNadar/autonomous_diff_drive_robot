// motor_control.hpp
#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

// Motor driver pin definitions
#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR 8
#define RIGHT_MOTOR_PWM 10
#define RIGHT_MOTOR_DIR 7

void initMotors();
void setMotorPWM(int left_pwm, int right_pwm);
void updateMotorPWM();

#endif // MOTOR_CONTROL_HPP
