#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>

// Initialize the BNO085 IMU
void initIMU();

// Update internal state with latest IMU readings
void updateIMU();

// Get raw IMU yaw angle (in radians)
float getImuX();
float getImuY();
float getImuTheta();

#endif // IMU_HPP
