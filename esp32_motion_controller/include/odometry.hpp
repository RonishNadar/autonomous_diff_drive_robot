#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <Arduino.h>

// Initialize odometry system (if needed)
void initOdometry();

// Compute new position using encoder data and fused heading
void computeOdometry();

// Accessors for current position and heading
float getOdomX();
float getOdomY();
float getOdomTheta();  // returns the fused heading (yaw)

#endif // ODOMETRY_HPP
