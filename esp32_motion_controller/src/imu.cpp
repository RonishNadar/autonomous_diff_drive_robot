#include "imu.hpp"
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <math.h>

// IMU object
BNO08x imu;
static bool imu_connected = false;  // ✅ New flag

// Internal state
static float x = 0.0, y = 0.0;
static float vx = 0.0, vy = 0.0;
static float theta = 0.0;

static unsigned long lastTime = 0;

#define GRAVITY 9.80665f
#define DEG_TO_RAD 0.01745329251f

float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void initIMU() {
    Wire.begin();

    if (!imu.begin()) {
        imu_connected = false;
        return;
    }

    imu.enableReport(SH2_ACCELEROMETER);
    imu.enableReport(SH2_GYROSCOPE_CALIBRATED);
    imu.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);

    lastTime = millis();
    imu_connected = true;
}

void updateIMU() {
    if (!imu_connected) return;
    if (!imu.getSensorEvent()) return;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float ax = imu.getAccelX() * GRAVITY;
    float ay = imu.getAccelY() * GRAVITY;
    float gz = imu.getGyroZ();
    float mx = imu.getMagX();
    float my = imu.getMagY();

    // Estimate orientation (theta)
    float gyroTheta = gz * DEG_TO_RAD * dt;
    float magTheta = atan2(my, mx);
    theta += 0.98f * gyroTheta + 0.02f * (magTheta - theta);
    theta = normalizeAngle(theta);

    float accX = ax * cos(theta) - ay * sin(theta);
    float accY = ax * sin(theta) + ay * cos(theta);

    vx += accX * dt;
    vy += accY * dt;

    x += vx * dt;
    y += vy * dt;
}

float getImuX() {
    return x;
}

float getImuY() {
    return y;
}

float getImuTheta() {
    return theta;
}