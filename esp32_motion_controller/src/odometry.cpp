#include "odometry.hpp"
#include "encoder.hpp"

static float x = 0.0, y = 0.0, theta = 0.0;
static float last_left = 0.0f;
static float last_right = 0.0f;

void initOdometry() {
    initEncoders();
    x = 0.0f;
    y = 0.0f;
    last_left = getLeftDistance();
    last_right = getRightDistance();
}

void computeOdometry() {
    float current_left = getLeftDistance();
    float current_right = getRightDistance();

    float delta_left = current_left - last_left;
    float delta_right = current_right - last_right;

    last_left = current_left;
    last_right = current_right;

    float delta_s = (delta_left + delta_right) / 2.0f;
    float deltaTheta = (delta_right - delta_left) / WHEEL_BASE;

    theta += deltaTheta;
    x += delta_s * cos(theta);
    y += delta_s * sin(theta);
}

float getOdomX() {
    return x;
}

float getOdomY() {
    return y;
}

float getOdomTheta() {
    return theta;
}
