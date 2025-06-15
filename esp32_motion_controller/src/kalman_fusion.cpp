#include "kalman_fusion.hpp"
#include "encoder.hpp"
#include "imu.hpp"

// Kalman filter states
static float x = 0, y = 0, theta = 0;
static float P_x = 1, P_y = 1, P_theta = 1; 

static float last_x = 0.0f, last_y = 0.0f, last_theta = 0.0f;
static unsigned long last_time = 0;

static float linear_velocity = 0.0f;
static float angular_velocity = 0.0f;

void initKalman() {
    x = getOdomX();      // Start with encoder values
    y = getOdomY();
    theta = getOdomTheta();
}

void updateKalman() {
    // Get measurements
    float z_x_enc = getOdomX();       // Encoder estimate
    float z_y_enc = getOdomY();
    float z_theta_enc = getOdomTheta();

    float z_x_imu = getImuX();    // IMU estimate
    float z_y_imu = getImuY();
    float z_theta_imu = getImuTheta();

    // Fuse X
    float Kx = P_x / (P_x + R);
    x = x + Kx * ((z_x_enc + z_x_imu) / 2.0f - x);
    P_x = (1 - Kx) * P_x + Q;

    // Fuse Y
    float Ky = P_y / (P_y + R);
    y = y + Ky * ((z_y_enc + z_y_imu) / 2.0f - y);
    P_y = (1 - Ky) * P_y + Q;

    // Fuse Theta
    float Kt = P_theta / (P_theta + R);
    theta = theta + Kt * ((z_theta_enc + z_theta_imu) / 2.0f - theta);
    P_theta = (1 - Kt) * P_theta + Q;
}

void updateVelocityFromOdometry(float current_x, float current_y, float current_theta) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;  // seconds

    if (dt <= 0.0001f) return;  // prevent division by zero

    float dx = current_x - last_x;
    float dy = current_y - last_y;
    float dtheta = current_theta - last_theta;

    // Normalize angle difference to [-PI, PI]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;

    linear_velocity = sqrt(dx * dx + dy * dy) / dt;
    angular_velocity = dtheta / dt;

    last_x = current_x;
    last_y = current_y;
    last_theta = current_theta;
    last_time = now;
}

float getKalmanX() {
    // return x;
    return getOdomX();
}

float getKalmanY() {
    // return y;
    return getOdomY();
}

float getKalmanTheta() {
    // return theta;
    return getOdomTheta();
}

float getLinearVelocity() {
    return linear_velocity;
}

float getAngularVelocity() {
    return angular_velocity;
}
