#ifndef KALMAN_FUSION_HPP
#define KALMAN_FUSION_HPP

// Tunable Kalman gains
#define R 0.5f   // Measurement noise
#define Q 0.01f  // Process noise

void initKalman();
void updateKalman();

float getKalmanX();
float getKalmanY();
float getKalmanTheta();

void updateVelocityFromOdometry(float current_x, float current_y, float current_theta);
float getLinearVelocity();
float getAngularVelocity();

#endif // KALMAN_FUSION_HPP
