#include "imu.hpp"
#include "odometry.hpp"
#include "kalman_fusion.hpp"
#include "motor_control.hpp"
#include "serial_comm.hpp"
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial monitor to start (optional)

    initMotors();      // Configure motor driver (e.g., L298N)
    initIMU();         // Set up BNO085 or other IMU
    initOdometry();    // Reset position variables
    initKalman();
    initSerial();      // Set up serial commands if any
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    // Run at 10 Hz
    if (now - lastUpdate >= 100) {
        lastUpdate = now;

        updateIMU();            // Read new IMU data
        computeOdometry();      // Fuse IMU + encoder for position
        updateKalman();
        updateVelocityFromOdometry(getKalmanX(), getKalmanY(), getKalmanTheta());
        processSerialInput();   // Optional: serial commands from host
        updateMotorPWM();       // Speed control logic (PID/etc.)
        sendOdometrySerial();   // Print or stream fused odometry
    }
}
