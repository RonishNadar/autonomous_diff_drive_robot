#include <Arduino.h>
#include <math.h>

// === Robot Geometry ===
#define WHEEL_RADIUS 0.033f     // meters
#define WHEEL_BASE   0.195f     // meters
#define TICKS_PER_REV 1250

// === Motor & Encoder Pins ===
#define ENCODER1_PIN_A 4
#define ENCODER1_PIN_B 5
#define ENCODER2_PIN_B 6
#define ENCODER2_PIN_A 7
#define MOTOR1_EN   15
#define MOTOR1_IN1  17
#define MOTOR1_IN2  16
#define MOTOR2_IN1  11
#define MOTOR2_IN2  10
#define MOTOR2_EN   12

#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ     20000
#define PWM_RES      8
#define MAX_PWM      255
#define MIN_PWM      70
#define MAX_VELOCITY 0.35f

// === PID gains ===
float Kp = 400.0f, Ki = 15.0f, Kd = 20.0f;

volatile int32_t encoder1_count = 0;
volatile int32_t encoder2_count = 0;
float target_v = 0.0f, target_w = 0.0f;
float integral1 = 0, integral2 = 0, last_err1 = 0, last_err2 = 0;
unsigned long last_time = 0;

// === Odometry State ===
float x = 0.0f, y = 0.0f, theta = 0.0f;

// === Feedforward Mapping - Forward ===
float pwmFromVelocity(float v_target) {
    float velocity[] = {
        0.026, 0.043, 0.060, 0.077, 0.092, 0.112, 0.130, 0.150, 0.170,
        0.188, 0.206, 0.226, 0.246, 0.265, 0.284, 0.303, 0.319, 0.338,
        0.356, 0.366, 0.371, 0.371
    };
    float pwm[] = {
        150, 155, 160, 165, 170, 175, 180, 185, 190,
        195, 200, 205, 210, 215, 220, 225, 230, 235,
        240, 245, 250, 255
    };
    int n = sizeof(velocity) / sizeof(float);
    if (v_target <= velocity[0]) return pwm[0];
    if (v_target >= velocity[n - 1]) return pwm[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (v_target >= velocity[i] && v_target <= velocity[i + 1]) {
            float t = (v_target - velocity[i]) / (velocity[i + 1] - velocity[i]);
            return pwm[i] + t * (pwm[i + 1] - pwm[i]);
        }
    }
    return 0;
}

// === Feedforward Mapping - Reverse ===
float pwmFromVelocityReverse(float v_target) {
    float velocity[] = {
        0.031, 0.049, 0.065, 0.082, 0.100, 0.117, 0.134, 0.153, 0.170,
        0.188, 0.205, 0.223, 0.240, 0.258, 0.276, 0.293, 0.311, 0.329,
        0.344, 0.353, 0.357, 0.357
    };
    float pwm[] = {
        150, 155, 160, 165, 170, 175, 180, 185, 190,
        195, 200, 205, 210, 215, 220, 225, 230, 235,
        240, 245, 250, 255
    };
    int n = sizeof(velocity) / sizeof(float);
    if (v_target <= velocity[0]) return pwm[0];
    if (v_target >= velocity[n - 1]) return pwm[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (v_target >= velocity[i] && v_target <= velocity[i + 1]) {
            float t = (v_target - velocity[i]) / (velocity[i + 1] - velocity[i]);
            return pwm[i] + t * (pwm[i + 1] - pwm[i]);
        }
    }
    return 0;
}

void IRAM_ATTR encoder1ISR() {
    bool A = digitalRead(ENCODER1_PIN_A);
    bool B = digitalRead(ENCODER1_PIN_B);
    encoder1_count += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
    bool A = digitalRead(ENCODER2_PIN_A);
    bool B = digitalRead(ENCODER2_PIN_B);
    encoder2_count += (A == B) ? 1 : -1;
}

int limitPWM(float pwm) {
    pwm = fabs(pwm);
    if (pwm > MAX_PWM) return MAX_PWM;
    if (pwm < MIN_PWM) return (pwm > 1e-2 ? MIN_PWM : 0);
    return (int)pwm;
}

void applyMotorPWM(int en_pin, int in1, int in2, int pwm_channel, int pwm, int dir) {
    digitalWrite(in1, dir > 0);
    digitalWrite(in2, dir < 0);
    ledcWrite(pwm_channel, pwm);
}

void handleSerialCommand(String input) {
    input.trim();
    if (input.startsWith("set ")) {
        input = input.substring(4);
        int space = input.indexOf(' ');
        if (space == -1) return;
        String key = input.substring(0, space);
        float val = input.substring(space + 1).toFloat();

        if (key == "v") {
            target_v = constrain(val, -MAX_VELOCITY, MAX_VELOCITY);
            Serial.printf("Linear velocity set to %.3f\n", target_v);
        } else if (key == "w") {
            target_w = val;
            Serial.printf("Angular velocity set to %.3f\n", target_w);
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(MOTOR1_IN1, OUTPUT); pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT); pinMode(MOTOR2_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR1_EN, PWM_CHANNEL1);
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR2_EN, PWM_CHANNEL2);

    pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);
    pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER2_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);

    last_time = millis();
    Serial.println("Use: set v <val>, set w <val>");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleSerialCommand(cmd);
    }

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.01f) return;
    last_time = now;

    int32_t c1 = encoder1_count;
    int32_t c2 = encoder2_count;
    encoder1_count = 0;
    encoder2_count = 0;

    float rps1 = (c1 / (float)TICKS_PER_REV) / dt;
    float rps2 = (c2 / (float)TICKS_PER_REV) / dt;
    float v1_actual = rps1 * 2 * PI * WHEEL_RADIUS;
    float v2_actual = rps2 * 2 * PI * WHEEL_RADIUS;

    float v1_target = target_v - (target_w * WHEEL_BASE / 2);
    float v2_target = target_v + (target_w * WHEEL_BASE / 2);

    float err1 = fabs(v1_target) - fabs(v1_actual);
    float err2 = fabs(v2_target) - fabs(v2_actual);
    integral1 += err1 * dt;
    integral2 += err2 * dt;
    float d1 = (err1 - last_err1) / dt;
    float d2 = (err2 - last_err2) / dt;
    float pid1 = Kp * err1 + Ki * integral1 + Kd * d1;
    float pid2 = Kp * err2 + Ki * integral2 + Kd * d2;
    last_err1 = err1;
    last_err2 = err2;

    int dir1 = (v1_target > 0) ? 1 : (v1_target < 0 ? -1 : 0);
    int dir2 = (v2_target > 0) ? 1 : (v2_target < 0 ? -1 : 0);

    float ff1 = (dir1 > 0) ? pwmFromVelocity(v1_target) : pwmFromVelocityReverse(-v1_target);
    float ff2 = (dir2 > 0) ? pwmFromVelocity(v2_target) : pwmFromVelocityReverse(-v2_target);
    int pwm1 = limitPWM(ff1 + pid1);
    int pwm2 = limitPWM(ff2 + pid2);

    applyMotorPWM(MOTOR1_EN, MOTOR1_IN1, MOTOR1_IN2, PWM_CHANNEL1, pwm1, dir1);
    applyMotorPWM(MOTOR2_EN, MOTOR2_IN1, MOTOR2_IN2, PWM_CHANNEL2, pwm2, dir2);

    // === Odometry update ===
    float v_avg = (v1_actual + v2_actual) / 2.0f;
    float w_avg = (v2_actual - v1_actual) / WHEEL_BASE;
    theta += w_avg * dt;
    x += v_avg * cos(theta) * dt;
    y += v_avg * sin(theta) * dt;

    Serial.printf("L: %.2f (%.2f) | R: %.2f (%.2f) | PWM: %d %d | Pose: x=%.2f y=%.2f th=%.2f\n",
                  v1_actual, v1_target, v2_actual, v2_target, pwm1, pwm2, x, y, theta);
    delay(100);
}
