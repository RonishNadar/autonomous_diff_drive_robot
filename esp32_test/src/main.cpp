#include <Arduino.h>

// === Motor and Encoder Pins ===
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define MOTOR_EN   15
#define MOTOR_IN1  16
#define MOTOR_IN2  17

// === Motor and Encoder Config ===
#define TICKS_PER_REV     1250
#define WHEEL_RADIUS      0.033f
#define PWM_CHANNEL       0
#define PWM_FREQ          20000
#define PWM_RESOLUTION    8
#define MIN_EFFECTIVE_PWM 70
#define MAX_PWM           255
#define MAX_VELOCITY      0.35f

// === PID parameters ===
// float Kp = 700.0f;
// float Ki = 10.0f;
// float Kd = 5.0f;
float Kp = 400.0f;
float Ki = 15.0f;
float Kd = 20.0f;


volatile int32_t encoder_count = 0;
float target_velocity = 0.0f;
float integral = 0.0f;
float last_error = 0.0f;
unsigned long last_time = 0;

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

// === Encoder ISR ===
void IRAM_ATTR encoderISR() {
    bool A = digitalRead(ENCODER_PIN_A);
    bool B = digitalRead(ENCODER_PIN_B);
    encoder_count += (A == B) ? 1 : -1;
}

// === PWM Limit Enforcement ===
int limitPWM(float pwm) {
    pwm = fabs(pwm);
    if (pwm > MAX_PWM) return MAX_PWM;
    if (pwm < MIN_EFFECTIVE_PWM) return (pwm > 1e-2 ? MIN_EFFECTIVE_PWM : 0);
    return (int)pwm;
}

// === Apply Motor Control ===
void applyMotorPWM(int pwm, int direction) {
    if (direction == 1) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
    } else if (direction == -1) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        pwm = 0;
    }
    ledcWrite(PWM_CHANNEL, pwm);
}

// === Setup ===
void setup() {
    Serial.begin(115200);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_EN, PWM_CHANNEL);

    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

    Serial.println("Enter velocity in m/s (±0.35):");
}

// === Main Loop ===
void loop() {
    static int32_t last_encoder = 0;
    static int last_direction = 0;

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        target_velocity = input.toFloat();
        if (target_velocity > MAX_VELOCITY) {
            target_velocity = MAX_VELOCITY;
            Serial.println("Clamped to max forward velocity: 0.35 m/s");
        } else if (target_velocity < -MAX_VELOCITY) {
            target_velocity = -MAX_VELOCITY;
            Serial.println("Clamped to max reverse velocity: -0.35 m/s");
        }
        Serial.printf("Target velocity set to: %.3f m/s\n", target_velocity);
    }

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.01f) return;

    int32_t current_encoder = encoder_count;
    int ticks = current_encoder - last_encoder;
    float ticks_per_sec = ticks / dt;
    float revs_per_sec = ticks_per_sec / TICKS_PER_REV;
    float angular_velocity = revs_per_sec * 2.0f * PI;
    float linear_velocity = angular_velocity * WHEEL_RADIUS;

    int direction = (target_velocity > 0) ? 1 :
                    (target_velocity < 0) ? -1 : 0;

    if (direction != last_direction) {
        integral = 0;
        last_error = 0;
        last_time = now;
        Serial.println("Direction changed — PID reset");
    }
    last_direction = direction;

    float ff_pwm = (target_velocity > 0)
        ? pwmFromVelocity(target_velocity)
        : pwmFromVelocityReverse(-target_velocity);

    float error = fabs(target_velocity) - fabs(linear_velocity);
    integral += error * dt;
    float derivative = (error - last_error) / dt;
    float pid_term = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;
    last_time = now;

    float total_pwm = ff_pwm + pid_term;  // PID assists only
    int pwm = limitPWM(total_pwm);
    applyMotorPWM(pwm, direction);

    Serial.printf("v: %.3f m/s | target: %.3f | PWM: %d | FF: %.1f | PID: %.1f\n",
                  linear_velocity, target_velocity, pwm, ff_pwm, pid_term);

    last_encoder = current_encoder;
    delay(100);
}
