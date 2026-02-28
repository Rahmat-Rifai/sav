#include <Arduino.h>

#define MOTOR_LEFT_PWM    5
#define MOTOR_LEFT_DIR1   6
#define MOTOR_LEFT_DIR2   7
#define MOTOR_RIGHT_PWM   9
#define MOTOR_RIGHT_DIR1  8
#define MOTOR_RIGHT_DIR2  10
#define ENCODER_LEFT_A    2
#define ENCODER_LEFT_B    3
#define ENCODER_RIGHT_A   18
#define ENCODER_RIGHT_B   19
#define ULTRASONIC_TRIG   12
#define ULTRASONIC_ECHO   13
#define LINE_SENSOR_1     A0
#define LINE_SENSOR_2     A1
#define LINE_SENSOR_3     A2
#define LINE_SENSOR_4     A3
#define LINE_SENSOR_5     A4
#define BUZZER_PIN        11
#define LED_BUILTIN_PIN   4
#define BUTTON_PIN        A5

class Leanbot {
private:
    volatile long leftEncoderCount;
    volatile long rightEncoderCount;
    static Leanbot* instance;

    void handleLeftEncoder() {
        int b = digitalRead(ENCODER_LEFT_B);
        if (digitalRead(ENCODER_LEFT_A) == b) {
            leftEncoderCount++;
        } else {
            leftEncoderCount--;
        }
    }

    void handleRightEncoder() {
        int b = digitalRead(ENCODER_RIGHT_B);
        if (digitalRead(ENCODER_RIGHT_A) == b) {
            rightEncoderCount++;
        } else {
            rightEncoderCount--;
        }
    }

public:
    Leanbot() : leftEncoderCount(0), rightEncoderCount(0) {
        instance = this;
    }

    void begin() {
        pinMode(MOTOR_LEFT_PWM, OUTPUT);
        pinMode(MOTOR_LEFT_DIR1, OUTPUT);
        pinMode(MOTOR_LEFT_DIR2, OUTPUT);
        pinMode(MOTOR_RIGHT_PWM, OUTPUT);
        pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
        pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

        pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
        pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
        pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
        pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);

        pinMode(ULTRASONIC_TRIG, OUTPUT);
        pinMode(ULTRASONIC_ECHO, INPUT);

        pinMode(LINE_SENSOR_1, INPUT);
        pinMode(LINE_SENSOR_2, INPUT);
        pinMode(LINE_SENSOR_3, INPUT);
        pinMode(LINE_SENSOR_4, INPUT);
        pinMode(LINE_SENSOR_5, INPUT);

        pinMode(BUZZER_PIN, OUTPUT);
        pinMode(LED_BUILTIN_PIN, OUTPUT);
        pinMode(BUTTON_PIN, INPUT_PULLUP);

        setMotorSpeeds(0, 0);
    }

    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        if (leftSpeed >= 0) {
            digitalWrite(MOTOR_LEFT_DIR1, HIGH);
            digitalWrite(MOTOR_LEFT_DIR2, LOW);
            analogWrite(MOTOR_LEFT_PWM, leftSpeed);
        } else {
            digitalWrite(MOTOR_LEFT_DIR1, LOW);
            digitalWrite(MOTOR_LEFT_DIR2, HIGH);
            analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
        }

        if (rightSpeed >= 0) {
            digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
            digitalWrite(MOTOR_RIGHT_DIR2, LOW);
            analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
        } else {
            digitalWrite(MOTOR_RIGHT_DIR1, LOW);
            digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
            analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
        }
    }

    float readUltrasonic() {
        digitalWrite(ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG, LOW);

        long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
        if (duration == 0) return -1.0;
        return duration * 0.034 / 2.0;
    }

    void readLineSensors(int* sensorValues) {
        sensorValues[0] = analogRead(LINE_SENSOR_1);
        sensorValues[1] = analogRead(LINE_SENSOR_2);
        sensorValues[2] = analogRead(LINE_SENSOR_3);
        sensorValues[3] = analogRead(LINE_SENSOR_4);
        sensorValues[4] = analogRead(LINE_SENSOR_5);
    }

    bool isButtonPressed() {
        return digitalRead(BUTTON_PIN) == LOW;
    }

    void beep(unsigned int frequency, unsigned long duration) {
        tone(BUZZER_PIN, frequency, duration);
    }

    void setLED(bool state) {
        digitalWrite(LED_BUILTIN_PIN, state ? HIGH : LOW);
    }

    long getLeftEncoderCount() {
        noInterrupts();
        long count = leftEncoderCount;
        interrupts();
        return count;
    }

    long getRightEncoderCount() {
        noInterrupts();
        long count = rightEncoderCount;
        interrupts();
        return count;
    }

    void resetEncoders() {
        noInterrupts();
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        interrupts();
    }

    static void leftEncoderISR() {
        if (instance) instance->handleLeftEncoder();
    }

    static void rightEncoderISR() {
        if (instance) instance->handleRightEncoder();
    }
};

Leanbot* Leanbot::instance = nullptr;
Leanbot robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
    robot.beep(1000, 500);
    robot.setLED(true);
    delay(500);
    robot.setLED(false);
}

void loop() {
    static bool running = false;

    if (robot.isButtonPressed()) {
        running = !running;
        robot.beep(2000, 200);
        delay(500);
    }

    if (running) {
    }
}
