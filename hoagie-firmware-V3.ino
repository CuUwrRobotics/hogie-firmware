#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include "MS5837.h"
#include "ArduPID.h"
#include <Servo.h>
#include <Vector.h>

/* Pin definitions */
const int PIN_SERVO_LEFT = 3;
const int PIN_PRESSURE_SENSOR = A0;
const RF24 SENDER(7, 8);
const RF24 RECEIVER(9, 10);
const byte S_ADDRESS[6] = "00001";
const byte R_ADDRESS[6] = "00002";

/* Constants */
const float PRESSURE_THRESHOLD = 100.0;
const uint64_t TIME_DIVE = 10000;
const float POOL_DEPTH = 10.0;
const uint8_t SERVO_DIVE = 0;
const uint8_t SERVO_RISE = 180;
const uint8_t SERVO_COAST = 90;

/* Global Variables */
MS5837 pressure_sensor;
ArduPID PID;
Servo myServo;
double PID_input, PID_output, setpoint = 2.5;
const double kP = 0, kI = 0, kD = 0;
uint64_t detectionStartTime = 0;
float depth = 0.0;
Vector<uint32_t> pressure_data;

struct packet {
    uint32_t time;
    float pressure;
};

void setup() {
    Serial.begin(9600);
    Wire.begin();
    myServo.attach(PIN_SERVO_LEFT);
    pressure_sensor.setModel(MS5837::MS5837_30BA);
    pressure_sensor.setFluidDensity(997);
    PID.begin(&PID_input, &PID_output, &setpoint, kP, kI, kD);
    PID.setOutputLimits(-90, 90);
    Serial.println("Hoagie Firmware V2 Initialized");
}

float getDepth() {
    float density = 1.025;
    float gravity = 9.81;
    float pressure = pressure_sensor.pressure();
    return (pressure / (density * gravity)) * 3.28084;
}

bool diveCompleted() {
    depth = getDepth();
    if (depth >= POOL_DEPTH) {
        if (detectionStartTime == 0) detectionStartTime = millis();
        return (millis() - detectionStartTime >= TIME_DIVE);
    }
    return false;
}

bool riseCompleted() {
    return getDepth() <= 0;
}

void transmit(const String &message) {
    SENDER.begin();
    SENDER.openWritingPipe(S_ADDRESS);
    SENDER.setPALevel(RF24_PA_MIN);
    SENDER.write(message.c_str(), message.length());
}

String receive() {
    RECEIVER.begin();
    RECEIVER.openReadingPipe(0, R_ADDRESS);
    RECEIVER.setPALevel(RF24_PA_MIN);
    RECEIVER.startListening();
    if (RECEIVER.available()) {
        char text[32] = "";
        RECEIVER.read(&text, sizeof(text));
        return String(text);
    }
    return "";
}

void loop() {
    static uint8_t state = 1;
    static uint64_t timer = millis();
    switch (state) {
        case 1: // Wait State
            Serial.println("Waiting for signal");
            transmit("Requesting time");
            if (receive() == "START") {
                state = 2;
                PID.start();
            }
            break;
        case 2: // Diving
            Serial.println("Diving");
            if (millis() - timer >= 5000) {
                pressure_data.push_back(getDepth());
                timer = millis();
            }
            PID_input = getDepth();
            PID.compute();
            myServo.write(PID_output + 90);
            if (diveCompleted()) {
                state = 3;
                PID.reset();
                setpoint = 0;
            }
            break;
        case 3: // Rising
            Serial.println("Rising");
            if (millis() - timer >= 5000) {
                pressure_data.push_back(getDepth());
                timer = millis();
            }
            PID_input = getDepth();
            PID.compute();
            myServo.write(PID_output + 90);
            if (riseCompleted()) {
                state = 1;
                PID.stop();
                transmit("Data Sent");
            }
            break;
        default:
            Serial.println("Error: Invalid state");
            while (1);
    }
}
