#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include "MS5837.h"
#include "ArduPID.h"
#include <Servo.h>
#include <Vector.h>
#include "printf.h"

// Packet struct
struct packet {
    packet() {companyName = 69420; seconds = 0;
        depth = 0.0; pressure = 0.0;}
    packet(uint64_t s, float d, float p) {
        companyName = 69420; seconds = s; depth = d; pressure = p;}
    uint32_t companyName;
    uint64_t seconds;
    float depth, pressure;
};

/* Pin definitions */
const int PIN_SERVO_LEFT = 5;
const int PIN_PRESSURE_SENSOR = A0;
#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };
int dcounter = 0;
int rcounter = 0;

/* Radio parameters */
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
//bool role = false;  // true = TX role, false = RX role

/* Constants */
const float PRESSURE_THRESHOLD = 100.0;
const uint64_t TIME_DIVE = 45;
uint64_t Time_of_Dive = 0;
const float POOL_DEPTH = 10.0;
const float TARGET_DEPTH = 2.5;
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
packet payload, temp;
Vector<packet> diveData;

// Initialization
void setup() {
    //transmit_int(75);
    Serial.begin(9600);
    Wire.begin();
    myServo.attach(PIN_SERVO_LEFT);
    pressure_sensor.setModel(MS5837::MS5837_30BA);
    pressure_sensor.setFluidDensity(997);
    PID.begin(&PID_input, &PID_output, &setpoint, kP, kI, kD);
    PID.setOutputLimits(-90, 90);
    if (!radio.begin()) {Serial.println(F("radio hardware is not responding!!"));}
    radio.setPALevel(RF24_PA_MAX);
    radio.openWritingPipe(address[radioNumber]);
    radio.openReadingPipe(1, address[!radioNumber]);
    radio.startListening();
    Serial.println("Hoagie Firmware V2 Initialized");
    transmit_int(76);
}

// Calculates depth from pressure sensor value
float getDepth() {
    float density = 1.025;
    float gravity = 9.81;
    float pressure = pressure_sensor.pressure();
    return (pressure / (density * gravity)) * 3.28084;
}

// Returns true if target diving depth has been reached
bool diveCompleted() {
    depth = getDepth();
    return depth >= TARGET_DEPTH;
}

// Returns true if target rising depth reaches or exceedes 0 (water surface)
bool riseCompleted() {
    return getDepth() <= 0;
}

// Sends packet from Hoagie to Topside
void transmit_packet(Vector<packet> d) {
    int s = d.size();
    radio.stopListening();
    for (int i; i < d.size(); i++) {
        radio.write(&d[i], sizeof(d[i]));
    }
    radio.startListening();
}

// Sends int from Hoagie to Topside
void transmit_int(int s) {
    radio.stopListening();
    uint8_t ss = sizeof(s);
    Serial.println(s);
    radio.write(&s, 4);
    radio.startListening();
}

// Recieves int from Topside
int receive_int() {
    uint8_t pipe;
    int payload;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(&payload, 4);
        return payload;
    }
    else return -1;
}

// Main loop
void loop() {
    static uint8_t state = 1;
    static uint64_t timer = millis();
    switch (state) {
        case 1: // Wait State
            Serial.println("Waiting for signal");
            transmit_int(75);
            if (receive_int() == 76) {
                state = 2;
            //    PID.start();
            }
            //transmit_int(77); // Timer Start
            //delay(150000); // Testing Timer, in ms
            //transmit_int(78); // Timer End
            //state = 2;
            break;
        case 2: // Diving
            Serial.println("Diving");
            transmit_int(81);
            if (millis() - timer >= 5000) {
                //pressure_data.push_back(getDepth());
                temp.seconds = timer;
                temp.depth = getDepth();
                temp.pressure = pressure_sensor.pressure();
                diveData.push_back(temp);
                timer = millis();
                dcounter++;
            }
            //PID_input = getDepth();
            //PID.compute();
            myServo.write(180);
            delay(15);
            if (diveCompleted() || dcounter > 3) {
                state = 4;
                //PID.reset();
                setpoint = 0;
                Time_of_Dive = timer / 100;
            }
            break;
        case 3: // Rising
            transmit_int(82);
            Serial.println("Rising");
            if (millis() - timer >= 5000) {
                //pressure_data.push_back(getDepth());
                temp.seconds = timer;
                temp.depth = getDepth();
                temp.pressure = pressure_sensor.pressure();
                diveData.push_back(temp);
                timer = millis();
                rcounter++;
            }
            //PID_input = getDepth();
            //PID.compute();
            myServo.write(0);
            delay(15);
            if (riseCompleted() || rcounter > 3) {
                state = 1;
                //PID.stop();
                //transmit("Data Sent");
                transmit_packet(diveData);
                transmit_int(91);
                diveData.clear();
            }
            break;
        case 4://waiting state
            //pressure_data.append(packet(trueTime(), analogRead(PIN_PRESSURE_SENSOR)));
            if ((TIME_DIVE - ((millis() / 100) - Time_of_Dive)) < 0) state = 3;
            break;
        default:
            Serial.println("Error: Invalid state");
            Serial.println(state);
            transmit_int(99);
            while (1);
    }
}

}
