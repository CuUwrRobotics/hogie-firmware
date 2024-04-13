#include <RH_ASK.h>
// #include <Servo.h>  // Arduino's Servo library did not work, timer conflicts with RadioHead
#include <Adafruit_SoftServo.h>  // We used this servo instead

const int RX_CS_PIN = 9;

// Servo myservo;
Adafruit_SoftServo myservo;

RH_ASK driver;  // reciever CS=10 , reciever data = 11, transmitter data = 12
//pins
int WATER_PIN = A0;
int SERVO_PIN = 3;
int IR_PIN = 2;
int BEAM_BREAK_PIN = A3;
const uint64_t DRIVE_TIME = 1000;
uint64_t motor_Stop = 0;

const int WATER_THRESHOLD = 300;

//variables
bool isWet = false;
bool isBottom = true;
int waterVal;                        //Analog value of water
static byte servoDiveCommand = 0;    // angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // angle value that the rise method sends to the servo
static byte servoCoastCommand = 90;
uint64_t unix_epoch_offset = 0;  //
uint64_t start_time = 0;
uint64_t current_time = start_time + current_time;
unsigned long ThenTime;  // comparing time
unsigned long NowTime;   // comparing time
//digitalRead(VOLTAGE_PIN,HIGH);


void setup() {

  Serial.begin(9600);
  pinMode(WATER_PIN, INPUT);
  pinMode(SERVO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  Serial.println("Setup...");

  if (!driver.init())
    Serial.println("radio init failed");

  pinMode(RX_CS_PIN, OUTPUT);
  digitalWrite(RX_CS_PIN, HIGH);
}


/**
 * recieve data from radio reciever , returns -1 if no data found
 */
int recieve() {
  uint8_t buf[sizeof(int8_t)] = { 0 };
  uint8_t buflen = sizeof(int8_t);
  // Non-blocking check
  if (!driver.recv(buf, &buflen)) {
      return -1;
  }
  Serial.print("::: ");
  Serial.print(buf[0]);
  Serial.print(" ");
  Serial.println(buf[1]);
  return buf[0];
}

void loop() {
  Serial.println("Awaiting servo command");
  int8_t command = 0;
  while ((command = recieve()) == -1) {
    myservo.refresh();
  }

  Serial.println("Got " + String(command));
  myservo.write((uint8_t)command);
  myservo.refresh();
}
