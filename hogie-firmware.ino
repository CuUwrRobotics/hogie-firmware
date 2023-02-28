
#include <RH_ASK.h>
// #include <Servo.h>  // Arduino's Servo library did not work, timer conflicts with RadioHead
#include <Adafruit_SoftServo.h> // We used this servo instead
#include <SPI.h>



// Servo myservo;
Adafruit_SoftServo myservo;

RH_ASK driver; // reciever CS=10 , reciever data = 11, transmitter data = 12 
//pins
int WATER_PIN = A0;
int SERVO_PIN = 9;
int IR_PIN = 2;
const long int DRIVE_TIME = 1000;
long int motor_Stop = 0;

//variables
bool isWet = false;
bool isBottom = true;
int waterVal;                        //Analog value of water
static byte servoDiveCommand = 0;    // angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // angle value that the rise method sends to the servo
static byte servoCoastCommand = 90;
long int time_elapsed = millis();  //
long int start_time = 0;
long int current_time = start_time + current_time;


void setup() {
  Serial.begin(9600);
  pinMode(WATER_PIN, INPUT);
  pinMode(SERVO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  if (!driver.init())
    Serial.println("radio init failed");
}

void dive() {
  if (motor_Stop == 0) {
    // Serial.println("diving");  // print status change to the serial port
    Serial.print("DIVE");
    //myservo.attach(SERVO_PIN);                    // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
    myservo.write(servoDiveCommand);  // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo)
    motor_Stop = millis() + DRIVE_TIME;
  } else if (motor_Stop < millis()) {
    myservo.write(servoCoastCommand);
    Serial.print("CDIV");
    //Serial.println("coasting (dive)");  // print status change to the serial port
  }
}

void rise() {
  if (motor_Stop == 0) {
    // Serial.println("rising");  // print status change to the serial port
    Serial.print("RISE");
    //myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
    myservo.write(servoRiseCommand);  // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
    motor_Stop = millis() + DRIVE_TIME;
  } else if (motor_Stop < millis()) {
    //Serial.println("coasting (rise)");  // print status change to the serial port
    myservo.write(servoCoastCommand);
    Serial.print("CRIS");
  }

  //delay(delayTime);
  //myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  // Serial.println("coasting (rise)");  // print status change to the serial port
}

void checkWet(bool &isWet) {
  waterVal = analogRead(WATER_PIN);

  if (waterVal <= 100) {
    //Serial.println("Float is dry");
    isWet = false;
  } else if (waterVal > 100) {
    // Serial.println("Float is wet");
    isWet = true;
  }
}

void checkBottom(bool &isBottom) {
  if (digitalRead(IR_PIN) == LOW) {
    //Serial.println("Float is at the Bottom");
    isBottom = true;
  }
  if (digitalRead(IR_PIN) == HIGH) {
    // Serial.println("Float is not at the Bottom");
    isBottom = false;
  }
}

/**
 * recieve data from radio reciever , returns -1 if no data found
 */
long int recieve() {
  uint8_t buf[sizeof(long int)];
  uint8_t buflen = sizeof(long int);
  // Non-blocking check
  if (driver.recv(buf, &buflen)) {
    return *((long *)buf);
  }
  return -1;
}

/**
 * transmits time over radio transmitter 
 */
void transmit(long time) {
  driver.send((uint8_t *)time, sizeof(time));
  driver.waitPacketSent();
}

int state = 0;

void loop() {
  checkWet(isWet);
  checkBottom(isBottom);
  static bool endstop_switch;
  Serial.println();
  Serial.print(state);
  Serial.print(",");
  Serial.print(isBottom);
  Serial.print(",");
  Serial.print(isWet);
  Serial.print(",");
  Serial.print(motor_Stop);
  Serial.print(",");


  switch (state) {
    case 0: /* Callibration State */

      int message = recieve();

      while (message != 0xDEADBEEF) {
        message = recieve();
      }
      state = 1;

      break;
    case 1: /* Wait State */
      unsigned long timbuktu = millis() + 1000;
      unsigned long timid;
      transmit(timid);
      while (millis() < timbuktu) {
        timid = recieve();
        if (timid == 010010) {
          state = 2;
          break;
        }
      }
      transmit(current_time);

      break;
    case 2: /* falling state */
      while (!isBottom) {
        dive();
      }
      state = 3;
      break;
    case 3:
      while (isWet) {
        rise();
      }
      state = 1;
      break;
  }
}
