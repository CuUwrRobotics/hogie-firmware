
#include <RH_ASK.h>
// #include <Servo.h>  // Arduino's Servo library did not work, timer conflicts with RadioHead
#include <Adafruit_SoftServo.h>  // We used this servo instead
#include <SPI.h>
//#include <TimeLib.h>
const int RX_CS_PIN = 9;

// Servo myservo;
Adafruit_SoftServo myservo;

RH_ASK driver;  // reciever CS=10 , reciever data = 11, transmitter data = 12
//pins
int WATER_PIN = A0;
int SERVO_PIN = 3;
int IR_PIN = 2;
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
unsigned long ThenTime; // comparing time 
unsigned long NowTime; // comparing time 
bool voltage; // Don't know what pin reading from yet 


void setup() {
  Serial.begin(9600);
  pinMode(WATER_PIN, INPUT);
  pinMode(SERVO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  myservo.attach(SERVO_PIN);

  if (!driver.init())
    Serial.println("radio init failed");

  pinMode(RX_CS_PIN, OUTPUT);
  digitalWrite(RX_CS_PIN, HIGH);
}

void dive() {
  myservo.write(servoDiveCommand);
  Serial.println("Servo is divin");
  transmit(trueTime());
  
/*
  Serial.println("line 54");
  if (!voltage ) {motor_Stop=1;}
  if (motor_Stop == 0) {
    Serial.println("line 57");
  ThenTime&=millis();
  NowTime=millis();
       
  while (((NowTime-5000) < ThenTime) && (voltage =1)) //arbitrary time spent in loop to define amount of water suck 
  {
    Serial.println("diving; motor_Stop=" + String((uint32_t)motor_Stop) + "; servoCoastCommand=" + String(servoCoastCommand) + "; waterVal=" + String(analogRead(WATER_PIN)) + "; digitalRead(IR_PIN)=" + String(digitalRead(IR_PIN)));  // print status change to the serial port
    //myservo.attach(SERVO_PIN); // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
    myservo.write(servoDiveCommand);  // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo)
    motor_Stop = millis() + DRIVE_TIME;
  }
  } else if (motor_Stop < millis()) {
    myservo.write(servoCoastCommand);
    Serial.println("diving (coast); motor_Stop=" + String((uint32_t)motor_Stop) + "; servoCoastCommand=" + String(servoCoastCommand) + "; waterVal=" + String(analogRead(WATER_PIN)) + "; digitalRead(IR_PIN)=" + String(digitalRead(IR_PIN)));
  }
*/
}

void rise() {
  if (motor_Stop == 0) {
    ThenTime= millis();
    NowTime= millis();
  millis();
  
  while ((NowTime-6000) < ThenTime) //arbitrary time spent in loop to define amount of water pushed out 
  {
    Serial.println("rising; motor_Stop=" + String((uint32_t)motor_Stop) + "; servoCoastCommand=" + String(servoCoastCommand) + "; waterVal=" + String(analogRead(WATER_PIN)) + "; digitalRead(IR_PIN)=" + String(digitalRead(IR_PIN)));
    //myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
    myservo.write(servoRiseCommand);  // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
    motor_Stop = millis() + DRIVE_TIME;
  }
  } else if (motor_Stop < millis()) {
    myservo.write(servoCoastCommand);
    Serial.println("rising (coast); motor_Stop=" + String((uint32_t)motor_Stop) + "; servoCoastCommand=" + String(servoCoastCommand) + "; waterVal=" + String(analogRead(WATER_PIN)) + "; digitalRead(IR_PIN)=" + String(digitalRead(IR_PIN)));
  }
  }
  //delay(delayTime);
  //myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  // Serial.println("coasting (rise)");  // print status change to the serial port


void checkWet(bool &isWet) {
  waterVal = analogRead(WATER_PIN);

  if (waterVal <= WATER_THRESHOLD) {
    //Serial.println("Float is dry");
    isWet = false;
  } else if (waterVal > WATER_THRESHOLD) {
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


void printull(uint64_t num) {
  for (int i = 0; i < sizeof(num); i++) {
    if ((uint8_t)((num >> (sizeof(num) - i - 1) * 8) & 0xFF) < 0x1F) {
      Serial.print(0);
    }
    Serial.print((uint8_t)((num >> (sizeof(num) - i - 1) * 8) & 0xFF), HEX);
  }
}

/**
 * recieve data from radio reciever , returns -1 if no data found
 */
uint64_t recieve() {
  uint8_t buf[sizeof(uint64_t)] = { 0 };
  uint8_t buflen = sizeof(uint64_t);
  long timeout = millis() + 500;
  // Non-blocking check
  digitalWrite(RX_CS_PIN, HIGH);
  while (!driver.recv(buf, &buflen)) {
    if(millis() > timeout) {
      return -1;
    }
  }
  Serial.print("RX: 0x");
  printull((uint64_t)(*((uint64_t *)buf)));
  Serial.println();
  return *((uint64_t *)buf);
}

/**
 * transmits time over radio transmitter 
 */
void transmit(uint64_t time) {
  digitalWrite(RX_CS_PIN, LOW);
  Serial.print("TX: 0x");
  printull(time);
  Serial.println();
  driver.send((uint8_t *)&time, sizeof(time));
  driver.waitPacketSent();
  digitalWrite(RX_CS_PIN, HIGH);
}

uint64_t trueTime() {
  return unix_epoch_offset + (millis()) / 1000;
}

int state = 0;

void loop() {
  checkWet(isWet);
  checkBottom(isBottom);
  static bool endstop_switch;
  Serial.print(state);
  Serial.print(",");
  Serial.print(isBottom);
  Serial.print(",");
  Serial.print(isWet);
  Serial.print(",");
  Serial.print((uint32_t)motor_Stop);
  Serial.println();

  uint64_t message_temp;

  switch (state) {
    case 0: /* Callibration State */
      Serial.println("state 0");
      
      
      
      message_temp = recieve();

      while (message_temp != 0xDEADBEEF) {
        message_temp = recieve();
        delay(10);
        
      }
      state = 1;
      break;
    case 1: /* Wait State */
      Serial.println("state 1");
      transmit(trueTime());
      message_temp = recieve();
      if (message_temp != -1) {
        if (message_temp == 0xFA110000) {
          Serial.println("Got drop");
          state = 2;
          break;
        } else {
          unix_epoch_offset = message_temp - (millis() / 1000);
          Serial.print("Calibrated offset: ");
          printull(unix_epoch_offset);
          Serial.print("; Calibrated offset gives: ");
          printull(trueTime());
          Serial.println();
        }
      }
      break;
    case 2: /* falling state */
      Serial.println("state 2");
      if (!isBottom) {
        dive();
      } else {
        state = 3;
      }
      break;
    case 3:
      Serial.println("state 3");
      if (isWet) {
        rise();
      } else {
        state = 1;
      }
      break;
    default:
      Serial.println("Reached default in state machine (very bad) :-(");
      while (1) {}
      break;
  }
}
