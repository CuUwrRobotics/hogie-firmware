
#include <RH_ASK.h>
#include <Adafruit_SoftServo.h>  // We used this servo instead
#include <SPI.h>


Adafruit_SoftServo myservo;

// Pin definitions
RH_ASK driver;  // reciever data = 11, transmitter data = 12
const int PIN_DIST_SENSOR = 2;
const int PIN_SERVO = 3;
const int PIN_RX_CS = 9;
const int PIN_WATER_SENSOR = A0;
const int PIN_SERVO_LIMIT = A3;

// Pre-measured timings and thresholds
const uint64_t TIME_DIVE = 10000;  // Time for the motor to pull the plunger back
const uint64_t TIME_RISE = TIME_DIVE;  // Time for the motor to push the plunger forwards

const int WATER_SENSOR_THRESHOLD = 300;

const uint8_t SERVO_DIVE_COMMAND = 0;    // Angle value that the servo takes as CW
const uint8_t SERVO_RISE_COMMAND = 179;  // Angle value that the servo takes as CCW
const uint8_t SERVO_COAST_COMMAND = 90;  // Angle value that the servo takes as steady

// Stores the offset from our epoch to the given unix epoch
uint64_t unix_epoch_offset = 0;

void refreshServo() {
  static unsigned long next_servo_refresh = 0;

  if (next_servo_refresh < millis()) {
    // Call 'refresh' only every 20ms (~50Hz)
    next_servo_refresh = millis() + 19;
    noInterrupts();  // The delay done in 'myservo.refresh()' is offset by around 30% by interrupts
    myservo.refresh();
    interrupts();
  }
}

void setup() {

  Serial.begin(9600);
  Serial.println("Setting up...");

  pinMode(PIN_WATER_SENSOR, INPUT);
  pinMode(PIN_SERVO, INPUT);
  pinMode(PIN_DIST_SENSOR, INPUT);
  pinMode(PIN_SERVO_LIMIT, INPUT_PULLUP);
  pinMode(PIN_RX_CS, OUTPUT);
  digitalWrite(PIN_RX_CS, HIGH);

  myservo.attach(PIN_SERVO);

  if (!driver.init())
    Serial.println("radio init failed");

  if (digitalRead(PIN_SERVO_LIMIT) == HIGH) {
    Serial.println("Waiting to un-break beam");
    myservo.write(SERVO_DIVE_COMMAND);
    while (digitalRead(PIN_SERVO_LIMIT) == HIGH) {
      refreshServo();
    }
  }
  Serial.println("Waiting to break beam");
  myservo.write(SERVO_RISE_COMMAND);
  while (digitalRead(PIN_SERVO_LIMIT) == LOW) {
    refreshServo();
  }
  Serial.println("Beam has been broken");
  myservo.write(SERVO_COAST_COMMAND);
  refreshServo();
}

bool bottomedOut() {
  return digitalRead(PIN_DIST_SENSOR) == LOW;
}

bool waterSensorWet() {
  return analogRead(PIN_WATER_SENSOR) > WATER_SENSOR_THRESHOLD;
}

bool diveCompleted() {
  static uint32_t motor_stop_time = 0;
  static bool coasting = false;

  if (!coasting) {
    if (motor_stop_time == 0) {
      // Just started diving; set up vars
      motor_stop_time = millis() + TIME_DIVE;
    }

    if (motor_stop_time > millis()) {
      // Not at motor_stop_time yet, keep moving the servo
      myservo.write(SERVO_DIVE_COMMAND);
    } else {
      // Enter coast mode
      myservo.write(SERVO_COAST_COMMAND);
      motor_stop_time = 0;  // Allow the time to reset next time we dive
      coasting = true;
    }
  } else {
    // Dive completed; coast until we reach the bottom
    myservo.write(SERVO_COAST_COMMAND);
    if (bottomedOut()) {
      coasting = false;  // Start the next dive by diving (not coasting)
      return true;
    }
  }
  return false;
}

bool riseCompleted() {
  static uint32_t motor_stop_time = 0;
  static bool coasting = false;

  if (!coasting) {
    if (motor_stop_time == 0) {
      // Just started rising; set up vars
      motor_stop_time = millis() + TIME_RISE;
    }

    if (motor_stop_time > millis()) {
      // Not at motor_stop_time yet, keep moving the servo
      myservo.write(SERVO_RISE_COMMAND);
    } else {
      // Enter coast mode
      myservo.write(SERVO_COAST_COMMAND);
      motor_stop_time = 0;  // Allow the time to reset next time we rise
      coasting = true;
    }
  } else {
    // Rise completed; coast until we reach the bottom
    myservo.write(SERVO_COAST_COMMAND);
    if (!waterSensorWet()) {
      coasting = false;  // Start the next rise by rising (not coasting)
      return true;
    }
  }
  return false;
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
  digitalWrite(PIN_RX_CS, HIGH);
  while (!driver.recv(buf, &buflen)) {
    // refreshServo();
    if (millis() > timeout) {
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
  digitalWrite(PIN_RX_CS, LOW);
  Serial.print("TX: 0x");
  printull(time);
  Serial.println();
  driver.send((uint8_t *)&time, sizeof(time));
  driver.waitPacketSent();
  digitalWrite(PIN_RX_CS, HIGH);
}

uint64_t trueTime() {
  return unix_epoch_offset + (millis()) / 1000;
}

int state = 0;

void loop() {
  refreshServo();

  static bool endstop_switch;
  static uint64_t message_temp;

  Serial.print("State #" + String(state));
  Serial.print(": b" + String((byte)bottomedOut()));
  Serial.print(" w" + String((byte)waterSensorWet()));
  
  Serial.println();


  switch (state) {
    case 0: /* Callibration State */
      // Serial.println("state: init");

      message_temp = recieve();
      Serial.println(String(int(message_temp)));
      if (message_temp == 0xDEADBEEF) {
        state = 1;
      }
      break;
    case 1: /* Wait State */
      Serial.println("state: wait for time");
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
      Serial.println("state: dive");
      if (diveCompleted()) {
        state = 3;
      }
      break;
    case 3:
      Serial.println("state: rise");
      if (riseCompleted()) {
        state = 1;
      }
      break;
    default:
      Serial.println("Reached default in state machine (very bad) :-(");
      while (1) {}
      break;
  }
}
