// Here lies Mr. Cady's Sanity 
/*
 * Hoagie Firmware V2
 * Jan 20 2024
 * Replaces ir and water sensor with pressure sensor
 */
/** 
 * Main functionality of this code:
 * Ingests pressure sensor data every 5 seconds
 * Stores it along with a timestamp
 * Controls a servo motor driving a syringe which takes in and expells water
 * Sends a message containing this data to topside computer via RF24LO1+ communications
*/
#include <SPI.h>
#include <RF24.h>
#include <time.h>
#include <RH_ASK.h>
#include <Arduino.h>
#include <nRF24L01.h>
#include <Adafruit_SoftServo.h>

/* Pin definitions (digital) */
const string TEAM_NAME = "CUUWR";
const int PIN_SERVO = 3;
const RF24 SENDER(7, 8);           // Transmitter: CE pin, CSN pin
const RF24 RECEIVER(9, 10);        // Receiver: CE pin, CSN pin
const byte S_ADDRESS[6] = "00001"; // Address of the transmitter
const byte R_ADDRESS[6] = "00002"; // Address of the receive
/* Pin definitions (analog) */
const int PIN_PRESSURE_SENSOR = A0;
const int PIN_SERVO_LIMIT = A3;

/* Pressure Sensor */
const float BL_PRESSURE = 101325.0;     // Baseline Barometric Pressure
const float PRESSURE_THRESHOLD = 100.0; // Pressure Threshold for Dive/Rise

/* Pre-measured timings and thresholds */
const uint64_t TIME_DIVE = 10000;     // Time to pull the plunger back
const uint64_t TIME_RISE = TIME_DIVE; // Time to push the plunger forwards
const float POOL_DEPTH = 10;          // feet

/* Servo Commands */
const uint8_t SERVO_DIVE = 0;
const uint8_t SERVO_RISE = 180;
const uint9_t SERVO_COAST = 90;

/** Refresh Servo function
 * refreshServo()
 */

struct packet
{
  u_int32_t time;
  float pressure;
};

void refreshServo()
{
  static unsigned long next_servo_refresh = 0;

  if (next_servo_refresh < millis())
  {
    // Call 'refresh' only every 20ms (~50Hz)
    next_servo_refresh = millis() + 19;
    noInterrupts(); // The delay done in 'myservo.refresh()' is offset by around 30% by interrupts
    myservo.refresh();
    interrupts();
  }
}

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_DIST_SENSOR, INPUT);
  pinMode(PIN_SERVO, 9E_SENSOR, INPUT);
  pinMode(PIN_SERVO_LIMIT, INPUT_PULLUP);

  myservo.attach(PIN_SERVO);

  // Additional setup for pressure sensor calibration if needed
  // For example, you may want to take a baseline pressure reading when the pool is empty
  // and use that as a reference for depth calculations.

  Serial.println("Setting up Hoagie Firmware V2...");
}

void transmit(string message)
{
  // Transmit Code
  SENDER.begin();
  SENDER.openWritingPipe(S_ADDRESS);
  SENDER.setPALevel(RF24_PA_MIN);

  const char text[] = message;
  SENDER.write(&text, sizeof(text));
  delay(1000);
  sender.closeWritingPipe();
}

void receive()
{
  // Receive Code
  RECEIVER.begin();
  RECEIVER.openReadingPipe(0, R_ADDRESS);
  RECEIVER.setPALevel(RF24_PA_MIN);
  RECEIVER.startListening();
  if (RECEIVER.available())
  {
    char text[32] = "";
    RECEIVER.read(&text, sizeof(text));
    return text;
  }
  else
  {
    return 0;
  }
}
/**
 * getDepth() function
 * This function returns the depth which is actually used rather sparingly,
 * you, dear reader, might expect that the depth would be more necessary but the
 * robot just communicates the pressure.
 *
 * Depth is converted using the standard function pressure / (density * gravity).
 * Pressure is read directly from the sensor via analog io pin.
 */
float getDepth()
{
  // Pressure Sensor to Depth Code
  float density = 1.025; // kg/m^3
  float gravity = 9.81;  // m/s^2
  float pressure = analogRead(PIN_PRESSURE_SENSOR);
  float depth = pressure / (density * gravity);
  // meters to feet
  depth = depth * 3.28084;
  return depth;
}

bool diveCompleted()
{
  // Dive Completed Code
  depth = getDepth();
  if (depth >= POOL_DEPTH)
  {
    if (detectionStartTime == 0)
      detectionStartTime = millis();
    if (millis() - detectionStartTime >= TIME_DIVE)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool riseCompleted()
{
  // Rise Completed Code
  depth = getDepth();
  if (depth <= 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
int64_t packetToBin(packet data[], TEAM_NAME)
{
  int64_t bin = 0;
  bin |= (TEAM_NAME << 56);
  bin |= (data[0].time << 48);
  bin |= (data[0].pressure << 32);
  bin |= (data[1].time << 16);
  bin |= (data[1].pressure);
  return bin;
}

void loop()
{
  refreshServo();
  static bool endstop_switch;
  static uint64_t message;
  static uint8_t state = 0;
  packet data[] = [128];
  // start timer
  static uint64_t timer = millis();

  switch (state)
  {
  case 0: /* Callibration State */
    // Serial.println("state: init");

    message_temp = recieve();
    Serial.println(String(int(message_temp)));
    if (message_temp == 0xDEADBEEF)
    {
      state = 1;
    }
    break;
  case 1: /* Wait State */
    Serial.println("state: wait for time");
    transmit(trueTime());
    message_temp = recieve();
    if (message_temp != -1)
    {
      if (message_temp == 0xFA110000)
      {
        Serial.println("Got drop");
        state = 2;
        break;
      }
      else
      {
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
    while (!diveCompleted())
    {
      // create a new packet every 5 seconds and add to data array
      if (millis() - timer >= 5000)
      {
        data.append(packet(trueTime(), analogRead(PIN_PRESSURE_SENSOR));
        timer = millis();
      }
    }
    if (diveCompleted())
    {
      state = 3;
    }
    break;

  case 3: // rising state
    Serial.println("state: rise");
    while (!riseCompleted())
    {
      // create a new packet every 5 seconds and add to data array
      if (millis() - timer >= 5000)
      {
        data.append(packet(trueTime(), analogRead(PIN_PRESSURE_SENSOR));
        timer = millis();
      }
    }
    if (riseCompleted())
    {
      state = 1;
      system.out.println("Sending...");
      try
      {
        message = packetToBin(data, TEAM_NAME);
        transmit(message);
      }
      catch (exception e)
      {
        system.out.println("Error sending data");
      }
    }
    break;

  default: // error state
    Serial.println("Reached default in state machine (very bad. you're lowkey fucked) :-(");
    while (1)
    {
    }
    break;
  }
}
}
