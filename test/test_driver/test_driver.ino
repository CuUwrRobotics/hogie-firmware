#include <RH_ASK.h>
RH_ASK driver;

const int RX_CS_PIN = 9;

void setup() {
  Serial.begin(9600);

  if (!driver.init())
    Serial.println("radio init failed");

  pinMode(RX_CS_PIN, OUTPUT);
  digitalWrite(RX_CS_PIN, HIGH);
}

/**
 * transmits time over radio transmitter 
 */
void transmit(int8_t time) {
  driver.send((int8_t *)&time, sizeof(time));
  driver.waitPacketSent();
}

void loop() {
  String angle = Serial.readStringUntil('\n');

  if (angle.length() == 0) return;

  int8_t intangle = angle.toInt();

  transmit(intangle);
  Serial.println("TX: " + String(intangle));
  delay(100);
  transmit(intangle);
  Serial.flush();
}
