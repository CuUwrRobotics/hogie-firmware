#include <RH_ASK.h>
RH_ASK driver;

const int RX_CS_PIN = 9;

void setup() {
  Serial.begin(9600);

  if (!driver.init())
    Serial.println("radio init failed");

  pinMode(3, INPUT_PULLUP);

  pinMode(RX_CS_PIN, OUTPUT);
  digitalWrite(RX_CS_PIN, HIGH);

  Serial.println("Ready");
}

void printull(uint64_t num) {
  for (int i = 0; i < sizeof(num); i++) {
    if ((uint8_t)((num >> (sizeof(num) - i - 1) * 8) & 0xFF) < 0x1F) {
      Serial.print(0);
    }
    Serial.print((uint8_t)((num >> (sizeof(num) - i - 1) * 8) & 0xFF), HEX);
  }
}

uint64_t getTime() {
  Serial.println("time");
  int rx = '\0', idx = 0;
  char buf[20] = { '\0' };
  while (rx != '\n' || idx >= 20) {
    rx = Serial.read();
    if (rx != -1) {
      buf[idx++] = rx;
    }
  }
  return (uint64_t)String(buf).toDouble();
}

uint64_t recieve() {
  uint8_t buf[sizeof(uint64_t)];
  uint8_t buflen = sizeof(uint64_t);
  // Non-blocking check
  if (driver.recv(buf, &buflen)) {
    Serial.print("RX: 0x");
    printull((uint64_t)(*((uint64_t *)buf)));
    Serial.println();
    return *((uint64_t *)buf);
  }
  return -1;
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
bool Within(uint64_t num, uint64_t cmp, uint64_t distance) {
  return (num < (cmp + distance)) && (num > (cmp - distance));
}


void loop() {
  bool IsPressed = false;
  while (digitalRead(3) == HIGH)
    ;
  Serial.println("Writing init message...");
  while (digitalRead(3) == LOW) {
    // Transmit while we wait for button release
    transmit(0xDEADBEEF);
  }

  Serial.println("Calibrating time...");

  uint64_t tx_time = 16776337352ull;  // About Feb 28 8:20 PM
  long TimeOut = millis() + 250;
  while (recieve() != tx_time) {
    // Recieve data for longer (doing it once per tx failed to recieve any data)
    TimeOut = millis() + 250;
    while (recieve() != tx_time && millis() < TimeOut) {}
    if (millis() < TimeOut) {
      // Wre left because of a valid transmission, not timeout
      break;
    }
    // Trainsmity periodically while we haven't confirmed the hoagie's time
    tx_time = getTime();
    transmit(tx_time);
  }
  Serial.println("Calibrated, press button to drop.");

  while (digitalRead(3) == LOW) {
    // Wait for button release
  }

  while (digitalRead(3) == HIGH) {
    // Wait for button press
  }

  // When we get here, the hogie has properly calibrated it's time
  while (true) {
    Serial.println("Writing drop message");

    while (digitalRead(3) == LOW) {
      transmit(0xFA110000);
      // Wait for button release
    }

    while (digitalRead(3) == HIGH) {
      // Wait for hogie until button press
      long rx_time = recieve();


      if (rx_time != -1) {
        // transmit -1 to release the hoagie's blocking recieve() function
        transmit(-1);

        Serial.println(rx_time);
      }
    }
  }
}
