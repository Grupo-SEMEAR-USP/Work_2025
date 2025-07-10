#include <Wire.h>

#define SLAVE_ADDRESS 0x42

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial.println("I2C Slave pronto.");
  Wire.onReceive(receiveEvent);
}

void loop() {
  delay(100);
}

void receiveEvent(int numBytes) {
  Serial.print("Recebido: ");
  while (Wire.available()) {
    uint8_t b = Wire.read();
    Serial.print("0x");
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  Serial.println();
}
