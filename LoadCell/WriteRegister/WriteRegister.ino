/**
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include <Wire.h>
#include <Adafruit_NAU7802.h>

#define TCAADDR 0x70

Adafruit_NAU7802 nau;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(){
  Wire.begin();
  
  Serial.begin(9600);

  for (uint8_t t=0; t<2; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }

  tcaselect(0);
  if (! nau.begin()) {
    Serial.println("Failed to find NAU7802");
  }
  Serial.println("Found NAU7802 1");

  tcaselect(1);
  if (! nau.begin()) {
    Serial.println("Failed to find NAU7802");
  }
  Serial.println("Found NAU7802 2");

  Serial.println("Time, T, F");
}

void loop() {
  Serial.print("t,");
  tcaselect(0);
  while (! nau.available()) {
    delay(1);
  }
  int32_t val = nau.read();
  Serial.print(val); Serial.print(","); 

  tcaselect(1);
  while (! nau.available()) {
    delay(1);
  }
  val = nau.read();
  Serial.println(val); //Serial.println(","); 
}