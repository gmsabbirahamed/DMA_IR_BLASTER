#ifndef SHT3X_CUSTOM_H
#define SHT3X_CUSTOM_H

#include <Wire.h>

class SHT3x {
public:
  SHT3x() {}
  
  // Initialize sensor (default I2C address is 0x44)
  bool begin(uint8_t address = 0x44) {
    Wire.begin();
    _address = address;
    return true;
  }

  // Read temperature in Celsius
  float readTemperature() {
    if (!readSensorData()) return NAN; // Handle read failure
    return (175 * ((float)data[0] * 256 + (float)data[1]) / 65535.0) - 45.0;
  }

  // Read humidity in percentage
  float readHumidity() {
    if (!readSensorData()) return NAN; // Handle read failure
    return (100 * ((float)data[3] * 256 + (float)data[4]) / 65535.0);
  }

private:
  uint8_t _address;
  uint8_t data[6];

  // Read data from sensor (6 bytes)
  bool readSensorData() {
    Wire.beginTransmission(_address);
    Wire.write(0x2C); // High repeatability measurement command
    Wire.write(0x06);
    Wire.endTransmission();
    delay(50); // Wait for the measurement

    Wire.requestFrom(_address, (uint8_t)6);
    if (Wire.available() == 6) {
      for (int i = 0; i < 6; i++) {
        data[i] = Wire.read();
      }
      return true;
    } else {
      return false; // Reading failed
    }
  }
};

#endif
