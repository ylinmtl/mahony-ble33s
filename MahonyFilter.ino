#include <Arduino_LSM9DS1.h>
#include "AHRS.h"

const float filterUpdateInterval = 1.0f / 200.0f;
const float serialSendInterval = 1.0f / 50.0f;

unsigned long prevFilterUpdate = 0;
unsigned long prevSerialSend = 0;

AHRS ahrs(filterUpdateInterval);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
    unsigned long now = micros();
    
    if ((now - prevFilterUpdate) >= filterUpdateInterval * 1e6) {
        float ax, ay, az, gx, gy, gz;
        ahrs.readIMU(ax, ay, az, gx, gy, gz);
        ahrs.MahonyAHRSupdate(-gx, -gy, -gz, ax, ay, az);
        prevFilterUpdate = now;
    }

    if ((now - prevSerialSend) >= serialSendInterval * 1e6) {
        Serial.print(ahrs.getRoll());
        Serial.print(",");
        Serial.print(ahrs.getPitch());
        Serial.print(",");
        Serial.println(ahrs.getYaw());
        prevSerialSend = now;
    }
}