#include <Arduino_LSM9DS1.h>
#include "scheduler.h"
#include "AHRS.h"

const float filterUpdateInterval = 1.0f / 200.0f;
const float serialSendInterval = 1.0f / 100.0f;

unsigned long prevFilterUpdate = 0;
unsigned long prevSerialSend = 0;

Scheduler scheduler;
AHRS ahrs;

void updateFilter() {
  float ax, ay, az, gx, gy, gz;
  ahrs.readIMU(ax, ay, az, gx, gy, gz);
  ahrs.MahonyAHRSupdate(-gx, -gy, -gz, ax, ay, az);
}

void sendSerial() {
  Serial.print(ahrs.getRoll());
  Serial.print(",");
  Serial.print(ahrs.getPitch());
  Serial.print(",");
  Serial.print(ahrs.getYaw());
  Serial.print(", FT");
  Serial.print(scheduler.getExecTime(updateFilter));
  Serial.print(", ST");
  Serial.println(scheduler.getExecTime(sendSerial));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  scheduler.addTask(updateFilter, filterUpdateInterval * 1e6);
  scheduler.addTask(sendSerial, serialSendInterval * 1e6);
}

void loop() {
  scheduler.run();
}