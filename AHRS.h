// AHRS.h
#ifndef AHRS_H
#define AHRS_H

#include <Arduino_LSM9DS1.h>

class AHRS {
private:
  // Mahony filter parameters
  const float twoKp = 2.0f * 5.0f;
  const float twoKi = 2.0f * 1.5f;

  float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

public:
  const float updateInterval;  // Filter update frequency, in seconds (1.0f / *.*Hz)

  AHRS(float interval)
    : updateInterval(interval) {}

  void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    gx = DEG_TO_RAD * gx;
    gy = DEG_TO_RAD * gy;
    gz = DEG_TO_RAD * gz;
  }

  void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalize accelerometer measurement
      recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Estimated direction of gravity and cross product of estimated direction and measured direction of gravity
      vx = q1 * q3 - q0 * q2;
      vy = q0 * q1 + q2 * q3;
      vz = q0 * q0 - 0.5f + q3 * q3;

      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Compute and apply integral feedback if enabled
      integralFBx += twoKi * ex * updateInterval;  // integral error scaled by Ki
      integralFBy += twoKi * ey * updateInterval;
      integralFBz += twoKi * ez * updateInterval;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;

      // Apply proportional feedback
      gx += twoKp * ex;
      gy += twoKp * ey;
      gz += twoKp * ez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * updateInterval);  // pre-multiply common factors
    gy *= (0.5f * updateInterval);
    gz *= (0.5f * updateInterval);
    float qa = q0;
    float qb = q1;
    float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  float getRoll() {
    return atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  }

  float getPitch() {
    return asin(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
  }

  float getYaw() {
    return atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
  }
};

#endif
