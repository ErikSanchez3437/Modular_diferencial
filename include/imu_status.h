#ifndef IMU_STATUS_H
#define IMU_STATUS_H

#include <cstdint>
struct ImuStatus {
  uint16_t delta; /**< The time between samples, in &micro;s. */
  float roll;     /**< The roll measured by the IMU, in degrees. */
  float pitch;    /**< The pitch measured by the IMU, in degrees. */
  float yaw;      /**< The yaw measured by the IMU, in degrees. */
  int16_t accX; /**< The x acceleration measured by the IMU. */
  int16_t accY; /**< The y acceleration measured by the IMU. */
  int16_t accZ; /**< The z acceleration measured by the IMU. */
};

#endif /* IMU_STATUS_H */
