#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H
#include <array>
#include <cstdint>

using std::array;

/**
 * The status of both motors. Created by the OpenCR board after reading the
 * corresponding variables from the motors
 */
struct MotorStatus {
  uint16_t delta; /**< The time between samples, in &micro;s. */
  int16_t lCur;   /**< The current reported by the left motor, in 2.69 mA. */
  int16_t rCur;   /**< The current reported by the right motor, in 2.69 mA. */
  int32_t lVel;   /**< The velocity reported by the left motor, in 0.229 rev
                        / min. */
  int32_t rVel;   /**< The velocity reported by the right motor, in 0.229
                         rev / min. */
  int32_t lPos;   /**< The position reported by the left motor, in 1 / 4096
                        rev. */
  int32_t rPos;   /**< The position reported by the right motor, in 1 / 4096
                         rev. */
  /**
   * Default constructor. Sets all the fields to zero.
   */
  MotorStatus();

  /**
   * Get the time that passed betweem this and the previous motor status.
   * \return the delta betweem the current and the previous state, in seconds.
   */
  float getDelta();
  /**
   * Get the current of both motors.
   * \return the current of the left and the right motor, in A.
   */
  array<float, 2> getCurrent();
  /**
   * Get the torque of both motors.
   * \return the torque of the left and the right motor, in Nm.
   */
  array<float, 2> getTorque();
  /**
   * Get the velocity of both motors.
   * \return the velocity of the left and the right motor, in rad/s.
   */
  array<float, 2> getVelocity();
  /**
   * Get the position of both motors.
   * \return the position of the left and the right motor, in rad.
   */
  array<float, 2> getPosition();
};

#endif /* MOTOR_STATUS_H */

