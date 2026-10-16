#ifndef VELOCITY_COMMAND_H
#define VELOCITY_COMMAND_H

#include <cstdint>
struct VelocityCommand {
  int32_t lVel; /**< The velocity desired on the left motor, in 0.229 rev/min. */
  int32_t rVel; /**< The velocity desired on the right motor, in 0.229 rev/min. */
};

#endif /* VELOCITY_COMMAND_H */
