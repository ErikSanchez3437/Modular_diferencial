#ifndef POSITION_COMMAND_H
#define POSITION_COMMAND_H

#include <cstdint>
struct PositionCommand {
  int32_t lPos; /**< The position desired on the left motor, in rev / 4096. */
  int32_t rPos; /**< The position desired on the right motor, in rev / 4096. */
};

#endif /* POSITION_COMMAND_H */
