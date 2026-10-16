#ifndef CURRENT_COMMAND_H
#define CURRENT_COMMAND_H

#include <cstdint>
struct CurrentCommand {
  int16_t lCur; /**< The current desired on the left motor, in 2.69 mA. */
  int16_t rCur; /**< The current desired on the right motor, in 2.69 mA. */
};

#endif /* CURRENT_COMMAND_H */
