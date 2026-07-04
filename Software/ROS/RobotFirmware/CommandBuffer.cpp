#include "CommandBuffer.h"

namespace CommandBuffer {

volatile MotionCommand buffer[Config::COMMAND_BUFFER_SIZE];
volatile uint8_t head = 0;
volatile uint8_t tail = 0;

} // namespace CommandBuffer
