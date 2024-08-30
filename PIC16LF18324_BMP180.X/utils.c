#include "utils.h"

uint8_t waiting(uint16_t loopct)
{
    // Busy loop. Sigh!
    for (uint8_t i = 0; i < 200; i++)
    {
        // Need this to avoid compiler opt-out
        asm("NOP");
    }

    // Enforce wait limit based on input value
    if (loopct > 50000)
    {
        return 0;
    }
    return 1;
}
