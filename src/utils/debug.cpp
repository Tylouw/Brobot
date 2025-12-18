#include "debug.h"

#if DEBUG_ENABLE
void debug_init()
{
    Serial.begin(DEBUG_BAUD);
    // Optional: give USB serial time to come up on some boards
    // delay(50);
    Serial.setTimeout(2);
    Serial.println();
    Serial.println("[I] Debug serial initialized");
}
#endif
