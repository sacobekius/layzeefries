#include "console.h"
#include "bleserial.h"

Console console;

size_t Console::write(uint8_t c)
{
    return write(&c, 1);
}

size_t Console::write(const uint8_t *buffer, size_t size)
{
    bleSerial.write(buffer, size);
    return Serial.write(buffer, size);
}
