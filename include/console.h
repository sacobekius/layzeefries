#ifndef LAYZEEFRIES_CONSOLE_H
#define LAYZEEFRIES_CONSOLE_H

#include <Arduino.h>

/* Spiegelt alle status/debug-output naar zowel de USB-serial als de BLE-seriele
 * verbinding, zodat je ook via een BLE-terminal-app kunt meelezen.
 */
class Console : public Print {
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write;
};

extern Console console;

#endif //LAYZEEFRIES_CONSOLE_H
