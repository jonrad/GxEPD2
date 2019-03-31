#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdio.h>
#include <stdlib.h>

class Controller
{
  public:
    virtual void spiBegin() = 0;
    virtual void spiBeginTransaction() = 0;
    virtual void spiEndTransaction() = 0;
    virtual uint8_t spiTransfer(uint8_t data) = 0;
    virtual void spiTransfer(void *buf, uint16_t count) = 0;

    virtual void delay(uint64_t ms) = 0;
    virtual void pinWrite(uint8_t pin, uint8_t value) = 0;
    virtual uint8_t pinRead(uint8_t pin) = 0;
};

class NullController : public Controller
{
  public:
    void spiBegin() {}
    void spiBeginTransaction() {}
    void spiEndTransaction() {}
    uint8_t spiTransfer(uint8_t data) { return 0; }
    void spiTransfer(void *buf, uint16_t count) {}

    void delay(uint64_t ms) {};
    void pinWrite(uint8_t pin, uint8_t value) {}
    uint8_t pinRead(uint8_t pin) { return 0; }
};

#endif