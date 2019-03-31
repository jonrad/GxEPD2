// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_EPD.h"

#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

GxEPD2_EPD::GxEPD2_EPD(int8_t cs, int8_t dc, int8_t rst, int8_t busy, int8_t busy_level, uint32_t busy_timeout,
                       uint16_t w, uint16_t h, GxEPD2::Panel p, bool c, bool pu, bool fpu) :
  WIDTH(w), HEIGHT(h), panel(p), hasColor(c), hasPartialUpdate(pu), hasFastPartialUpdate(fpu),
  _cs(cs), _dc(dc), _rst(rst), _busy(busy), _busy_level(busy_level), _busy_timeout(busy_timeout), _diag_enabled(false),
  _controller(new NullController())
{
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate)
{
  init(serial_diag_bitrate, true, false);
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate, bool initial, bool pulldown_rst_mode)
{
  _initial = initial;
  _pulldown_rst_mode = pulldown_rst_mode;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  if (serial_diag_bitrate > 0)
  {
    Serial.begin(serial_diag_bitrate);
    _diag_enabled = true;
  }
  if (_cs >= 0)
  {
    _controller->pinWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);
  }
  if (_dc >= 0)
  {
    _controller->pinWrite(_dc, HIGH);
    pinMode(_dc, OUTPUT);
  }
  _reset();
  if (_busy >= 0)
  {
    pinMode(_busy, INPUT);
  }
  _controller->spiBegin();
}

void GxEPD2_EPD::_reset()
{
  if (_rst >= 0)
  {
    if (_pulldown_rst_mode)
    {
      _controller->pinWrite(_rst, LOW);
      pinMode(_rst, OUTPUT);
      _controller->delay(20);
      pinMode(_rst, INPUT_PULLUP);
      _controller->delay(200);
    }
    else
    {
      _controller->pinWrite(_rst, HIGH);
      pinMode(_rst, OUTPUT);
      _controller->delay(20);
      _controller->pinWrite(_rst, LOW);
      _controller->delay(20);
      _controller->pinWrite(_rst, HIGH);
      _controller->delay(200);
    }
    _hibernating = false;
  }
}

void GxEPD2_EPD::_waitWhileBusy(const char* comment, uint16_t busy_time)
{
  if (_busy >= 0)
  {
    _controller->delay(1); // add some margin to become active
    unsigned long start = micros();
    while (1)
    {
      if (_controller->pinRead(_busy) != _busy_level) break;
      _controller->delay(1);
      if (micros() - start > _busy_timeout)
      {
        Serial.println("Busy Timeout!");
        break;
      }
    }
    if (comment)
    {
#if !defined(DISABLE_DIAGNOSTIC_OUTPUT)
      if (_diag_enabled)
      {
        unsigned long elapsed = micros() - start;
        Serial.print(comment);
        Serial.print(" : ");
        Serial.println(elapsed);
      }
#endif
    }
    (void) start;
  }
  else _controller->delay(busy_time);
}

void GxEPD2_EPD::_writeCommand(uint8_t c)
{
  _controller->spiBeginTransaction();
  if (_dc >= 0) _controller->pinWrite(_dc, LOW);
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  _controller->spiTransfer(c);
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  if (_dc >= 0) _controller->pinWrite(_dc, HIGH);
  _controller->spiEndTransaction();
}

void GxEPD2_EPD::_writeData(uint8_t d)
{
  _controller->spiBeginTransaction();
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  _controller->spiTransfer(d);
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  _controller->spiEndTransaction();
}

void GxEPD2_EPD::_writeData(const uint8_t* data, uint16_t n)
{
  _controller->spiBeginTransaction();
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  for (uint16_t i = 0; i < n; i++)
  {
    _controller->spiTransfer(*data++);
  }
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  _controller->spiEndTransaction();
}

void GxEPD2_EPD::_writeDataPGM(const uint8_t* data, uint16_t n)
{
  _controller->spiBeginTransaction();
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  for (uint16_t i = 0; i < n; i++)
  {
    _controller->spiTransfer(pgm_read_byte(&*data++));
  }
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  _controller->spiEndTransaction();
}

void GxEPD2_EPD::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  _controller->spiBeginTransaction();
  if (_dc >= 0) _controller->pinWrite(_dc, LOW);
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  _controller->spiTransfer(*pCommandData++);
  if (_dc >= 0) _controller->pinWrite(_dc, HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _controller->spiTransfer(*pCommandData++);
  }
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  _controller->spiEndTransaction();
}

void GxEPD2_EPD::_writeCommandDataPGM(const uint8_t* pCommandData, uint8_t datalen)
{
  _controller->spiBeginTransaction();
  if (_dc >= 0) _controller->pinWrite(_dc, LOW);
  if (_cs >= 0) _controller->pinWrite(_cs, LOW);
  _controller->spiTransfer(pgm_read_byte(&*pCommandData++));
  if (_dc >= 0) _controller->pinWrite(_dc, HIGH);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    _controller->spiTransfer(pgm_read_byte(&*pCommandData++));
  }
  if (_cs >= 0) _controller->pinWrite(_cs, HIGH);
  _controller->spiEndTransaction();
}
