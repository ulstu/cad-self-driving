/*
 * Simple interface to the Fly Sky IBus RC system.
 */

#include <Arduino.h>
#include "FlySkyIBus.h"

FlySkyIBus IBus;

void FlySkyIBus::begin(HardwareSerial& serial)
{
  serial.begin(115200);
  begin((Stream&)serial);
}

void FlySkyIBus::begin(Stream& stream)
{
  this->stream = &stream;
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;
}

void FlySkyIBus::loop(void)
{
  while (stream->available() > 0)
  {
    uint32_t now = millis();
    if (now - last >= PROTOCOL_TIMEGAP)
    {
      state = GET_PREAM;
    }
    last = now;
    
    uint8_t v = stream->read();
    switch (state)
    {
      case GET_PREAM:
        if (v == PROTOCOL_CMD)
        {
          ptr = 0;
          len = 28;
          chksum = 0x00;
          state = GET_DATA;
        }
        else
        {
          state = DISCARD;
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        if (ptr % 2){
          chksum += v;
        } else {
          chksum += v << 8;
        }
        if (ptr == len)
        {
          state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        uint16_t ch;
        // Validate checksum
        if (chksum == (v << 8) + lchksum) {
            for (uint8_t i = 0; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
              channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
            }
        }
        state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
  }
}

uint16_t FlySkyIBus::readChannel(uint8_t channelNr)
{
  if (channelNr < PROTOCOL_CHANNELS) {
    return channel[channelNr];
  }
  else {
    return 0;
  }
}

bool FlySkyIBus::available() {
  if (millis() - last < 200) {
    return true;
  } else {
    return false;
  }
}
