#include "MagneticSensorSSC.h"


MagneticSensorSSC::MagneticSensorSSC(spi_device_handle_t* spiHandler){
  this->spiHandler = spiHandler;
  encTrans.length = 32;
  encTrans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  encTrans.tx_data[0] = 0b10000000;
  encTrans.tx_data[1] = 0b00100001;
  encTrans.tx_data[3] = 0;
  encTrans.tx_data[4] = 0;
}


void MagneticSensorSSC::init(){
  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSSC::getSensorAngle(){
  spi_device_transmit(*spiHandler, &encTrans);
  uint16_t result = encTrans.rx_data[3];
  result |= encTrans.rx_data[2] << 8;
  return ((float)result) / 32767.0 * _2PI;
}
