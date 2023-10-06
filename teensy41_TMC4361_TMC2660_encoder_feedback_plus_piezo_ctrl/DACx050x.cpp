/*
   DACx050x.cpp
   This file contains the class for operationg the x050x DACs from TI
   https://www.ti.com/lit/ds/symlink/dac60508.pdf
   The internal vref is 2.5V; by setting the gain we can achieve outputs
   between 0 and 1.75V (half gain), 0 and 2.5V, (unit gain) and 0 and 5V (double gain)

   Class Members:
    Public:
      Functions:
        DACx050x: Initialize values
          Arguments: uint8_t cs
        begin:    Initialize the device
          Arguments: SPIClass *port
        write:    Write a 2 byte value to an address
          Arguments: uint8_t register, uint16_t data
          Returns: uint16_t response
        NOT IMPLEMENTED set_gain: Set the gain (1/2, 1, or 2)
          Arguments: uint8_t gain, uint8_t register_bits
        output:   Write an analog output to a specific channel
          Arguments: uint8_t register_num, uint16_t value
        reset:    Reset the DAC
    Private:
      Variables:
        spi_:     SPI bus object
        settings_: SPI settings
        cs_:      CS pin (active low)

*/

#include "DACx050x.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: DACx050x() initializes the DAC's CS pin

  OPERATION:   We use the member initializer list to get the CS pin

  ARGUMENTS:
      uint8_t cs:      CS pin

  RETURNS: NONE

  INPUTS / OUTPUTS: NONE

  LOCAL VARIABLES: NONE

  SHARED VARIABLES:
     uint8_t cs_: cs pin

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
DACx050x::DACx050x(uint8_t cs)
  : cs_(cs) {
  // Clear the setpoints array
  for(uint8_t i = 0; i < DACx050x_NUM_CHANNEL; i++){
    setpoints[i] = 0;
  }
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: write() writes arbitrary data to an arbitrary address and returns the response.

  OPERATION:   We first initialize an SPI communication, then send the address to the device, then the data. We close the communication.

  ARGUMENTS:
      uint8_t reg:   The register, 4 bits
      uint16_t data: The 16 bit payload

  RETURNS:
    uint16_t response: The 16 bit response

  INPUTS:  Data is received on the MISO line

  OUTPUTS: The CS, CLK, and MOSI pins are used as output

  LOCAL VARIABLES: None

  SHARED VARIABLES:
     SPIClass *spi_: pointer to SPI object
     SPISettings settings_: SPI settiings object

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint16_t DACx050x::write(uint8_t reg, uint16_t data) {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  spi_->transfer(reg);
  uint16_t response = spi_->transfer16(data);
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();

  return response;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: begin() initializes the SPI bus, stores the settings, and initializes communication with the device.

  OPERATION:   We store the SPI bus object pointer and settings to a local variable. We then configure the device.
               The configuration is hard-coded for now.

  ARGUMENTS:
      SPIClass &port:      Pointer to SPI object

  RETURNS: None

  INPUTS:  Data is received on the MISO line

  OUTPUTS: The CS, CLK, and MOSI pins are used as output

  LOCAL VARIABLES:
    uint16_t datagram: Data to send over SPI

  SHARED VARIABLES:
     SPIClass *spi_: pointer to SPI object
     SPISettings settings_: SPI settiings object

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
void DACx050x::begin(SPIClass &port) {
  uint16_t datagram;

  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);

  spi_ = &port;
  settings_ = SPISettings(DACx050x_CLK, MSBFIRST, SPI_MODE2);

  this->reset();

  datagram = 0;
  this->write(DACx050x_CONFIG, datagram);

  datagram = DAC_x050x_BCST_MASK;
  this->write(DACx050x_SYNC, datagram);

  datagram =  (0x00 << 8) + 0x80;// Set double gain on channel 7, leave the rest at no gain
  this->write(DACx050x_GAIN, datagram);
  return;
}

void DACx050x::output(uint8_t channel_num, uint16_t value) {
  uint8_t addr = 0b1000 | channel_num;
  setpoints[channel_num] = value;
  this->write(addr, value);
}

uint16_t DACx050x::get_setpoint(uint8_t channel_num){
  return setpoints[channel_num];
}

void DACx050x::reset() {
  for(uint8_t i = 0; i < DACx050x_NUM_CHANNEL; i++){
    setpoints[i] = 0;
  }
  this->write(DACx050x_TRIGGER, DACx050x_SOFT_RST);
}
