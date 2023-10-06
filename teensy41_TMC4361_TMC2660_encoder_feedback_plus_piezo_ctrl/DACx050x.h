/*
   DACx050x.h
   This file contains the class for operationg the x050x DACs from TI
   https://www.ti.com/lit/ds/symlink/dac60508.pdf
   The internal vref is 2.5V; by setting the gain we can achieve outputs 
   between 0 and 1.75V (half gain), 0 and 2.5V, (unit gain) and 0 and 5V (double gain)

   Class Members:
    Public:
      Functions:
        DACx050x: Initialize values
        begin:    Initialize the device
        write:    Write a 2 byte value to an address
        NOT IMPLEMENTED set_gain: Set the gain (1/2, 1, or 2)
        output:   Write an analog output to a specific channel
        reset:    Reset the DAC
    Private:
      Variables:
        spi_:      SPI bus object
        settings_: SPI settings
        cs_:       CS pin (active low)

*/

#ifndef DACx050x_H
#define DACx050x_H

#include <Arduino.h>
#include <SPI.h>

// CRC polynomial
#define DACx050x_CRC 0b100000111

// Define register and field map
// Config
#define DACx050x_CONFIG  0b0011
// Config fields
#define DACx050x_ALM_SEL (1<<13) // 0: ~{ALARM} pin is CRC_ERR; 1: ~{ALARM} pin is REF_ALARM, default 0
#define DACx050x_ALM_EN  (1<<12) // 0: ~{ALWRM} pin is push-pull; 1: ~{ALARM} is open drain, default 0
#define DACx050x_CRC_EN  (1<<11) // 0: CRC disabled; 1: CRC enabled, default 0
#define DACx050x_FSDO    (1<<10) // 0: Update serial data out at normal speed; 1: Update a half-clock faster, default 0
#define DACx050x_DSDO    (1<<9)  // 0: Enable SDO output (tri-state when CS high, output otherwise); 1: SDO always tri-state, default 0
#define DACx050x_DREF    (1<<8)  // 0: Enable internal ref; 1: disable internal reference, default 0
#define DACx050x_DDAC_MASK 0b11111111 // Low 8 bits are for the 8 channels; set a bit high to disable, low to enable default 0

// Sync
#define DACx050x_SYNC 0b0010
// Sync fields
#define DAC_x050x_BCST_MASK 0xFF00 // Set bits high to update when the broadcast register is written to. Default 1
#define DAC_x050x_SYNC_MAXK 0x00FF // Set bits high to update in response to LDAC trigger, low to update at CS rising edge. Default 0

// Gain 
#define DACx050x_GAIN    0b0100
// Gain fields
#define DACx050x_DIV_EN (1<<8) // Set bit 8 to divide the internal ref by 2
#define DACx050x_GAIN_MASK 0b11111111 //  Low 8 bits are for the 8 channels; set a bit high to set the gain to 2, gain is 1 otherwise

// Trigger
#define DACx050x_TRIGGER 0b0101
// Trigger fields
#define DACx050x_LDAC    (1<<4)  // 1: Synchronoulsy load DACs configed as such in SYNC register; 0: do nothing
#define DACx050x_SOFT_RST 0b0101 // Set bits [3..0] to this to soft reset the device

// Broadcast
#define DACx050x_BCST 0b0110 // Used to set multiple values at once
// DAC field is just 12-16 bit output

// Status
#define DACx050x_STAT 0b0111
// Status fields
#define DACx050x_ALM  1 // Read only. High: (Vref/DIV) - VDD below threshold

// DAC Channel 
#define DACx050x_CHANNEL 0b1000 // add the channel number to this
#define DACx050x_NUM_CHANNEL 8
// DAC field is just 12-16 bit output


// Bit 0 sets the output gain setting (high for 2x)
// Bit 1 sets the ref divide setting (high for /2)
// Note -  the divider is set for all channels while the output gain is channel-specific
// Note -  not recommended to have DIV 0 and gain 0
#define DACx050x_HALF_GAIN   0b10
#define DACx050x_UNIT_GAIN   0b11
#define DACx050x_DOUBLE_GAIN 0b01

#define DACx050x_CLK 1000000

class DACx050x {
  public:
    DACx050x(uint8_t cs);
    void begin(SPIClass &port);
    // void set_gain(uint8_t gain, uint8_t channel_bits);
    void output(uint8_t channel_num, uint16_t value);
    uint16_t get_setpoint(uint8_t channel_num);
    uint16_t write(uint8_t reg, uint16_t data);
    void reset();

  private:
    SPIClass *spi_;
    uint8_t cs_;
    SPISettings settings_;
    uint16_t setpoints[DACx050x_NUM_CHANNEL];
};

#endif /*DACx050x_H*/
