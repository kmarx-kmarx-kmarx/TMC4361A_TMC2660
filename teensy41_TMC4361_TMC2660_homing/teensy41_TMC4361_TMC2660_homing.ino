/*
    Implement homing to one of the limit switches

    Author: Kevin Marx
    Created on: 7/5/2022
*/

#include <SPI.h>
#include "TMC4361A.h" // These files come from the TCM4361A manufacturer

// Define the chip select pins for the TMC4361 connected to the microcontroller
const int pin_TMC4361_CS[4] = {41, 36, 35, 34};
// Define the clock pin. The clock speed of the TMC4361 is slower than the Teensy's clock speed
const int pin_TMC4361_CLK = 37;
// Define the CS pin for the 3.3 to 5V level shifter
const int pin_DAC80508_CS = 33;

const int clk_Hz_TMC4361 = 16000000;


void setup() {
  SerialUSB.begin(20000000);

  // temp 3.3V
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);

  // CS pin for DAC80508
  pinMode(pin_DAC80508_CS, OUTPUT);
  digitalWrite(pin_DAC80508_CS, HIGH);

  // Clock for TMC4361 and TMC2660
  pinMode(pin_TMC4361_CLK, OUTPUT);
  analogWriteFrequency(pin_TMC4361_CLK, clk_Hz_TMC4361);
  analogWrite(pin_TMC4361_CLK, 128);

  // CS pin for TMC4361
  for (int i = 0; i < 4; i++)
  {
    pinMode(pin_TMC4361_CS[i], OUTPUT);
    digitalWrite(pin_TMC4361_CS[i], HIGH);
  }

  // SPI
  SPI.begin();
  delayMicroseconds(5000);

  // initilize TMC4361
  for (int i = 0; i < 4; i++) {
    // TMC4361
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_CLK_FREQ | 0x80, clk_Hz_TMC4361); // clock frequency
    // SPI configuration
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_SPIOUT_CONF | 0x80, 0x4440108A); // SPI_OUT_CONF
    // cover datagram for TMC2660
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_COVER_LOW_WR | 0x80, 0x000900C3);
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_COVER_LOW_WR | 0x80, 0x000A0000);
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_COVER_LOW_WR | 0x80, 0x000C000A);
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_COVER_LOW_WR | 0x80, 0x000E00A0); // SDOFF = 1
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_COVER_LOW_WR | 0x80, 0x000D001F); // current scaling: 0b11111 (max)
    // current open loop scaling
    TMC4361_sendData(pin_TMC4361_CS[i], 0x06 | 0x80, (0x01 << 24) + (0xA0 << 16) + (0xA0 << 8) + 0xA0); // hold (0x01), drv1(0x20), drv2(0x20), boost(0x20)
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_CURRENT_CONF | 0x80, 0x00000003); // disable current scaling for hold and drive
    //ramp mode
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_RAMPMODE | 0x80, 0b010); // S-shaped ramp
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_VMAX | 0x80, 0); // set initial velocity to 0
    // clear xactual
    TMC4361_sendData(pin_TMC4361_CS[i], TMC4361A_XACTUAL | 0x80, 0x00000000);

    enableLimitSwitch(pin_TMC4361_CS[i]); // enable limit switch reading
  }
}

void loop() {

}




/****************************************************************************************

 ****************************************************************************************/
void TMC4361_sendData(int pin_CS, unsigned long address, unsigned long datagram)
{

  unsigned long i_datagram = 0;

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_CS, LOW);
  delayMicroseconds(100);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8; // current open loop scaling
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(pin_CS, HIGH);
  SPI.endTransaction();
  return;
}

uint32_t TMC4361_transceiveData(uint8_t pin_CS, unsigned long address, unsigned long datagram)
{
  uint32_t i_datagram = 0;

  SPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_CS, LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(pin_CS, HIGH);
  SPI.endTransaction();

  return i_datagram;
}

unsigned long getXACTUAL(uint8_t pin_CS)
{
  delay(20);
  unsigned long i_datagram = 0;
  unsigned long address = TMC4361A_XACTUAL;
  unsigned long datagram = 0;

  i_datagram = TMC4361_transceiveData(pin_CS, address, datagram);

  return i_datagram;
}

void enableLimitSwitch(uint8_t pin_CS) {
  // Enable both the left and right limit switches
  delay(20);
  // Set whether they are low active (set bit to 0) or high active (1)
  unsigned long pol_datagram = (1 << TMC4361A_POL_STOP_LEFT_SHIFT) | (1 << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  unsigned long en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  TMC4361_sendData(pin_CS, TMC4361A_REFERENCE_CONF | 0x80, pol_datagram);
  TMC4361_sendData(pin_CS, TMC4361A_REFERENCE_CONF | 0x80, en_datagram | pol_datagram);


  return;
}

uint8_t readLimitSwitches(uint8_t pin_CS) {
  // Read both limit switches. Set bit 0 if the left switch is pressed and bit 1 if the right switch is pressed
  unsigned long i_datagram = 0;
  unsigned long address = TMC4361A_STATUS;
  unsigned long datagram = 0;
  // Get the datagram
  i_datagram = TMC4361_transceiveData(pin_CS, address, datagram);
  // Mask off everything except the button states
  i_datagram &= (TMC4361A_STOPL_ACTIVE_F_MASK | TMC4361A_STOPR_ACTIVE_F_MASK);
  // Shift the button state down to bits 0 and 1
  i_datagram >>= TMC4361A_STOPL_ACTIVE_F_SHIFT;
  // Get rid of the high bits
  uint8_t result = i_datagram & 0xff;

  return result;
}

void setBits(uint8_t pin_CS, uint8_t address, uint32_t mask, uint32_t shift, uint32_t dat){
  // Set the bits of any register at any location without changing the other bits
  // First, read the bits at the register
}

void rotate(uint8_t pin_CS, int32_t velocity){
  // Set ramp mode - set the ramp/position mode bit to 1
  uint8_t address = TMC4361A_RAMPMODE;
  
}
