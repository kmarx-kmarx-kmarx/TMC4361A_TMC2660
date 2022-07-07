/*
    Identify which limit switch is the "right" or "left".
    The motor current is turned off and the carriage can be
    manually pushed to either limit switch. Check the
    serial monitor to see which switch was activated.

    Author: Kevin Marx
    Created on: 7/5/2022
*/

#include "TMC4361A.h" // These files come from the TCM4361A manufacturer
#include <SPI.h>

// Define the chip select pins for the TMC4361 connected to the microcontroller
const uint8_t pin_TMC4361_CS[4] = {41, 36, 35, 34};
// Define the clock pin. The clock speed of the TMC4361 is slower than the Teensy's clock speed
const uint8_t pin_TMC4361_CLK = 37;
// Define the CS pin for the 3.3 to 5V level shifter
const uint8_t pin_DAC80508_CS = 33;

ConfigurationTypeDef tmc4361_configs[4];
TMC4361ATypeDef tmc4361[4];
const uint32_t clk_Hz_TMC4361 = 16000000;

#define LEFT_SW 0b01
#define RGHT_SW 0b10

void setup() {
  SerialUSB.begin(20000000);

  // Supply 3.3V to the level shifter. Future revisions will have power hardwired
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);

  // CS pin for DAC80508level shifter
  pinMode(pin_DAC80508_CS, OUTPUT);
  digitalWrite(pin_DAC80508_CS, HIGH);

  // Clock for TMC4361 and TMC2660
  pinMode(pin_TMC4361_CLK, OUTPUT);
  analogWriteFrequency(pin_TMC4361_CLK, clk_Hz_TMC4361);
  analogWrite(pin_TMC4361_CLK, 128); // 50% duty

  // Initialize TMC4361
  for (int i = 0; i < 4; i++)
  {
    // Initialize the tmc4361 with their channel number and default configuration
    tmc4361A_init(&tmc4361[i], pin_TMC4361_CS[i], &tmc4361_configs[i], tmc4361A_defaultRegisterResetState);
    // Set the chip select pins
    pinMode(pin_TMC4361_CS[i], OUTPUT);
    digitalWrite(pin_TMC4361_CS[i], HIGH);
  }

  // SPI
  SPI.begin();
  delayMicroseconds(5000);

  // initilize TMC4361
  for (int i = 0; i < 4; i++) {
    // TMC4361
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_CLK_FREQ | TMC_WRITE_BIT, clk_Hz_TMC4361);
    // SPI configuration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_SPIOUT_CONF | TMC_WRITE_BIT, 0x4440108A);
    // cover datagram for TMC2660
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000900C3);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000A0000);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000C000A);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000E00A0); // SDOFF = 1
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000D001F); // current scaling: 0b11111 (max)
    // current open loop scaling
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_SCALE_VALUES | TMC_WRITE_BIT, (0xFF << TMC4361A_HOLD_SCALE_VAL_SHIFT) +   // Set hold scale value (0 to 255)
                      (0xFF << TMC4361A_DRV2_SCALE_VAL_SHIFT) +   // Set DRV2 scale  (0 to 255)
                      (0xFF << TMC4361A_DRV1_SCALE_VAL_SHIFT) +   // Set DRV1 scale  (0 to 255)
                      (0xFF << TMC4361A_BOOST_SCALE_VAL_SHIFT)); // Set boost scale (0 to 255)

    tmc4361A_setBits(&tmc4361[i], TMC4361A_CURRENT_CONF | TMC_WRITE_BIT, TMC4361A_DRIVE_CURRENT_SCALE_EN_MASK); // keep drive current scale
    tmc4361A_setBits(&tmc4361[i], TMC4361A_CURRENT_CONF | TMC_WRITE_BIT, TMC4361A_HOLD_CURRENT_SCALE_EN_MASK);  // keep hold current scale

    enableLimitSwitch(&tmc4361[i]); // enable limit switch reading

    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, TMC4361A_LATCH_X_ON_ACTIVE_L_MASK); // store position when we hit left bound
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, TMC4361A_LATCH_X_ON_ACTIVE_R_MASK); // store position when we hit right bound

    // enable homing
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, 0b1100 << TMC4361A_HOME_EVENT_SHIFT);// define home behavior
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, TMC4361A_STOP_LEFT_IS_HOME_MASK);    // use stop left as home
    tmc4361A_setBits(&tmc4361[i], TMC4361A_HOME_SAFETY_MARGIN | TMC_WRITE_BIT, 1 << 6);                         // have a safety margin around home
  }

  // Home all the motors
  int32_t x_home[4];
  for (int i = 0; i < 1; i++) {
    Serial.print("Homing ");
    Serial.println(pin_TMC4361_CS[i]);
    x_home[i] = homing_lft(&tmc4361[i], 10000, 200000);
    Serial.print("x_home: ");
    Serial.println(x_home[i]);
  }
}

void loop() {

}




/****************************************************************************************

 ****************************************************************************************/
void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length) {
  // Initialize SPI transfer
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(channel, LOW);
  delayMicroseconds(100);
  // Write each byte and overwrite data[] with the response
  for (size_t i = 0; i < length; i++) {
    data[i] = SPI.transfer(data[i]);
  }
  // End the transaction
  digitalWrite(channel, HIGH);
  SPI.endTransaction();
  return;
}

void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat) {
  // Set the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  uint32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // OR with the bits we want to set
  datagram |= dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address | TMC_WRITE_BIT, datagram);

  return;
}
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat) {
  // Reset the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  uint32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // AND with the bits with the negation of the bits we want to clear
  datagram &= ~dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address | TMC_WRITE_BIT, datagram);

  return;
}

void enableLimitSwitch(TMC4361ATypeDef *tmc4361A) {
  // Enable both the left and right limit switches
  // Set whether they are low active (set bit to 0) or high active (1)
  unsigned long pol_datagram = (1 << TMC4361A_POL_STOP_LEFT_SHIFT) | (1 << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  unsigned long en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, pol_datagram);
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, en_datagram);

  return;
}

uint8_t readSwitchEvent(TMC4361ATypeDef *tmc4361A) {
  // Read both limit switches. Set bit 0 if the left switch is pressed and bit 1 if the right switch is pressed
  unsigned long i_datagram = 0;
  unsigned long address = TMC4361A_EVENTS;

  // Get the datagram
  i_datagram = tmc4361A_readInt(tmc4361A, address);
  // Mask off everything except the button states
  i_datagram &= (TMC4361A_STOPL_EVENT_MASK | TMC4361A_STOPR_EVENT_MASK);
  // Shift the button state down to bits 0 and 1
  i_datagram >>= TMC4361A_STOPL_EVENT_SHIFT;
  // Get rid of the high bits
  uint8_t result = i_datagram & 0xff;

  return result;
}

int32_t homing_lft(TMC4361ATypeDef *tmc4361A, int32_t v_slow, int32_t v_fast) {
  // Homing routine
  // First, check if we are already at a limit switch
  uint8_t eventstate = readSwitchEvent(tmc4361);
  if (eventstate == LEFT_SW) {
    // If we are at the left limit switch, go right until we are no longer hitting it
    tmc4361A_rotate(tmc4361, v_fast);
    while (eventstate != 0) {
      // Try clearing the event
      tmc4361A_rstBits(tmc4361, TMC4361A_EVENTS, (TMC4361A_STOPL_EVENT_MASK | TMC4361A_STOPR_EVENT_MASK));
      delay(5);
      eventstate = readSwitchEvent(tmc4361);
    }
    // Wait until we hit the right limit switch
    while (eventstate == 0) {
      delay(5);
      eventstate = readSwitchEvent(tmc4361);
    }
  }
  // Now we are either at the right limit switch or somewhere in the middle
  // Enable home tracking
  tmc4361A_setBits(tmc4361, TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, TMC4361A_START_HOME_TRACKING_MASK);
  // Move back to the left slowly
  tmc4361A_rotate(tmc4361, -v_slow);
  while (eventstate != 0) {
    // Try clearing the "hit right" event 
    tmc4361A_rstBits(tmc4361, TMC4361A_EVENTS, (TMC4361A_STOPL_EVENT_MASK | TMC4361A_STOPR_EVENT_MASK));
    delay(5);
    eventstate = readSwitchEvent(tmc4361);
  }
  // Wait until we hit the left limit switch
  while (eventstate == 0) {
    delay(5);
    eventstate = readSwitchEvent(tmc4361);
  }

  // Read the latched X_HOME
  return tmc4361A_readInt(tmc4361, TMC4361A_X_HOME);
}
