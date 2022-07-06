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
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR | TMC_WRITE_BIT, 0x000900C3);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR | TMC_WRITE_BIT, 0x000A0000);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR | TMC_WRITE_BIT, 0x000C000A);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR | TMC_WRITE_BIT, 0x000E00A0); // SDOFF = 1
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR | TMC_WRITE_BIT, 0x000D001F); // current scaling: 0b11111 (max)
    // current open loop scaling
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_SCALE_VALUES | TMC_WRITE_BIT, 0x00000000); //no current for hold or drive
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_CURRENT_CONF | TMC_WRITE_BIT, TMC4361A_HOLD_CURRENT_SCALE_EN_MASK | TMC4361A_DRIVE_CURRENT_SCALE_EN_MASK); // keep hold and drive current at 0

    enableLimitSwitch(pin_TMC4361_CS[i]); // enable limit switch reading
  }
}

void loop() {
  // Run each motor one at a time
  uint8_t switchstate = 0;
  uint8_t prevstate  = 0;

  for (int i = 0; i < 1; i++) {
    Serial.println("moving " + String(pin_TMC4361_CS[i]));

    // Idle until we hit a switch
    switchstate = readLimitSwitches(pin_TMC4361_CS[i]);
    while (switchstate == 0) {
      // Delay a little
      delay(5);
      // Update switchstate
      switchstate = readLimitSwitches(pin_TMC4361_CS[i]);
    }
    Serial.print("Hit switch: ");
    if (switchstate == 1) {
      Serial.println("left");
    }
    else {
      Serial.println("right");
    }
    prevstate = switchstate;
    TMC4361_transceiveData(pin_TMC4361_CS[i], TMC4361A_STATUS | TMC_WRITE_BIT, 0);

    // Repeat going backwards
    switchstate = readLimitSwitches(pin_TMC4361_CS[i]);
    while (switchstate == 0 || switchstate == prevstate) {
      delay(5);
      switchstate = readLimitSwitches(pin_TMC4361_CS[i]);
    }
    Serial.print("Hit switch: ");
    if (switchstate == 1) {
      Serial.println("left");
    }
    else {
      Serial.println("right");
    }

    prevstate = 0;
    switchstate = 0;
    TMC4361_transceiveData(pin_TMC4361_CS[i], TMC4361A_STATUS | TMC_WRITE_BIT, 0);
    delay(2000);
  }
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

void enableLimitSwitch(TMC4361ATypeDef *tmc4361A) {
  // Enable both the left and right limit switches
  // Set whether they are low active (set bit to 0) or high active (1)
  unsigned long pol_datagram = (1 << TMC4361A_POL_STOP_LEFT_SHIFT) | (1 << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  unsigned long en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  tmc4361A_writeInt(tmc4361A, TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, pol_datagram);
  tmc4361A_writeInt(tmc4361A, TMC4361A_REFERENCE_CONF | TMC_WRITE_BIT, en_datagram | pol_datagram);
  
  return;
}

unsigned long readLimitSwitches(uint8_t pin_CS) {
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
