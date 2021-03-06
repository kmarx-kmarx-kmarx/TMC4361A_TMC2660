/*
    Find home, the max position, then take position arguments over serial and ramp to the setpoint.
    This code adds to TMC4361.h and TMC4361.cpp.

    Author: Kevin Marx
    Created on: 7/7/2022
*/

#include "TMC4361A.h" // These files come from the TCM4361A manufacturer
#include <SPI.h>

// Define the chip select pins for the TMC4361 connected to the microcontroller
#define N_MOTOR 1
const uint8_t pin_TMC4361_CS[4] = {41, 36, 35, 34};
// Define the clock pin. The clock speed of the TMC4361 is slower than the Teensy's clock speed
const uint8_t pin_TMC4361_CLK = 37;
// Define the CS pin for the 3.3 to 5V level shifter
const uint8_t pin_DAC80508_CS = 33;

ConfigurationTypeDef tmc4361_configs[4];
TMC4361ATypeDef tmc4361[4];
const uint32_t clk_Hz_TMC4361 = 16000000;

uint8_t prevstate[4] = {1, 1, 1, 1};

#define LEFT_SW 0b01
#define RGHT_SW 0b10

const int32_t vslow =  50000;
const int32_t vfast = 200000;
const int32_t xrng  = 100000;

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
  for (int i = 0; i < N_MOTOR; i++)
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
  for (int i = 0; i < N_MOTOR; i++) {
    // TMC4361
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_CLK_FREQ, clk_Hz_TMC4361);
    // SPI configuration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_SPIOUT_CONF, 0x4440108A);
    // cover datagram for TMC2660
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000900C3);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000A0000);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000C000A);
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000E00A0); // SDOFF = 1
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_COVER_LOW_WR, 0x000D001F); // current scaling: 0b11111 (max)
    // current open loop scaling
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_SCALE_VALUES, (0xFF << TMC4361A_HOLD_SCALE_VAL_SHIFT) +   // Set hold scale value (0 to 255)
                      (0xFF << TMC4361A_DRV2_SCALE_VAL_SHIFT) +   // Set DRV2 scale  (0 to 255)
                      (0xFF << TMC4361A_DRV1_SCALE_VAL_SHIFT) +   // Set DRV1 scale  (0 to 255)
                      (0xFF << TMC4361A_BOOST_SCALE_VAL_SHIFT)); // Set boost scale (0 to 255)

    tmc4361A_setBits(&tmc4361[i], TMC4361A_CURRENT_CONF, TMC4361A_DRIVE_CURRENT_SCALE_EN_MASK); // keep drive current scale
    tmc4361A_setBits(&tmc4361[i], TMC4361A_CURRENT_CONF, TMC4361A_HOLD_CURRENT_SCALE_EN_MASK);  // keep hold current scale

    enableLimitSwitch(&tmc4361[i]); // enable limit switch reading

    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_L_MASK); // store position when we hit left bound
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_R_MASK); // store position when we hit right bound

    // enable homing
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF, 0b1100 << TMC4361A_HOME_EVENT_SHIFT);// define home behavior
    tmc4361A_setBits(&tmc4361[i], TMC4361A_REFERENCE_CONF, TMC4361A_STOP_LEFT_IS_HOME_MASK);    // use stop left as home
    tmc4361A_setBits(&tmc4361[i], TMC4361A_HOME_SAFETY_MARGIN, 1 << 6);                         // have a safety margin around home
  }

  // Home all the motors
  uint8_t eventstate = 0;
  for (int i = 0; i < N_MOTOR; i++) {
    Serial.print("Homing ");
    Serial.println(pin_TMC4361_CS[i]);
    homing_lft(&tmc4361[i], vslow, vfast);
    Serial.print("x_home: ");
    Serial.println(tmc4361[i].xhome);
    // Homing left -- xmin and xhome are the same
    tmc4361[i].xmin = tmc4361[i].xhome;
    // Find max right value
    tmc4361A_rotate(&tmc4361[i], vslow);
    while (eventstate != RGHT_SW) {
      delay(5);
      eventstate = readSwitchEvent(&tmc4361[i]);
    }
    tmc4361[i].xmax = tmc4361A_readInt(&tmc4361[i], TMC4361A_X_LATCH_RD);
    Serial.print("xmax: ");
    Serial.println(tmc4361[i].xmax);


  }

  Serial.print("Enter 'p' then a motor index 0 to ");
  Serial.print(N_MOTOR - 1);
  Serial.print(" and a number between 0 and ");
  Serial.print(xrng);
  Serial.println(" separated by spaces in a single line");

  for (int i = 0; i < N_MOTOR; i++) {
    // set target position to actual position - no motion initially
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_X_TARGET, tmc4361[i].xmax);
    // ramp mode
    tmc4361A_setBits(&tmc4361[i], TMC4361A_RAMPMODE, 0b110); // positioning mode, s-shaped ramp
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_BOW1, 0x0001FFFF); // determines the value which increases the absolute acceleration value.
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_BOW2, 0x001FFFFF); // determines the value which decreases the absolute acceleration value.
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_BOW3, 0x001FFFFF); // determines the value which increases the absolute deceleration value.
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_BOW4, 0x0001FFFF); // determines the value which decreases the absolute deceleration value.
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_AMAX, 0x001FFFFF); // max acceleration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_DMAX, 0x001FFFFF); // max decelleration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_ASTART, 0); // initial acceleration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_DFINAL, 0); // final decelleration
    tmc4361A_writeInt(&tmc4361[i], TMC4361A_VMAX, 0x04FFFF00); // max speed
  }
}

void loop() {
  int32_t index, target;
  char cmd;

  // Wait until serial is available

  if (Serial.available()) {
    cmd = Serial.read();
    if (cmd == 'p') {
      index = Serial.parseInt();
      target = Serial.parseInt();
      // Display error if out of bounds
      if (index < 0 || index >= N_MOTOR) {
        Serial.print("Index ");
        Serial.print(index);
        Serial.println(" is out of bounds");
        return;
      }
      // Otherwise, read next value
      // Display error if out of bounds
      if (target < 0 || target > xrng) {
        Serial.print("Target ");
        Serial.print(target);
        Serial.println(" is out of bounds");
        return;
      }
      Serial.print("Idx: ");
      Serial.println(index);
      Serial.print("Target: ");
      Serial.println(target);
      Serial.print("Motor CS pin: ");
      Serial.println(tmc4361[index].config->channel);
      Serial.print("Motor xmax: ");
      Serial.println(tmc4361[index].xmax);
      Serial.print("Motor xmin: ");
      Serial.println(tmc4361[index].xmin);
      Serial.print("Actual Target: ");
      int32_t diff = tmc4361[index].xmax - tmc4361[index].xmin;
      float scaler = (float)diff / (float)xrng;
      target = target * scaler + tmc4361[index].xmin;
      Serial.println(target);
      Serial.print("Current Position: ");
      Serial.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_XACTUAL));
      tmc4361A_readInt(&tmc4361[index], TMC4361A_EVENTS);
      tmc4361A_writeInt(&tmc4361[index], TMC4361A_X_TARGET, target);
      tmc4361A_readInt(&tmc4361[index], TMC4361A_EVENTS);
    }
  }

  // Show results
  for (int i = 0; i < N_MOTOR; i++) {
    target  = tmc4361A_readInt(&tmc4361[i], TMC4361A_STATUS) & TMC4361A_TARGET_REACHED_MASK;
    if(target != 0 && prevstate[i] == 0){
      Serial.print("Motor with CS pin ");
      Serial.print(tmc4361[i].config->channel);
      Serial.println(" has reached its target");
    }
    prevstate[i] = target;
    // todo: implement using events register
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

void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat) {
  // Set the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  uint32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // OR with the bits we want to set
  datagram |= dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address, datagram);

  return;
}
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat) {
  // Reset the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  uint32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // AND with the bits with the negation of the bits we want to clear
  datagram &= ~dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address, datagram);

  return;
}

void enableLimitSwitch(TMC4361ATypeDef *tmc4361A) {
  // Enable both the left and right limit switches
  // Set whether they are low active (set bit to 0) or high active (1)
  unsigned long pol_datagram = (1 << TMC4361A_POL_STOP_LEFT_SHIFT) | (1 << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  unsigned long en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, pol_datagram);
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, en_datagram);

  return;
}

uint8_t readLimitSwitches(TMC4361ATypeDef *tmc4361A) {
  // Read both limit switches. Set bit 0 if the left switch is pressed and bit 1 if the right switch is pressed
  unsigned long i_datagram = 0;
  unsigned long address = TMC4361A_STATUS;

  // Get the datagram
  i_datagram = tmc4361A_readInt(tmc4361A, address);
  // Mask off everything except the button states
  i_datagram &= (TMC4361A_STOPL_ACTIVE_F_MASK | TMC4361A_STOPR_ACTIVE_F_MASK);
  // Shift the button state down to bits 0 and 1
  i_datagram >>= TMC4361A_STOPL_ACTIVE_F_SHIFT;
  // Get rid of the high bits
  uint8_t result = i_datagram & 0xff;

  return result;
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

void homing_lft(TMC4361ATypeDef *tmc4361A, int32_t v_slow, int32_t v_fast) {
  // Homing routine
  // First, check if we are already at a limit switch
  uint8_t eventstate = readSwitchEvent(tmc4361);
  if (eventstate == LEFT_SW) {
    Serial.println("At left switch already!");
    // If we are at the left limit switch, go right until we are no longer hitting it
    tmc4361A_rotate(tmc4361, v_fast);
    while (eventstate != 0) {
      // Try clearing the event
      tmc4361A_rstBits(tmc4361, TMC4361A_EVENTS, (TMC4361A_STOPL_EVENT_MASK | TMC4361A_STOPR_EVENT_MASK));
      delay(5);
      eventstate = readSwitchEvent(tmc4361);
    }
    // Wait until we aren't hitting the left limit switch
    eventstate = readLimitSwitches(tmc4361);
    while (eventstate == LEFT_SW) {
      delay(5);
      eventstate = readLimitSwitches(tmc4361);
    }
    delay(200);
    Serial.println("Backed up from left switch");
  }
  // Now we are either at the right limit switch, somewhere in the middle, or somewhere close to the left limit switch
  // Move back to the left slowly
  Serial.println("Moving left");
  tmc4361A_rotate(tmc4361, -v_slow);
  eventstate = readLimitSwitches(tmc4361);
  while (eventstate == RGHT_SW) {
    // Try clearing the "hit right" event
    tmc4361A_rstBits(tmc4361, TMC4361A_EVENTS, (TMC4361A_STOPL_EVENT_MASK | TMC4361A_STOPR_EVENT_MASK));
    delay(50);
    eventstate = readLimitSwitches(tmc4361);
  }
  Serial.println("Cleared events");
  // Enable home tracking
  tmc4361A_setBits(tmc4361, TMC4361A_REFERENCE_CONF, TMC4361A_START_HOME_TRACKING_MASK);
  // Wait until we hit the left limit switch
  while (eventstate != LEFT_SW) {
    delay(5);
    eventstate = readSwitchEvent(tmc4361);
  }
  // Read the latched X_HOME
  tmc4361->xhome = tmc4361A_readInt(tmc4361, TMC4361A_X_HOME);

  return;
}
