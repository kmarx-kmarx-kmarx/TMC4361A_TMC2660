/*
   Utils.cpp
   This file contains higher-level functions for interacting with the TMC4362A.
   Some functions are low-level helpers and do not need to be accessed directly by the user
    User-facing functions:
      tcm4361A_tcm2660_init:   Initialize the TCM4361A and TCM2660
      setMaxSpeed:             Write the target velocity to the TCM4361A in units microsteps per second
      setMaxAcceleration:      Write the maximum acceleration in units microsteps per second squared
      moveTo:                  Move to the target absolute position in units microsteps
      move:                    Move to a position relative to the current position in units microsteps
      currentPosition:         Return the current position in units microsteps
      targetPosition:          Return the target position in units microsteps
      setCurrentPosition:      Set the current position to a specific value in units microsteps
      stop:                    Halt operation by setting the target position to the current position
      isRunning:               Returns true if the motor is moving
      mmToMicrosteps:          Convert from millimeters to units microsteps
      microstepsTomm:          Convert from microsteps to units millimeters
      enableLimitSwitch:       Enables reading from limit switches and using limit switches as automatic stop indicators.
      enableHomingLimit:       Enables using the limit switch or homing
      readLimitSwitches:       Read limit switch current state

    For internal use:
      tmc4361A_readWriteArray: Used for low-level SPI communication with the TMC4361A
      tmc4361A_setBits:        Implements some of the features of tmc4361A_readWriteCover in an easier to use way; it sets bits in a register without disturbing the other bits
      tmc4361A_setBits:        Implements some of the features of tmc4361A_readWriteCover in an easier to use way; it clears bits in a register without disturbing the other bits
      readSwitchEvent:         Read events created by the limit switches
      sRampInit:               Write all parameters for the s-shaped ramp
      setSRampParam:           Set and write an individual parameter for the s-shaped ramp
*/
#include "Utils.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: tmc4361A_readWriteArray() sends a number of bytes to a target device over SPI. Functions in TMC4316A.cpp depend on this function.
  
  OPERATION:   This function mediates a SPI transaction by first setting the CS pin of the target device low, then sending bytes from an array one at a time over SPI and storing the data back into the original array. Once the transaction is over, the CS pin is brought high again.
  
  ARGUMENTS: 
      uint8_t channel: CS pin number
      uint8_t *data:   Pointer to data array
      size_t length:   Number of bytes in the data array
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      uint8_t *data: Values in this array are overwritten 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: SPI.h
  -----------------------------------------------------------------------------
*/ 
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


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: tmc4361A_setBits() sets bits in a register without affecting the other bits.
  
  OPERATION:   We first read the register data then OR the register with the bits we want to set. Then, it writes the data to the address.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint8_t address:           Address of the register we want to write to
      int32_t dat:               Data we want to write to the array
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      int32_t datagram: Used to hold both the data read from the register and the data we want to write to the register
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat) {
  // Set the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  int32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // OR with the bits we want to set
  datagram |= dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address, datagram);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: tmc4361A_rstBits() clears bits in a register without affecting the other bits.
  
  OPERATION:   We first read the register data then AND the register with the negation of bits we want to set. Then, it writes the data to the address.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint8_t address:           Address of the register we want to write to
      int32_t dat:               Data we want to write to the array
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      int32_t datagram: Used to hold both the data read from the register and the data we want to write to the register
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat) {
  // Reset the bits in dat without disturbing any other bits in the register
  // Read the bits already there
  int32_t datagram = tmc4361A_readInt(tmc4361A, address);
  // AND with the bits with the negation of the bits we want to clear
  datagram &= ~dat;
  // Write
  tmc4361A_writeInt(tmc4361A, address, datagram);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: tcm4361A_tcm2660_init() initializes the TCM4361A and TCM2660
  
  OPERATION:   We write several bytes to the two ICs to configure their behaviors.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint32_t clk_Hz_TMC4361:   Clock frequency we are driving the ICs at
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void tcm4361A_tcm2660_init(TMC4361ATypeDef *tmc4361A, uint32_t clk_Hz_TMC4361) {
  // TMC4361
  tmc4361A_writeInt(tmc4361A, TMC4361A_CLK_FREQ, clk_Hz_TMC4361);
  // SPI configuration
  tmc4361A_writeInt(tmc4361A, TMC4361A_SPIOUT_CONF, 0x4440108A);
  // cover datagram for TMC2660
  tmc4361A_writeInt(tmc4361A, TMC4361A_COVER_LOW_WR, 0x000900C3);
  tmc4361A_writeInt(tmc4361A, TMC4361A_COVER_LOW_WR, 0x000A0000);
  tmc4361A_writeInt(tmc4361A, TMC4361A_COVER_LOW_WR, 0x000C000A);
  tmc4361A_writeInt(tmc4361A, TMC4361A_COVER_LOW_WR, 0x000E00A0); // SDOFF = 1 -> SPI mode
  tmc4361A_writeInt(tmc4361A, TMC4361A_COVER_LOW_WR, 0x000D0000 | TCM2660_CSCALE); // enable SFILT; SG2 = 0; set current scale
  // current open loop scaling
  tmc4361A_writeInt(tmc4361A, TMC4361A_SCALE_VALUES, (TMC4361A_HOLD_SCALE_VAL << TMC4361A_HOLD_SCALE_VAL_SHIFT) +   // Set hold scale value (0 to 255)
                    (TMC4361A_DRV2_SCALE_VAL << TMC4361A_DRV2_SCALE_VAL_SHIFT) +   // Set DRV2 scale  (0 to 255)
                    (TMC4361A_DRV1_SCALE_VAL << TMC4361A_DRV1_SCALE_VAL_SHIFT) +   // Set DRV1 scale  (0 to 255)
                    (TMC4361A_BOOST_SCALE_VAL << TMC4361A_BOOST_SCALE_VAL_SHIFT)); // Set boost scale (0 to 255)

  tmc4361A_setBits(tmc4361A, TMC4361A_CURRENT_CONF, TMC4361A_DRIVE_CURRENT_SCALE_EN_MASK); // keep drive current scale
  tmc4361A_setBits(tmc4361A, TMC4361A_CURRENT_CONF, TMC4361A_HOLD_CURRENT_SCALE_EN_MASK);  // keep hold current scale

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: enableLimitSwitch() enables the left and right limit switches
  
  OPERATION:   We format the switch polarity variables into a datagram and send it to the TCM4361. We then enable position latching when the limit switch is hit
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint8_t pol_lft:           Polarity of the left switch - 0 if active low, 1 if active high
      uint8_t pol_rht:           Polarity of right
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      uint32_t pol_datagram, en_datagram: store datagrams to write to the TCM4361.
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void enableLimitSwitch(TMC4361ATypeDef *tmc4361A, uint8_t pol_lft, uint8_t pol_rht) {
  // Enable both the left and right limit switches
  pol_lft &= 1; // mask off unwanted bits
  pol_rht &= 1;
  // Set whether they are low active (set bit to 0) or high active (1)
  uint32_t pol_datagram = (pol_lft << TMC4361A_POL_STOP_LEFT_SHIFT) | (pol_rht << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  uint32_t en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, pol_datagram);
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, en_datagram);

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_L_MASK); // store position when we hit left bound
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_R_MASK); // store position when we hit right bound

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: enableHomingLimit() enables using either the left or right limit switch for homing
  
  OPERATION:   We format the switch polarity variables and target switch into a datagram and send it to the TCM4361. We then enable position latching when the limit switch is hit
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint8_t sw:                Which switch we are using for homing - left or right
      uint8_t pol_lft:           Polarity of the left switch - 0 if active low, 1 if active high
      uint8_t pol_rht:           Polarity of right
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void enableHomingLimit(TMC4361ATypeDef *tmc4361A, uint8_t sw, uint8_t pol_lft, uint8_t pol_rht) {
  if (sw == LEFT_SW) {
    if (pol_lft != 0) {
      // If the left switch is active high, HOME_REF = 0 indicates positive direction in reference to X_HOME
      tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, 0b1100 << TMC4361A_HOME_EVENT_SHIFT);
    }
    else {
      // if active low, HOME_REF = 0 indicates negative direction in reference to X_HOME
      tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, 0b0011 << TMC4361A_HOME_EVENT_SHIFT);
    }
    // use stop left as home
    tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_STOP_LEFT_IS_HOME_MASK);
  }
  else {
    if (pol_rht != 0) {
      // If the right switch is active high, HOME_REF = 0 indicates positive direction in reference to X_HOME
      tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, 0b0011 << TMC4361A_HOME_EVENT_SHIFT);
    }
    else {
      // if active low, HOME_REF = 0 indicates negative direction in reference to X_HOME
      tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, 0b1100 << TMC4361A_HOME_EVENT_SHIFT);
    }
    // use stop right as home
    tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_STOP_RIGHT_IS_HOME_MASK);
  }
  // have a safety margin around home
  tmc4361A_setBits(tmc4361A, TMC4361A_HOME_SAFETY_MARGIN, 1 << 2);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: readLimitSwitches() reads the limit switches and returns their state in the two low bits of a byte.
                00 - both not pressed
                01 - left switch pressed
                10 - right
                11 - both
  
  OPERATION:   We read the status register, mast the irrelevant bits, and shift the relevant bits down
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: 
      uint8_t result: Byte containing the button state in the last two bits
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      uint32_t i_datagram: data received from the TCM4361A
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
uint8_t readLimitSwitches(TMC4361ATypeDef *tmc4361A) {
  // Read both limit switches. Set bit 0 if the left switch is pressed and bit 1 if the right switch is pressed
  uint32_t i_datagram = 0;

  // Get the datagram
  i_datagram = tmc4361A_readInt(tmc4361A, TMC4361A_STATUS);
  // Mask off everything except the button states
  i_datagram &= (TMC4361A_STOPL_ACTIVE_F_MASK | TMC4361A_STOPR_ACTIVE_F_MASK);
  // Shift the button state down to bits 0 and 1
  i_datagram >>= TMC4361A_STOPL_ACTIVE_F_SHIFT;
  // Get rid of the high bits
  uint8_t result = i_datagram & 0xff;

  return result;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: readLimitSwitches() checks whether there was a switch event and returns their state in the two low bits of a byte.
                00 - both not pressed
                01 - left switch pressed
                10 - right
                11 - both
  
  OPERATION:   We read the events register, mast the irrelevant bits, and shift the relevant bits down
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: 
      uint8_t result: Byte containing the button state in the last two bits
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      uint32_t i_datagram: data received from the TCM4361A
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
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

void homeLeft(TMC4361ATypeDef *tmc4361A, int32_t v_slow, int32_t v_fast) {
  // Homing routine
  // First, move right a little, making sure not to hit the right
  uint8_t eventstate = readLimitSwitches(tmc4361A);
  if (eventstate != RGHT_SW) {
    // move right
    tmc4361A_readInt(tmc4361A, TMC4361A_EVENTS);
    tmc4361A_rotate(tmc4361A, v_fast);
    tmc4361A_readInt(tmc4361A, TMC4361A_EVENTS);
    // Wait until we aren't hitting the left limit switch
    eventstate = readLimitSwitches(tmc4361A);
    while (eventstate == LEFT_SW) {
      readSwitchEvent(tmc4361A);
      delay(5);
      eventstate = readLimitSwitches(tmc4361A);
    }
    // Keep going a little to have clearance from left. It's OK if we hit the right limit switch.
    delay(200);
  }
  // Now we are either at the right limit switch, somewhere in the middle, or somewhere close to the left limit switch
  // Move back to the left slowly
  tmc4361A_rotate(tmc4361A, -v_slow);
  // In case we are at the right switch, clear the event
  eventstate = readLimitSwitches(tmc4361A);
  while (eventstate == RGHT_SW) {
    // Try clearing the "hit right" event
    readSwitchEvent(tmc4361A);
    delay(5);
    eventstate = readLimitSwitches(tmc4361A);
  }
  // Wait until we hit the left limit switch
  while (eventstate != LEFT_SW) {
    delay(5);
    eventstate = readSwitchEvent(tmc4361A);
  }
  // Set the current position to 0; this is the home position
  tmc4361A_writeInt(tmc4361A, TMC4361A_XACTUAL, 0);
  tmc4361A_writeInt(tmc4361A, TMC4361A_X_HOME, 0);
  tmc4361A->xhome = tmc4361A_readInt(tmc4361A, TMC4361A_X_HOME);
  tmc4361A->xmin = tmc4361A->xhome;

  return;
}

void findRight(TMC4361ATypeDef *tmc4361A, int32_t v_slow) {
  // Move to the right to find xmax
  tmc4361A_rotate(tmc4361A, v_slow);
  uint8_t eventstate;
  while (eventstate != RGHT_SW) {
    delay(5);
    eventstate = readSwitchEvent(tmc4361A );
  }
  tmc4361A->xmax = tmc4361A_readInt(tmc4361A, TMC4361A_X_LATCH_RD);
  tmc4361A_writeInt(tmc4361A, TMC4361A_X_TARGET, tmc4361A->xmax);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: sRampInit() writes the ramp parameters to the TMC4361A.
  
  OPERATION:   We read the data from the shared struct and write them one at a time to the TCM4361A
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from and written to the struct
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void sRampInit(TMC4361ATypeDef *tmc4361A) {
  tmc4361A_setBits(tmc4361A, TMC4361A_RAMPMODE, 0b110); // positioning mode, s-shaped ramp
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW1, tmc4361A->rampParam[BOW1_IDX]); // determines the value which increases the absolute acceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW2, tmc4361A->rampParam[BOW2_IDX]); // determines the value which decreases the absolute acceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW3, tmc4361A->rampParam[BOW3_IDX]); // determines the value which increases the absolute deceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW4, tmc4361A->rampParam[BOW4_IDX]); // determines the value which decreases the absolute deceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_AMAX, tmc4361A->rampParam[AMAX_IDX]); // max acceleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_DMAX, tmc4361A->rampParam[DMAX_IDX]); // max decelleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_ASTART, tmc4361A->rampParam[ASTART_IDX]); // initial acceleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_DFINAL, tmc4361A->rampParam[DFINAL_IDX]); // final decelleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_VMAX, tmc4361A->rampParam[VMAX_IDX]); // max speed

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setSRampParam() writes a single ramp parameter to the TMC4361A.
  
  OPERATION:   We change a variable in the shared struct and call sRampInit() to write the data.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      uint8_t idx:               Which parameter to change
      int32_t param:             The new value of the parameter
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from and written to the struct
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void setSRampParam(TMC4361ATypeDef *tmc4361A, uint8_t idx, int32_t param) {
  // Ensure idx is in range
  if (idx >= N_PARAM) {
    return;
  }

  tmc4361A->rampParam[idx] = param;
  sRampInit(tmc4361A);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setMaxSpeed() writes a single ramp parameter to the TMC4361A.
  
  OPERATION:   We first verify the new velocity value is in bounds, then we change the variable in the shared struct and call sRampInit() to write the data.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      int32_t velocity:          The velocity in units microsteps per second
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from and written to the struct
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
void setMaxSpeed(TMC4361ATypeDef *tmc4361A, int32_t velocity) {
  velocity = tmc4361A_discardVelocityDecimals(velocity);
  setSRampParam(tmc4361A, VMAX_IDX, velocity);
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setMaxAcceleration() writes a single ramp parameter to the TMC4361A.
  
  OPERATION:   We first verify the new acceleration value is in bounds, then we change the variable in the shared struct. We also change bows 1 through 4 to ensure we hit max acceleration. Then we call sRampInit() to write the data.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      int32_t acceleration:      The acceleration in microsteps per second squared
  
  RETURNS: 
      uint8_t err: Return ERR_OUT_OF_RANGE or NO_ERR depending on what happened.
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from and written to the struct
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
int8_t setMaxAcceleration(TMC4361ATypeDef *tmc4361A, uint32_t acceleration) {
  if (acceleration > ((1 << 22) - 1)) {
    return ERR_OUT_OF_RANGE;
  }
  // Calculate what the jerks should be given amax and vmax
  // Minimize the time a = amax under the constraint BOW1 = BOW2 = BOW3 = BOW4
  float bowval = (float)acceleration * (float)acceleration / (float)tmc4361A->rampParam[VMAX_IDX];
  int32_t bow = bowval;
  bow = min((1 << 24) - 1, bow);

  tmc4361A->rampParam[BOW1_IDX] = bow;
  tmc4361A->rampParam[BOW2_IDX] = bow;
  tmc4361A->rampParam[BOW3_IDX] = bow;
  tmc4361A->rampParam[BOW4_IDX] = bow;
  tmc4361A->rampParam[AMAX_IDX] = acceleration;
  tmc4361A->rampParam[DMAX_IDX] = acceleration;

  sRampInit(tmc4361A);

  return NO_ERR;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: moveTo() writes the new setpoint to the TMC4361A.
  
  OPERATION:   We first verify the new position value is in bounds, then we clear the event register, send the data to the TMC4613, clear the event register again, and read the current position to refresh it.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      int32_t x_pos:             The target position in microsteps
  
  RETURNS: 
      uint8_t err: Return ERR_OUT_OF_RANGE or NO_ERR depending on what happened.
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
int8_t moveTo(TMC4361ATypeDef *tmc4361A, int32_t x_pos) {
  // ensure we are in positioning mode

  TMC4361A_FIELD_WRITE(tmc4361A, TMC4361A_RAMPMODE, TMC4361A_OPERATION_MODE_MASK, TMC4361A_OPERATION_MODE_SHIFT, 1);
  if (x_pos < tmc4361A->xmin || x_pos > tmc4361A->xmax) {
    return ERR_OUT_OF_RANGE;
  }
  // Read events before and after to clear the register
  tmc4361A_readInt(tmc4361A, TMC4361A_EVENTS);
  tmc4361A_writeInt(tmc4361A, TMC4361A_X_TARGET, x_pos);
  tmc4361A_readInt(tmc4361A, TMC4361A_EVENTS);
  // Read X_ACTUAL to get it to refresh
  tmc4361A_readInt(tmc4361A, TMC4361A_XACTUAL);

  return NO_ERR;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: move() writes the new setpoint relative to the current position to the TMC4361A.
  
  OPERATION:   We first convert the relative position to an absolute position and call moveTo()
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      int32_t x_pos:             The target position in microsteps
  
  RETURNS: 
      uint8_t err: Return ERR_OUT_OF_RANGE or NO_ERR depending on what happened.
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      int32_t current: current position
      int32_t target:  calculated absolute position
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
int8_t move(TMC4361ATypeDef *tmc4361A, int32_t x_pos) {
  int32_t current = currentPosition(tmc4361A);
  int32_t target = current + x_pos;

  return moveTo(tmc4361A, target);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: currentPosition() reads the current position
  
  OPERATION:   We read the data in the XACTUAL register
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: 
      int32_t xpos: The position in units microsteps
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/ 
int32_t currentPosition(TMC4361ATypeDef *tmc4361A) {
  return tmc4361A_readInt(tmc4361A, TMC4361A_XACTUAL);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: targetPosition() reads the target position
  
  OPERATION:   We read the data in the X_TARGET register
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: 
      int32_t xpos: The position in units microsteps
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/
int32_t targetPosition(TMC4361ATypeDef *tmc4361A) {
  return tmc4361A_readInt(tmc4361A, TMC4361A_X_TARGET);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setCurrentPosition() overwrites the current position with a new position. This doesn't change the physical position of the motor.
  
  OPERATION:   We change the motor driver struct varaibles to reflect the new offset and update the TMC4361A.
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      int32_t current: stores the current position
      int32_t diff:    stores the difference between the current and target position
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from and written to the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/
void setCurrentPosition(TMC4361ATypeDef *tmc4361A, int32_t position) {
  int32_t current = currentPosition(tmc4361A);
  int32_t dif = position - current;
  // change motor parameters on the teensy
  tmc4361A->xmax -= dif;
  tmc4361A->xmin -= dif;
  tmc4361A->xhome -= dif;
  // change motor parameters on the driver
  tmc4361A_writeInt(tmc4361A, TMC4361A_XACTUAL, position);
  // Ensure the motor doesn't move
  moveTo(tmc4361A, position);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: stop() halts motor motion
  
  OPERATION:   We move the motor to position 0 relative to its current position
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: None
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/
void stop(TMC4361ATypeDef *tmc4361A) {
  move(tmc4361A, 0);
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: isRunning() checks whether the motor is moving and returns either true or false
  
  OPERATION:   We check if the motor hit its target. If so, we return true. We then check the acceleration and velocity; if they are both zero we also return true. Then, otherwise, return false
  
  ARGUMENTS: 
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
  
  RETURNS: 
      bool moving: true if moving, false otherwise
  
  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively
  
  LOCAL VARIABLES: 
      int32_t stat_reg: The status register
  
  SHARED VARIABLES: 
      TMC4361ATypeDef *tmc4361A: Values are read from the struct 
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: TCM4316A.h
  -----------------------------------------------------------------------------
*/
bool isRunning(TMC4361ATypeDef *tmc4361A) {
  int32_t stat_reg = tmc4361A_readInt(tmc4361A, TMC4361A_STATUS);

  // We aren't running if target is reached OR (velocity = 0 and acceleration == 0)
  if (stat_reg & TMC4361A_TARGET_REACHED_MASK != 0) {
    return true;
  }
  stat_reg &= (TMC4361A_VEL_STATE_F_MASK | TMC4361A_RAMP_STATE_F_MASK);
  if (stat_reg == 0) {
    return true;
  }

  // Otherwise, return false
  return false;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: mmToMicrosteps() convers a position in units mm to a position in units microsteps
  
  OPERATION:   We multiply the mm by a conversion factor and cast to int32_t
  
  ARGUMENTS: 
      float mm: posiiton in mm
  
  RETURNS: 
      int32_t microsteps: the position in units microsteps
  
  INPUTS / OUTPUTS: None
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: None
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
int32_t mmToMicrosteps(float mm) {
  return mm * ((float)(MICROSTEPS * STEP_PER_REV)) / ((float)(PITCH));
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: microstepsTomm() convers a position in units microsteps to a position in units mm
  
  OPERATION:   We cast the microsteps to a float and multiply the microsteps by a conversion factor
  
  ARGUMENTS: 
      int32_t microsteps: the position in units microsteps

  
  RETURNS: 
      float mm: posiiton in mm

  
  INPUTS / OUTPUTS: None
  
  LOCAL VARIABLES: None
  
  SHARED VARIABLES: None
  
  GLOBAL VARIABLES: None
  
  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
float   microstepsTomm(int32_t microsteps) {
  float temp = microsteps * ((float)(PITCH)) / ((float)(MICROSTEPS * STEP_PER_REV));
  return temp;
}
