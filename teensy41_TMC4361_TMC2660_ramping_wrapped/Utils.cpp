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

void enableLimitSwitch(TMC4361ATypeDef *tmc4361A, uint8_t pol_lft, uint8_t pol_rht) {
  // Enable both the left and right limit switches
  pol_lft &= 1; // mask off unwanted bits
  pol_rht &= 1;
  // Set whether they are low active (set bit to 0) or high active (1)
  unsigned long pol_datagram = (pol_lft << TMC4361A_POL_STOP_LEFT_SHIFT) | (pol_rht << TMC4361A_POL_STOP_RIGHT_SHIFT);
  // Enable both left and right stops
  unsigned long en_datagram = TMC4361A_STOP_LEFT_EN_MASK | TMC4361A_STOP_RIGHT_EN_MASK;

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, pol_datagram);
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, en_datagram);

  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_L_MASK); // store position when we hit left bound
  tmc4361A_setBits(tmc4361A, TMC4361A_REFERENCE_CONF, TMC4361A_LATCH_X_ON_ACTIVE_R_MASK); // store position when we hit right bound

  return;
}

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

void setSRampParam(TMC4361ATypeDef *tmc4361A, uint8_t idx, int32_t param) {
  // Ensure idx is in range
  if (idx >= N_PARAM) {
    return;
  }

  tmc4361A->rampParam[idx] = param;
  sRampInit(tmc4361A);

  return;
}

void setMaxSpeed(TMC4361ATypeDef *tmc4361A, int32_t velocity) {
  velocity = tmc4361A_discardVelocityDecimals(velocity);
  setSRampParam(tmc4361A, VMAX_IDX, velocity);
  return;
}

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
int8_t move(TMC4361ATypeDef *tmc4361A, int32_t x_pos) {
  int32_t current = currentPosition(tmc4361A);
  int32_t target = current + x_pos;

  return moveTo(tmc4361A, target);
}
int32_t currentPosition(TMC4361ATypeDef *tmc4361A) {
  return tmc4361A_readInt(tmc4361A, TMC4361A_XACTUAL);
}
int32_t targetPosition(TMC4361ATypeDef *tmc4361A) {
  return tmc4361A_readInt(tmc4361A, TMC4361A_X_TARGET);
}
void setCurrentPosition(TMC4361ATypeDef *tmc4361A, int32_t position) {
  int32_t current = currentPosition(tmc4361A);
  int32_t dif = position - current;
  // change motor parameters on the teensy
  tmc4361A->xmax -= dif;
  tmc4361A->xmin -= dif;
  tmc4361A->xhome -= dif;
  // change motor parameters on the driver
  tmc4361A_writeInt(tmc4361A, TMC4361A_XACTUAL, position);

  return;
}
void stop(TMC4361ATypeDef *tmc4361A) {
  move(tmc4361A, 0);
  return;
}
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
int32_t mmToMicrosteps(float mm) {
  return mm * ((float)(MICROSTEPS * STEP_PER_REV)) / ((float)(PITCH));
}

float   microstepsTomm(int32_t microsteps) {
  float temp = microsteps * ((float)(PITCH)) / ((float)(MICROSTEPS * STEP_PER_REV));
  return temp;
}
