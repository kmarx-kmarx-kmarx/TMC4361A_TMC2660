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
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW1, tmc4361A->rampParam[0]); // determines the value which increases the absolute acceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW2, tmc4361A->rampParam[1]); // determines the value which decreases the absolute acceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW3, tmc4361A->rampParam[2]); // determines the value which increases the absolute deceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_BOW4, tmc4361A->rampParam[3]); // determines the value which decreases the absolute deceleration value.
  tmc4361A_writeInt(tmc4361A, TMC4361A_AMAX, tmc4361A->rampParam[4]); // max acceleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_DMAX, tmc4361A->rampParam[5]); // max decelleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_ASTART, tmc4361A->rampParam[6]); // initial acceleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_DFINAL, tmc4361A->rampParam[7]); // final decelleration
  tmc4361A_writeInt(tmc4361A, TMC4361A_VMAX, tmc4361A->rampParam[8]); // max speed

  return;
}
