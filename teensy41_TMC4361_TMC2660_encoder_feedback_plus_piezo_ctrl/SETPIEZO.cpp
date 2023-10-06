#include "SETPIEZO.h"
//
//int32_t prev_target = s32_MAX;
//uint16_t prev_setpoint = 0;
/*
  -----------------------------------------------------------------------------
  DESCRIPTION: piezo_set_offset() sets the DAC to adjust the stage position by a small amount using the piezo.

  OPERATION:

  ARGUMENTS:
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      DACx050x *dac: Pointer to the DAC class
      int32_t initial_pos: the initial encoder reading
      int32_t mstep_expand: the amount of microsteps/encoder steps we want to expand by
      uint32_t err: Tolerated error from expected in units microsteps
      bool correct: do error correction step if true

  RETURNS:
      bool success: Return true if we were able to get below the err without exceeding the number of retries.

  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively. The DAC outputs a value which in turn moves the piezo.

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      TMC4361ATypeDef *tmc4361A: Values are read from the struct

  GLOBAL VARIABLES: None

  DEPENDENCIES: tmc4316A.h
                DACx050x.h
  -----------------------------------------------------------------------------
*/
bool piezo_set_offset(TMC4361ATypeDef *tmc4361A, DACx050x *dac, int32_t initial_pos, int32_t mstep_expand, uint32_t err, bool correct) {
  err = err/2;
  err = max(err, 1);
  uint32_t t0 = millis();
  int32_t new_target = initial_pos + mstep_expand;
  int32_t reading = tmc4361A_read_encoder(tmc4361A, 0);
  int32_t initial_target = float(abs(mstep_expand)) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
  int32_t undershoot_range_ustep = float(PIEZO_DAC_UNDERSHOOT) * float(tmc4361A->dac_fullscale_msteps) / float(u16_MAX);
  uint16_t dac_target;
  int16_t step_size;

  // check if there is a DAC - return out
  if (tmc4361A->dac_idx == NO_DAC)
    return false;
  // If our current position is too far out of range, move open loop to the target
  reading = reading - initial_target;
  reading = abs(reading);
  if (float(reading) >= float(undershoot_range_ustep)) {
    if (initial_target <= PIEZO_DAC_UNDERSHOOT)
      initial_target = 0;
    else
      initial_target -= PIEZO_DAC_UNDERSHOOT;
    dac_target = constrain(initial_target, 1, u16_MAX - 1);
    dac->output(tmc4361A->dac_idx, dac_target);
    delay(PIEZO_SETTLE_TIME_MS);
  }
  // otherwise, get the previous dac position
  else{
    dac_target = dac->get_setpoint(tmc4361A->dac_idx);
  }

  // End here if we aren't doing error correction
  if (!correct) {
    initial_target = float(abs(mstep_expand)) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
    dac_target = constrain(initial_target, 1, u16_MAX - 1);
    dac->output(tmc4361A->dac_idx, dac_target);
    delay(PIEZO_SETTLE_TIME_MS);
    return true;
  }
  reading = tmc4361A_read_encoder(tmc4361A, 0);
  reading = reading - new_target;
  // If we overshot instead of undershot, return an error
  if (float(reading) < -float(err)) {
    //SerialUSB.println("Overshoot open loop");
    return false;
  }

  step_size = float(reading/2) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
  step_size = max(1, step_size);
  //SerialUSB.print("Step size: ");
  //SerialUSB.println(step_size);

  while (((millis() - t0) < PIEZO_TIMEOUT_MS) && (float(reading) > float(err)) && (dac_target <= (u16_MAX - step_size))) {
    // prevent over/underflow:
    if ((step_size > 0) && (float(dac_target) > (float(u16_MAX) - float(step_size)))) // `a + x` would overflow
      dac_target = u16_MAX;
    else if ((step_size < 0) && (float(dac_target) <  float(-step_size)))
      dac_target = 0;
    else
      dac_target += step_size;
    dac_target = constrain(dac_target, 0, u16_MAX);
    dac->output(tmc4361A->dac_idx, dac_target);
    delayMicroseconds(PIEZO_INC_INTERVAL_US);
    reading = tmc4361A_read_encoder(tmc4361A, 0);
    reading = reading - new_target;
//    SerialUSB.print("Err: ");
//    SerialUSB.println(reading);
    step_size = float(reading/3) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
//    SerialUSB.print("Step size: ");
//    SerialUSB.println(step_size);
  }

  //  // try hongquan's suggestion
  //  mstep_expand -= reading;
  //  // consider - only do if undershoot
  //  // combine w undershoot only
  //  initial_target = float(abs(mstep_expand)) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
  //  dac_target = constrain(initial_target, 0, u16_MAX - 1);
  //  dac->output(tmc4361A->dac_idx, dac_target);
  //  delay(PIEZO_SETTLE_TIME_MS);
  // if the reading is below the target, give up
  //  else if (reading < new_target) {
  //    prev_target = reading;
  //    prev_setpoint = dac_target;
  //    return false;
  //  }
  //  else if (prev_target == s32_MAX) {
  //    prev_target = reading;
  //    prev_setpoint = dac_target;
  //    return false;
  //  }
  //  // Try to use the previous reading and current reading
  //  m = (double(prev_target) - double(reading)) / (double(prev_setpoint) - double(dac_target));
  //  b = -m * double(dac_target) + double(reading);
  //  //SerialUSB.println(m);
  //  //SerialUSB.println(b);
  //  // Find a linear approximation
  //  d = (double(new_target) - b) / m;
  //  dac_target = constrain(d, 0, u16_MAX);
  //  initial_target = float(abs(mstep_expand)) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps);
  //  //dac_target = max(dac_target, initial_target);
  //  //SerialUSB.print("final: ");
  //  //SerialUSB.println(d);
  //  dac->output(tmc4361A->dac_idx, initial_target);
  //  delay(PIEZO_SETTLE_TIME_MS);
  //  reading = abs(tmc4361A_read_encoder(tmc4361A, 0) - new_target);
  //
  //  // save new position/setting
  //  prev_target = reading;
  //  prev_setpoint = dac_target;
  reading = abs(reading);
  return (float(reading) < float(err)); // Return true only if we have error below threshold
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: piezo_calibrate_limits() measures the position when outputting 0V and when outputting 5V to relate the
               piezo stpoint to position.

  OPERATION:   We first disable PID. Next, we set the DAC voltage to 0, take a measurement, set the voltage to 5V, and take another measurement.
               We repeat this operation and average the positions at 0V and at 5V and subtract them.

  ARGUMENTS:
      TMC4361ATypeDef *tmc4361A: Pointer to a struct containing motor driver info
      DACx050x *dac: Pointer to the DAC class
      uint8_t n_measurements: Number of measurements to take

  RETURNS:
      bool success: Return true if we were able to get below the err without exceeding the number of retries.

  INPUTS / OUTPUTS: The CS pin and SPI MISO and MOSI pins output, input, and output data respectively. The DAC outputs a value which in turn moves the piezo. We also disable PID on the motor

  LOCAL VARIABLES: None

  SHARED VARIABLES:
      TMC4361ATypeDef *tmc4361A: Values are read from the struct
      DACx050x *dac: We access members of the DAC class.

  GLOBAL VARIABLES: None

  DEPENDENCIES: tmc4316A.h
                DACx050x.h
  -----------------------------------------------------------------------------
*/
void piezo_calibrate_limits(TMC4361ATypeDef *tmc4361A, DACx050x *dac, uint8_t n_measurements) {
  double measurement_max = 0;
  double measurement_min = 0;
  // Disable PID - we don't want the motor moving
  tmc4361A_set_PID(tmc4361A, PID_DISABLE);
  // check if there is a DAC - return out
  if (tmc4361A->dac_idx == NO_DAC) {
    return;
  }
  // Measure the distances
  for (uint8_t i  = 0; i < n_measurements; i++) {
    dac->output(tmc4361A->dac_idx, 0);
    delay(PIEZO_SETTLE_TIME_MS * 10);
    measurement_min += tmc4361A_read_encoder(tmc4361A,  5);
    dac->output(tmc4361A->dac_idx, u16_MAX);
    delay(PIEZO_SETTLE_TIME_MS * 10);
    measurement_max += tmc4361A_read_encoder(tmc4361A,  5);
  }

  int32_t diff = abs((measurement_min / n_measurements) - (measurement_max / n_measurements));
  tmc4361A->dac_fullscale_msteps = diff;

  return;
}
