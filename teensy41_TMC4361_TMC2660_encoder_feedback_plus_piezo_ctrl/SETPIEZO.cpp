#include "SETPIEZO.h"
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
bool piezo_set_offset(TMC4361ATypeDef *tmc4361A, DACx050x *dac, int32_t initial_pos, int32_t mstep_expand, uint32_t err) {
  int32_t new_target = initial_pos + mstep_expand;
  int32_t error;
  uint8_t attempts = 0;
  uint16_t dac_target = constrain(float(mstep_expand) * float(u16_MAX) / float(tmc4361A->dac_fullscale_msteps), 0, u16_MAX);
  double m, b; // for linear fit

  // check if there is a DAC - return out
  if (tmc4361A->dac_idx == NO_DAC)
    return false;

  // First, move to opposite end to unstick the stage
  if (abs(mstep_expand) >= (tmc4361A->dac_fullscale_msteps / 2))
    dac->output(tmc4361A->dac_idx, 0);
  else
    dac->output(tmc4361A->dac_idx, u16_MAX);
  // We just want to unstick the stage so it's ok if the stage doesn't fully settle
  delay(PIEZO_SETTLE_TIME_MS / 2);
  
  // Next, go to the target
  SerialUSB.print("Initial target: ");
  SerialUSB.println(dac_target);
  dac->output(tmc4361A->dac_idx, dac_target);
  delay(PIEZO_SETTLE_TIME_MS);
  error = abs(tmc4361A_read_encoder(tmc4361A, 0) - new_target);
  // If it's good enough, we are done
  if (error < err)
    return true;
    
  // Otherwise, use the current position and the nearest extreme point to estimate the slope and offset
  if (abs(mstep_expand) > (tmc4361A->dac_fullscale_msteps / 2)) {
    // If we are closer to the far end, use the far end as reference
    // We have one point at (dac_target, encoder) and another at (u16_MAX, dac_fullscale+initial)
    m = (-tmc4361A->dac_fullscale_msteps + initial_pos - error) / (u16_MAX - dac_target);
    b = -m * dac_target + err;
  }
  else {
    // If we are closer to the far end, use the near end as reference
    // We have one point at (dac_target, encoder) and another at (0, initial)
    m = (error - initial_pos) / (dac_target - 0);
    b = -m * dac_target + err;
  }
  SerialUSB.print("New target: ");
  SerialUSB.println((float(mstep_expand) - b) / m);
  dac_target = constrain((float(mstep_expand) - b) / m, 0, u16_MAX);
  dac->output(tmc4361A->dac_idx, dac_target);
  delay(PIEZO_SETTLE_TIME_MS);
  error = abs(tmc4361A_read_encoder(tmc4361A, 0) - new_target);

  return (error < err); // Return true only if we have error below threshold
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
