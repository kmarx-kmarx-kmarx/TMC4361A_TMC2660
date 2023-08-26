/*
    teensy41_TMC4361_TMC2660_encoder_feedback_plus_piezo_control.ino:
        z motor only
        This project implements PID-controlled positioning-based control on a Teensy 4.1 using the TMC4361A motor controller and TMC2660 motor driver.
        These two ICs have a somewhat complicated API for setup and operation; this project provides a wrapper to make the API easier to use.
        All functionality is implemented in the .h and .cpp files #included in this project; this file demonstrates their functionality with a human-usable serial interface.
        This demonstration assumes the motor is set up with two limit switches in the default configuration.
        To use this demonstration, set the parameters as described below.

        We also have control over the piezo using the DAC.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

    Shared Variables:
        ConfigurationTypeDef tmc4361_configs: Configurations for the motor controllers
        TMC4361ATypeDef tmc4361:              Motor structs contain motor state and parameters

    Motor Parameters:
        N_MOTOR: Number of TMC4361A motor controllers connected to the same SPI bus
        pin_TMC4361_CS[N_MOTOR]: Chip Select pins for each tmc4316A. Each motor controller should have a unique pin
        pin_TMC4361_CLK: SPI clock pin; the TMC4361A and TMC2660 need lower clock speeds than the Teensy. This is in units Hz.
        lft_sw_pol[N_MOTOR]: Define left limit switch behaviors for each motor. Set to 1 for active high switches, 0 for active low.
        rht_sw_pol[N_MOTOR]: Same as above for the right switch
        TMC4361_homing_sw[N_MOTOR]: Set whether the left or right switch is the homing switch for each motor.
        vslow: Movement speed in microsteps per second when calibrating. Moing slowly when calibrating gives more accurate results

    Dependencies:
        TMC4361A.h: Low-level definitions and functions for motor interface
        Utils.h:    Higher level functions for motor interface

    Author: Kevin Marx
    Created on: 7/7/2022
*/

#include "TMC4361A.h"
#include "TMC4361A_TMC2660_Utils.h"
#include "octopi.h"
#include "DACx050x.h"
#include "SETPIEZO.h"
// Config the DAC
DACx050x dac(33);

// Motor Parameters: connect 1 motor
#define N_MOTOR 1
const uint8_t pin_TMC4361_CS[N_MOTOR] = {35};//, 36, 35};//, 34}; // leaving other pins here so it's easy to switch over to controlling all 4 motors
const uint8_t pin_TMC4361_CLK = 37;
const uint32_t clk_Hz_TMC4361 = 16000000;
const uint8_t lft_sw_pol[N_MOTOR] = {0};//, 1, 1};//,1,1};
const uint8_t rht_sw_pol[N_MOTOR] = {0};//, 1, 1};//,1,1};
const uint8_t TMC4361_homing_sw[N_MOTOR] = {RGHT_SW};//, LEFT_SW, LEFT_SW}; //, LEFT_SW, LEFT_SW};
const int32_t vslow = 0x01FFFC00;
#define PID_DEVIATION_ERROR     180000
#define PID_CHECK_INTERVAL_MS      500
uint32_t pid_check_time = 0;

#define N_TESTPOINTS 3 // for linearity test
#define DO_LINEARITY_TEST false

void trianglewave(int32_t dac_idx, int32_t time_ms);
void squarewave(int32_t dac_idx, int32_t time_ms);

// Configs and motor structs
ConfigurationTypeDef tmc4361_configs[N_MOTOR];
TMC4361ATypeDef tmc4361[N_MOTOR];

// Initialization routines
void setup() {
  // Initialize serial on the Teensy
  SerialUSB.begin(20000000);
  delay(1000);

  // Initialize clock
  pinMode(pin_TMC4361_CLK, OUTPUT);
  analogWriteFrequency(pin_TMC4361_CLK, clk_Hz_TMC4361);
  analogWrite(pin_TMC4361_CLK, 128); // 50% duty

  // Initialize TMC4361 structs with default values and initialize CS pins
  for (int i = 0; i < N_MOTOR; i++) {
    // Initialize the tmc4361 with their channel number and default configuration
    tmc4361A_init(&tmc4361[i], pin_TMC4361_CS[i], &tmc4361_configs[i], tmc4361A_defaultRegisterResetState);
    // Set the chip select pins
    pinMode(pin_TMC4361_CS[i], OUTPUT);
    digitalWrite(pin_TMC4361_CS[i], HIGH);
  }

  // Motor configurations
  // 0.22 ohm -> 1 A; 0.15 ohm -> 1.47 A
  tmc4361A_tmc2660_config(&tmc4361[0], (Z_MOTOR_RMS_CURRENT_mA / 1000)*R_sense_z / 0.2298, Z_MOTOR_I_HOLD, 1, 1, 1, SCREW_PITCH_Z_MM, FULLSTEPS_PER_REV_Z, MICROSTEPPING_Z, 7, 2475); // need to make current scaling on TMC2660 is > 16 (out of 31)


  // Initialize SPI - included in Utils.h
  SPI.begin();
  // Give some time to finish initialization before using the SPI bus
  delayMicroseconds(50);
  // Initialize the DAC
  dac.begin(SPI);

  // initilize TMC4361 and TMC2660 - turn on functionality
  for (int i = 0; i < N_MOTOR; i++) {
    // set up ICs with SPI control and other parameters
    tmc4361A_tmc2660_init(&tmc4361[i], clk_Hz_TMC4361);
    // enable limit switch reading
    tmc4361A_enableLimitSwitch(&tmc4361[i], rht_sw_pol[i], RGHT_SW, false);
    tmc4361A_enableLimitSwitch(&tmc4361[i], lft_sw_pol[i], LEFT_SW, false);
  }


  for (int i = 0; i < N_MOTOR; i++) {
    // initialize ramp with default values
    tmc4361A_setMaxSpeed(&tmc4361[i], tmc4361A_vmmToMicrosteps(&tmc4361[i], MAX_VELOCITY_Z_mm));
    tmc4361A_setMaxAcceleration(&tmc4361[i], tmc4361A_ammToMicrosteps(&tmc4361[i], MAX_ACCELERATION_Z_mm));
    tmc4361[i].rampParam[ASTART_IDX] = 0;
    tmc4361[i].rampParam[DFINAL_IDX] = 0;

    tmc4361A_sRampInit(&tmc4361[i]);
  }
  //print all params
  for (int index = 0; index < N_MOTOR; index++) {
    SerialUSB.print("TMC4361A_BOW1 (microsteps/s^3, mm/s^3): ");
    SerialUSB.print(tmc4361[index].rampParam[BOW1_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW1_IDX]));

    SerialUSB.print("TMC4361A_BOW2 (microsteps/s^3, mm/s^3): ");
    SerialUSB.print(tmc4361[index].rampParam[BOW2_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW2_IDX]));

    SerialUSB.print("TMC4361A_BOW3 (microsteps/s^3, mm/s^3): ");
    SerialUSB.print(tmc4361[index].rampParam[BOW3_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW3_IDX]));

    SerialUSB.print("TMC4361A_BOW4 (microsteps/s^3, mm/s^3): ");
    SerialUSB.print(tmc4361[index].rampParam[BOW4_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW4_IDX]));

    SerialUSB.print("TMC4361A_AMAX (microsteps/s^2, mm/s^2): ");
    SerialUSB.print(tmc4361[index].rampParam[AMAX_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[AMAX_IDX]));

    SerialUSB.print("TMC4361A_DMAX (microsteps/s^2, mm/s^2): ");
    SerialUSB.print(tmc4361[index].rampParam[DMAX_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[DMAX_IDX]));


    SerialUSB.print("TMC4361A_ASTART (microsteps/s^2, mm/s^2): ");
    SerialUSB.print(tmc4361[index].rampParam[ASTART_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[ASTART_IDX]));

    SerialUSB.print("TMC4361A_DFINAL (microsteps/s^2, mm/s^2): ");
    SerialUSB.print(tmc4361[index].rampParam[DFINAL_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[DFINAL_IDX]));

    SerialUSB.print("TMC4361A_VMAX (microsteps/s, mm/s): ");
    SerialUSB.print(tmc4361[index].rampParam[VMAX_IDX]);
    SerialUSB.print(", ");
    SerialUSB.println(tmc4361A_vmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[VMAX_IDX]));
  }

  tmc4361A_enableHomingLimit(&tmc4361[0], rht_sw_pol[0], TMC4361_homing_sw[0]);
  // Perform homing
  SerialUSB.println("Begin inward (right)");
  tmc4361A_moveToExtreme(&tmc4361[0], tmc4361A_vmmToMicrosteps(&tmc4361[0], MAX_VELOCITY_Z_mm), RGHT_DIR);
  tmc4361A_setCurrentPosition(&tmc4361[0], 0);

  // Move to the center - set the middle as home so distance can range from +3mm to -3 mm
  int32_t target = tmc4361A_xmmToMicrosteps(&tmc4361[0], Z_NEG_LIMIT_MM * 1.01);
  SerialUSB.print("Moving to center ");
  SerialUSB.println(target);
  tmc4361A_moveTo(&tmc4361[0], target);
  // Wait until we reach our target
  while (tmc4361A_currentPosition(&tmc4361[0]) != target) {
    delay(50);
  }
  // Our center position is home, set to 0
  tmc4361A_setCurrentPosition(&tmc4361[0], 0);
  tmc4361[0].xhome = 0;
  // Physical limit switch (left) is now at -3 mm
  // Set virtual limit switch (right) relative to current position at +3 mm
  target = tmc4361A_xmmToMicrosteps(&tmc4361[0], Z_POS_LIMIT_MM * 1.01);
  tmc4361[0].xmin = -target;
  tmc4361A_setVirtualStop(&tmc4361[0], RGHT_SW, target);

  // Initialze the encoder
  //                                    A/B transitions per rev and filter params
  tmc4361A_init_ABN_encoder(&tmc4361[0], 3000, 32, 4, 512, true);
  // Transitions per rev calculated by =(max mstep - min mstep)/(max encoder - min encoder)*previous transition per rev setting
  // Synchronize encoder and microsteps
  tmc4361A_setCurrentPosition(&tmc4361[0], tmc4361A_read_encoder(&tmc4361[0], N_ENC_AVG_EXP));
  // Calibrate the piezo
  piezo_calibrate_limits(&tmc4361[0], &dac, 1);
  SerialUSB.print("Calibrated piezo: ");
  SerialUSB.println(tmc4361[0].dac_fullscale_msteps);

  SerialUSB.print("Initialized encoder. Current position: ");
  SerialUSB.print(tmc4361A_currentPosition(&tmc4361[0]));
  SerialUSB.print(", encoder position ");
  SerialUSB.println(tmc4361A_read_encoder(&tmc4361[0], N_ENC_AVG_EXP));
  // Print out motor stats
  for (int i = 0; i < N_MOTOR; i++) {
    SerialUSB.print("Motor ");
    SerialUSB.print(i);
    SerialUSB.print(" on pin ");
    SerialUSB.println(pin_TMC4361_CS[i]);
    SerialUSB.print("Min value (microstep): ");
    SerialUSB.println(tmc4361[i].xmin);
    SerialUSB.print("Home pos (microstep):  ");
    SerialUSB.println(tmc4361[i].xhome);
    SerialUSB.print("Max value (microstep): ");
    SerialUSB.println(tmc4361[i].xmax);
    SerialUSB.print("Min value (millimeter): ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[i], tmc4361[i].xmin));
    SerialUSB.print("Home pos (millimeter):  ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[i], tmc4361[i].xhome));
    SerialUSB.print("Max value (millimeter): ");
    SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[i], tmc4361[i].xmax));
  }
  // Perform linearity test
  if (DO_LINEARITY_TEST) {
    int32_t msteps[N_TESTPOINTS];
    int32_t encoder_readings[N_TESTPOINTS];
    int32_t endpoint = tmc4361A_xmmToMicrosteps(&tmc4361[0], Z_NEG_LIMIT_MM);
    int32_t startpoint = tmc4361A_xmmToMicrosteps(&tmc4361[0], Z_POS_LIMIT_MM);
    SerialUSB.print("Start at ");
    SerialUSB.print(startpoint);
    SerialUSB.print(", end at ");
    SerialUSB.println(endpoint);
    int8_t err = tmc4361A_measure_linearity(&tmc4361[0], encoder_readings, msteps, N_TESTPOINTS, startpoint, endpoint, 5000);
    switch (err) {
      case NO_ERR:
        SerialUSB.println("Success");
        break;
      case ERR_OUT_OF_RANGE:
        SerialUSB.println("Out of range");
        break;
      case ERR_TIMEOUT:
        SerialUSB.println("Movement timed out");
        break;
      default:
        SerialUSB.println("Unknown error");
    }
    for (int i = 0; i < N_TESTPOINTS; i++) {
      SerialUSB.print(msteps[i]);
      SerialUSB.print(", ");
      SerialUSB.println(encoder_readings[i]);
    }
  }
  // Enable PID
  SerialUSB.println("Enable PID");
  // target, pid, err
  tmc4361A_init_PID(&tmc4361[0], 25, 25, 512, 64, 0, tmc4361A_vmmToMicrosteps(&tmc4361[0], MAX_VELOCITY_Z_mm), 4096, 2);
  tmc4361A_set_PID(&tmc4361[0], PID_BPG0);
  // Print serial commands
  //  SerialUSB.println("Syntax:");
  //  SerialUSB.println("[s|S|r|R|o|O|q|Q] <index> <setpoint>");
  //  SerialUSB.println("s: Set the absolute target position in units microsteps");
  //  SerialUSB.println("S: Set the absolute target position in units millimeters");
  //  SerialUSB.println("r: Set the relative target position in units microsteps");
  //  SerialUSB.println("R: Set the relative target position in units millimeters");
  //  SerialUSB.println("o: Set the absolute target position in units microsteps with anti-stick feature");
  //  SerialUSB.println("O: Set the absolute target position in units millimeters with anti-stick feature");
  //  SerialUSB.println("q: Set the relative target position in units microsteps with anti-stick feature");
  //  SerialUSB.println("Q: Set the relative target position in units millimeters with anti-stick feature");
  //
  //  SerialUSB.println("[u|U] <index> <setpoint>");
  //  SerialUSB.println("u: Set the target velocity in units microsteps/s");
  //  SerialUSB.println("U: Set the target velocity in units millimeters/s");
  //
  //  SerialUSB.println("w <index> [a|A|v|V|k|K] <value>");
  //  SerialUSB.println("w: indicates start of write command");
  //  SerialUSB.println("a: set maximum acceleration in pulses per second^2");
  //  SerialUSB.println("A: same as 'a' but in units mm per second^2");
  //  SerialUSB.println("v: set maximum velocity in pulses per second");
  //  SerialUSB.println("V: same as 'v' but in units mm per second");
  //  SerialUSB.println("k: overwrite current position in units microsteps");
  //  SerialUSB.println("K: same as 'k' but in units mm");
  //
  //  SerialUSB.println("b <index>");
  //  SerialUSB.println("b: print ramp parameters with units microsteps and mm");
  //
  //  SerialUSB.println("[x|X] <index>");
  //  SerialUSB.println("x: print motor position in units microsteps");
  //  SerialUSB.println("X: print motor position in units millimeters");
  //
  //  SerialUSB.println("[t|T] <index>");
  //  SerialUSB.println("t: print target position in units microsteps");
  //  SerialUSB.println("T: print target position in units millimeters");
  //
  //  SerialUSB.println("e <index>");
  //  SerialUSB.println("e: get the encoder position");
  //
  //  SerialUSB.println("[d|D] <dac index> <intensity>");
  //  SerialUSB.println("d: set DAC output intensity (0 to 65535)");
  //  SerialUSB.println("D: set DAC output voltage in volts (0 to 5)");
  //
  //  SerialUSB.println("[n|N|m|M] <dac index> <n steps> <n avg>");
  //  SerialUSB.println("n: Lerp the piezo voltage from 0 to 5V along n steps points and report the encoder reading (averaged over 2^n)");
  //  SerialUSB.println("   Prints DAC setpoint and actual encoder value");
  //  SerialUSB.println("N: Do triange wave");
  //  SerialUSB.println("[m|M]: same as [n|N] but lerping across the range using setpoints in units microsteps and the new modified newton's method");
  //  SerialUSB.println("   Prints encoder value setpoint and actual encoder value");
  //
  //  SerialUSB.println("[i|I] <dac index> <loop time, seconds>");
  //  SerialUSB.println("i: Output a triangle wave for a given amount of time");
  //  SerialUSB.println("I: Output a triangle wave for a given amount of time");
  //
  //  SerialUSB.println("[p|P] <index>");
  //  SerialUSB.println("p: disable PID");
  //  SerialUSB.println("P: enable PID");
  //
  //  SerialUSB.print("Index selects which motor or DAC to read, in range 1 to ");
  //  SerialUSB.println(N_MOTOR);
}

void loop() {
  int32_t index, target, index_max;
  float tmp;
  char cmd;

  // Wait until serial is available
  if (SerialUSB.available()) {
    cmd = SerialUSB.read();
    index = SerialUSB.parseInt();
    // Check command
    if (cmd < 'A' || cmd > 'z') {
      // Exit loop if cmd is not a letter
      return;
    }
    // Parse index first; if parseInt times out, index = 0
    index_max = N_MOTOR;
    if ((cmd == 'd') || (cmd == 'D') || (cmd == 'n') || (cmd == 'N') || (cmd == 'm') || (cmd == 'M') || (cmd == 'i') || (cmd == 'I'))
      index_max = 8;

    if (index <= 0 || index > index_max) {
      SerialUSB.println("Index OOB or timeout");
      return;
    }
    index--;
    // debugging - status
    // SerialUSB.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_STATUS), BIN);
    // SerialUSB.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_EVENTS), BIN);
    // Parse cmd
    switch (cmd) {
      case 'i':
        tmp = SerialUSB.parseFloat();
        target = tmp * 1000;
        SerialUSB.print("Triangle wave output on ");
        SerialUSB.print(index);
        SerialUSB.print(" for ");
        SerialUSB.print(target);
        SerialUSB.println(" milliseconds.");
        trianglewave(index, target);
      case 'I':
        tmp = SerialUSB.parseFloat();
        target = tmp * 1000;
        SerialUSB.print("Square wave output on ");
        SerialUSB.print(index);
        SerialUSB.print(" for ");
        SerialUSB.print(target);
        SerialUSB.println(" milliseconds.");
        squarewave(index, target);
      case 'p':
        SerialUSB.println("Disable PID");
        tmc4361A_set_PID(&tmc4361[index], PID_DISABLE);
        break;
      case 'P':
        SerialUSB.println("Enable PID");
        tmc4361A_set_PID(&tmc4361[index], PID_BPG0);
        break;
      case 'b': // Print
        SerialUSB.print("TMC4361A_BOW1 (microsteps/s^3, mm/s^3): ");
        SerialUSB.print(tmc4361[index].rampParam[BOW1_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW1_IDX]));

        SerialUSB.print("TMC4361A_BOW2 (microsteps/s^3, mm/s^3): ");
        SerialUSB.print(tmc4361[index].rampParam[BOW2_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW2_IDX]));

        SerialUSB.print("TMC4361A_BOW3 (microsteps/s^3, mm/s^3): ");
        SerialUSB.print(tmc4361[index].rampParam[BOW3_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW3_IDX]));

        SerialUSB.print("TMC4361A_BOW4 (microsteps/s^3, mm/s^3): ");
        SerialUSB.print(tmc4361[index].rampParam[BOW4_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_xmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[BOW4_IDX]));

        SerialUSB.print("TMC4361A_AMAX (microsteps/s^2, mm/s^2): ");
        SerialUSB.print(tmc4361[index].rampParam[AMAX_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[AMAX_IDX]));

        SerialUSB.print("TMC4361A_DMAX (microsteps/s^2, mm/s^2): ");
        SerialUSB.print(tmc4361[index].rampParam[DMAX_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[DMAX_IDX]));


        SerialUSB.print("TMC4361A_ASTART (microsteps/s^2, mm/s^2): ");
        SerialUSB.print(tmc4361[index].rampParam[ASTART_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[ASTART_IDX]));

        SerialUSB.print("TMC4361A_DFINAL (microsteps/s^2, mm/s^2): ");
        SerialUSB.print(tmc4361[index].rampParam[DFINAL_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_amicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[DFINAL_IDX]));

        SerialUSB.print("TMC4361A_VMAX (microsteps/s, mm/s): ");
        SerialUSB.print(tmc4361[index].rampParam[VMAX_IDX]);
        SerialUSB.print(", ");
        SerialUSB.println(tmc4361A_vmicrostepsTomm(&tmc4361[index], tmc4361[index].rampParam[VMAX_IDX]));
        break;
      case 'S': // Movement
      case 's':
        SerialUSB.print("Abs Move ");
        if (cmd == 'S') {
          tmp = SerialUSB.parseFloat();
          target = tmc4361A_xmmToMicrosteps(&tmc4361[index], tmp);
        }
        else {
          target = SerialUSB.parseInt();
        }
        SerialUSB.println(tmc4361A_moveTo(&tmc4361[index], target));
        break;
      case 'R':
      case 'r':
        SerialUSB.print("Rel Move ");
        if (cmd == 'R') {
          tmp = SerialUSB.parseFloat();
          target = tmc4361A_xmmToMicrosteps(&tmc4361[index], tmp);
        }
        else {
          target = SerialUSB.parseInt();
        }
        SerialUSB.println(tmc4361A_move(&tmc4361[index], target));
        break;
      case 'O': // Movement
      case 'o':
        SerialUSB.println("Abs Move no stick");
        if (cmd == 'O') {
          tmp = SerialUSB.parseFloat();
          target = tmc4361A_xmmToMicrosteps(&tmc4361[index], tmp);
        }
        else {
          target = SerialUSB.parseInt();
        }
        SerialUSB.print(tmc4361A_moveTo_no_stick(&tmc4361[index], target, tmc4361A_xmmToMicrosteps(&tmc4361[index], .1), PID_DEVIATION_ERROR, 10000));
        break;
      case 'Q':
      case 'q':
        SerialUSB.println("Rel Move no stick");
        if (cmd == 'Q') {
          tmp = SerialUSB.parseFloat();
          target = tmc4361A_xmmToMicrosteps(&tmc4361[index], tmp);
        }
        else {
          target = SerialUSB.parseInt();
        }
        SerialUSB.print(tmc4361A_move_no_stick(&tmc4361[index], target, tmc4361A_xmmToMicrosteps(&tmc4361[index], .1), PID_DEVIATION_ERROR, 10000));
        break;

      case 'U':
      case 'u':
        SerialUSB.print("Set Velocity: ");
        if (cmd == 'U') {
          tmp = SerialUSB.parseFloat();
          target = tmc4361A_vmmToMicrosteps(&tmc4361[index], tmp);
        }
        else {
          target = SerialUSB.parseInt();
        }
        SerialUSB.println(target);
        tmc4361A_setSpeed(&tmc4361[index], target);
        break;

      case 'x': // Current position
      case 'X':
        SerialUSB.print("Current pos: ");
        target = tmc4361A_currentPosition(&tmc4361[index]);

        if (cmd == 'X') {
          tmp = tmc4361A_xmicrostepsTomm(&tmc4361[index], target);
          SerialUSB.println(tmp);
        }
        else {
          SerialUSB.println(target);
        }
        break;

      case 't': // Target position
      case 'T':
        SerialUSB.print("Target pos: ");
        target = tmc4361A_targetPosition(&tmc4361[index]);

        if (cmd == 'T') {
          tmp = tmc4361A_xmicrostepsTomm(&tmc4361[index], target);
          SerialUSB.println(tmp);
        }
        else {
          SerialUSB.println(target);
        }
        break;

      case 'w':
        SerialUSB.println("Write parameter:");
        // If we have a write command, store the command for processing later
        SerialUSB.read();       // Read and throw out the space
        cmd = SerialUSB.read(); // Store the command
        index_max = max(N_MOTOR, 8) + 1; // Set flag to continue parsing the command
        break;

      case 'e':
        SerialUSB.print("Encoder reading: ");
        SerialUSB.println(tmc4361A_read_encoder(&tmc4361[index],  N_ENC_AVG_EXP));
        break;
      case 'D':
      case 'd':
        if (cmd == 'D') {
          tmp = SerialUSB.parseFloat();
          tmp = constrain(tmp, 0, 5);
          tmp = tmp / 5;
          target = tmp * u16_MAX;
        }
        else {
          target = SerialUSB.parseInt();
        }

        dac.output(index, target);
        SerialUSB.print("Setting output ");
        SerialUSB.print(index);
        SerialUSB.print(": ");
        SerialUSB.println(target);
        break;
      case 'n':
      case 'N':
      case 'm':
      case 'M':
        // Initialize variables. We are in a switch statement so we have to separate delclaration and assignment
        bool do_triangle, do_error_correction;
        int32_t final_target, initial_position;
        uint32_t target_idx;
        // Read from serial
        target = abs(SerialUSB.parseInt());    // number of steps
        index_max = SerialUSB.parseInt(); // number of averages

        // if N or M, double the final target so we do a triangle
        do_triangle = ((cmd == 'N') || (cmd == 'M'));
        final_target = do_triangle ? (target * 2) : target;
        // Set bool for whether we are doing error corrected movement or normal movement
        do_error_correction = ((cmd == 'm') || (cmd == 'M'));
        // Get the initial position - used for error corrected movement
        dac.output(index, 0);
        delay(100);
        initial_position = tmc4361A_read_encoder(&tmc4361[0],  index_max);

        for (uint32_t i = 0; i < final_target; i++) {
          // Triangle - if we hit the peak, wrap back around
          if(i >= target){
            target_idx = final_target - i - 1;
          }
          
          if (!do_error_correction) {
            // Do open-loop assuming linear behavior
            tmp = target_idx * u16_MAX / (target-1);
            dac.output(index, tmp);
            delay(6);
          }
          else {
            // set the step - must be negative given configuration
            tmp = -(target_idx/(target - 1)) * tmc4361[0].dac_fullscale_msteps;
            SerialUSB.print("0 Initial target: ");
            SerialUSB.println(tmp);
            // Set driver, DAC, initial position, desired expansion, and tolerated error in units usteps. We want 150 nm error - 0.00015 mm
            piezo_set_offset(&tmc4361[0], &dac, initial_position, int32_t(tmp), tmc4361A_xmmToMicrosteps(&tmc4361[index], 0.00015)); 
            // tmp is now the target position
            tmp += initial_position;
          }
          SerialUSB.print(int32_t(tmp));
          SerialUSB.print(", ");
          SerialUSB.println(tmc4361A_read_encoder(&tmc4361[0],  index_max));
        }
        
        dac.output(index, 0);
        break;
      default:
        SerialUSB.println("Not recognized");
    }
    if (index_max > max(N_MOTOR, 8)) {
      switch (cmd) {
        case 'A': // set acceleration
        case 'a':
          SerialUSB.print("AMAX: ");
          if (cmd == 'A') {
            tmp = SerialUSB.parseFloat();
            target = tmc4361A_ammToMicrosteps(&tmc4361[index], tmp);
          }
          else {
            target = SerialUSB.parseInt();
          }
          SerialUSB.println(target);
          tmc4361A_setMaxAcceleration(&tmc4361[index], target);
          break;

        case 'V':
        case 'v':
          if (cmd == 'V') {
            SerialUSB.print("VMAX: ");
            tmp = SerialUSB.parseFloat();
            target = tmc4361A_vmmToMicrosteps(&tmc4361[index], tmp);
          }
          else {
            target = SerialUSB.parseInt();
          }
          SerialUSB.println(target);
          tmc4361A_setMaxSpeed(&tmc4361[index], target);
          break;

        case 'K':
        case 'k':
          if (cmd == 'K') {
            SerialUSB.print("X: ");
            tmp = SerialUSB.parseFloat();
            target = tmc4361A_xmmToMicrosteps(&tmc4361[index], tmp);
          }
          else {
            target = SerialUSB.parseInt();
          }
          SerialUSB.println(target);
          tmc4361A_setCurrentPosition(&tmc4361[index], target);
          break;
        default:
          SerialUSB.println("Not recognized");
      }
    }
  }
  // SerialUSB.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_STATUS), BIN);

  // Every PID_CHECK_INTERVAL_MS, check the deviation error and disable PID if it exceeds the threshold
  uint32_t corrected_time = pid_check_time - (pid_check_time % PID_CHECK_INTERVAL_MS);
  uint32_t deviation;
  if ((millis() - corrected_time) > PID_CHECK_INTERVAL_MS) {
    pid_check_time = millis();
    for (int i = 0; i < N_MOTOR; i++) {
      deviation = abs(tmc4361A_read_deviation(&tmc4361[i]));
      if ((deviation >= PID_DEVIATION_ERROR) || (tmc4361A_readLimitSwitches(&tmc4361[0]) != 0)) {
        SerialUSB.println(deviation);
        tmc4361A_stop(&tmc4361[i]);
        tmc4361A_set_PID(&tmc4361[i], PID_DISABLE);
        SerialUSB.print("PID disabled for motor ");
        SerialUSB.print(i + 1);
        SerialUSB.print(". Re-enable with 'P ");
        SerialUSB.print(i + 1);
        SerialUSB.println("'.");
      }
    }

    // Also get stop switches
    // SerialUSB.print("Stop switches: ");
    // SerialUSB.println(tmc4361A_readLimitSwitches(&tmc4361[0]), BIN);
  }
}

void trianglewave(int32_t dac_idx, int32_t time_ms) {
  uint32_t t0 = millis();
  uint32_t intensity = 0;
  while ((millis() - t0) < time_ms) {
    intensity = (micros() % (1 << 17));
    if (intensity > u16_MAX) {
      intensity = (1 << 17) - intensity;
    }
    dac.output(dac_idx, intensity);
    delayMicroseconds(487);
  }
  delay(500);
  dac.output(dac_idx, 0);
}
void squarewave(int32_t dac_idx, int32_t time_ms) {
  uint32_t t0 = millis();
  while ((millis() - t0) < time_ms) {
    dac.output(dac_idx, u16_MAX);
    delay(100);
    dac.output(dac_idx, 0);
    delay(100);
  }
}
