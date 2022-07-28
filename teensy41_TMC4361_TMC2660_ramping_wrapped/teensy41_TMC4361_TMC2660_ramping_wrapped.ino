/*
    teensy41_TMC4361_TMC2660_ramping_wrapped.ino:
        This project implements positioning-based control on a Teensy 4.1 using the TMC4361A motor controller and TMC2660 motor driver.
        These 2 ICs have a somewhat complicated API for setup and operation; this project provides a wrapper to make the API easier to use.
        All functionality is implemented in the .h and .cpp files #included in this project; this file demonstrates their functionality with a human-usable serial interface.
        This demonstration assumes the motor is set up with two limit switches.
        To use this demonstration, set the parameters as described below.

     DESCRIPTION: This file sets up and runs the main control loop.
        setup(): Initialize Serial and communications with the motor controller IC
        loop():  Read commands over Serial and perform them

    Shared Variables:
        ConfigurationTypeDef tmc4361_configs: Configurations for the motor controllers
        TMC4361ATypeDef tmc4361:              Motor structs contain motor state and parameters
        uint8_t prevstate[N_MOTOR]:           Used for tracking motor position. Future versions of this code will use the TMC4361A EVENTS register instead of tracking state on the Teensy

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
#include "Utils.h"

// Motor Parameters: connect 1 motor
#define N_MOTOR 1
const uint8_t pin_TMC4361_CS[N_MOTOR] = {41}; //, 36, 35, 34}; // leaving other pins here so it's easy to switch over to controlling all 4 motors
const uint8_t pin_TMC4361_CLK = 37;
const uint32_t clk_Hz_TMC4361 = 16000000;
const uint8_t lft_sw_pol[N_MOTOR] = {1}; // , 1,1,1};
const uint8_t rht_sw_pol[N_MOTOR] = {1}; // , 1,1,1};
const uint8_t TMC4361_homing_sw[N_MOTOR] = {LEFT_SW}; //, LEFT_SW, LEFT_SW, LEFT_SW};
const int32_t vslow =  0x007FFF00;

// Cofigs and motor structs
ConfigurationTypeDef tmc4361_configs[N_MOTOR];
TMC4361ATypeDef tmc4361[N_MOTOR];
// Keep track of previous state to see if motor hit target

void setup() {
  // Initialize serial on the Teensy
  SerialUSB.begin(20000000);
  Serial.setTimeout(200);
  // The Teensy operates at 3.3V while the TMC4361A and TMC2660 operate at 5V
  // Supply 3.3V to the level shifters. Future board revisions will have power hardwired
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);

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

  // Initialize SPI - included in Utils.h
  SPI.begin();
  // Give some time to finish initialization before using the SPI bus
  delayMicroseconds(5000);

  // initilize TMC4361 and TMC2660 - turn on functionality
  for (int i = 0; i < N_MOTOR; i++) {
    // set up ICs with SPI control and other parameters
    tmc4361A_tmc2660_init(&tmc4361[i], clk_Hz_TMC4361);
    // enable limit switch reading
    enableLimitSwitch(&tmc4361[i], lft_sw_pol[i], rht_sw_pol[i]);
    // enable homing using a limit switch
    enableHomingLimit(&tmc4361[i], TMC4361_homing_sw[i], lft_sw_pol[i], rht_sw_pol[i]);
  }

  // Home all the motors depending on their requirements
  // First, move the first motor all the way right
  moveToExtreme(&tmc4361[0], vslow, RGHT_DIR);
  // Then all the way left
  moveToExtreme(&tmc4361[0], vslow, LEFT_DIR);
  // Set left as home
  setHome(&tmc4361[0]);

  for (int i = 0; i < N_MOTOR; i++) {
    // initialize ramp with default values
    setMaxSpeed(&tmc4361[i], 0x04FFFF00);
    setMaxAcceleration(&tmc4361[i], 0x1FFFFF);
    tmc4361[i].rampParam[ASTART_IDX] = 0;
    tmc4361[i].rampParam[DFINAL_IDX] = 0;

    sRampInit(&tmc4361[i]);
  }

  // Set the home position to be 0
  setCurrentPosition(&tmc4361[0], 0);
  // Go to 0
  moveTo(&tmc4361[0], 0);

  // Print out motor stats
  for (int i = 0; i < N_MOTOR; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" on pin ");
    Serial.println(pin_TMC4361_CS[i]);
    Serial.print("Min value (microstep): ");
    Serial.println(tmc4361[i].xmin);
    Serial.print("Home pos (microstep):  ");
    Serial.println(tmc4361[i].xhome);
    Serial.print("Max value (microstep): ");
    Serial.println(tmc4361[i].xmax);
    Serial.print("Min value (millimeter): ");
    Serial.println((float)tmc4361[i].xmin * (float)PITCH / (MICROSTEPS * STEP_PER_REV));
    Serial.print("Home pos (millimeter):  ");
    Serial.println((float)tmc4361[i].xhome * (float)PITCH / (MICROSTEPS * STEP_PER_REV));
    Serial.print("Max value (millimeter): ");
    Serial.println((float)tmc4361[i].xmax * (float)PITCH / (MICROSTEPS * STEP_PER_REV));
  }
  // Print serial commands
  Serial.println("Syntax:");
  Serial.println("[s|S|r|R] <index> <setpoint>");
  Serial.println("s: Set the absolute target position in units microsteps");
  Serial.println("S: Set the absolute target position in units millimeters");
  Serial.println("r: Set the relative target position in units microsteps");
  Serial.println("R: Set the relative target position in units millimeters");

  Serial.println("[u|U] <index> <setpoint>");
  Serial.println("u: Set the target velocity in units microsteps/s");
  Serial.println("U: Set the target velocity in units millimeters/s");

  Serial.println("w <index> [a|A|v|V] <value>");
  Serial.println("w: indicates start of write command");
  Serial.println("a: set maximum acceleration in pulses per second^2");
  Serial.println("A: same as 'a' but in units mm per second^2");
  Serial.println("v: set maximum velocity in pulses per second");
  Serial.println("V: same as 'v' but in units mm per second");

  Serial.println("p <index>");
  Serial.println("p: print ramp parameters with units microsteps and mm");

  Serial.println("[x|X] <index>");
  Serial.println("x: print motor position in units microsteps");
  Serial.println("X: print motor position in units millimeters");

  Serial.println("[t|T] <index>");
  Serial.println("t: print target position in units microsteps");
  Serial.println("T: print target position in units millimeters");

  Serial.print("Index selects which motor to read, in range 1 to ");
  Serial.println(N_MOTOR);
}

void loop() {
  int32_t index, target;
  float tmp;
  char cmd;

  // Wait until serial is available
  if (Serial.available()) {
    cmd = Serial.read();
    index = Serial.parseInt();
    // Check command
    if (cmd < 'A' || cmd > 'z') {
      // Exit loop if cmd is not a letter
      return;
    }
    // Parse index first; if parseInt times out, index = 0
    if (index <= 0 || index > N_MOTOR) {
      Serial.println("Index OOB or timeout");
      return;
    }
    index--;
    // Parse cmd
    switch (cmd) {
      case 'p': // Print
        Serial.print("TMC4361A_BOW1 (microsteps/s^3, mm/s^3): ");
        Serial.print(tmc4361[index].rampParam[BOW1_IDX]);
        Serial.print(", ");
        Serial.println(xmicrostepsTomm(tmc4361[index].rampParam[BOW1_IDX]));

        Serial.print("TMC4361A_BOW2 (microsteps/s^3, mm/s^3): ");
        Serial.print(tmc4361[index].rampParam[BOW2_IDX]);
        Serial.print(", ");
        Serial.println(xmicrostepsTomm(tmc4361[index].rampParam[BOW2_IDX]));

        Serial.print("TMC4361A_BOW3 (microsteps/s^3, mm/s^3): ");
        Serial.print(tmc4361[index].rampParam[BOW3_IDX]);
        Serial.print(", ");
        Serial.println(xmicrostepsTomm(tmc4361[index].rampParam[BOW3_IDX]));

        Serial.print("TMC4361A_BOW4 (microsteps/s^3, mm/s^3): ");
        Serial.print(tmc4361[index].rampParam[BOW4_IDX]);
        Serial.print(", ");
        Serial.println(xmicrostepsTomm(tmc4361[index].rampParam[BOW4_IDX]));

        Serial.print("TMC4361A_AMAX (microsteps/s^2, mm/s^2): ");
        Serial.print(tmc4361[index].rampParam[AMAX_IDX]);
        Serial.print(", ");
        Serial.println(amicrostepsTomm(tmc4361[index].rampParam[AMAX_IDX]));

        Serial.print("TMC4361A_DMAX (microsteps/s^2, mm/s^2): ");
        Serial.print(tmc4361[index].rampParam[DMAX_IDX]);
        Serial.print(", ");
        Serial.println(amicrostepsTomm(tmc4361[index].rampParam[DMAX_IDX]));


        Serial.print("TMC4361A_ASTART (microsteps/s^2, mm/s^2): ");
        Serial.print(tmc4361[index].rampParam[ASTART_IDX]);
        Serial.print(", ");
        Serial.println(amicrostepsTomm(tmc4361[index].rampParam[ASTART_IDX]));

        Serial.print("TMC4361A_DFINAL (microsteps/s^2, mm/s^2): ");
        Serial.print(tmc4361[index].rampParam[DFINAL_IDX]);
        Serial.print(", ");
        Serial.println(amicrostepsTomm(tmc4361[index].rampParam[DFINAL_IDX]));

        Serial.print("TMC4361A_VMAX (microsteps/s, mm/s): ");
        Serial.print(tmc4361[index].rampParam[VMAX_IDX]);
        Serial.print(", ");
        Serial.println(vmicrostepsTomm(tmc4361[index].rampParam[VMAX_IDX]));
        break;
      case 'S': // Movement
      case 's':
        Serial.println("Abs Move");
        if (cmd == 'S') {
          tmp = Serial.parseFloat();
          target = xmmToMicrosteps(tmp);
        }
        else {
          target = Serial.parseInt();
        }
        moveTo(&tmc4361[index], target);
        break;
      case 'R':
      case 'r':
        Serial.println("Rel Move");
        if (cmd == 'R') {
          tmp = Serial.parseFloat();
          target = xmmToMicrosteps(tmp);
        }
        else {
          target = Serial.parseInt();
        }
        move(&tmc4361[index], target);
        break;

      case 'U':
      case 'u':
        Serial.print("Set Velocity: ");
        if (cmd == 'U') {
          tmp = Serial.parseFloat();
          target = vmmToMicrosteps(tmp);
        }
        else {
          target = Serial.parseInt();
        }
        Serial.println(target);
        setSpeed(&tmc4361[index], target);
        break;

      case 'x': // Current position
      case 'X':
        Serial.print("Current pos: ");
        target = currentPosition(&tmc4361[index]);

        if (cmd == 'X') {
          tmp = xmicrostepsTomm(target);
          Serial.println(tmp);
        }
        else {
          Serial.println(target);
        }
        break;

      case 't': // Target position
      case 'T':
        Serial.print("Target pos: ");
        target = targetPosition(&tmc4361[index]);

        if (cmd == 'T') {
          tmp = xmicrostepsTomm(target);
          Serial.println(tmp);
        }
        else {
          Serial.println(target);
        }
        break;

      case 'w':
        Serial.println("Write parameter:");
        // If we have a write command, store the command for processing later
        Serial.read();       // Read and throw out the space
        cmd = Serial.read(); // Store the command
        break;
      default:
        Serial.println("Not recognized");
    }
  }

  // Next, process the write command
  switch (cmd) {
    case 'A': // set acceleration
    case 'a':
      Serial.print("AMAX: ");
      if (cmd == 'A') {
        tmp = Serial.parseFloat();
        target = ammToMicrosteps(tmp);
      }
      else {
        target = Serial.parseInt();
      }
      Serial.println(target);
      setMaxAcceleration(&tmc4361[index], target);
      break;

    case 'V':
    case 'v':
      if (cmd == 'V') {
        Serial.print("VMAX: ");
        tmp = Serial.parseFloat();
        target = vmmToMicrosteps(tmp);
      }
      else {
        target = Serial.parseInt();
      }
      Serial.println(target);
      setMaxSpeed(&tmc4361[index], target);
      break;

    default:
      break;
  }
}
