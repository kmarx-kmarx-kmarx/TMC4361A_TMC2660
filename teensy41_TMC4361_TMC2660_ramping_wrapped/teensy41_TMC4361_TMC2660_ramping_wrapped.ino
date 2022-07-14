/*
    Find home, the max position, then take position arguments over serial and ramp to the setpoint.
    This code adds to TMC4361.h and TMC4361.cpp.

    Author: Kevin Marx
    Created on: 7/7/2022
*/

#include "TMC4361A.h" // These files come from the TCM4361A manufacturer
#include "Utils.h"    // Adds more functionality

// Define the chip select pins for the TMC4361 connected to the microcontroller
#define N_MOTOR 1
const uint8_t pin_TMC4361_CS[N_MOTOR] = {41}; //, 36, 35, 34};
// Define the clock pin. The clock speed of the TMC4361 is slower than the Teensy's clock speed
const uint8_t pin_TMC4361_CLK = 37;
const uint32_t clk_Hz_TMC4361 = 16000000;

// Define the CS pin for the 3.3 to 5V level shifter
const uint8_t pin_DAC80508_CS = 33;

// define limit switch polarity, 1 for active high, 0 for active low
const uint8_t lft_sw_pol[N_MOTOR] = {1}; // , 1,1,1};
const uint8_t rht_sw_pol[N_MOTOR] = {1}; // , 1,1,1};
// define the switchs we are using for homing
const uint8_t TMC4361_homing_sw[N_MOTOR] = {LEFT_SW}; //, 36, 35, 34};

// Set calibration behavior
const int32_t vslow =  50000;
const int32_t vfast = 200000;

// Cofigs and motor structs
ConfigurationTypeDef tmc4361_configs[N_MOTOR];
TMC4361ATypeDef tmc4361[N_MOTOR];
// Keep track of previous state to see if motor hit target
uint8_t prevstate[N_MOTOR] = {1}; //, 1, 1, 1};

void setup() {
  SerialUSB.begin(20000000);
  Serial.setTimeout(200);
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

  // Initialize TMC4361 structs with default values and initialize CS pins
  for (int i = 0; i < N_MOTOR; i++) {
    // Initialize the tmc4361 with their channel number and default configuration
    tmc4361A_init(&tmc4361[i], pin_TMC4361_CS[i], &tmc4361_configs[i], tmc4361A_defaultRegisterResetState);
    // Set the chip select pins
    pinMode(pin_TMC4361_CS[i], OUTPUT);
    digitalWrite(pin_TMC4361_CS[i], HIGH);
  }

  // SPI - included in Utils.h
  SPI.begin();
  delayMicroseconds(5000);

  // initilize TMC4361 and TMC2660 - turn on functionality
  for (int i = 0; i < N_MOTOR; i++) {
    // set up ICs with SPI control and other parameters
    tcm4361A_tcm2660_init(&tmc4361[i], clk_Hz_TMC4361);
    // enable limit switch reading
    enableLimitSwitch(&tmc4361[i], lft_sw_pol[i], rht_sw_pol[i]);
    // enable homing using a limit switch
    enableHomingLimit(&tmc4361[i], TMC4361_homing_sw[i], lft_sw_pol[i], rht_sw_pol[i]);
  }

  // Home all the motors depending on their requirements
  homeLeft(&tmc4361[0], vslow, vfast);
  findRight(&tmc4361[0], vslow);

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

  Serial.println("Syntax:");
  Serial.println("[s|S] <index> <setpoint>");
  Serial.println("s: setpoint is in units microsteps");
  Serial.println("S: setpoint is in units millimeters");
  Serial.print("Index selects which motor to move, in range 1 to ");
  Serial.println(N_MOTOR);
  Serial.println("Setpoint sets the absolute position along the track in range min to max. ");
  Serial.println("w <index> [a|A|b|B|c|C] <value>");
  Serial.println("w: indicates start of write command");
  Serial.print("Index selects which motor to move, in range 1 to ");
  Serial.println(N_MOTOR);
  Serial.println("a: set BOW 1 and BOW 4 (positive jerk when accelerating, negative jerk when decellerating) in pulses per second^3");
  Serial.println("A: same as 'a' but in units mm per second^3");
  Serial.println("b: set BOW 2 and BOW 3 (negative jerk when accelerating, positive jerk when decellerating) in pulses per second^3");
  Serial.println("B: same as 'b' but in units mm per second^3");
  Serial.println("c: set max velocity in pulses per second * 2^-8");
  Serial.println("C: same as 'c' but in units mm per second ");
  Serial.println("p <index>");
  Serial.println("p: print ramp parameters");
  Serial.print("Index selects which motor to move, in range 1 to ");
  Serial.println(N_MOTOR);
  Serial.println("[x|X] <index>");
  Serial.println("x: Output in units microsteps");
  Serial.println("X: Output in units millimeters");
  Serial.print("Index selects which motor to move, in range 1 to ");
  Serial.println(N_MOTOR);
  Serial.println("Print out the current position of the specified motor");
  Serial.println("[t|T] <index>");
  Serial.println("t: Output in units microsteps");
  Serial.println("T: Output in units millimeters");
  Serial.print("Index selects which motor to move, in range 1 to ");
  Serial.println(N_MOTOR);
  Serial.println("Print out the target position of the specified motor");

  for (int i = 0; i < N_MOTOR; i++) {
    // initialize ramp with default values
    tmc4361[i].rampParam[BOW1_IDX] = 0x01FFFF;
    tmc4361[i].rampParam[BOW2_IDX] = 0x1FFFFF;
    tmc4361[i].rampParam[BOW3_IDX] = 0x1FFFFF;
    tmc4361[i].rampParam[BOW4_IDX] = 0x01FFFF;
    tmc4361[i].rampParam[AMAX_IDX] = 0x1FFFFF;
    tmc4361[i].rampParam[DMAX_IDX] = 0x1FFFFF;
    tmc4361[i].rampParam[ASTART_IDX] = 0;
    tmc4361[i].rampParam[DFINAL_IDX] = 0;
    tmc4361[i].rampParam[VMAX_IDX] = 0x04FFFF00;

    sRampInit(&tmc4361[i]);
  }
}

void loop() {
  int32_t index, target;
  float tmp;
  char cmd;

  // Wait until serial is available
  if (Serial.available()) {
    cmd = Serial.read();
    index = Serial.parseInt();
    // Parse index first; if parseInt times out, index = 0
    if (index <= 0 || index > N_MOTOR) {
      Serial.println("Index OOB or timeout");
      return;
    }
    index--;
    // Parse cmd
    switch (cmd) {
      case '\n':
      case ' ':
        break;
      case 'p':
        Serial.print("TMC4361A_BOW1: ");
        Serial.println(tmc4361[index].rampParam[BOW1_IDX]);
        Serial.print("TMC4361A_BOW2: ");
        Serial.println(tmc4361[index].rampParam[BOW2_IDX]);
        Serial.print("TMC4361A_BOW3: ");
        Serial.println(tmc4361[index].rampParam[BOW3_IDX]);
        Serial.print("TMC4361A_BOW4: ");
        Serial.println(tmc4361[index].rampParam[BOW4_IDX]);
        Serial.print("TMC4361A_AMAX: ");
        Serial.println(tmc4361[index].rampParam[AMAX_IDX]);
        Serial.print("TMC4361A_DMAX: ");
        Serial.println(tmc4361[index].rampParam[DMAX_IDX]);
        Serial.print("TMC4361A_ASTART: ");
        Serial.println(tmc4361[index].rampParam[ASTART_IDX]);
        Serial.print("TMC4361A_DFINAL: ");
        Serial.println(tmc4361[index].rampParam[DFINAL_IDX]);
        Serial.print("TMC4361A_VMAX: ");
        Serial.println(tmc4361[index].rampParam[VMAX_IDX]);
      case 'S':
      case 's':
        // Change from mm to microsteps
        if (cmd == 'S') {
          tmp = Serial.parseFloat();
          tmp = tmp * ((float)(MICROSTEPS * STEP_PER_REV)) / ((float)(PITCH));
          target = tmp;
        }
        else {
          target = Serial.parseInt();
        }
        // Check if in bounds
        if (target < tmc4361[index].xmin || target > tmc4361[index].xmax) {
          Serial.println("Target OOB");
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
        Serial.print("Current Position: ");
        Serial.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_XACTUAL));
        // Go to target
        Serial.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_EVENTS), BIN);
        tmc4361A_writeInt(&tmc4361[index], TMC4361A_X_TARGET, target);
        Serial.println(tmc4361A_readInt(&tmc4361[index], TMC4361A_EVENTS), BIN);
        Serial.print("Time: ");
        Serial.println(millis());
        break;

      case 'w':
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
    case 'a':
    case 'A':
    case 'b':
    case 'B':
      // change from mm/s^3 to microsteps/s^3
      if (cmd == 'A' || cmd == 'B') {
        tmp = Serial.parseFloat();
        tmp = tmp * ((float)(MICROSTEPS * STEP_PER_REV)) / ((float)(PITCH));
        target = tmp;
      }
      else {
        target = Serial.parseInt();
      }
      Serial.print("The target is ");
      Serial.println(target);
      // Check if above limit
      if (target > BOWMAX) {
        Serial.print("Target too high, must be below ");
        Serial.println(BOWMAX);
        break;
      }
      if (cmd == 'a' || cmd == 'A') {
        // Set the bow parameters - bow 1 and bow 4; indices 0 and 3

        setSRampParam(&tmc4361[index], BOW1_IDX, target);
        setSRampParam(&tmc4361[index], BOW4_IDX, target);
        Serial.print("Set bows 1 and 4");
      }
      else {
        // Set the bow parameters - bow 2 and bow 3; indices 1 and 2
        setSRampParam(&tmc4361[index], BOW2_IDX, target);
        setSRampParam(&tmc4361[index], BOW3_IDX, target);
        Serial.print("Set bows 1 and 4");
      }

      break;

    case 'c':
    case 'C':
      Serial.println("Setting VMax");
      // change from mm/s to microsteps/s * 2^-8
      if (cmd == 'C') {
        Serial.println("Convert from mm to microsteps");
        tmp = Serial.parseFloat();
        tmp = tmp * ((float)(MICROSTEPS * STEP_PER_REV)) / ((float)(PITCH));
        tmp *= (1 << 8);
        target = tmp;
      }
      else {
        target = Serial.parseInt();
      }
      Serial.print("The target is ");
      Serial.println(target);

      if (abs(target) > (1 << 8) * (clk_Hz_TMC4361 / 4)) {
        Serial.print("Target too high, must be below ");
        Serial.println((1 << 8)*clk_Hz_TMC4361 / 4);
        break;
      }

      // Set the VMAX parameter
      setSRampParam(&tmc4361[index], VMAX_IDX, target);
      Serial.print("Set VMAX to ");
      Serial.println(tmc4361[index].rampParam[VMAX_IDX]);
      break;
    default:
      break;
  }

  // Show results
  for (int i = 0; i < N_MOTOR; i++) {
    // todo: implement using events register instead of status register (ideally no need for prevstate[])

    target  = tmc4361A_readInt(&tmc4361[i], TMC4361A_STATUS) & TMC4361A_TARGET_REACHED_MASK;
    if (target != 0 && prevstate[i] == 0) {
      Serial.print("Motor with CS pin ");
      Serial.print(tmc4361[i].config->channel);
      Serial.println(" has reached its target");
      Serial.print("Time: ");
      Serial.println(millis());
    }
    prevstate[i] = target;
  }
}
