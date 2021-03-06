/*
   Utils.h
   Contains utilities for running the TCM4361 and TMC2660

    Created on: 7/8/2022
        Author: Kevin Marx
*/
#ifndef TMC_UTILS_H_
#define TMC_UTILS_H_

#include <stddef.h>
#include <stdint.h>
#include <SPI.h>
#include "TMC4361A.h"

// Motor settings - can modify these
#define PITCH      (float)2.54 // carriage parameter - 1 rotation is 2.54 mm
#define MICROSTEPS 256         // motor driver parameter - 1 rotation has 256 microsteps
#define STEP_PER_REV 200       // motor parameter - number of steps per full revolution

// Current Scale Values - can modify these
#define TMC2660_CSCALE           0x1F  // Current scale value on the TCM2660, ranges from 0x00 to 0x1F
#define TMC4361A_HOLD_SCALE_VAL  0xFF  // 0 to 255 
#define TMC4361A_DRV2_SCALE_VAL  0xFF
#define TMC4361A_DRV1_SCALE_VAL  0xFF
#define TMC4361A_BOOST_SCALE_VAL 0xFF

// Error handling macros
#define NO_ERR            0
#define ERR_OUT_OF_RANGE -1
#define ERR_MISC         -2

// Functions for user-facing API
void tmc4361A_tmc2660_init(TMC4361ATypeDef *tmc4361A, uint32_t clk_Hz_TMC4361);
void setMaxSpeed(TMC4361ATypeDef *tmc4361A, int32_t velocity);
void setSpeed(TMC4361ATypeDef *tmc4361A, int32_t velocity);
int32_t speed(TMC4361ATypeDef *tmc4361A);
int32_t acceleration(TMC4361ATypeDef *tmc4361A);
int8_t setMaxAcceleration(TMC4361ATypeDef *tmc4361A, uint32_t acceleration);
int8_t moveTo(TMC4361ATypeDef *tmc4361A, int32_t x_pos);
int8_t move(TMC4361ATypeDef *tmc4361A, int32_t x_pos);
int32_t currentPosition(TMC4361ATypeDef *tmc4361A);
int32_t targetPosition(TMC4361ATypeDef *tmc4361A);
void setCurrentPosition(TMC4361ATypeDef *tmc4361A, int32_t position);
void stop(TMC4361ATypeDef *tmc4361A);
bool isRunning(TMC4361ATypeDef *tmc4361A);
int32_t xmmToMicrosteps(float mm);
float   xmicrostepsTomm(int32_t microsteps);
int32_t vmmToMicrosteps(float mm);
float   vmicrostepsTomm(int32_t microsteps);
int32_t ammToMicrosteps(float mm);
float   amicrostepsTomm(int32_t microsteps);
void enableLimitSwitch(TMC4361ATypeDef *tmc4361A, uint8_t pol_lft, uint8_t pol_rht);
void enableHomingLimit(TMC4361ATypeDef *tmc4361A, uint8_t sw, uint8_t pol_lft, uint8_t pol_rht);
uint8_t readLimitSwitches(TMC4361ATypeDef *tmc4361A);
void setHome(TMC4361ATypeDef *tmc4361A);
void moveToExtreme(TMC4361ATypeDef *tmc4361A, int32_t vel, int8_t dir);


// The following does not need to be accessed by the end user
#define LEFT_SW 0b01
#define RGHT_SW 0b10
#define LEFT_DIR -1
#define RGHT_DIR  1
#define BOWMAX 0x1FFFFFF   // (1<<24 - 1) 

void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat);
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat);
uint8_t readSwitchEvent(TMC4361ATypeDef *tmc4361A);
void sRampInit(TMC4361ATypeDef *tmc4361A);
void setSRampParam(TMC4361ATypeDef *tmc4361A, uint8_t idx, int32_t param);
void adjustBows(TMC4361ATypeDef *tmc4361A);

#endif /* TMC_UTILS_H_ */
