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
#define TCM2660_CSCALE           0x1F  // Current scale value on the TCM2660, ranges from 0x00 to 0x1F
#define TMC4361A_HOLD_SCALE_VAL  0xFF  // 0 to 255 
#define TMC4361A_DRV2_SCALE_VAL  0xFF
#define TMC4361A_DRV1_SCALE_VAL  0xFF
#define TMC4361A_BOOST_SCALE_VAL 0xFF

// Error handling macros
#define NO_ERR            0
#define ERR_OUT_OF_RANGE -1
#define ERR_MISC         -2

// Functions for user-facing API
void tcm4361A_tcm2660_init(TMC4361ATypeDef *tmc4361A, uint32_t clk_Hz_TMC4361);
void homeLeft(TMC4361ATypeDef *tmc4361A, int32_t v_slow, int32_t v_fast); // todo: generalize
void findRight(TMC4361ATypeDef *tmc4361A, int32_t v_slow);                // todo: generalize
void setMaxSpeed(TMC4361ATypeDef *tmc4361A, int32_t velocity);
int8_t setMaxAcceleration(TMC4361ATypeDef *tmc4361A, uint32_t acceleration);
int8_t moveTo(TMC4361ATypeDef *tmc4361A, int32_t x_pos);
int8_t move(TMC4361ATypeDef *tmc4361A, int32_t x_pos);
int32_t currentPosition(TMC4361ATypeDef *tmc4361A);
int32_t targetPosition(TMC4361ATypeDef *tmc4361A);
void setCurrentPosition(TMC4361ATypeDef *tmc4361A, int32_t position);
void stop(TMC4361ATypeDef *tmc4361A);
bool isRunning(TMC4361ATypeDef *tmc4361A);
int32_t mmToMicrosteps(float mm);
float   microstepsTomm(int32_t microsteps);
void enableLimitSwitch(TMC4361ATypeDef *tmc4361A, uint8_t pol_lft, uint8_t pol_rht);
void enableHomingLimit(TMC4361ATypeDef *tmc4361A, uint8_t sw, uint8_t pol_lft, uint8_t pol_rht);
uint8_t readLimitSwitches(TMC4361ATypeDef *tmc4361A);


// The following does not need to be accessed by the end user
#define LEFT_SW 0b01
#define RGHT_SW 0b10
#define BOWMAX 0x1FFFFFF   // (1<<24 - 1) 

void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat);
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, int32_t dat);
uint8_t readSwitchEvent(TMC4361ATypeDef *tmc4361A);
void sRampInit(TMC4361ATypeDef *tmc4361A);
void setSRampParam(TMC4361ATypeDef *tmc4361A, uint8_t idx, int32_t param);

#endif /* TMC_UTILS_H_ */
