/*
   Utils.h
   Contains utilities for running the TCM4361and TMC2660

    Created on: 7/8/2022
        Author: Kevin Marx
*/
#ifndef TMC_UTILS_H_
#define TMC_UTILS_H_

#include <stddef.h>
#include <stdint.h>
#include <SPI.h>
#include "TMC4361A.h" // These files come from the TCM4361A manufacturer


#define LEFT_SW 0b01
#define RGHT_SW 0b10

// Current Scale Values
#define TCM2660_CSCALE           0x1F  // Current scale value on the TCM2660, ranges from 0x00 to 0x1F
#define TMC4361A_HOLD_SCALE_VAL  0xFF  // 0 to 255 
#define TMC4361A_DRV2_SCALE_VAL  0xFF
#define TMC4361A_DRV1_SCALE_VAL  0xFF
#define TMC4361A_BOOST_SCALE_VAL 0xFF


void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
void tmc4361A_setBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat);
void tmc4361A_rstBits(TMC4361ATypeDef *tmc4361A, uint8_t address, uint32_t dat);
void tcm4361A_tcm2660_init(TMC4361ATypeDef *tmc4361A, uint32_t clk_Hz_TMC4361);
void enableLimitSwitch(TMC4361ATypeDef *tmc4361A, uint8_t pol_lft, uint8_t pol_rht);
void enableHomingLimit(TMC4361ATypeDef *tmc4361A, uint8_t sw, uint8_t pol_lft, uint8_t pol_rht);
uint8_t readLimitSwitches(TMC4361ATypeDef *tmc4361A);
uint8_t readSwitchEvent(TMC4361ATypeDef *tmc4361A);
void homeLeft(TMC4361ATypeDef *tmc4361A, int32_t v_slow, int32_t v_fast);
void findRight(TMC4361ATypeDef *tmc4361A, int32_t v_slow);
void sRampInit(TMC4361ATypeDef *tmc4361A, int32_t Bow1, int32_t Bow2, int32_t Bow3, int32_t Bow4, int32_t AMAX, int32_t DMAX, int32_t ASTART, int32_t DFINAL, int32_t VMAX);

#endif /* TMC_UTILS_H_ */
