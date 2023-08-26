#include "DACx050x.h"
#include "TMC4361A.h"
#include "TMC4361A_TMC2660_Utils.h"

#define PIEZO_N_RETRIES      1
#define PIEZO_SETTLE_TIME_MS 6

bool piezo_set_offset(TMC4361ATypeDef *tmc4361A, DACx050x *dac, int32_t initial_pos, int32_t mstep_expand, uint32_t err);
void piezo_calibrate_limits(TMC4361ATypeDef *tmc4361A, DACx050x *dac, uint8_t n_measurements);
