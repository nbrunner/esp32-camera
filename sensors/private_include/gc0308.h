#pragma once

#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCCB_ID                       0x21

#define GC032A_CHIP_ID                0X232A
#define GC320A_CHIP_MIDH              0xf0      // The upper 8 bits of the factory ID
#define GC320A_CHIP_MIDL              0xf1      // The lower 8 bits of the factory ID

#define GC0308_CHIP_ID                0x9b

int gc0308_init(sensor_t *sensor);

#ifdef __cplusplus
}
#endif
