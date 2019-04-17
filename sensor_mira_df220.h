

#ifndef SENSOR_MIRA_DF220_H__
#define SENSOR_MIRA_DF220_H__

#include "sensor.h"
#include "df220.h"

#define DF220_ADDR_DEFAULT UINT8_C(0x27)

int rt_hw_df220_init(const char *name, struct rt_sensor_config *cfg);

#endif
