/********************************************************************************
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#ifndef WATER_VALVES_H 
#define WATER_VALVES_H

#ifdef __cplusplus
extern "C" {
#endif


#define SLEEP_TIME_MS 					1000


#define WATER_VALVE_IN_ON_SLEEP_TIME	100
#define WATER_VALVE_IN_OFF_SLEEP_TIME	50
#define WATER_VALVE_OUT_ON_SLEEP_TIME	100
#define WATER_VALVE_OUT_OFF_SLEEP_TIME	50


void water_valves_init(void);
void water_valves_test(void); 
void water_valve_in(uint32_t WaterValveInCount, uint8_t SleepTime);
void water_valve_out(uint32_t WaterValveOutCount, uint8_t SleepTime);

#ifdef __cplusplus
}
#endif

#endif

/********************************************************************************
 * 
 ********************************************************************************/