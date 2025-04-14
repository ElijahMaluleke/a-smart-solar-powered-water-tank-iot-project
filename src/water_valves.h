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
void water_valves_test(uint32_t msleepTime, uint32_t valvesOnCount); 
void water_valve_in(bool WaterValveInCount);
void water_valve_out(bool WaterValveInCount);

#ifdef __cplusplus
}
#endif

#endif

/********************************************************************************
 * 
 ********************************************************************************/