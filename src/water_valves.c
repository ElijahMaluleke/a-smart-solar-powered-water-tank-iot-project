/********************************************************************************
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include "water_valves.h"
#include "pins_definitions.h"

/********************************************************************************
 * 
 ********************************************************************************/
#define WV_DEVICE_GPIO0 DT_NODELABEL(gpio0)
#define WV_DEVICE_GPIO1 DT_NODELABEL(gpio1)
#define WV_DEVICE_GPIO2 DT_NODELABEL(gpio2)

/********************************************************************************
 * 
 ********************************************************************************/
// 
const struct device *wv_gpio0_dev = DEVICE_DT_GET(WV_DEVICE_GPIO0);
// 
const struct device *wv_gpio1_dev = DEVICE_DT_GET(WV_DEVICE_GPIO1);
// 
const struct device *wv_gpio2_dev = DEVICE_DT_GET(WV_DEVICE_GPIO2);

/********************************************************************************
 * @} water_valves_init
 ********************************************************************************/
void water_valves_init(void) 
{
	// set up water_valves pins 
	gpio_pin_configure(wv_gpio2_dev, WATER_VALVE_IN, GPIO_OUTPUT_HIGH);	
	//gpio_pin_set(wv_gpio2_dev, WATER_VALVE_IN, true);
	// set up water_valves pins 
	gpio_pin_configure(wv_gpio2_dev, WATER_VALVE_OUT, GPIO_OUTPUT_HIGH);	
	//gpio_pin_set(wv_gpio2_dev, WATER_VALVE_OUT, true);

}

/********************************************************************************
 * @} water_valves_test
 ********************************************************************************/
void water_valves_test(uint32_t msleepTime, uint32_t valvesOnCount) 
{
	uint32_t i;
	
	for(i = 0; i < valvesOnCount; i++) 
	{
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_IN, false);
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_OUT, false);
		k_msleep(msleepTime);
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_IN, true);
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_OUT, true);
		k_msleep(msleepTime);
	}
}

/********************************************************************************
 * @} water_valve_in
 ********************************************************************************/
void water_valve_in(bool WaterValveInCount) 
{
	if(WaterValveInCount == ON) 
	{
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_IN, false);
		gpio_pin_set(wv_gpio2_dev, GREEN_LED, true);
	}
	else 	
	{
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_IN, true);
		gpio_pin_set(wv_gpio2_dev, GREEN_LED, false);
	}
}

/********************************************************************************
 * @} water_valve_out
 ********************************************************************************/
void water_valve_out(bool WaterValveOutCount) 
{
	if(WaterValveOutCount == ON) 
	{
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_OUT, false);
	}
	else 	
	{
		gpio_pin_set(wv_gpio2_dev, WATER_VALVE_OUT, true);
	}
}

	