/* main.c - Synchronization demo */

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
#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_systick.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <nrfx_saadc.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

#include "pins_definitions.h"
#include "water_valves.h"

/********************************************************************************
 * The synchronization demo has two threads that utilize semaphores and sleeping
 * to take turns printing a greeting message at a controlled rate. The demo
 * shows both the static and dynamic approaches for spawning a thread; a real
 * world application would likely use the static approach for both threads.
 ********************************************************************************/
#define PIN_THREADS (IS_ENABLED(CONFIG_SMP) && IS_ENABLED(CONFIG_SCHED_CPU_MASK))

/* size of stack area used by each thread */
#define STACKSIZE 													1024

/* scheduling priority used by each thread */
#define PRIORITY 														7

/* delay between greetings (in ms) */
#define SLEEPTIME  							 						500
/* 2200 msec = 2.2 sec */
#define PRODUCER_SLEEP_TIME_MS 							2200

/* Stack size for both the producer and consumer threads */
#define PRODUCER_THREAD_PRIORITY 						6
#define CONSUMER_THREAD_PRIORITY 						7

#define CONFIG_APP_VERSION									"1.0.0"
#define DEVICE_NAME            							CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         						(sizeof(DEVICE_NAME) - 1)

#define RUN_LED_BLINK_INTERVAL  						1000
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 											1000
#define SLEEP_TIME_S												1000
#define SLEEP_TIME_HALF_S										500
#define SLEEP_TIME_QUOTA_S									250

#define THREAD_YIELD_TIME 									100000

#define ULTRASONIC_SENSOR										0x80

#define DEVICE_NAME				CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN			(sizeof(DEVICE_NAME) - 1)

/********************************************************************************/
//
// Define the battery sample interval 
#define BATTERY_SAMPLE_INTERVAL_MS 					5000

#define CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL	4

// node label
#define DEVICE_GPIO0 												DT_NODELABEL(gpio0)
#define DEVICE_GPIO1 												DT_NODELABEL(gpio1)
#define DEVICE_GPIO2 												DT_NODELABEL(gpio2)

/********************************************************************************
 *
 ********************************************************************************/
static bool get_object_ultrasonic_range_sensor(float* dist);
static bool get_water_ultrasonic_range_sensor(float* dist);
static void timer2_init(void);
static void set_conversion_factor(void);
static void object_distance_proximity(uint32_t Distance, uint8_t Sensor) ;
static void water_level_proximity(uint32_t Level, uint8_t Sensor) ;
static void configure_saadc(void);
static void timer0_handler(struct k_timer *dummy);
// Add forward declaration of timer callback handler 
static void battery_sample_timer_handler(struct k_timer *timer);
static void bat_status_led(uint32_t msleep_time, uint8_t BlinkCount);
static int are_devices_ready(void);
static void flash_all_leds(void);
static void beep(uint32_t BeepCount, uint32_t FirstBeepDelay, uint32_t LastBeepDelay);
static void test_nrfx_systick_delay(void);
static void configure_all_gpios(void);
static int bt_init(void);
static int bt_send(void);

/********************************************************************************
 *
 ********************************************************************************/
// Declare the buffer to hold the SAAD sample value
static int16_t sample;
// counter */
static volatile uint32_t tCount = 0;
static volatile uint32_t tCountTemp = 0;
// count to us (micro seconds) conversion factor
static volatile float countToUs = 1;
static float dist_ultrasonic;
static uint8_t prescaler = 1;
static uint16_t comp1 = 500;
// 
static int bat_volt = 0;
static bool bat_low_status = false;
static bool sensors_status = true;
static bool pir_status = false;
uint32_t object_distance_sensor = 0;
uint32_t water_level_sensor = 0;

/********************************************************************************
 * Define the data type of the message
 ********************************************************************************/
typedef struct 
{
	//
	uint32_t ultrasonic_sensor_reading;
	
} SensorReading;

/********************************************************************************
 *
 ********************************************************************************/
static struct k_timer timer_sensors;
static struct gpio_callback pir_cb_data;

// 
const struct device *gpio0_dev = DEVICE_DT_GET(DEVICE_GPIO0);
// 
const struct device *gpio1_dev = DEVICE_DT_GET(DEVICE_GPIO1);
// 
const struct device *gpio2_dev = DEVICE_DT_GET(DEVICE_GPIO2);

// Declare the struct to hold the configuration for the SAADC channel used to sample the battery voltage 
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AVDD, 0);

/********************************************************************************
 *
 ********************************************************************************/

// Define the battery sample timer instance 
K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);
//
LOG_MODULE_REGISTER(a_smart_white_cane, CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL);

/********************************************************************************
 * 
 ********************************************************************************/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};


static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

/********************************************************************************
 * 
 ********************************************************************************/
static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);

	printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

/********************************************************************************
 * 
 ********************************************************************************/
static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	char message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	memcpy(message, data, MIN(sizeof(message) - 1, len));
	printk("%s() - Len: %d, Message: %s\n", __func__, len, message);
}

/********************************************************************************
 * 
 ********************************************************************************/
struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = received,
};

/********************************************************************************
 * set conversion factor
 ********************************************************************************/ 
static void set_conversion_factor(void) 
{
	//
	countToUs = 0.0625*comp1*(1 << prescaler);
}

/********************************************************************************
 *
 ********************************************************************************/
ISR_DIRECT_DECLARE(timer2_handler) 
{
	// 
	if((NRF_TIMER20->EVENTS_COMPARE[1]) && ((NRF_TIMER20->INTENSET) & 
	    (TIMER_INTENSET_COMPARE1_Msk))) {
		  NRF_TIMER20->TASKS_CLEAR = 1;
		  NRF_TIMER20->EVENTS_COMPARE[1] = 0;
		  tCount++;
		  //printk("Timer count: >%d<\n", tCount); // printk
		  //printf("Timer count: >%X<\n", tCount); // printf
		  //LOG_INF("Timer count: >%d<\n", tCount); // LOG_INF

	}
	ISR_DIRECT_PM();
	return 1;
}

/********************************************************************************
 * Set up and start Timer1
 * // set prescalar n
 * // f = 16 MHz / 2^(n)
 *
 * // 16 MHz clock generates timer tick every 1/(16000000) s = 62.5 nano s
 * // With compare enabled, the interrupt is fired every: 62.5 * comp1 nano s
 * // = 0.0625*comp1 micro seconds
 * // multiply this by 2^(prescalar)
 ********************************************************************************/
static void timer2_init(void) 
{
	//
	IRQ_DIRECT_CONNECT(TIMER20_IRQn, IRQ_PRIO_LOWEST, timer2_handler, 0);
	irq_enable(TIMER20_IRQn);
	NRF_TIMER20->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER20->TASKS_CLEAR = 1;
	NRF_TIMER20->PRESCALER = prescaler;
	NRF_TIMER20->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	NRF_TIMER20->CC[1] = comp1;
	NRF_TIMER20->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos;
	NRF_TIMER20->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos;
	set_conversion_factor();
	printf("timer tick = %f us\n", (double)countToUs);
	NRF_TIMER20->TASKS_START = 1;
}

/********************************************************************************
 * Stop Timer1
 ********************************************************************************/
static void stop_timer(void) 
{
	// 
	NRF_TIMER20->TASKS_STOP = 1;
}

/********************************************************************************
 * Start Timer1
 ********************************************************************************/
static void start_timer(void) 
{
	// 
	NRF_TIMER20->TASKS_START = 1;
}

/********************************************************************************
 * Create the expiry function for the timer
 ********************************************************************************/
/*static void timer0_handler(struct k_timer *dummy) 
{
	// Interrupt Context - Sysetm Timer ISR 
	static bool flip = true;
	if (flip) {
		gpio_pin_toggle(gpio2_dev, LED_ONE);
	} else {
		gpio_pin_toggle(gpio2_dev, LED_ONE);
	}

	flip = !flip;
}*/

/******************************************************************************** 
 * Implement timer callback handler function 
 ********************************************************************************/
void battery_sample_timer_handler(struct k_timer *timer) 
{
  // Trigger the sampling
  nrfx_err_t err = nrfx_saadc_mode_trigger();
  if(err != NRFX_SUCCESS) {
    printk("nrfx_saadc_mode_trigger error: %08x", err);
    return;
  }

  // Calculate and print voltage 
  int battery_voltage = ((600*6) * sample) / ((1<<12));
	bat_volt = battery_voltage;
	
  //printk("SAADC sample: %d\n", sample);
  printk("Battery Voltage: %d mV\n", bat_volt);
	bat_low_status = true;
}

/******************************************************************************** 
 * 
 ********************************************************************************/
static void configure_saadc(void) 
{
  // Connect ADC interrupt to nrfx interrupt handler 
  IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)), DT_IRQ(DT_NODELABEL(adc), priority), nrfx_isr, nrfx_saadc_irq_handler, 0);
        
  // Connect ADC interrupt to nrfx interrupt handler */
  nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
  if(err != NRFX_SUCCESS) {
      printk("nrfx_saadc_mode_trigger error: %08x\n", err);
      return;
  }

  // Configure the SAADC channel
  channel.channel_config.gain = NRF_SAADC_GAIN1_2;
  //channel.channel_config.resistor_p = NRF_SAADC_AIN6;
  //channel.channel_config.input_positive = NRF_SAADC_AIN6;
  err = nrfx_saadc_channels_config(&channel, 1);
  if(err != NRFX_SUCCESS) {
		printk("nrfx_saadc_channels_config error: %08x\n", err);
	  return;
	}

  // Configure nrfx_SAADC driver in simple and blocking mode 
  err = nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
                                  
  if(err != NRFX_SUCCESS) 
  {
    printk("nrfx_saadc_simple_mode_set error: %08x\n", err);
    return;
  }
        
  // Set buffer where sample will be stored 
  err = nrfx_saadc_buffer_set(&sample, 1);
  if(err != NRFX_SUCCESS) 
  {
    printk("nrfx_saadc_buffer_set error: %08x\n", err);
    return;
  }

  // Start periodic timer for battery sampling 
	k_timer_start(&battery_sample_timer, K_NO_WAIT, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));
}

/********************************************************************************
 * @} get_ultrasonic_range_sensor
 ********************************************************************************/
static bool get_object_ultrasonic_range_sensor(float* dist) 
{	
	// 
	sensors_status = false;
	gpio_pin_set(gpio2_dev, O_TRIG_ULTRASONIC_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio2_dev, O_TRIG_ULTRASONIC_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, O_ECHO_ULTRASONIC_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, O_ECHO_ULTRASONIC_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) 
	{
		return true;
	}
	else 
	{
		return false;
	}
}

/********************************************************************************
 * @} get_ultrasonic_range_sensor
 ********************************************************************************/
static bool get_water_ultrasonic_range_sensor(float* dist) 
{	
	// 
	sensors_status = false;
	gpio_pin_set(gpio1_dev, W_TRIG_ULTRASONIC_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio1_dev, W_TRIG_ULTRASONIC_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, W_ECHO_ULTRASONIC_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, W_ECHO_ULTRASONIC_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) 
	{
		return true;
	}
	else 
	{
		return false;
	}
}

/********************************************************************************
 * @} object_proximity
 ********************************************************************************/
static void object_distance_proximity(uint32_t Distance, uint8_t Sensor) 
{
	//
	switch(Sensor) 
	{	
		case ULTRASONIC_SENSOR:
		{
			if((Distance >= 0)&&(Distance <= 10)){
				water_valve_out(1 , 18);
			}
			else if((Distance >= 10)&&(Distance <= 20)){
				water_valve_out(1 , 16);
			}
			else if((Distance >= 20)&&(Distance <= 30)){
				water_valve_out(1 , 14);
			}
			else if((Distance >= 30)&&(Distance <= 40)){
				water_valve_out(1 , 12);
			}
			else if((Distance >= 40)&&(Distance <= 50)){
				water_valve_out(1 , 10);
			}
			else if((Distance >= 60)&&(Distance <= 60)){
				water_valve_out(1 , 8);
			}
			else if((Distance >= 70)&&(Distance <= 70)){
				water_valve_out(1 , 6);
			}
			else if((Distance >= 80)&&(Distance <= 90)){
				water_valve_out(1 , 4);
			}
			if((Distance >= 90)&&(Distance <= 100)){
				water_valve_out(1 , 18);
			}
			else{
			}
		}
		break;
	
		default:
		break;
	}	
}

/********************************************************************************
 * @} object_proximity
 ********************************************************************************/
static void water_level_proximity(uint32_t Level, uint8_t Sensor) 
{
	//
	switch(Sensor) 
	{	
		case ULTRASONIC_SENSOR:
		{
			if((Level >= 0)&&(Level <= 10)){
				water_valve_in(1 , 18);
			}
			else if((Level >= 10)&&(Level <= 20)){
				water_valve_in(1 , 16);
			}
			else if((Level >= 20)&&(Level <= 30)){
				water_valve_in(1 , 14);
			}
			else if((Level >= 30)&&(Level <= 40)){
				water_valve_in(1 , 12);
			}
			else if((Level >= 40)&&(Level <= 50)){
				water_valve_in(1 , 10);
			}
			else if((Level >= 60)&&(Level <= 60)){
				water_valve_in(1 , 8);
			}
			else if((Level >= 70)&&(Level <= 70)){
				water_valve_in(1 , 6);
			}
			else if((Level >= 80)&&(Level <= 90)){
				water_valve_in(1 , 4);
			}
			if((Level >= 90)&&(Level <= 100)){
				water_valve_in(1 , 18);
			}
			else{
			}
		}
		break;
	
		default:
		break;
	}	
}

/********************************************************************************
 * timer_sensors_exp_fnct
 ********************************************************************************/
static void timer_sensors_exp_fnct(struct k_timer *timer_id) 
{

	printk("timer_sensors_expiry_function\n");
	if(sensors_status) {
		k_timer_start(&timer_sensors, K_MSEC(1000), K_NO_WAIT);
	}
	else{
		k_timer_start(&timer_sensors, K_MSEC(200), K_NO_WAIT);
	}
	gpio_pin_toggle(gpio2_dev, SENSORS_STATUS_LED);	
}

/********************************************************************************
 * Define the callback function
 ********************************************************************************/
void pir_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) 
{	
	//
	pir_status = true;
	printk("\n\nObect detected!\n");
}

/********************************************************************************
 * 
 ********************************************************************************/
static void bat_status_led(uint32_t msleep_time, uint8_t BlinkCount) 
{
	uint8_t i;
	// 
	for(i = 0; i < BlinkCount; i++) 
	{
		gpio_pin_set(gpio1_dev, BAT_STATUS_LED, false);
		k_msleep(msleep_time);
		gpio_pin_set(gpio1_dev, BAT_STATUS_LED, true);
		k_msleep(msleep_time);	
	}
}

/********************************************************************************
 * @param my_name      thread identification string
 * @param my_sem       thread's own semaphore
 * @param other_sem    other thread's semaphore
 ********************************************************************************/
void hello_loop(const char *my_name, struct k_sem *my_sem, struct k_sem *other_sem)
{
	const char *tname;
	uint8_t cpu;
	struct k_thread *current_thread;

	while (1) 
	{
		// take my semaphore 
		k_sem_take(my_sem, K_FOREVER);

		current_thread = k_current_get();
		tname = k_thread_name_get(current_thread);
		#if CONFIG_SMP
			cpu = arch_curr_cpu()->id;
		#else
			cpu = 0;
		#endif
		// say "hello"
		if (tname == NULL) {
			printk("%s: Hello World from cpu %d on %s!\n", my_name, cpu, CONFIG_BOARD);
		} else {
			printk("%s: Hello World from cpu %d on %s!\n", tname, cpu, CONFIG_BOARD);
		}

		/* wait a while, then let other thread have a turn */
		k_busy_wait(THREAD_YIELD_TIME);
		k_msleep(SLEEPTIME);
		k_sem_give(other_sem);
	}
}

// define semaphores 
K_SEM_DEFINE(thread_a_sem, 1, 1);	// starts off "available" 
K_SEM_DEFINE(thread_b_sem, 0, 1);	// starts off "not available" 

/********************************************************************************
 * thread_a is a dynamic thread that is spawned in main
 ********************************************************************************/
void thread_a_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	// invoke routine to ping-pong hello messages with thread_b 
	hello_loop(__func__, &thread_a_sem, &thread_b_sem);
}
K_THREAD_STACK_DEFINE(thread_a_stack_area, STACKSIZE);
static struct k_thread thread_a_data;

/********************************************************************************
 * thread_b is a static thread spawned immediately
 ********************************************************************************/
void thread_b_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	// invoke routine to ping-pong hello messages with thread_a 
	hello_loop(__func__, &thread_b_sem, &thread_a_sem);
}
K_THREAD_DEFINE(thread_b, STACKSIZE, thread_b_entry_point, NULL, NULL, NULL, PRIORITY, 0, 0);
extern const k_tid_t thread_b;
	
/********************************************************************************
 * test_nrfx_systick_delay
 ********************************************************************************/
static void test_nrfx_systick_delay(void)
{
	uint64_t i = 0;
	uint64_t j = 0;
	//
	for(i = 0; i < 5; i++) 
	{
		gpio_pin_set(gpio2_dev, LED_ONE, true);
		gpio_pin_set(gpio1_dev, LED_TWO, true);
		gpio_pin_set(gpio2_dev, LED_THREE, true);
		gpio_pin_set(gpio1_dev, LED_FOUR, true);
		
		for(j = 0; j < 1000; j++) 
		{
			nrfx_systick_delay_us(1000);
		}
		gpio_pin_set(gpio2_dev, LED_ONE, false);
		gpio_pin_set(gpio1_dev, LED_TWO, false);
		gpio_pin_set(gpio2_dev, LED_THREE, false);
		gpio_pin_set(gpio1_dev, LED_FOUR, false);
		
		for(j = 0; j < 1000; j++) 
		{
			nrfx_systick_delay_us(1000);
		}
	} 
}
	
/********************************************************************************
 * flash_all_leds
 ********************************************************************************/
static void flash_all_leds(void)
{
	uint64_t i = 0;
	//
	for(i = 0; i < 5; i++) 
	{
		gpio_pin_set(gpio2_dev, LED_ONE, true);
		gpio_pin_set(gpio1_dev, LED_TWO, true);
		gpio_pin_set(gpio2_dev, LED_THREE, true);
		gpio_pin_set(gpio1_dev, LED_FOUR, true);
		k_msleep(SLEEP_TIME_MS);
		gpio_pin_set(gpio2_dev, LED_ONE, false);
		gpio_pin_set(gpio1_dev, LED_TWO, false);
		gpio_pin_set(gpio2_dev, LED_THREE, false);
		gpio_pin_set(gpio1_dev, LED_FOUR, false);
		k_msleep(SLEEP_TIME_MS);
	} 
}

/********************************************************************************
 * beep 
 ********************************************************************************/
static void beep(uint32_t BeepCount, uint32_t FirstBeepDelay, uint32_t LastBeepDelay)
{
	uint64_t i;
	//
	for(i = 0; i < BeepCount; i++) 
	{
		gpio_pin_set(gpio2_dev, BUZZER, false);
		k_msleep(FirstBeepDelay);
		gpio_pin_set(gpio2_dev, BUZZER, true);
		k_msleep(LastBeepDelay);
	} 
}

/********************************************************************************
 * bt init
 ********************************************************************************/
static int bt_init(void)
{
	int err;

	printk("Bluetooth Peripheral NUS Init\n");

	// bt nus cb register
	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("Failed to register NUS callback: %d\n", err);
		return err;
	}

	// bt enable
	err = bt_enable(NULL);
	if (err) {
		printk("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	// bt le adv start
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Failed to start advertising: %d\n", err);
		return err;
	}

	// 
	printk("Initialization complete\n");
	return 0;
}

/********************************************************************************
 * bt send
 ********************************************************************************/
static int bt_send(void)
{
	int err;

	//
	const char *water_level = "83%\n";
	const char *water_valve_in_status = "Open\n";
	const char *water_valve_out_status = "Closed\n";
	const char *water_flow_sensor = "Flowing\n";

	k_sleep(K_SECONDS(3));

	//
	err = bt_nus_send(NULL, water_level, strlen(water_level));
	printk("Data send - Result: %d\n", err);

	if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
		return err;
	}
	k_sleep(K_SECONDS(1));

	//
	err = bt_nus_send(NULL, water_valve_in_status, strlen(water_valve_in_status));
	printk("Data send - Result: %d\n", err);

	if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
		return err;
	}
	k_sleep(K_SECONDS(1));

	//
	err = bt_nus_send(NULL, water_valve_out_status, strlen(water_valve_out_status));
	printk("Data send - Result: %d\n", err);

	if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
		return err;
	}
	k_sleep(K_SECONDS(1));

	//
	err = bt_nus_send(NULL, water_flow_sensor, strlen(water_flow_sensor));
	printk("Data send - Result: %d\n", err);

	if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
		return err;
	}
	return 0;
}

/********************************************************************************
 * are_devices_ready
 ********************************************************************************/
static int are_devices_ready(void)
{
	int ret = 0;
	// 
	ret = device_is_ready(gpio0_dev);
	if (!ret) 
	{
		return ret;
	}
	//
	ret = device_is_ready(gpio1_dev); 
	if (!ret) 
	{
		return ret;
	}
	//
	ret = device_is_ready(gpio2_dev); 
	if (!ret) 
	{
		return ret;
	}
	return ret;
}

/********************************************************************************
 * configure_all_gpios
 ********************************************************************************/
static void configure_all_gpios(void)
{
	//
	gpio_pin_configure(gpio2_dev, OLED_I2C_SCL, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio2_dev, OLED_I2C_SDL, GPIO_OUTPUT_INACTIVE);
	//
	gpio_pin_configure(gpio2_dev, LED_ONE, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, LED_TWO, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio2_dev, LED_THREE, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, LED_FOUR, GPIO_OUTPUT_INACTIVE);
	// set up ultrasonic sensor pins 
	gpio_pin_configure(gpio1_dev, O_ECHO_ULTRASONIC_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_configure(gpio2_dev, O_TRIG_ULTRASONIC_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set(gpio2_dev, O_TRIG_ULTRASONIC_SENSOR, false);
	// set up ultrasonic sensor pins 
	gpio_pin_configure(gpio1_dev, W_ECHO_ULTRASONIC_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_configure(gpio1_dev, W_TRIG_ULTRASONIC_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set(gpio1_dev, W_TRIG_ULTRASONIC_SENSOR, false);
	//
	gpio_pin_configure(gpio2_dev, WATER_LEVEL_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_configure(gpio2_dev, WATER_FLOW_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);
	// set up buzzer pins 
	gpio_pin_configure(gpio2_dev, BUZZER, GPIO_OUTPUT_LOW);	
	gpio_pin_set(gpio2_dev, BUZZER, true);
}

/********************************************************************************
 *
 ********************************************************************************/
int main(void)
{
	int ret = 0;
	int blink_status = 0;
	tCount = 0;
	ARG_UNUSED(blink_status);
	ARG_UNUSED(start_timer);
	ARG_UNUSED(stop_timer);

	/*-------------------------------------------------------------------------
	 *
	 -------------------------------------------------------------------------*/
	printk("\n\nA Smart Solar-Powered Water Tank Level and Tap Monitor IoT ");
	printk("Project in Rural Areas Application started, version: %s\n",	CONFIG_APP_VERSION);
	
	if(!are_devices_ready()) {
		return ret;
	}
	
	// configure
	configure_all_gpios();
	water_valves_init();
	nrfx_systick_init();
	printk("Timer2 Init\n");
	timer2_init();
	// test
	test_nrfx_systick_delay();
	beep(5, SLEEP_TIME_MS, SLEEP_TIME_MS);
	water_valves_test(); 

	// set up PIR MODULE pins 
	gpio_pin_configure(gpio1_dev, PIR_MODULE_PIN, GPIO_INPUT | GPIO_PULL_DOWN);	//
	// Configure the interrupt on the PIR MODULE pin 
	gpio_pin_interrupt_configure(gpio1_dev, PIR_MODULE_PIN, GPIO_INT_EDGE_RISING);
	// Initialize the static struct gpio_callback variable 
	gpio_init_callback(&pir_cb_data, pir_interrupt_handler, BIT(PIR_MODULE_PIN));
	// Add the callback function by calling gpio_add_callback() 
	gpio_add_callback(gpio1_dev, &pir_cb_data);   

	bt_init();
	configure_saadc();   

	/*-------------------------------------------------------------------------
	 *
	 -------------------------------------------------------------------------*/ 
	for(;;)  
	{  
		//  
		uint8_t ultrasonic_sensor = 0x80;   

 		// get distance ultrasonic sensor
		if(get_object_ultrasonic_range_sensor(&dist_ultrasonic))  
		{
			object_distance_sensor = (uint32_t)dist_ultrasonic;
			printf("Object Distance = %d cm\n", object_distance_sensor);
			object_distance_proximity(object_distance_sensor, ultrasonic_sensor);
		}
		else 
 		{
			object_distance_sensor = (uint32_t)dist_ultrasonic;
			printf("Object Distance < 400.0 %d cm\n", object_distance_sensor);
		}      

 		/*----------------------------------------------------------------------*/
		//  
		
 		// get water level sensor
		if(get_water_ultrasonic_range_sensor(&dist_ultrasonic))  
		{
			water_level_sensor = (uint32_t)dist_ultrasonic;
			printf("Water Level = %d cm\n", water_level_sensor);
			water_level_proximity(water_level_sensor, ultrasonic_sensor);
		}
		else 
 		{
			water_level_sensor = (uint32_t)dist_ultrasonic;
			printf("Water Level < 400.0 %d cm\n", water_level_sensor);
		}       
 
		/*----------------------------------------------------------------------*/
		// pir status
		if(pir_status)  
		{
			pir_status = false;
			gpio_pin_set(gpio2_dev, PIR_MODULE_LED, true);
			k_msleep(SLEEP_TIME_MS);
			gpio_pin_set(gpio2_dev, PIR_MODULE_LED, false);  
			k_msleep(SLEEP_TIME_MS);
		}    
    
		/*----------------------------------------------------------------------*/		
		// bat volt
		if((bat_volt <= 1250) && (bat_low_status == true)) {
			printf("Battery Low = %d mV\n", bat_volt);
			water_valve_in(2, 6);
			bat_status_led(125, 6);
			bat_low_status = false;
		}
		  
    //
		bt_send();   
   
		// 
		//k_msleep(SLEEP_TIME_MS * 2);
	} 

	/*-------------------------------------------------------------------------
	 *
	 -------------------------------------------------------------------------*/
	// 
	//printk("Init Software Timers\n");
	//k_timer_init(&timer_sensors, timer_sensors_exp_fnct, NULL);
	//k_timer_start(&timer_sensors, K_MSEC(1000), K_NO_WAIT);
	// start periodic timer that expires once every 0.5 second 
	//k_timer_start(&timer0, K_MSEC(500), K_MSEC(500));

	/*-------------------------------------------------------------------------
	 *
	 -------------------------------------------------------------------------*/
	k_thread_create(&thread_a_data, thread_a_stack_area,
	K_THREAD_STACK_SIZEOF(thread_a_stack_area), thread_a_entry_point, NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);
			
	k_thread_name_set(&thread_a_data, "thread_a");

	#if PIN_THREADS
		if (arch_num_cpus() > 1) 
		{
			k_thread_cpu_pin(&thread_a_data, 0);

		/*-----------------------------------------------------------------------
		 * Thread b is a static thread that is spawned immediately. 
		 * This means that the
		 * following `k_thread_cpu_pin` call can fail with `-EINVAL` 
		 * if the thread is
		 * actively running. Let's suspend the thread and resume it after
		 * the affinity mask is set.
		 -----------------------------------------------------------------------*/
			k_thread_suspend(thread_b);
			k_thread_cpu_pin(thread_b, 1);
			k_thread_resume(thread_b);
		}
	#endif

	//configure_saadc();
	k_thread_start(&thread_a_data);
	return 0;
}

/********************************************************************************
 *
 ********************************************************************************/
