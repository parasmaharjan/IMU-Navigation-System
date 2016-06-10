/**
 * \file
 *
 * \brief SAM UART Quick Start
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "sentral/sentral_interface.h"
#include "fastmath.h"

struct uart_module uart_instance;

static uint8_t string_input[8];
volatile static bool read_complete_flag = false;
volatile static bool write_complete_flag = false;
uint32_t task_1 = 0;
uint32_t task_2 = 0;

#define CONF_TIMER_RELOAD_VALUE 26000000/1000

static void timer_callback(void)
{
	task_1++;
	task_2++;
}

static void configure_gpio_pins(void)
{
	struct gpio_config config_gpio_pin;
	gpio_get_config_defaults(&config_gpio_pin);
	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
}

static void configure_timer(void)
{
	struct timer_config config_timer;
	timer_get_config_defaults(&config_timer);
	config_timer.reload_value = CONF_TIMER_RELOAD_VALUE;
	timer_init(&config_timer);
	timer_enable();
}

static void configure_timer_callback(void)
{
	timer_register_callback(timer_callback);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

static void uart_read_complete_callback(struct uart_module *const module)
{
	read_complete_flag = true;
}

static void uart_write_complete_callback(struct uart_module *const module)
{
	write_complete_flag = true;
}

static void configure_uart(void)
{
	struct uart_config config_uart;

	uart_get_config_defaults(&config_uart);

	config_uart.baud_rate = 115200;
	config_uart.pin_number_pad[0] = EDBG_CDC_SERCOM_PIN_PAD0;
	config_uart.pin_number_pad[1] = EDBG_CDC_SERCOM_PIN_PAD1;
	config_uart.pin_number_pad[2] = EDBG_CDC_SERCOM_PIN_PAD2;
	config_uart.pin_number_pad[3] = EDBG_CDC_SERCOM_PIN_PAD3;
	
	config_uart.pinmux_sel_pad[0] = EDBG_CDC_SERCOM_MUX_PAD0;
	config_uart.pinmux_sel_pad[1] = EDBG_CDC_SERCOM_MUX_PAD1;
	config_uart.pinmux_sel_pad[2] = EDBG_CDC_SERCOM_MUX_PAD2;
	config_uart.pinmux_sel_pad[3] = EDBG_CDC_SERCOM_MUX_PAD3;

	while (uart_init(&uart_instance,
			EDBG_CDC_MODULE, &config_uart) != STATUS_OK) {
	}
	
	stdio_serial_init(&uart_instance, EDBG_CDC_MODULE, &config_uart);
	
}

extern void SysTick_Handler(void)
{
	printf("Handler\n\r");	
}

uint8_t rawSensorData[6];
char temp_x[10], temp_y[10], temp_z[10], temp_roll[10],temp_pitch[10], temp_heading[10];
char q0[10],q1[10],q2[10],q3[10];
char buffer[100];
float ax = 0.0, ay = 0.0, az = 0.0;
float acc_mag = 0.0, acc_mac_filter = 0.0, acc_mag_previous = 0.0;
float fXg, fYg, fZg;
float alpha = 0.1;
float roll, pitch, heading;

float quat[4];
int16_t accelCount[3];
float ax, ay, az;
int16_t gyroCount[3];
float gx, gy, gz;

void readQuatData(float * destination)
{
	uint8_t rawData[16];
	uint32_t data;
	for(uint8_t i = 0; i < 16; i++) {
		rawData[i] = i2c_sentral_read_reg(0x00 + i);
	}
	data = ((rawData[3] << 24)|(rawData[2] << 16)|(rawData[1] << 8) | rawData[0]) ;
	memcpy(&destination[0], &data, 4);
	data = ((rawData[7] << 24)|(rawData[6] << 16)|(rawData[5] << 8) | rawData[4]) ;
	memcpy(&destination[1], &data, 4);
	data = ((rawData[11] << 24)|(rawData[10] << 16)|(rawData[9] << 8) | rawData[8]) ;
	memcpy(&destination[2], &data, 4);
	data = ((rawData[15] << 24)|(rawData[14] << 16)|(rawData[13] << 8) | rawData[12]) ;
	memcpy(&destination[3], &data, 4);
}

void readMagData(int16_t * destination)
{
	uint8_t rawData[6];
	for(uint8_t i = 0; i < 6; i++) {
		rawData[i] = i2c_sentral_read_reg(0x12 + i);
	}
	destination[0] = (int16_t)((rawData[1] << 8) | rawData[0]) ;
	destination[1] = (int16_t)((rawData[3] << 8) | rawData[2]) ;
	destination[2] = (int16_t)((rawData[5] << 8) | rawData[4]) ;
}

void readAccelData(int16_t * destination)
{
	uint8_t rawData[6];
	for(uint8_t i = 0; i < 6; i++) {
		rawData[i] = i2c_sentral_read_reg(0x1A + i);
	}
	destination[0] = (int16_t)((rawData[1] << 8) | rawData[0]) ;
	destination[1] = (int16_t)((rawData[3] << 8) | rawData[2]) ;
	destination[2] = (int16_t)((rawData[5] << 8) | rawData[4]) ;
}

void readGyroData(int16_t * destination)
{
	uint8_t rawData[6];
	for(uint8_t i = 0; i < 6; i++) {
		rawData[i] = i2c_sentral_read_reg(0x22 + i);
	}
	destination[0] = (int16_t)((rawData[1] << 8) | rawData[0]) ;
	destination[1] = (int16_t)((rawData[3] << 8) | rawData[2]) ;
	destination[2] = (int16_t)((rawData[5] << 8) | rawData[4]) ;
}

double atan2(double y, double x)
{
	if(x > 0){
		return atan(y/x);
	}else if(x < 0 & y >= 0){
		return (atan(y/x)+(22/7));
	}else if(x <0 & y < 0){
		return (atan(y/x)-(22/7));
	}else if (x == 0 & y > 0){
		return (22/14);
	}else if(x == 0 & y < 0){
		return (-(22/14));
	}else
		return 0.0;
} 

int main(void)
{
	/**
	 * For make this QS work, disable the systick to stop task switch.
	 * Should not do it if you want the BLE functions.
	 */
	
	//SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
	
	configure_gpio_pins();
	
	configure_uart();
	
	//printf("New Program started\n\rSystick: %d\n\rCore Clock: %d\n\r",SysTick_Config(26000), system_clock_get_value());
	
	configure_timer();
	
	configure_timer_callback();
	
	i2c_init_sentral();
	
	init_sentral();
	
	while (true)
	{
		if(task_1 > 100)
		{
			gpio_pin_toggle_output_level(LED_0_PIN);
			task_1 = 0;
		}
		if(task_2 > 50)
		{
// 			readAccelData(&accelCount);
// 			ax = (float)accelCount[0]/16384.0;
// 			ay = (float)accelCount[1]/16384.0;
// 			az = (float)accelCount[2]/16384.0;
// 			
// 			acc_mag = sqrt(ax*ax+ay*ay+az*az);
// 			
// 			acc_mac_filter = 0.1 * (acc_mac_filter + acc_mag - acc_mag_previous);
// 			acc_mag_previous = acc_mag;
			
			
			/*Quaternion data*/
			readQuatData(quat);
			gcvt(quat[0] , 5 , q0);
			gcvt(quat[1] , 5 , q1);
			gcvt(quat[2] , 5 , q2);
			gcvt(quat[3] , 5 , q3);
			//printf("%s:%s:%s:%s\n\r",q0,q1,q2,q3);
			
			heading = atan2((quat[0]*quat[0] - quat[1] * quat[1] -quat[2] * quat[2] + quat[3] * quat[3]) , (2 * (quat[0] * quat[1] + quat[2] * quat[3])));
			pitch = asin((-2) * (quat[0] * quat[2] - quat[1] * quat[3]));
			roll = atan2((- quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]) , (2 * (quat[0] * quat[3] + quat[1] * quat[2])));
			
			gcvt(roll,5,temp_roll);
			gcvt(pitch,5,temp_pitch);
			gcvt(heading,5,temp_heading);
			
			printf("%s:%s:%s\n\r", temp_roll, temp_pitch, temp_heading);
// 			
// 			/*Raw Accelerometer data*/
// 			readAccelData(&accelCount);
// 			ax = (float)accelCount[0]/16384.0;
// 			ay = (float)accelCount[1]/16384.0;
// 			az = (float)accelCount[2]/16384.0;
// 			
// 			//Low Pass Filter
// 			fXg = ax * alpha + (fXg * (1.0 - alpha));
// 			fYg = ay * alpha + (fYg * (1.0 - alpha));
// 			fZg = az * alpha + (fZg * (1.0 - alpha));
// 
// 			//Roll & Pitch Equations
// 			roll  = (atan(-fYg/ fZg)*180.0)/3.14;
// 			pitch = (atan(fXg/ sqrt(fYg*fYg + fZg*fZg))*180.0)/3.14;
// 			
// 			gcvt(ax,5,temp_x);
// 			gcvt(ay,5,temp_y);
// 			gcvt(az,5,temp_z);
// 
// 			gcvt(roll,5,temp_roll);
// 			gcvt(pitch,5,temp_pitch);
			
			//printf("%s:%s\n\r",temp_pitch,temp_roll);
			
			task_2 = 0;
		}
	}
	
	uint8_t string1[] = " Hello World!\r\n";
	uart_write_buffer_wait(&uart_instance, string1, sizeof(string1));

	uint8_t string3[] = " Test callback functions, please input 8 number or character!\r\n";
	uart_register_callback(&uart_instance, uart_write_complete_callback,
		UART_TX_COMPLETE);
	uart_enable_callback(&uart_instance, UART_TX_COMPLETE);
	uart_write_buffer_job(&uart_instance, string3, sizeof(string3));
	while (!write_complete_flag);

	uart_register_callback(&uart_instance, uart_read_complete_callback,
		UART_RX_COMPLETE);
	uart_enable_callback(&uart_instance, UART_RX_COMPLETE);
	uart_read_buffer_job(&uart_instance, string_input, sizeof(string_input));
	while (!read_complete_flag);

	uint8_t string4[] = " Data received: ";
	uart_write_buffer_wait(&uart_instance, string4, sizeof(string4));
	uart_write_buffer_wait(&uart_instance, string_input, sizeof(string_input));

	uint8_t temp;

	uint8_t string2[] = "\r\n Enter while loop, echo back your input!\r\n";
	uart_write_buffer_wait(&uart_instance, string2, sizeof(string2));

	while (true) {
		if (uart_read_wait(&uart_instance, &temp) == STATUS_OK) {
			while (uart_write_wait(&uart_instance, temp) != STATUS_OK) {
			}
		}
	}
}
