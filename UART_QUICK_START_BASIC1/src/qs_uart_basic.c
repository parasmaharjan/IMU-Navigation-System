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
char buffer[10];

struct uart_module uart_instance;

static uint8_t string_input[8];
volatile static bool read_complete_flag = false;
volatile static bool write_complete_flag = false;
uint32_t task_1 = 0;
uint32_t task_2 = 0;
uint32_t msTick = 0;

#define CONF_TIMER_RELOAD_VALUE 26000000/1000

static void timer_callback(void)
{
	msTick++;
	task_1++;
	task_2++;
}

void delay_ms(uint32_t d)
{
	msTick = 0;
	while(msTick < d);
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

int16_t dataAccGyro[7]; // 3- acc data 1-acc timestamp 3 gyro data
int16_t dataSentral[10]; // 3 - mag, 3 - acc, 1 - acc timestamp, 3 - gyro

void rawSentralData(int16_t * destination){
	uint8_t rawData[22];
	uint32_t data;
	int16_t data_16;
	for(uint8_t i = 0; i < 22; i++) {
		rawData[i] = i2c_sentral_read_reg(0x12 + i);
	}
		
	// Magnetometer - +-1000 uT ------> 200/65536 uT = 1 * 10/32.768 milli Gauss
	destination[0] = (int16_t)((rawData[1] << 8) | rawData[0]);
	destination[1] = (int16_t)((rawData[3] << 8) | rawData[2]);
	destination[2] = (int16_t)((rawData[5] << 8) | rawData[4]);
	
	// Accelerometer
	destination[3] = (int16_t)((rawData[9] << 8) | rawData[8]);
	destination[4] = (int16_t)((rawData[11] << 8) | rawData[10]);
	destination[5] = (int16_t)((rawData[13] << 8) | rawData[12]);
	
	// Acc time stamp
	destination[6] = (int16_t)((rawData[15] << 8) | rawData[14]);
	
	// Gyroscope
	destination[7] = (int16_t)((rawData[17] << 8) | rawData[16]);
	destination[8] = (int16_t)((rawData[19] << 8) | rawData[18]);
	destination[9] = (int16_t)((rawData[21] << 8) | rawData[20]);
}

void readSentralData(float * destination){
	uint8_t rawData[42];
	uint32_t data;
	int16_t data_16;
	uint16_t data_u16;
	static uint16_t previous_time = 0;
	for(uint8_t i = 0; i < 42; i++) {
		rawData[i] = i2c_sentral_read_reg(0x00 + i);
	}
	
	// Quaternion data
	data = ((rawData[3] << 24)|(rawData[2] << 16)|(rawData[1] << 8) | rawData[0]) ;
	memcpy(&destination[0], &data, 4);
	data = ((rawData[7] << 24)|(rawData[6] << 16)|(rawData[5] << 8) | rawData[4]) ;
	memcpy(&destination[1], &data, 4);
	data = ((rawData[11] << 24)|(rawData[10] << 16)|(rawData[9] << 8) | rawData[8]) ;
	memcpy(&destination[2], &data, 4);
	data = ((rawData[15] << 24)|(rawData[14] << 16)|(rawData[13] << 8) | rawData[12]) ;
	memcpy(&destination[3], &data, 4);
	
	// Magnetometer - +-1000 uT ------> 200/65536 uT = 1 * 10/32.768 milli Gauss
	data_16 = (int16_t)((rawData[19] << 8) | rawData[18]);
	destination[4] = (float)(data_16) * 10.0 / 32.768;
	data_16 = (int16_t)((rawData[21] << 8) | rawData[20]);
	destination[5] = (float)(data_16) * 10.0 / 32.768;
	data_16 = (int16_t)((rawData[23] << 8) | rawData[22]);
	destination[6] = (float)(data_16) * 10.0 / 32.768;
	
	// Accelerometer
	data_16 = (int16_t)((rawData[27] << 8) | rawData[26]);
	destination[7] = (float)(data_16) / 16384.0;
	data_16 = (int16_t)((rawData[29] << 8) | rawData[28]);
	destination[8] = (float)(data_16) / 16384.0;
	data_16 = (int16_t)((rawData[31] << 8) | rawData[30]);
	destination[9] = (float)(data_16) / 16384.0;
	
	// Gyroscope
	data_16 = (int16_t)((rawData[35] << 8) | rawData[34]);
	destination[10] = (float)(data_16) / 16.4;
	data_16 = (int16_t)((rawData[37] << 8) | rawData[36]);
	destination[11] = (float)(data_16) / 16.4;
	data_16 = (int16_t)((rawData[39] << 8) | rawData[38]);
	destination[12] = (float)(data_16) / 16.4;	
	
	// Gyro time stamp
	data_u16 = (uint16_t)((rawData[41] << 8) | rawData[40]);
// 	if(data_u16 > previous_time)
// 		printf("%d\n\r", (data_u16-previous_time));
// 	else
// 		printf("%d\n\r", (65536 - previous_time + data_u16));	
	
	if(data_u16 > previous_time)
		destination[13] += (float)(data_u16 - previous_time) / 32000.0;
	else
		destination[13] += (float)(65536 - previous_time + data_u16) / 32000.0;
	
// 	gcvt(destination[13], 5, buffer);
// 	printf("%s\n\r", buffer);
	
	previous_time = data_u16;
}

void rawAccGryoData(int16_t * destination){
	uint8_t rawData[15];
	for (uint8_t i = 0; i < 15; i++)
	{
		rawData[i] = i2c_sentral_read_reg(0x1A + i);
	}
	
	destination[0] = (int16_t)((rawData[1] << 8) | rawData[0]) ; // acc data
	destination[1] = (int16_t)((rawData[3] << 8) | rawData[2]) ;
	destination[2] = (int16_t)((rawData[5] << 8) | rawData[4]) ;
	
	destination[3] = (int16_t)((rawData[7] << 8) | rawData[6]) ; //timestamp
	
	destination[4] = (int16_t)((rawData[9] << 8) | rawData[8]) ; //gyro data
	destination[5] = (int16_t)((rawData[11] << 8) | rawData[10]) ;
	destination[6] = (int16_t)((rawData[13] << 8) | rawData[12]) ;
	
}

float aef[3]; //globle variable that returns value of rotated quaternion
static float cq[4];

static void quat_conj(float q[4]){
	cq[0] = q[0];
	cq[1] = -q[1];
	cq[2] = -q[2];
	cq[3] = -q[3];
}

void acc_to_earth_frame(float acc[3], float q[4]){
	float acq[4];
	float a[4];
	float quat[4];
	//including 0 to make 1x4 matrix
	a[0] = 0.0;
	a[1] = acc[0];
	a[2] = acc[1];
	a[3] = acc[2];

	//calculate conjugate
	quat_conj(q);

	// quat product
	acq[0] = a[0] * cq[0] - a[1] * cq[1] - a[2] * cq[2] - a[3] * cq[3];
	acq[1] = a[0] * cq[1] - a[1] * cq[0] - a[2] * cq[3] - a[3] * cq[2];
	acq[2] = a[0] * cq[2] - a[1] * cq[3] - a[2] * cq[0] - a[3] * cq[1];
	acq[3] = a[0] * cq[3] - a[1] * cq[2] - a[2] * cq[1] - a[3] * cq[0];

	// quat product
	quat[0] = a[0] * q[0] - a[1] * q[1] - a[2] * q[2] - a[3] * q[3];
	quat[1] = a[0] * q[1] - a[1] * q[0] - a[2] * q[3] - a[3] * q[2];
	quat[2] = a[0] * q[2] - a[1] * q[3] - a[2] * q[0] - a[3] * q[1];
	quat[3] = a[0] * q[3] - a[1] * q[2] - a[2] * q[1] - a[3] * q[0];

	//copy to aef
	aef[0] = quat[1];
	aef[1] = quat[2];
	aef[2] = quat[3];
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

#include "Kalman.h"
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double dt;

float roll, pitch, heading;
float data[14];

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

int main(void)
{
	system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
	
	configure_gpio_pins();
	
	configure_uart();
	
	configure_timer();
	
	configure_timer_callback();
	
	i2c_init_sentral();
	
	init_sentral();
	
	readSentralData(data);
	heading = atan2((data[0]*data[0] - data[1] * data[1] -data[2] * data[2] + data[3] * data[3]) , (2 * (data[0] * data[1] + data[2] * data[3]))) * 57.32;
	pitch = asin((-2) * (data[0] * data[2] - data[1] * data[3])) * 57.32;
	roll = atan2((- data[0] * data[0] - data[1] * data[1] + data[2] * data[2] + data[3] * data[3]) , (2 * (data[0] * data[3] + data[1] * data[2]))) * 57.32;
	
	
	setAngle(roll);
	gyroXangle = roll;
	compAngleX = roll;
	
	while (true)
	{
		if(task_1 > 100)
		{
			gpio_pin_toggle_output_level(LED_0_PIN);
			task_1 = 0;
		}
		if(task_2 > 100)
		{
			readSentralData(data);
			
			accX = data[7];
			accY = data[8];
			accZ = data[9];
			gyroX = data[10];
			gyroY = data[11];
			gyroZ = data[12];
			dt = data[13];
			dt = 0.1;
				
			roll  = atan2(accY, accZ) * 57.32;
			pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * 57.32;
			heading = 0;
			
// 			heading = atan2((data[0]*data[0] - data[1] * data[1] -data[2] * data[2] + data[3] * data[3]) , (2 * (data[0] * data[1] + data[2] * data[3]))) * 57.32;
// 			pitch = asin((-2) * (data[0] * data[2] - data[1] * data[3])) * 57.32;
// 			roll = atan2((- data[0] * data[0] - data[1] * data[1] + data[2] * data[2] + data[3] * data[3]) , (2 * (data[0] * data[3] + data[1] * data[2]))) * 57.32;
			
// 			if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
// 				setAngle(roll);
// 				compAngleX = roll;
// 				kalAngleX = roll;
// 				gyroXangle = roll;
// 			}
// 			else
				kalAngleX = kalmanFilterX(roll, gyroX, dt); // Calculate the angle using a Kalman filter
				kalAngleY = kalmanFilterY(pitch, gyroY, dt);
				kalAngleZ = kalmanFilterZ(heading, gyroZ, dt);
			
			gyroXangle += gyroX * dt; // Calculate gyro angle without any filter
			gyroZangle += gyroZ * dt;
			gyroYangle += gyroY * dt;
			
			compAngleX = 0.93 * (compAngleX + gyroX * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
			compAngleY = 0.93 * (compAngleY + gyroY * dt) + 0.07 * pitch;
			compAngleZ = 0.93 * (compAngleZ + gyroZ * dt) + 0.07 * heading;
			
// 			// Reset the gyro angle when it has drifted too much
// 			if (gyroXangle < -180 || gyroXangle > 180)
// 				gyroXangle = kalAngleX;
			
// 			gcvt(heading, 5, buffer);
// 			printf(buffer);
// 			printf("\t");
// 			gcvt(gyroZangle, 5, buffer);
// 			printf(buffer);
// 			printf("\t");
// 			gcvt(compAngleZ, 5, buffer);
// 			printf(buffer);
// 			printf("\t");
// 			gcvt(kalAngleZ, 5, buffer);
// 			printf(buffer);
// 			printf("\n\r");

			gcvt(roll, 5, buffer);
			printf(buffer);
			printf("\t");
			gcvt(gyroXangle, 5, buffer);
			printf(buffer);
			printf("\t");
			gcvt(compAngleX, 5, buffer);
			printf(buffer);
			printf("\t");
			gcvt(kalAngleX, 5, buffer);
			printf(buffer);
			printf("\n\r");
						
			task_2 = 0;
		}
		
// 		rawSentralData(&dataSentral);
// 		printf("%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\n\r",(int16_t)dataSentral[0],(int16_t)dataSentral[1],(int16_t)dataSentral[2],(int16_t)dataSentral[3],(int16_t)dataSentral[4],(int16_t)dataSentral[5],(uint16_t)dataSentral[6],(int16_t)dataSentral[7],(int16_t)dataSentral[8],(int16_t)dataSentral[9]);
		
	}
}
