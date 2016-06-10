/*
 * IncFile1.h
 *
 * Created: 5/3/2016 9:40:26 AM
 *  Author: LOMAS
 */ 


#ifndef SENTRAL_INTERFACE_H_
#define SENTRAL_INTERFACE_H_

#include <compiler.h>
#include <stdio.h>
#include <string.h>
#include <i2c_master.h>
#include "samb11g18a.h"

#define SENTRAL_I2C_MODULE  I2C1

#define SENTRAL_PIN_PAD0 PIN_LP_GPIO_14_MUX4_I2C1_SDA
#define SENTRAL_PIN_PAD1 PIN_LP_GPIO_15_MUX4_I2C1_SCL

#define SENTRAL_MUX_PAD0 MUX_LP_GPIO_14_MUX4_I2C1_SDA
#define SENTRAL_MUX_PAD1 MUX_LP_GPIO_15_MUX4_I2C1_SCL


#define SENTRAL_ADDRESS					0x28 // Slave Address of SENtral

/* --- SENtral Status Registers --- */
#define SENTRAL_RST_REQ_REG				0x9B
#define SENTRAL_EEPROM_STATUS_REG		0x37
#define SENTRAL_ALGORITHM_STATUS_REG	0x38
#define SENTRAL_PASS_THROUGH_STATUS_REG 0x9E

/* --- SENtral Control Registers --- */
#define SENTRAL_SET_MAG_RATE_REG		0x55
#define SENTRAL_SET_ACCL_RATE_REG		0x56
#define SENTRAL_SET_GYRO_RATE_REG 		0x57
#define SENTRAL_SET_RATE_DIV_REG		0x32
#define SENTRAL_ALGORITHM_CTRL_REG		0x54
#define SENTRAL_ENABLE_EVENTS_REG		0x33
#define SENTRAL_HOST_CTRL_REG			0x34
#define SENTRAL_PASS_THROUGH_CTRL_REG	0xA0

#define SLAVE_ADDRESS SENTRAL_ADDRESS

/* --- SENtral Commands  --- */

#define SENTRAL_RESET_CMD				0X01

/* Results Length in Bytes */
#define  SENTRAL_RESULT_LENGTH			0x2A // there are 42 Bytes of data out from SENtral

/*
 * Initialize I2C bus for communicating with SENtral Module
 */
void i2c_init_sentral(void);
/*
 * I2C Based Functions 
 */
void i2c_sentral_read(uint8_t regAddr, uint8_t *getByte, uint8_t readByteLength);
uint8_t i2c_sentral_read_reg(uint8_t);
void i2c_sentral_write_register(uint8_t regAddr, uint8_t *writeByte, uint8_t writeByteLen);

/* 
 * Other Utility functions 
 */
void reset_sentral(void);
uint8_t config_sentral(void);
uint8_t check_sentral_eeprom_status(void);
/* 
 * Initialize SENtral Module !
 */
void init_sentral(void);

/*
 * Function for reading raw data 
 * From SENtral
 */
void sentral_read_raw_data(uint8_t *);

/*	
 *	Reading Heading, Pitch and Roll 
 *  data from SENtral Module  
 */
float sentral_read_heading(void);
float sentral_read_pitch(void);
float sentral_read_roll(void);

/* 
 * Delay Prototype 
 */
/*void delay(uint32_t count);*/


#endif /* SENTRAL_INTERFACE_H_ */