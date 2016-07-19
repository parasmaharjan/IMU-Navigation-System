/*
 * CFile1.c
 *
 * Created: 5/3/2016 9:41:08 AM
 *  Author: LOMAS
 */ 

#include "sentral_interface.h"

struct i2c_master_module i2c_master_instance;

#if 1	// This is the configuration information found in the 
		// SENtral User manual
		// This is recommended configuration
	static const uint8_t Configuration[][2]=
	{
		{SENTRAL_SET_MAG_RATE_REG, 0x64},		// Magn Rate 100Hz
		{SENTRAL_SET_ACCL_RATE_REG, 0x0A},		// Accel Rate 100Hz
		{SENTRAL_SET_GYRO_RATE_REG, 0x0F},		// Gyro rate 150Hz		
		{SENTRAL_SET_RATE_DIV_REG, 0x01},		// ODR divided by 1
		{SENTRAL_PASS_THROUGH_CTRL_REG, 0x00},	
		{SENTRAL_ALGORITHM_CTRL_REG, 0x02},		// Get Heading Pitch and Roll along with raw data
		{SENTRAL_ENABLE_EVENTS_REG, 0x7},		// Interrupts on data available 
												// but not cared when interrupt signal lines are not connected to Host
		{SENTRAL_HOST_CTRL_REG, 0x01}
	};
#endif

#if 0	// This is the configuration found in the sample code 
		// written for STM32
	static const uint8_t Configuration[][2]=
	{
		{SENTRAL_SET_MAG_RATE_REG, 30},			
		{SENTRAL_SET_ACCL_RATE_REG, 10},
		{SENTRAL_SET_GYRO_RATE_REG, 10},
		{SENTRAL_SET_RATE_DIV_REG, 0x00},		// ODR divided by 1
		{SENTRAL_PASS_THROUGH_CTRL_REG, 0x00},
		{SENTRAL_ALGORITHM_CTRL_REG, 0x02},		// Get Heading Pitch and Roll along with raw data
		{SENTRAL_ENABLE_EVENTS_REG, 0x7},		// Interrupts on data available
		// but not cared when interrupt signal lines are not connected to Host
		{SENTRAL_HOST_CTRL_REG, 0x01}
	};
#endif

/* Delay for this function only */
static void delay(uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		for (uint32_t j = 0; j < 1000; j++)
		asm volatile ("nop");
	}
	return;
}

void i2c_init_sentral(void)
{
	uint32_t time_overflow = 0;
	/* Initialize config structure and software module */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	
	config_i2c_master.pin_number_pad0 = SENTRAL_PIN_PAD0;
	config_i2c_master.pin_number_pad1 = SENTRAL_PIN_PAD1;
	config_i2c_master.pinmux_sel_pad0 = SENTRAL_MUX_PAD0;
	config_i2c_master.pinmux_sel_pad1 = SENTRAL_MUX_PAD1;
	
	config_i2c_master.clock_source = I2C_CLOCK_SOURCE_SELECT_CLOCK_3;
	config_i2c_master.clock_divider = 0x8;
	
	/* Initialize and enable device with config */
	while(i2c_master_init(&i2c_master_instance, SENTRAL_I2C_MODULE, &config_i2c_master) != STATUS_OK) {
		if(time_overflow++ > 10000) break;
	}
	i2c_enable(i2c_master_instance.hw);	
	/*delay(100);*/
	return;
}

/*---- Function definition for writing n bytes starting from particular register  -----*/
void i2c_sentral_write_register(uint8_t regAddr, uint8_t *writeByte, uint8_t writeByteLen) {

	uint8_t writeRegBytes[writeByteLen + 1], i;
	writeRegBytes[0] = regAddr;
	for(i= 0; i < writeByteLen; i++ ) {
		writeRegBytes[i+1] = writeByte[i];
	}

	struct i2c_master_packet writeRegAddr = {
		.address = SLAVE_ADDRESS,
		.data_length = sizeof(writeRegBytes),
		.data = writeRegBytes,
	};

	i2c_master_write_packet_wait(&i2c_master_instance, &writeRegAddr);
}

/* --- Function definition for reading N bytes of data starting from particular register ---*/
void i2c_sentral_read(uint8_t regAddr, uint8_t *getByte, uint8_t readByteLength) {
	
	struct i2c_master_packet writeRegAddr = {
		.address = SLAVE_ADDRESS,
		.data_length = 1,
		.data = &regAddr,
	};
	
	struct i2c_master_packet readBytes = {
		.address = SLAVE_ADDRESS,
		.data_length = readByteLength,
		.data = getByte,
	};
	i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &writeRegAddr);
	i2c_master_read_packet_wait(&i2c_master_instance, &readBytes);
	return;
}

/* --- Function definition for reading single bytes from particular register ---*/
uint8_t i2c_sentral_read_reg(uint8_t regAdr) {
	
	struct i2c_master_packet writeRgAddr = {
		.address = SLAVE_ADDRESS,
		.data_length = 1,
		.data = &regAdr,
	};
	uint8_t getByt;
	struct i2c_master_packet rdBytes = {
		.address = SLAVE_ADDRESS,
		.data_length = 1,
		.data = &getByt,
	};
	i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &writeRgAddr);
	i2c_master_read_packet_wait(&i2c_master_instance, &rdBytes);
	return getByt;
}


void reset_sentral(void) {	
	uint8_t myByte = SENTRAL_RESET_CMD;
	i2c_sentral_write_register(SENTRAL_RST_REQ_REG, &myByte, 1);
	return;
}

uint8_t config_sentral(void) {
	uint8_t testByte, i, count = 0;
	for(i = 0; i < sizeof(Configuration)/2; i++) {
		i2c_sentral_write_register(Configuration[i][0], (uint8_t *)&Configuration[i][1], 1);
		testByte = i2c_sentral_read_reg(Configuration[i][0]);	
			
		if(testByte != Configuration[i][1]) {
			count++;
			////printf("Error Uploading byte at register address : %d \n\r", Configuration[i][0]);
		}
	}
	
	/* Read the register and test values manually !! */
	#if 0
		////printf
		
		
		
		
	("Register Content of Algorithm Status Register : %d\r\n", i2c_sentral_read_reg(SENTRAL_ALGORITHM_STATUS_REG));	
		//printf("Register Content of Pass Through Status Register : %d\r\n", i2c_sentral_read_reg(SENTRAL_PASS_THROUGH_STATUS_REG));
	#endif
	
	if(i2c_sentral_read_reg(SENTRAL_ALGORITHM_STATUS_REG) & 0x01)	{
		//printf("Device in standby mode \r\n");
	}
	if(i2c_sentral_read_reg(SENTRAL_PASS_THROUGH_STATUS_REG) & 0x01){
		//printf("Device in Pass-Through mode \r\n");
	}
	return count;
}

uint8_t check_sentral_eeprom_status(void) {
	uint8_t myByte = i2c_sentral_read_reg(SENTRAL_EEPROM_STATUS_REG);	
	//printf("SENtral Status Reg Content : %d \r\n", myByte);
	if(myByte & 0x01) {
		////printf("EEPROM detected!\r\n");
		if(myByte & 0x02) {
			////printf("Done EEPROM upload...\r\n");
			if(myByte & 0x04){
				////printf("Uploaded with error / CRC Error !\r\n");
				} else {
				////printf("Uploaded Successful!\r\n");
				return true;
			}
			} else {
			////printf("EEPROM upload not done...\r\n");
			return false;
			delay(100);  // Wait for a moment
		}
		} else {
		////printf("EEPROM Not detected..\r\n");
		return false;
	}
	return 0;
}

void init_sentral(void) {	
	
	static uint8_t reset_count = 0;
	reset_count++;
	if(check_sentral_eeprom_status()) {
		if(0 < config_sentral()) {
			////printf("Error uploading data !");			
			} else {
			////printf("Done uploading control bytes into SENtral registers !\r\n");
		}
	} else {
		reset_sentral();
		delay(500);
		////printf("Resetting SENtral for %d time(s)!\r\n", reset_count);
		/* If the device is not configured correctly 
		 * Keep sending reset upload the command values 
		 * recursively.
		 */
		init_sentral(); 
	}
	////printf("Done initialization of SENtral.\r\n");
	return ;
}
void sentral_read_raw_data(uint8_t *data_buf) {
	i2c_sentral_read(0x00, data_buf, SENTRAL_RESULT_LENGTH);
	return;
}

float sentral_read_heading(void) {
	uint8_t QX[4];
	float qx;
	i2c_sentral_read(0x00,QX,sizeof(float));	
	memcpy(&qx,QX,sizeof(float));
	return qx;
}

float sentral_read_pitch(void) {
	uint8_t QY[4];
	float qy;
	i2c_sentral_read(0x04,QY,sizeof(float));	
	memcpy(&qy,QY,sizeof(float));	
	return qy;
}

float sentral_read_roll(void) {
	uint8_t QZ[4];
	float qz;
	i2c_sentral_read(0x08,QZ,sizeof(float));	
	memcpy(&qz,QZ,sizeof(float));	
	return qz;
}
