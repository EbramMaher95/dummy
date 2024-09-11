/*
 * 24C32.c
 *
 *  Created on: Aug 30, 2024
 *      Author: Perom
 */

#include "24C32.h"

//defining the slave address -> from the datasheet of the eeprom module (hardware dependent)
#define eeprom_SLA	0b1010000

#define eeprom_write_address 123

#define start_address 0b00000000

#define EEPROM_I2C_TIMEOUT 100

static uint8_t I2C__write(uint8_t slave, uint8_t *data, uint8_t len,
		eeprom24c32_t *eeprom) {

	//master send data to the slave (RTC)
	HAL_StatusTypeDef OK = HAL_I2C_Master_Transmit(eeprom->i2c_bus,
			(eeprom_SLA << 1), data, len, 100);

	//check for the acknowledgment flag -> if 1 returned -> not acknowledged
	return (OK == HAL_OK) ? 1 : 0;
}

static uint8_t I2C__read(uint8_t slave, uint8_t *data, uint8_t len,
		eeprom24c32_t *eeprom) {

	//master receive data from the slave (RTC)
	HAL_StatusTypeDef OK = HAL_I2C_Master_Receive(eeprom->i2c_bus,
			(eeprom_SLA << 1), data, len, 100);

	//check for the acknowledgment flag -> if 1 returned -> not acknowledged
	return (OK == HAL_OK) ? 1 : 0;
}

//initializing the eeprom
eeprom_state_t eeprom24c32_init(eeprom24c32_t *eeprom,
		I2C_HandleTypeDef *i2c_bus) {

	uint8_t status = 1;
	eeprom->i2c_bus = i2c_bus;

	// Clear buffer and write to EEPROM
	eeprom->i2c_buffer[0] = 0x00;

	// Return the appropriate status
	return (status == 1) ? eeprom_OK : eeprom_NOK;

}

/*
 HAL_StatusTypeDef eeprom24c32_write(eeprom24c32_t *eeprom,
 uint32_t *buffer_data, uint16_t mem_address) {


 HAL_StatusTypeDef status = HAL_I2C_Mem_Write(eeprom->i2c_bus,
 (eeprom_SLA << 1), mem_address, I2C_MEMADD_SIZE_16BIT,
 (uint8_t*) buffer_data, 1, EEPROM_I2C_TIMEOUT);
 HAL_Delay(20);  // According to EEPROM datasheet
 return status;
 }



 HAL_StatusTypeDef eeprom24c32_read(eeprom24c32_t *eeprom, uint16_t *data,
 uint16_t mem_address) {
 uint8_t read_data = 0;
 HAL_StatusTypeDef status = HAL_I2C_Mem_Read(eeprom->i2c_bus,
 (eeprom_SLA << 1), mem_address, I2C_MEMADD_SIZE_16BIT, &read_data,
 1, EEPROM_I2C_TIMEOUT);
 *data = read_data;
 return status;
 }
 */

//write to the eeprom function
uint8_t eeprom24c32_write(eeprom24c32_t *eeprom, uint32_t *buffer_data,
		uint16_t mem_ddress) {
	uint8_t status;

	//for testing
	//eeprom->i2c_buffer[0] = 8;

	status = HAL_I2C_Mem_Write(eeprom->i2c_bus, (eeprom_SLA << 1), mem_ddress,
			2, (uint8_t*)buffer_data, 1, 100);

	//delay according to the datasheet
	HAL_Delay(20);

	return status;

}

//read data from the eeprom
uint16_t eeprom24c32_read(eeprom24c32_t *eeprom, uint16_t *data,
		uint16_t mem_ddress) {

	uint8_t status;
	uint8_t read_data;

	status = HAL_I2C_Mem_Read(eeprom->i2c_bus, (eeprom_SLA << 1), mem_ddress, 2,
			&read_data, 1, 100);

	// Store the read byte into the data variable
	*data = read_data;

	return status;

}

