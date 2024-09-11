/*
 * 24C32.h
 *
 *  Created on: Aug 30, 2024
 *      Author: Perom
 */

#ifndef DEVICE_DRIVERS_24C32_EEPROM_24C32_H_
#define DEVICE_DRIVERS_24C32_EEPROM_24C32_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

//structure for the definition
typedef struct {

	//buffer variable for hours, minutes, seconds, no. of doses, Am or PM, 12 or 24h
	uint32_t i2c_buffer[7];



	//

	//variable for the memory address
	uint16_t mem_add;
	/*
	 //variables for sec, min, hours, day, date, month, year
	 uint8_t sec, min, hour, day, date, month;

	 //the size of year is larger because uint8 cannot hold values starting with 2000
	 uint16_t year;

	 //variable for checking if running or not
	 uint8_t ch; //ch is for clock hold

	 uint8_t format :1;

	 uint8_t AM_PM :1;
	 */
	//
	I2C_HandleTypeDef *i2c_bus;

} eeprom24c32_t;

//enum for eeprom status
typedef enum {
	eeprom_OK, eeprom_NOK,
} eeprom_state_t;

//initializing the eeprom
eeprom_state_t eeprom24c32_init(eeprom24c32_t *eeprom,
		I2C_HandleTypeDef *i2c_bus);

//write data to the eeprom function
uint8_t eeprom24c32_write(eeprom24c32_t *eeprom, uint32_t *buffer_data,
		uint16_t mem_ddress);
//read data from the eeprom
uint16_t eeprom24c32_read(eeprom24c32_t *eeprom, uint16_t *data,
		uint16_t mem_ddress);

#endif /* DEVICE_DRIVERS_24C32_EEPROM_24C32_H_ */
