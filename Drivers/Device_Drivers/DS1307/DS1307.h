/*
 * DS1307.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Perom
 */

#ifndef DEVICE_DRIVERS_DS1307_DS1307_H_
#define DEVICE_DRIVERS_DS1307_DS1307_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

//structure for the definition
typedef struct {

	//buffer variable
	uint8_t i2c_buffer[8];

	//variables for sec, min, hours, day, date, month, year
	uint8_t sec, min, hour, day, date, month;

	//the size of year is larger because uint8 cannot hold values starting with 2000
	uint16_t year;

	//variable for checking if running or not
	uint8_t ch; //ch is for clock hold

	uint8_t format :1;

	uint8_t AM_PM :1;

	//
	I2C_HandleTypeDef *i2c_bus;

} ds1307_t;

//enum for RTC status
typedef enum {
	DS1307_OK, DS1307_NOK,
} DS1307_state_t;

//initializing the RTC
DS1307_state_t Ds1307_init(ds1307_t *clock, I2C_HandleTypeDef *i2c_bus);

//set the time function
DS1307_state_t Ds1307_set(ds1307_t *clock);

//read the time function
DS1307_state_t Ds1307_read(ds1307_t *clock);

#endif /* DEVICE_DRIVERS_DS1307_DS1307_H_ */
