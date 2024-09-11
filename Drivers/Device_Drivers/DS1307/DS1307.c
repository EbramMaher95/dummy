/*
 * DS1307.c
 *
 *  Created on: Jun 30, 2024
 *      Author: Perom
 */

#include "DS1307.h"

//defining the slave address -> from the datasheet of the RTC module (hardware dependent)
#define dev_SLA	0b1101000

#define start_address 0b00000000

static uint8_t I2C__write(uint8_t slave, uint8_t *data, uint8_t len,
		ds1307_t *clock) {

	//master send data to the slave (RTC)
	HAL_StatusTypeDef OK = HAL_I2C_Master_Transmit(clock->i2c_bus,
			(dev_SLA << 1), data, len, 100);

	//check for the acknowledgment flag -> if 1 returned -> not acknowledged
	return (OK == HAL_OK) ? 1 : 0;
}

static uint8_t I2C__read(uint8_t slave, uint8_t *data, uint8_t len,
		ds1307_t *clock) {

	//master receive data from the slave (RTC)
	HAL_StatusTypeDef OK = HAL_I2C_Master_Receive(clock->i2c_bus,
			(dev_SLA << 1), data, len, 100);

	//check for the acknowledgment flag -> if 1 returned -> not acknowledged
	return (OK == HAL_OK) ? 1 : 0;
}

uint8_t BCD2DEC(uint8_t bcd) {

	return ((bcd >> 4) * 10 + (0xF & bcd));

}

uint8_t DEC2BCD(uint8_t dec) {

	uint8_t div = dec / 10;
	uint8_t rem = dec % 10;

	return (div << 4 | rem);
}

//initializing the RTC
DS1307_state_t Ds1307_init(ds1307_t *clock, I2C_HandleTypeDef *i2c_bus) {

	uint8_t status = 1;

	clock->i2c_bus = i2c_bus;

	clock->i2c_buffer[0] = 0x00;
	status &= I2C__write(dev_SLA, clock->i2c_buffer, 1, clock);

	status &= I2C__read(dev_SLA, clock->i2c_buffer, 1, clock);

	if (((clock->i2c_buffer[0]) & (1 << 7)) == 0) {

	} else {

		clock->i2c_buffer[0] = 0x00; 	//reg base address

		clock->i2c_buffer[1] = 0; 		//clock hold bit

		status &= I2C__write(dev_SLA, clock->i2c_buffer, 2, clock);
	}

	if (status == 1) {

		return DS1307_OK;
	} else {
		return DS1307_NOK;
	}

}

//set the time function
DS1307_state_t Ds1307_set(ds1307_t *clock) {

	clock->i2c_buffer[0] = start_address;

	clock->i2c_buffer[1] = DEC2BCD(clock->sec);

	clock->i2c_buffer[2] = DEC2BCD(clock->min);

	//in case of PMAM is selected (12H format)
	if (clock->format == 1) {

		clock->i2c_buffer[3] = DEC2BCD(clock->hour) | ((clock->format) << 6)
				| ((clock->AM_PM) << 5);
	}

	//case of 24h format
	else {

		clock->i2c_buffer[3] = DEC2BCD(clock->hour) | ((clock->AM_PM) << 5);

	}

	clock->i2c_buffer[4] = DEC2BCD(clock->day) & 0x7;

	clock->i2c_buffer[5] = DEC2BCD(clock->date) & 0x3f;

	clock->i2c_buffer[6] = DEC2BCD(clock->month) & 0x1f;

	clock->i2c_buffer[7] = DEC2BCD(clock->year - 2000);

	if (I2C__write(dev_SLA, clock->i2c_buffer, 8, clock) == 1) {

		return DS1307_OK;
	} else {

		return DS1307_NOK;
	}
}

//read the time function
DS1307_state_t Ds1307_read(ds1307_t *clock) {
	clock->i2c_buffer[0] = start_address;

	    if (I2C__write(dev_SLA, clock->i2c_buffer, 1, clock) == 1) {
	        if (I2C__read(dev_SLA, clock->i2c_buffer, 7, clock) == 1) {
	            clock->sec = BCD2DEC(clock->i2c_buffer[0] & 0x7F);
	            clock->min = BCD2DEC(clock->i2c_buffer[1]);
	            clock->format = (clock->i2c_buffer[2] & 0b01000000) >> 6;

	            if (clock->format == 1) { // 12h format
	                clock->hour = BCD2DEC(clock->i2c_buffer[2] & 0b00011111);
	                clock->AM_PM = (clock->i2c_buffer[2] & 0b00100000) >> 5;
	            } else { // 24h format
	                clock->hour = BCD2DEC(clock->i2c_buffer[2] & 0b00111111);
	                clock->AM_PM = (clock->hour > 11) ? 1 : 0;
	            }

	            clock->day = BCD2DEC(clock->i2c_buffer[3]);
	            clock->date = BCD2DEC(clock->i2c_buffer[4]);
	            clock->month = BCD2DEC(clock->i2c_buffer[5]);
	            clock->year = BCD2DEC(clock->i2c_buffer[6]) + 2000;

	            return DS1307_OK;
	        }
	    }
	    return DS1307_NOK;
	}


