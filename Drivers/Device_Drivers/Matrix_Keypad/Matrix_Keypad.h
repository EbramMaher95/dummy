/*
 * Matrix_Keypad.h
 *
 *  Created on: Jan 31, 2024
 *      Author: ramys
 */

#ifndef DEVICE_DRIVERS_MATRIX_KEYPAD_H_
#define DEVICE_DRIVERS_MATRIX_KEYPAD_H_

#include <stdint.h>

//this is where the MCAL drivers are called
#include "stm32f1xx_hal.h"

typedef struct {
	/// @brief defines the number of rows
	uint8_t Rows;

	/// @brief defines the number of columns
	uint8_t Columns;

	/// @brief Holds the status of all keys
	/// 32-bits means the max number of keys is 32
	uint32_t KeysStatus;

	/// @brief the GPIO handle where the input pins are attached
	GPIO_TypeDef *IntputPort;

	/// @brief the starting index of the first input pin
	uint32_t InputStartingPin;

	/// @brief The GPIO Handle where the output pins are attached
	GPIO_TypeDef *OutputPort;

	/// @brief the starting index of the first Output Pin
	uint32_t OutputStartingPin;

	/// @brief internally used to hold the 1s mask used for IO operations
	uint16_t IP_Mask;
	uint16_t OP_Mask;
} Matrix_Keypad_t;

/// @brief Configres the hardware to use keypad
/// @param keypad
void Keypad_Init(Matrix_Keypad_t *keypad);

/// @brief Function periodically called by the user to refresh the status of the keys
/// @param keypad
void Keypad_Refresh(Matrix_Keypad_t *keypad);

/// @fn uint8_t Keypad_Get_Key(Matrix_Keypad_t*, uint8_t)
/// @brief
/// retrieves the logic level of a specific key
/// @param keypad ptr to the keypad structure
/// @param key numbr of the key to be read (starting from key #0
/// @return 1 if the key was pressed and o otherwise
uint8_t Keypad_Get_Key(Matrix_Keypad_t *keypad, uint8_t key);
#endif /* DEVICE_DRIVERS_MATRIX_KEYPAD_H_ */
