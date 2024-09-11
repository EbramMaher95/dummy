/*
 * Matrix_Keypad.c
 *
 *  Created on: Jan 31, 2024
 *      Author: ramys
 */

#include "Matrix_Keypad.h"

/// @brief Reads the input status from the input port
static uint16_t Keypad_GetInputs(Matrix_Keypad_t *keypad) {
	uint16_t Input = keypad->IntputPort->IDR & keypad->IP_Mask;
	return (Input >> keypad->InputStartingPin);
}

///@brief Write the status of output ports
static void Keypad_SetOutputs(Matrix_Keypad_t *keypad, uint16_t OutputStat) {
	//turn off the pins
	keypad->OutputPort->BRR |= keypad->OP_Mask;
	//turn on only the needed ones according to the required output
	keypad->OutputPort->BSRR |= (OutputStat << keypad->OutputStartingPin);
}

/// @fn void Keypad_Delay(uint32_t)
/// @brief
/// Halts the program execution for a certain time
/// @param delay_ms delay value in milliseconds
static void Keypad_Delay(uint32_t delay_ms) {
	HAL_Delay(delay_ms);
}

void Keypad_Refresh(Matrix_Keypad_t *keypad) {
// temporary variable to store the new status of keys
	uint32_t TempKeys = 0;
	uint16_t OutputStat = 1 << (keypad->Rows - 1);

	uint16_t x = 0;
	do {

		TempKeys <<= keypad->Columns;
		// send the output status to the hardware
		Keypad_SetOutputs(keypad, OutputStat);
		Keypad_Delay(1);
		// read the input status from the hardware

		TempKeys |= Keypad_GetInputs(keypad);
		OutputStat >>= 1;
		x++;
	} while (x < keypad->Rows);

	keypad->KeysStatus = TempKeys;
}

void Keypad_Init(Matrix_Keypad_t *keypad) {
	GPIO_InitTypeDef config;

//put a logic 1 in every position where an input pin is selected
	uint16_t Pins = 0;

//configure the columns gpio pins as inputs
//enable pull down resistors to eliminate the use of external ones
	for (int x = 0; x < keypad->Columns; x++) {
		Pins |= (1 << (x + keypad->InputStartingPin));
	}
//Load the input pin Mask
	keypad->IP_Mask = Pins;

	config.Mode = GPIO_MODE_INPUT;
	config.Pull = GPIO_PULLDOWN;
	config.Pin = Pins;
	HAL_GPIO_Init(keypad->IntputPort, &config);

//configure the rows gpio pins as outputs
	Pins = 0;
	for (int x = 0; x < keypad->Rows; x++) {
		Pins |= (1 << (x + keypad->OutputStartingPin));
	}

//Load the output pin Mask
	keypad->OP_Mask = Pins;

	config.Mode = GPIO_MODE_OUTPUT_PP;
	config.Speed = GPIO_SPEED_FREQ_LOW;
	config.Pin = Pins;
	HAL_GPIO_Init(keypad->IntputPort, &config);

//turn off all the outputs
	Keypad_SetOutputs(keypad, 0);
}

uint8_t Keypad_Get_Key(Matrix_Keypad_t *keypad, uint8_t key) {
	return (keypad->KeysStatus & (1 << key)) ? 1 : 0;
}
