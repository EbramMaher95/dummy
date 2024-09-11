/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//the keypad structure
Matrix_Keypad_t kp = { .Rows = 4, .Columns = 4, .IntputPort = GPIOB,
		.OutputPort = GPIOB, .InputStartingPin = 6, .OutputStartingPin = 12 };
//the LCD structure:
Alcd_t lcd =
		{ .RS_GPIO = GPIOA, .RS_GPIO_Pin = GPIO_PIN_4, .EN_GPIO = GPIOA,
				.EN_GPIO_Pin = GPIO_PIN_5, .Data_GPIO = GPIOA,
				.Data_GPIO_Start_Pin = 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
ds1307_t CLK;

eeprom24c32_t memory;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	//initialize device drivers
	//first we initialize the lcd to display the updates
	//the LCD will be connected to: --> can be found in line 32 in the main.c file
	//A0,1,2,3 --> Data pins
	//A4:RS, A5:EN
	Alcd_Init(&lcd, 2, 16);

	//clear display
	Alcd_Clear(&lcd);

	Keypad_Init(&kp);

	//initialize the RTC
	Ds1307_init(&CLK, &hi2c2);

	//initialize the eeprom
	eeprom24c32_init(&memory, &hi2c2);

	//to initiate the base (counter)
	HAL_TIM_Base_Start(&htim1);

	//enable the OC pin (PWM pin)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//to change the duty cycle -> CCR
	//range is from 999 to 1999 (according to calculations)

	//the servo is set at 0 degree upon starting
	TIM1->CCR1 = 999;

	//a flag if button is pressed
	uint8_t password_comp_flag, error_code, delay_flag;
	int8_t status;
	password_comp_flag = 0;
	status = 0;
	error_code = 0;

	uint8_t dose_h, dose_m, dose_s, dose_num;

	//finite state machine section
	uint32_t current_tick;

	uint32_t dosing_tick = 0;

	uint32_t general_delay;

	// a string to save the RTC time
	char timeString[50];

	/*password section*/
	char menu_pass[] = "1234";
	char entered_password[5] = ""; // User input buffer (4 digits + null terminator)
	char time_date_buffer[5];
	uint16_t buffer;
	uint8_t input_index = 0;

	general_delay = HAL_GetTick() + 250;

	/**
	 * testing section
	 */

	/*
	 //to change the duty cycle -> CCR
	 //range is from 999 to 1999 (according to calculations)
	 TIM1->CCR1 = 1999;
	 GPIO_InitTypeDef c = { .Mode = GPIO_MODE_OUTPUT_PP, .Pin = GPIO_PIN_13,
	 .Speed = GPIO_SPEED_LOW };

	 HAL_GPIO_Init(GPIOC, &c);

	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);

	 char str[16];
	 int16_t message;
	 uint8_t value;
	 eeprom24c32_read(&memory, &value, &memory.i2c_buffer[2]);

	 Alcd_Clear(&lcd);
	 // Display ADC value on the LCD
	 message = sprintf(str, "mem = %d", value);
	 Alcd_PutAt_n(&lcd, 0, 0, str, message);

	 HAL_Delay(2000);

	 value = 255;

	 while (value == 255) {

	 Keypad_Refresh(&kp);
	 if (Keypad_Get_Key(&kp, 12)) {
	 value = 12;
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	 } else if (Keypad_Get_Key(&kp, 10)) {
	 value = 10;
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	 } else if (Keypad_Get_Key(&kp, 1)) {
	 value = 1;
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	 }

	 }

	 Alcd_Clear(&lcd);
	 Alcd_PutAt(&lcd, 0, 0, "exit while");

	 HAL_Delay(2000);

	 eeprom24c32_write(&memory, &value, &memory.i2c_buffer[2]);

	 Alcd_Clear(&lcd);
	 // Display ADC value on the LCD
	 message = sprintf(str, "value = %d", value);
	 Alcd_PutAt_n(&lcd, 0, 0, str, message);
	 */

	/**
	 * end of testing section
	 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//get the current tick number
		current_tick = HAL_GetTick();
		//view the time parameters -> status 14
		while ((status == 0) && (current_tick >= general_delay)) {

			//get the current tick number
			current_tick = HAL_GetTick();
			//clear the lcd
			Alcd_Clear(&lcd);
			Alcd_PutAt(&lcd, 0, 0, "Dose @");

			//reading dosing hours
			eeprom24c32_read(&memory, &dose_h, dosing_time_hours);

			snprintf(timeString, sizeof(timeString), "%02d", status);
			Alcd_PutAt_n(&lcd, 0, 14, timeString, strlen(timeString));

			snprintf(timeString, sizeof(timeString), "%02d:", dose_h);
			Alcd_PutAt_n(&lcd, 1, 0, timeString, strlen(timeString));

			eeprom24c32_read(&memory, &dose_m, dosing_time_minutes);
			snprintf(timeString, sizeof(timeString), "%02d:", dose_m);

			Alcd_PutAt_n(&lcd, 1, 3, timeString, strlen(timeString));

			eeprom24c32_read(&memory, &dose_s, dosing_time_seconds);
			snprintf(timeString, sizeof(timeString), "%02d", dose_s);

			Alcd_PutAt_n(&lcd, 1, 6, timeString, strlen(timeString));

			//check if back or next is selected
			Keypad_Refresh(&kp);
			//in case back is selected
			if (Keypad_Get_Key(&kp, kp_button_save_menu)
					&& (current_tick >= general_delay)) {

				//back to previous menu
				status = 10;

			}

			general_delay = HAL_GetTick() + 250;
		}


		while ((status == 10) && (current_tick >= general_delay)) {

			//get the current tick number
			current_tick = HAL_GetTick();
			//clear the lcd
			Alcd_Clear(&lcd);
			Alcd_PutAt(&lcd, 0, 0, "enter h:");

			snprintf(timeString, sizeof(timeString), "%02d", status);
			Alcd_PutAt_n(&lcd, 0, 14, timeString, strlen(timeString));

			Keypad_Refresh(&kp);

			if (Keypad_Get_Key(&kp, kp_button_1)
					&& (current_tick >= general_delay)) {

				dose_h = 12;

				eeprom24c32_write(&memory, &dose_h, 0x0000);

				status = 11;

			}

			general_delay = HAL_GetTick() + 250;
		}


		while ((status == 11) && (current_tick >= general_delay)) {

			//get the current tick number
			current_tick = HAL_GetTick();
			//clear the lcd
			Alcd_Clear(&lcd);
			Alcd_PutAt(&lcd, 0, 0, "enter m:");

			snprintf(timeString, sizeof(timeString), "%02d", status);
			Alcd_PutAt_n(&lcd, 0, 14, timeString, strlen(timeString));

			Keypad_Refresh(&kp);

			if (Keypad_Get_Key(&kp, kp_button_2)
					&& (current_tick >= general_delay)) {

				dose_m = 20;

				eeprom24c32_write(&memory, &dose_m, 0x0010);

				status = 12;

			}

			general_delay = HAL_GetTick() + 250;
		}


		while ((status == 12) && (current_tick >= general_delay)) {

			//get the current tick number
			current_tick = HAL_GetTick();
			//clear the lcd
			Alcd_Clear(&lcd);
			Alcd_PutAt(&lcd, 0, 0, "enter sec:");

			snprintf(timeString, sizeof(timeString), "%02d", status);
			Alcd_PutAt_n(&lcd, 0, 14, timeString, strlen(timeString));

			Keypad_Refresh(&kp);

			if (Keypad_Get_Key(&kp, kp_button_3)
					&& (current_tick >= general_delay)) {

				dose_h = 50;

				eeprom24c32_write(&memory, &dose_s, 0x0100);

				status = 0;

			}

			general_delay = HAL_GetTick() + 250;
		}


//
//
	}			//end of while 1

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 19999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
