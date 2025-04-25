/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Avionics Trigger Board, project Firehorn, EPFL Rocket Team
  * @author			: Cyprien Lacassagne
  * @version		: 0.1.0
  * @date			: 2025.03.11
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "intranet_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PYRO_ON_MIN_FOR_TRIGGER 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static uint32_t mem[TRB_NB_REG];
static uint8_t reg_addr = 0x00;
static uint8_t reg_addr_rcvd = 0;
static uint8_t rx_bytes_count = 0;
static uint8_t tx_bytes_count = 0;
static uint8_t rx_buffer[NET_XFER_SIZE];
static uint8_t tx_buffer[NET_XFER_SIZE];

static uint8_t woken_up = 0;
static uint8_t triggered = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void process_rx_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c) {
	reg_addr_rcvd = 0;
	HAL_I2C_EnableListen_IT(hi2c);
}

// Called when detecting TRB address match on I2C bus
// Will temporarily wake up the CPU during sleep mode
void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, &reg_addr, 1, I2C_FIRST_FRAME);
	}else {
		// Transmit data from requested register
		uint8_t tx_byte = mem[reg_addr] & 0xFF;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) &tx_byte, 1, I2C_FIRST_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c) {
	// First time this callback is called is after receiving the register address

	// If received 4 bytes of data, store it in memory and exit callback
	if (rx_bytes_count >= NET_XFER_SIZE) {
		process_rx_data();
		reg_addr = 0;
		rx_bytes_count = 0;
		return;
	}

	// Prevent writing to read-only registers
	if (reg_addr != TRB_IS_WOKEN_UP && reg_addr != TRB_HAS_TRIGGERED) {
		if (rx_bytes_count < NET_XFER_SIZE - 1) {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_buffer[rx_bytes_count], 1, I2C_NEXT_FRAME);
			++rx_bytes_count;
		}else {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_buffer[rx_bytes_count], 1, I2C_LAST_FRAME);
			++rx_bytes_count;
		}
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c) {
	++tx_bytes_count;
	if (tx_bytes_count >= NET_XFER_SIZE) {
		tx_bytes_count = 0;
		return;
	}

	if (tx_bytes_count < NET_XFER_SIZE - 1) {
		uint8_t tx_byte = (mem[reg_addr] >> tx_bytes_count * 8) & 0xFF;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) &tx_byte, 1, I2C_NEXT_FRAME);
	}else {
		uint8_t tx_byte = (mem[reg_addr] >> tx_bytes_count * 8) & 0xFF;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) &tx_byte, 1, I2C_LAST_FRAME);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
	reg_addr= 0;
	reg_addr_rcvd = 0;

	uint32_t code = HAL_I2C_GetError(hi2c);

	if (code == HAL_I2C_ERROR_AF) {
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
	}

	if (code == HAL_I2C_ERROR_BERR) {
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);
	}

	HAL_I2C_EnableListen_IT(hi2c);
}

void process_rx_data(void) {
	mem[reg_addr] = 0;
	for (uint8_t i = 0; i < NET_XFER_SIZE; ++i) {
		mem[reg_addr] += rx_buffer[i] << 8*i;
	}
	// If the received packet is WAKEUP command, disable SleepOnExit to stay in RUN mode
	if (reg_addr == TRB_WAKE_UP && mem[reg_addr] == NET_CMD_ON && !woken_up) {
		HAL_ResumeTick();
		HAL_PWR_DisableSleepOnExit();
		woken_up = 1;
		mem[TRB_IS_WOKEN_UP] = NET_CMD_ON;
	}
}

static void blink_led(const uint32_t delta_ms) {
	static uint8_t is_led_on = 0;
	static uint8_t first_pulse = 0;
	static uint32_t led_on_ms = 0;
	static uint32_t led_off_ms = 0;

	is_led_on = HAL_GPIO_ReadPin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin) == GPIO_PIN_SET;
	if (is_led_on) {
		led_on_ms += delta_ms;
	}else {
		led_off_ms += delta_ms;
	}

	if (mem[TRB_CLEAR_TO_TRIGGER] == NET_CMD_ON) {
		if (led_on_ms > 50) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_RESET);
			led_on_ms = 0;
		}else if (led_off_ms > (first_pulse ? 50 : 850)) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_SET);
			led_off_ms = 0;
			first_pulse = !first_pulse;
		}
	}else {
		if (led_on_ms > 50) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_RESET);
			led_on_ms = 0;
		}else if (led_off_ms > 950) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_SET);
			led_off_ms = 0;
		}
	}
}

static void listen_pyros(uint32_t delta_ms) {
	static uint32_t pyro1_ms = 0;
	static uint32_t pyro2_ms = 0;
	static uint32_t pyro3_ms = 0;

//	if (triggered) {
//		HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, GPIO_PIN_RESET);
//		return;
//	}

	if ((mem[TRB_PYROS] & TRB_PYRO1) == NET_CMD_ON) {
		HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_SET);
		pyro1_ms += delta_ms;
	}else {
		HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_RESET);
		pyro1_ms = 0;
	}

	if (((mem[TRB_PYROS] & TRB_PYRO2) >> 8) == NET_CMD_ON) {
		HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_SET);
		pyro2_ms += delta_ms;
	}else {
		HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_RESET);
		pyro2_ms = 0;
	}

	if (((mem[TRB_PYROS] & TRB_PYRO3) >> 16) == NET_CMD_ON) {
		HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, GPIO_PIN_SET);
		pyro3_ms += delta_ms;
	}else {
		HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, GPIO_PIN_RESET);
		pyro3_ms = 0;
	}

	if (pyro1_ms > PYRO_ON_MIN_FOR_TRIGGER || pyro2_ms > PYRO_ON_MIN_FOR_TRIGGER
			|| pyro3_ms > PYRO_ON_MIN_FOR_TRIGGER) {
		triggered = 1;
		mem[TRB_HAS_TRIGGERED] = NET_CMD_ON;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	// Enable I2C address callback and independent processing of R/W
	if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	// Reset all GPIOs
	HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, GPIO_PIN_RESET);

	// Enter SLEEP mode. SleepOnExit keeps the MCU in SLEEP only to process interrupts (I2C, etc)
	// Wake up to RUN mode is done when the command WAKE_UP from Master is received.
//	HAL_SuspendTick();
//	HAL_PWR_EnableSleepOnExit();
//	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t now_ms = HAL_GetTick();
	while (1)
	{
		static uint32_t old_ms = 0;
		static uint32_t delta_ms = 0;

		old_ms = now_ms;
		now_ms = HAL_GetTick();
		delta_ms = now_ms - old_ms;

		if (mem[TRB_CLEAR_TO_TRIGGER] == NET_CMD_ON) {
			listen_pyros(delta_ms);
		}

		blink_led(delta_ms);

		HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 20;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PYRO1_Pin|PYRO2_Pin|PYRO3_Pin|LED_ACTIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PYRO1_Pin PYRO2_Pin PYRO3_Pin */
  GPIO_InitStruct.Pin = PYRO1_Pin|PYRO2_Pin|PYRO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ACTIVE_Pin */
  GPIO_InitStruct.Pin = LED_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ACTIVE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(PYRO3_GPIO_Port, PYRO3_Pin);
	  HAL_Delay(100);
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
