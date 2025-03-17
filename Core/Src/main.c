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

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
static uint32_t mem[TRB_NB_REG];
static uint8_t reg_addr = 0x00;
static uint8_t reg_addr_rcvd = 0;
static uint8_t rx_buffer[NET_XFER_SIZE];
static uint8_t tx_buffer[NET_XFER_SIZE];

static uint8_t woken_up = 0;
static uint8_t triggered = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

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
		// First byte is always the register address
		if (!reg_addr_rcvd) {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &reg_addr, 1, I2C_NEXT_FRAME);
		}
	}else {
		// Transmit data from requested register
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) &mem[reg_addr], NET_XFER_SIZE, I2C_LAST_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c) {
	if (!reg_addr_rcvd) {
		// After receiving the register address, receive the data
		reg_addr_rcvd = 1;
		// Prevent writing to read-only registers
		if (reg_addr != TRB_IS_WOKEN_UP && reg_addr != TRB_HAS_TRIGGERED) {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buffer, NET_XFER_SIZE, I2C_LAST_FRAME);
		}
	}else {
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
		reg_addr = 0x00;
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
		}else if (led_off_ms > (first_pulse ? 50 : 1000)) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_SET);
			led_off_ms = 0;
			first_pulse = !first_pulse;
		}
	}else {
		if (led_on_ms > 50) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_RESET);
			led_on_ms = 0;
		}else if (led_off_ms > 1000) {
			HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_SET);
			led_off_ms = 0;
		}
	}
}

static void listen_pyros(uint32_t delta_ms) {
	static uint32_t pyro1_ms = 0;
	static uint32_t pyro2_ms = 0;
	static uint32_t pyro3_ms = 0;

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
	MX_USB_PCD_Init();
	/* USER CODE BEGIN 2 */
	// Enable I2C address callback and independent processing of R/W
	HAL_I2C_EnableListen_IT(&hi2c1);
	HAL_GPIO_WritePin(LED_ACTIVE_GPIO_Port, LED_ACTIVE_Pin, GPIO_PIN_RESET);

	// Enter SLEEP mode. SleepOnExit keeps the MCU in SLEEP only to process interrupts (I2C, etc)
	// Wake up to RUN mode is done when the command WAKE_UP from Master is received.
	HAL_SuspendTick();
	HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		static uint32_t now_ms = HAL_GetTick();
		static uint32_t old_ms = 0;
		static uint32_t delta_ms = 0;

		old_ms = now_ms;
		now_ms = HAL_GetTick();
		delta_ms = now - old;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = NET_ADDR_TRB << 1;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
