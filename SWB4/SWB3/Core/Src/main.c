/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define LOGI(fmt, ...) printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) printf("[WARN] " fmt "\r\n", ##__VA_ARGS__)
#define LOGE(fmt, ...) printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLASH_USER_START_ADDR  ((uint32_t)0x0803F800U) // constant giving the start address of the flash area we’ll use
#define FLASH_USER_END_ADDR    ((uint32_t)0x08040000U) // end address of that region. The code will only operate inside this range.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define RX_BUF_SIZE 128u
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

#define TX_BUF_SIZE 256u
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;

volatile uint32_t custom_tick = 0;



uint32_t blink_delay = 500;
uint8_t use_irq = 0;
uint8_t has_error = 0;

uint32_t t_led = 0;
uint32_t t_log = 0;
uint32_t t_double = 0;
uint8_t d_step = 0;

uint32_t t_btn = 0;
uint8_t clicks = 0;
uint8_t last_btn = 1;
uint8_t auto_blink = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Erase configured flash region (whole pages)
HAL_StatusTypeDef flash_erase_region(void)
{
    FLASH_EraseInitTypeDef EraseInit = {0};
    uint32_t PageError = 0; // store any page number that failed to erase
    const uint32_t pageSize = FLASH_PAGE_SIZE;
    if (FLASH_USER_START_ADDR >= FLASH_USER_END_ADDR) return HAL_ERROR;

    uint32_t startPage = (FLASH_USER_START_ADDR - FLASH_BASE) / pageSize;
    uint32_t nbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR + pageSize - 1) / pageSize;

    HAL_FLASH_Unlock(); // unlock flash control registers so we can erase/program.

    EraseInit.TypeErase = FLASH_TYPEERASE_PAGES; // erase by pages
    EraseInit.Banks = FLASH_BANK_1; // different nucleo boards may have different amount of data banks
    EraseInit.Page = startPage; // start page to erase, because defined region is one page long (2kb)
    EraseInit.NbPages = nbPages;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&EraseInit, &PageError);
    HAL_FLASH_Lock(); // lock it back

    return st;
}

HAL_StatusTypeDef flash_write_doubleword(uint32_t addr, uint64_t value)
{
    if ((addr < FLASH_USER_START_ADDR) || ((addr + 8) > FLASH_USER_END_ADDR) || (addr & 7U)) {
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, value);
    HAL_FLASH_Lock();
    return st;
}

// Read bytes from flash into buffer
HAL_StatusTypeDef flash_read_bytes(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if ((addr < FLASH_USER_START_ADDR) || ((addr + len) > FLASH_USER_END_ADDR)) {
        return HAL_ERROR;
    }
    memcpy(buf, (void*)addr, len); // copy from the flash addr to the buff
    return HAL_OK;
}

int __io_putchar(int ch) {
	uint8_t c = (uint8_t) ch;
	uint8_t need_kick = 0;
	__disable_irq();

	uint16_t next_head = (tx_head + 1) & (TX_BUF_SIZE - 1);
	if (next_head != tx_tail) {
		tx_buf[tx_head] = c;
		tx_head = next_head;

		if (!tx_busy) {
			tx_busy = 1;
			need_kick = 1;
		}
	}
	__enable_irq();

	if (need_kick) {
		HAL_UART_Transmit_IT(&huart2, &tx_buf[tx_tail], 1);
	}
	return 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
	if (huart == &huart2) {
		tx_tail = (tx_tail + 1) & (TX_BUF_SIZE - 1);

		if (tx_tail != tx_head) {
			HAL_UART_Transmit_IT(&huart2, &tx_buf[tx_tail], 1);
		} else {
			tx_busy = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	if (huart == &huart2) {
		uint16_t next_head = (rx_head + 1) & (RX_BUF_SIZE - 1);
		if (next_head != rx_tail) {
			rx_buf[rx_head] = rx_byte;
			rx_head = next_head;
		}

		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}

void process_command(char* cmd) {
	uint32_t addr;
	uint32_t len;
	uint64_t val;

	if(strcmp(cmd, "gpio status") == 0) {
		LOGI("BTN: %d, LED: %d\r\n", HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin), HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin));

	} else if (strcmp(cmd, "led on") == 0) {

		auto_blink = 0;
		d_step = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		LOGI("LED ON");



	} else if (strcmp(cmd, "led off") == 0) {
		auto_blink = 0;
		d_step = 0;

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		LOGI("LED OFF (auto blink off)");

	} else if (strcmp(cmd, "led toggle") == 0) {

		auto_blink = 0;
		d_step = 0;


		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		LOGI("LED OFF (auto blink off)");

	} else if (strcmp(cmd, "led auto") == 0) {
		auto_blink = 1;
		LOGI("auto blink on");

	} else if (strcmp(cmd, "button mode poll") == 0) {
		use_irq = 0;
		LOGI("Mode set to POLL");
	} else if (strcmp(cmd, "button mode irq") == 0) {
		use_irq = 1;
		LOGI("Mode set to IRQ");
	} else if (strcmp(cmd, "flash erase") == 0) {

		if(flash_erase_region() == HAL_OK) {
			LOGI("INFO: Flash Erased");
		} else {
			LOGE("Flash Erase Failed");
		}
	} else if (strncmp(cmd, "flash write ", 12) == 0) {
		if (sscanf(cmd + 12, "%lx %llx", &addr, &val) == 2) {
			if (flash_write_doubleword(addr, val) == HAL_OK) {
				LOGI("Flash Write OK");
			} else {
				LOGE("Flash Write Failed");
			}
		} else {
			has_error = 2;
			LOGE("Invalid write parameters");
		}
	} else if (strncmp(cmd, "flash read ", 11) == 0) {
		if (sscanf(cmd + 11, "%lx %lu", &addr, &len) == 2) {
			uint8_t buf[16] = {0};
			if(flash_read_bytes(addr, buf, (len>16) ? 16 : len) == HAL_OK) {
				for(int i = 0; i <((len > 16) ? 16 : len); i++) {
					printf("%02X ", buf[i]);
				}
				printf("\r\n");
			} else {
				LOGE("Flash Read Failed");
			}
		} else {
			has_error = 2;
			LOGE("Invalid read parameters");
		}
	} else {
		LOGE("Unknown command: '%s'", cmd);
		has_error = 1;
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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  HAL_TIM_Base_Start_IT(&htim6);

  LOGI("--- System Loaded ---");
  uint64_t startup;
  if (flash_read_bytes(FLASH_USER_START_ADDR, (uint8_t*)&startup, 8) == HAL_OK) {
	  if (startup == 0xFFFFFFFFFFFFFFFF) {
		 LOGI("Flash Record Status: EMPTY");
	  } else {
		 LOGI("Flash Record Status: OK");
	  }
  } else {
	  LOGE("Flash Record Status: FAIL");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char cmd[32];
  uint32_t idx = 0;



  while (1)
  {
	  uint32_t now = custom_tick;


	  if (auto_blink == 1) {
		  if (has_error == 1) {
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  } else if (has_error == 2) {
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  } else if (now - t_led >= blink_delay) {
			  t_led = now;
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  }

		  if (now - t_double >= 2200) {
			  t_double = now;
			  d_step = 1;
		  }
		  if (d_step == 1) {
		  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		  		  d_step = 2;
		  	  } else if (d_step == 2 && now - t_double > 50) {
		  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  		  d_step = 3;

		  	  } else if (d_step == 3 && now - t_double > 100) {
		  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		  		  d_step = 4;
		  	  } else if (d_step == 4 && now - t_double > 150) {
		  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  		  d_step = 0;
		  	  }
	  }



	  // log
	  if (now - t_log >= 1000){
		  t_log = now;
		  LOGI("Delay: %lu ms | Error: %d | Mode: %d\r\n", blink_delay, has_error, use_irq);
		  if (has_error > 0) {
			  has_error = 0;
		  }
	  }



	  if (use_irq == 0) {
		  uint8_t btn = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);

		  if (btn == 0 && last_btn == 1) {
			  clicks++;
			  t_btn = now;
		  }

		  last_btn = btn;

		  if (clicks > 0 && (now -t_btn > 400)) {
			  if (clicks == 1 && blink_delay > 100) {
				  blink_delay -= 100;
			  } if (clicks >= 2) {
				  blink_delay += 100;
			  }
			  clicks = 0;
		  }
	  }









	  // UART Buffer
	  while (rx_tail != rx_head) {
		  uint8_t b = rx_buf[rx_tail];
		  rx_tail = (rx_tail + 1) & (RX_BUF_SIZE - 1);

		  if (b == '\b' || b == 0x7F) {
			  if (idx > 0) {
				  idx--;
				  continue;
			  }
		  }

		  if (b == '\n' || b == '\r') {
			  if (idx > 0) {
				  cmd[idx] = 0;
				  process_command(cmd);
			  }
			  idx = 0;
			  continue;
		  }

		  if (idx < 31) {
			 cmd[idx] = (char)b;
			 idx++;
		  } else {
			 idx = 0;
		  }
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_RX_Pin LPUART1_TX_Pin */
  GPIO_InitStruct.Pin = LPUART1_RX_Pin|LPUART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
	        custom_tick++; // Add 1 millisecond
	    }
}

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
#ifdef USE_FULL_ASSERT
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
