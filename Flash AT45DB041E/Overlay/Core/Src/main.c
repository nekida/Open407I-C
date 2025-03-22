/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
#include <string.h>

#include "../../AT45DBxx/AT45DBxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
typedef enum {
  ERR_RD_ID = 0,
  ERR_GT_ADDR,
  ERR_WR,
  ERR_RD
} error_t;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void error_handler (error_t err)
{
  const error_t error = err;
  UNUSED (error);
  HAL_GPIO_WritePin (USERLED1_GPIO_Port, USERLED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin (USERLED2_GPIO_Port, USERLED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (USERLED3_GPIO_Port, USERLED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin (USERLED4_GPIO_Port, USERLED4_Pin, GPIO_PIN_RESET);
  while (1) {

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Read ID
  const size_t id_num = 5;
  uint8_t at45dbxx_id[id_num];
  memset (at45dbxx_id, 0, id_num);
  
  HAL_StatusTypeDef status = HAL_OK;

  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit (&hspi1, &at45dbxx_opcode_get_id, sizeof (at45dbxx_opcode_get_id), HAL_MAX_DELAY);
  status = HAL_SPI_Receive (&hspi1, at45dbxx_id, sizeof (at45dbxx_id), HAL_MAX_DELAY);
  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK)
    error_handler (ERR_RD_ID);

  // Write
  const uint16_t page_addr = 0x123;
  const size_t page_addr_len = 3;
  uint8_t page_addr_arr[page_addr_len];
  memset (page_addr_arr, 0, page_addr_len);

  const char * test_message[] = "Test message";

  const size_t cmd = 1;
  const size_t end_of_str = 1;
  const size_t len_message = strlen (test_message) + end_of_str + cmd;
  uint8_t tx_data[len_message];

  tx_data[0] = at45dbxx_opcode_wr_page_on_buf1;
  memcpy (&tx_data[1], test_message, len_message - cmd);

  if (!at45dbxx_get_addr_packed (page_addr, page_addr_arr, page_addr_len))
    error_handler (ERR_GT_ADDR);

  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  // status = HAL_SPI_Transmit (&hspi1, &at45dbxx_opcode_wr_page_on_buf1, sizeof (at45dbxx_opcode_wr_page_on_buf1), HAL_MAX_DELAY);
  status = HAL_SPI_Transmit (&hspi1, page_addr_arr, page_addr_len, HAL_MAX_DELAY);
  status = HAL_SPI_Transmit (&hspi1, (uint8_t *)tx_data, len_message, HAL_MAX_DELAY);
  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK)
    error_handler (ERR_WR);

  HAL_Delay (500);

  // Read
  uint8_t rx_test_message[len_message];
  rx_test_message[0] = at45dbxx_opcode_cont_arr_rd_hf;
  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  // status = HAL_SPI_Transmit (&hspi1, &at45dbxx_opcode_cont_arr_rd_hf, sizeof (at45dbxx_opcode_cont_arr_rd_hf), HAL_MAX_DELAY);
  status = HAL_SPI_Transmit (&hspi1, page_addr_arr, page_addr_len, HAL_MAX_DELAY);
  status = HAL_SPI_Receive (&hspi1, (uint8_t *)rx_test_message, len_message, HAL_MAX_DELAY);
  HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK)
    error_handler (ERR_RD);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin (USERLED1_GPIO_Port, USERLED1_Pin);
    HAL_Delay (300);
    HAL_GPIO_TogglePin (USERLED2_GPIO_Port, USERLED2_Pin);
    HAL_Delay (300);
    HAL_GPIO_TogglePin (USERLED3_GPIO_Port, USERLED3_Pin);
    HAL_Delay (300);
    HAL_GPIO_TogglePin (USERLED4_GPIO_Port, USERLED4_Pin);
    HAL_Delay (300);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
