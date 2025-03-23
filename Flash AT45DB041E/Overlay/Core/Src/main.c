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

#define N_REGIONS	1	// # of overlay regions in RAM
#define N_OVLY		4	// Total # of overlays (in all regions)

#define OVERLAY(region,ov,sym)	{ region, &ov, &__load_start_ ## sym, &__load_stop_ ## sym, 0, sym }
#define LOADREF(sym) __load_start_ ## sym, __load_stop_ ## sym 

extern unsigned long overlay1;	// Provides address of overlay region 1

// Function load addresses
extern char LOADREF(fee), LOADREF(fie), LOADREF(foo), LOADREF(fum);

/*********************************************************************
 * Overlayed functions
 *********************************************************************/
 int fee(int arg) __attribute__((noinline,section(".ov_fee")));
 int fie(int arg) __attribute__((noinline,section(".ov_fie")));
 int foo(int arg) __attribute__((noinline,section(".ov_foo")));
 int fum(int arg) __attribute__((noinline,section(".ov_fum")));
 
/*********************************************************************
 * Overlay Table and macros
 *********************************************************************/

 typedef struct {
	short		regionx;// Overlay region index
	void		*vma;	// Overlay's mapped address
	char		*start;	// Load start address
	char		*stop;	// Load stop address
	unsigned long	size;	// Size in bytes
	void		*func;	// Function pointer
} s_overlay;

// Overlay table:
static s_overlay overlays[N_OVLY] = {
	OVERLAY(0, overlay1, fee),
	OVERLAY(0, overlay1, fie),
	OVERLAY(0, overlay1, foo),
	OVERLAY(0, overlay1, fum)
};

// Overlay cache:
static s_overlay *cur_overlay[N_REGIONS] = { 0 };

/*********************************************************************
 * Overlay lookup: Returns func ptr, after copying code if necessary
 *********************************************************************/

static void *module_lookup (void *module) {
	unsigned regionx;
	s_overlay *ovl = 0;

	// std_printf("module_lookup(%p):\n",module);

	for ( unsigned ux=0; ux<N_OVLY; ++ux ) {
		if ( overlays[ux].start == module ) {
			regionx = overlays[ux].regionx;
			ovl = &overlays[ux];
			break;
		}
	}

	if ( !ovl )
		return 0;		// Not found

	if ( !cur_overlay[regionx] || cur_overlay[regionx] != ovl ) {
		if ( ovl->size == 0 )
			ovl->size = (char *)ovl->stop - (char *)ovl->start;
		cur_overlay[regionx] = ovl;
		memcpy(ovl->vma,ovl->start,ovl->size);
	}
	return ovl->func;
}


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

  // Initialized
  at45dbxx_init (&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);

  // Read ID
  const size_t id_num = 5;
  uint8_t at45dbxx_id[id_num];
  memset (at45dbxx_id, 0, id_num);
  
  if (!at45dbxx_rd_id (at45dbxx_id, id_num))
    error_handler (ERR_RD_ID);

  HAL_Delay (500);

  uint8_t tx_test_message[] = "Test message";
  const size_t len_message = sizeof (tx_test_message);
  uint8_t rx_test_message[len_message];
  memset (rx_test_message, 0, len_message);

  at45dbxx_config_t at45dbxx_config_wr = { .opcode = AT45DBXX_OPCODE_WR_PAGE_ON_BUF1, .addr = 0x123, .msg = tx_test_message, .msg_len = len_message };
  at45dbxx_config_t at45dbxx_config_rd = { .opcode = AT45DBXX_OPCODE_CONT_ARR_RD_HF, .addr = 0x123, .msg = rx_test_message, .msg_len = len_message };

  // Write
  if (!at45dbxx_wr_data (&at45dbxx_config_wr))
    error_handler (ERR_WR);

  HAL_Delay (500);

  // Read
  if (!at45dbxx_rd_data (&at45dbxx_config_rd))
    error_handler (ERR_RD);

  HAL_Delay (500);
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
/*********************************************************************
 * Overlay function fee()
 *********************************************************************/

 int fee (int arg) {
   static const char format[] 
     __attribute__((section(".ov_fee_static")))
     = "***********\n"
       "fee(0x%04X)\n"
       "***********\n";
 
   // std_printf(format,arg);
   return arg + 0x0001;
 }
 
 /*********************************************************************
  * Overlay function fie()
  *********************************************************************/
 
 int fie (int arg) {
   // std_printf("fie(0x%04X)\n",arg);
   return arg + 0x0010;
 }
 
 /*********************************************************************
  * Overlay function foo()
  *********************************************************************/
 
 int foo (int arg) {
   // std_printf("foo(0x%04X)\n",arg);
   return arg + 0x0200;
 }
 
 /*********************************************************************
  * Overlay function fum()
  *********************************************************************/
 
 int fum (int arg) {
   // std_printf("fum(0x%04X)\n",arg);
   return arg + 0x3000;
 }

 /*********************************************************************
 * Stub functions for calling the overlay functions:
 *********************************************************************/

static int
fee_stub(int arg) {
	int (*feep)(int arg) = module_lookup(&__load_start_fee);

	return feep(arg);
}

static int
fie_stub(int arg) {
	int (*fiep)(int arg) = module_lookup(&__load_start_fie);

	return fiep(arg);
}

static int
foo_stub(int arg) {
	int (*foop)(int arg) = module_lookup(&__load_start_foo);

	return foop(arg);
}

static int
fum_stub(int arg) {
	int (*fump)(int arg) = module_lookup(&__load_start_fum);

	return fump(arg);
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
