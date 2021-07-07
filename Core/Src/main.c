/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include <stdio.h>
#include "lsm3.h"

ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
uint8_t activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char *activities[AI_NETWORK_OUT_1_SIZE] =
    {
    "stable", "walk", "run"
    };
static void AI_Init(ai_handle w_addr, ai_handle act_addr);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float *values, uint32_t len);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
LIS3DSH_DataRaw raw;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int fd, char *ptr, int len)
    {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
    }

static void AI_Init(ai_handle w_addr, ai_handle act_addr)
    {
    ai_error err;

    /* 1 - Create an instance of the model */
    err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE)
	{
	printf("ai_network_create error - type=%d code=%d\r\n", err.type,
		err.code);
	Error_Handler();
	}

    /* 2 - Initialize the instance */
    const ai_network_params params =
	{
	AI_NETWORK_DATA_WEIGHTS(w_addr),
    AI_NETWORK_DATA_ACTIVATIONS(act_addr)
    }
    ;

    if (!ai_network_init(network, &params))
	{
	err = ai_network_get_error(network);
	printf("ai_network_init error - type=%d code=%d\r\n", err.type,
		err.code);
	Error_Handler();
	}
    }

static void AI_Run(float *pIn, float *pOut)
    {
    ai_i32 batch;
    ai_error err;

    /* 1 - Create the AI buffer IO handlers with the default definition */
    ai_buffer ai_input[AI_NETWORK_IN_NUM] = AI_NETWORK_IN
    ;
    ai_buffer ai_output[AI_NETWORK_OUT_NUM] = AI_NETWORK_OUT
    ;

    /* 2 - Update IO handlers with the data payload */
    ai_input[0].n_batches = 1;
    ai_input[0].data = AI_HANDLE_PTR(pIn);
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(pOut);

    batch = ai_network_run(network, ai_input, ai_output);
    if (batch != 1)
	{
	err = ai_network_get_error(network);
	printf("AI ai_network_run error - type=%d code=%d\r\n", err.type,
		err.code);
	Error_Handler();
	}
    }

static uint32_t argmax(const float *values, uint32_t len)
    {
    float max_value = values[0];
    uint32_t max_index = 0;
    for (uint32_t i = 1; i < len; i++)
	{
	if (values[i] > max_value)
	    {
	    max_value = values[i];
	    max_index = i;
	    }
	}
    return max_index;
    }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t write_index = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
    static u8 start = 0;
    if (GPIO_Pin == GPIO_PIN_0)
	{
	LIS3DSH_GetDataRaw(&raw);
//	HAL_UART_Transmit(&huart2, (u8*) &raw, sizeof(raw), 0xFFFF);
//	printf("% 5d, % 5d, % 5d\r\n", raw.x, raw.y, raw.z);
	/* Note: window overlapping can be managed here */
	aiInData[write_index + 0] = (float) raw.x / 32000.0f;
	aiInData[write_index + 1] = (float) raw.y / 32000.0f;
	aiInData[write_index + 2] = (float) raw.z / 32000.0f;
	write_index += 3;

	if (write_index == AI_NETWORK_IN_1_SIZE)
	    {
	    write_index = 0;

	    start = 1;
	    }
	if (start != 0)
	    {
//	    printf("Running inference\r\n");
	    AI_Run(aiInData, aiOutData);

	    /* Output results */
	    for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++)
		{
		printf("%8.2f ", aiOutData[i]);
		}
	    printf("\n");
//	    uint32_t class = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
//	    printf(": %d - %s\r\n", (int) class, activities[class]);
	    }

	}
    }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_DMA_Init();
    MX_CRC_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_Delay(200);
    AI_Init(ai_network_data_weights_get(), activations);
    LIS3DSH_init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
	{
	HAL_Delay(40);
//	LIS3DSH_GetDataRaw(&raw);
//	HAL_UART_Transmit(&huart2, (u8*) &raw, sizeof(raw), 0xFFFF);
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
    RCC_OscInitTypeDef RCC_OscInitStruct =
	{
	0
	};
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{
	0
	};

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
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    while (1)
	{
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	HAL_Delay(50); /* wait 50 ms */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
