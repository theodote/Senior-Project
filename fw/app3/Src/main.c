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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
// #include "dsp/transform_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { false, true } bool;
typedef enum { NORMAL, CIRCULAR } dmaType;

#define MAX_BUFS  16  // I yield.
typedef struct multiBuffer {
  uint16_t numBufs;
  uint16_t bufSize;
  dmaType dmaMode;
  volatile bool dmaDone;
  volatile bool finished;
  union {
    struct {
      volatile uint16_t dmaInd;
      volatile uint16_t activeInds[MAX_BUFS - 1];
    };
    volatile uint16_t indices[MAX_BUFS];
  };
  volatile void* buffers;
} MultiBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALF_FRAME_SIZE   512
#define FULL_FRAME_SIZE   2 * HALF_FRAME_SIZE
#define ADC_MAX           4096
#define ADC_MAX_F         4096.0f
#define ADC_MID           2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NEXT(n,m)               ((n) == (m-1) ? 0 : (n+1))
#define PREV(n,m)               ((n) == 0 ? (m-1) : (n-1))

#define ACTIVE_BUF(b, t, n) ((t(*)[b.bufSize]) (b.buffers))[b.activeInds[n]]
#define DMA_BUF(b, t)       ((t(*)[b.bufSize]) (b.buffers))[b.dmaInd]

#define INT16_TO_FLOAT(n)       (1.0f / ADC_MAX_F * (n - ADC_MID))
#define FLOAT_TO_INT16(n)       ((int16_t)(ADC_MAX * n) + ADC_MID)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t adcData[2 * HALF_FRAME_SIZE] = {0};
MultiBuffer adcBuf = {2, HALF_FRAME_SIZE, CIRCULAR, false, false, {.indices = {0, 1}}, (void*)adcData};
int16_t dacData[2 * HALF_FRAME_SIZE] = {0};
MultiBuffer dacBuf = {2, HALF_FRAME_SIZE, CIRCULAR, false, false, {.indices = {0, 1}}, (void*)dacData};

float inData[2 * HALF_FRAME_SIZE] = {0.0};
MultiBuffer inBuf = {2, HALF_FRAME_SIZE, NORMAL, false, false, {.indices = {0, 1}}, (void*)inData};
float outData[2 * HALF_FRAME_SIZE] = {0.0};
MultiBuffer outBuf = {2, HALF_FRAME_SIZE, NORMAL, false, false, {.indices = {0, 1}}, (void*)outData};

arm_rfft_fast_instance_f32 fft;
float frequency[HALF_FRAME_SIZE] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MultiBufferRotate(MultiBuffer* mb) {
  for (int i = 0; i < mb->numBufs; i++) {
    mb->indices[i] = NEXT(mb->indices[i], mb->numBufs);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  MultiBufferRotate(&adcBuf);
  adcBuf.dmaDone = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  MultiBufferRotate(&adcBuf);
  adcBuf.dmaDone = true;
}

void processData() {
  if (adcBuf.dmaDone == true) {
    for (int i = 0; i < adcBuf.bufSize; i++) {
      DMA_BUF(inBuf, float)[i] = INT16_TO_FLOAT( ACTIVE_BUF(adcBuf, int16_t, 0)[i] );
    }
    adcBuf.dmaDone = false;
    MultiBufferRotate(&inBuf);
    // memcpy(&(DMA_BUF(outBuf, float)), &(ACTIVE_BUF(inBuf, float, 0)), HALF_FRAME_SIZE * sizeof(float));
    arm_rfft_fast_f32(&fft, ACTIVE_BUF(inBuf, float, 0), frequency, 0);
    arm_rfft_fast_f32(&fft, frequency, ACTIVE_BUF(outBuf, float, 0), 1);

    MultiBufferRotate(&outBuf);

    for (int i = 0; i < adcBuf.bufSize; i++) {
      DMA_BUF(dacBuf, int16_t)[i] = FLOAT_TO_INT16( ACTIVE_BUF(outBuf, float, 0)[i] );
    }
    MultiBufferRotate(&dacBuf);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 2 * HALF_FRAME_SIZE);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacData, 2 * HALF_FRAME_SIZE, DAC_ALIGN_12B_R);

  arm_rfft_fast_init_f32(&fft, HALF_FRAME_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    processData();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
