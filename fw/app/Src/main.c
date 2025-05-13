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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {false, true} bool;

typedef enum {OLD, PREV, NEW} processFrame;
typedef enum {LEFT, RIGHT} rawFrame;



#define NUM_PARAMETERS  5
typedef enum {
  THRESHOLD = 0,
  VOLUME,
  SUBTRACTIONS,
  ATTENUATION,
  COYOTE
} ParameterType;

typedef struct {
  const ParameterType type;
  int32_t counter;
  const float32_t min;
  const float32_t max;
  const float32_t step;
  volatile float32_t value;
  volatile float32_t baseValue;
} Parameter;

typedef struct {
  ParameterType curr;
  Parameter parameters[NUM_PARAMETERS];
} UserControl;



#define MAX_BUFS  16
typedef struct {
  const uint16_t numBufs;
  const uint16_t bufSize;
  const enum { NORMAL, CIRCULAR } dmaMode;
  volatile bool dmaDone;
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
#define FLOAT_HALF_FRAME  HALF_FRAME_SIZE * sizeof(float)
#define FULL_FRAME_SIZE   2 * HALF_FRAME_SIZE
#define FLOAT_FULL_FRAME  FULL_FRAME_SIZE * sizeof(float)

// mix signed and unsigned of same rank - UNSIGNED WINS. DO NOT USE U
#define ADC_MAX           4096
#define ADC_MAX_F         4096.0f
#define ADC_MID           2048

#define CNT_MAX           65536
#define CNT_MID           32768

#define BOUNCE_TIME       168 * 50  // kHz * ms. prescaler of 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b)            ((a) < (b) ? a : b)         // MIND THE SIDE EFFECTS!
#define MAX(a,b)            ((a) > (b) ? a : b)         // MIND THE SIDE EFFECTS!
#define NEXT(n,m)           ((n) == (m-1) ? 0 : (n+1))  // MIND THE SIDE EFFECTS!
#define PREV(n,m)           ((n) == 0 ? (m-1) : (n-1))  // MIND THE SIDE EFFECTS!

#define READY_RAW_BUF(b, n) ((int16_t(*)[b.bufSize]) (b.buffers))[b.activeInds[n]]
#define LOAD_RAW_BUF(b)     ((int16_t(*)[b.bufSize]) (b.buffers))[b.dmaInd]
#define READY_BUF(b, n)     ((float32_t(*)[b.bufSize]) (b.buffers))[b.activeInds[n]]
#define LOAD_BUF(b)         ((float32_t(*)[b.bufSize]) (b.buffers))[b.dmaInd]

#define INT16_TO_FLOAT(n)   (1.0f / ADC_MAX_F * (n - ADC_MID))
#define FLOAT_TO_INT16(n)   ((int16_t)(ADC_MAX * n) + ADC_MID)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t adcData [2 * HALF_FRAME_SIZE] = {0};
int16_t dacData [2 * HALF_FRAME_SIZE] = {0};
MultiBuffer adcBuf = {2, HALF_FRAME_SIZE, CIRCULAR, false, {.indices = {0, 1}}, (void*)adcData};
MultiBuffer dacBuf = {2, HALF_FRAME_SIZE, CIRCULAR, false, {.indices = {0, 1}}, (void*)dacData};

float32_t inData    [3 * HALF_FRAME_SIZE] = {0.0};
float32_t outData   [3 * HALF_FRAME_SIZE] = {0.0};
MultiBuffer inBuf =  {3, HALF_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2}}, (void*)inData};
MultiBuffer outBuf = {3, HALF_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2}}, (void*)outData};

float32_t inFrames  [4 * FULL_FRAME_SIZE] = {0.0};
float32_t outFrames [4 * FULL_FRAME_SIZE] = {0.0};
MultiBuffer inFrameBuf =  {4, FULL_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2, 3}}, (void*)inFrames};
MultiBuffer outFrameBuf = {4, FULL_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2, 3}}, (void*)outFrames};

float32_t hann      [FULL_FRAME_SIZE] = {0.0};
float32_t tempHalf1 [HALF_FRAME_SIZE] = {0.0};
float32_t tempHalf2 [HALF_FRAME_SIZE] = {0.0};
float32_t temp1     [FULL_FRAME_SIZE] = {0.0};
float32_t temp2     [FULL_FRAME_SIZE] = {0.0};

float32_t X         [4 * FULL_FRAME_SIZE] = {0.0};
MultiBuffer XBuf =    {4, FULL_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2, 3}}, (void*)X};
float32_t Xmag      [4 * HALF_FRAME_SIZE] = {0.0};
MultiBuffer XmagBuf = {4, HALF_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2, 3}}, (void*)Xmag};
float32_t Smag      [4 * HALF_FRAME_SIZE] = {0.0};
MultiBuffer SmagBuf = {4, HALF_FRAME_SIZE, NORMAL, false, {.indices = {0, 1, 2, 3}}, (void*)Smag};
float32_t uMag      [HALF_FRAME_SIZE] = {0.0};

float32_t meanRatio = 0.0;
float32_t logMeanRatio = 0.0;
float32_t coolDown = 0.0;
float32_t threshold = 0.0;
bool speech = false;
bool speechBefore = false;
bool forceuMag = true;
bool mute = false;
bool bypass = false;

arm_rfft_fast_instance_f32 fft;

UserControl ctrl = {THRESHOLD, 0, {
  {THRESHOLD,     0,  -25.0,  0.0,  1.0, -20.0, -20.0},
  {VOLUME,        0,   0.0,   1.0,  0.05, 0.9,   0.9},
  {SUBTRACTIONS,  0,   0.0,   10.0, 1.0,  2.0,   2.0},
  {ATTENUATION,   0,   0.00,  0.10, 0.01, 0.03,  0.03},
  {COYOTE,        0,   0.0,   50.0, 5.0,  30.0,  30.0}
}};
bool justPressed = false;     // protect timer from improper restarting
uint8_t clickCount = 0;
uint8_t holdCount = 0;
uint16_t pinPressed = B1_Pin;
uint16_t pinOld = B1_Pin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void arm_hanning_f32(float32_t * pDst, uint32_t blockSize) {
  // copied verbatim from GitHub source. ST provides a limited CMSIS DSP implementation.
  float32_t k = 2.0f / ((float32_t) blockSize);
  float32_t w;

  for (uint32_t i=0; i < blockSize; i++) {
    w = PI * i * k;
    w = 0.5f * (1.0f - cosf (w));
    pDst[i] = w;
  }
}



static int16_t clampInt16(int16_t value, int16_t min, int16_t max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}

static void MultiBufferRotate(MultiBuffer* mb) {
  for (int i = 0; i < mb->numBufs; i++) {
    mb->indices[i] = NEXT(mb->indices[i], mb->numBufs);
  }
}



static float32_t UserControlValue(const UserControl* control, ParameterType type) {
  return control->parameters[type].value;
}

static int16_t UserControlUpdate(UserControl* control) {
  ParameterType* type = &(control->curr);
  Parameter* param = &(control->parameters[*type]);

  int32_t count = (int32_t)(TIM2->CNT) - CNT_MID;
  float32_t newValue = param->baseValue + count * param->step;

  if (newValue > param->max || newValue < param->min) {
    TIM2->CNT = (uint32_t)(param->counter + CNT_MID);
  } else {
    param->value = newValue;
    param->counter = count;
  }

  return count;
}

static ParameterType UserControlSwitch(UserControl* control) {
  ParameterType* type = &(control->curr);
  *type = NEXT(*(type), NUM_PARAMETERS);
  TIM2->CNT = (uint32_t)(CNT_MID + control->parameters[*type].counter);
  UserControlUpdate(control);
  return *type;
}



void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (justPressed == false || TIM10->CNT > BOUNCE_TIME) {
    pinOld = pinPressed;
    pinPressed = pin;
    if (pinPressed != pinOld) {
      clickCount = 0;
    }
    clickCount += 1;
  }
  if (justPressed == false) {
    justPressed = true;
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {

    if (pinPressed == B1_Pin) {   // double press
      bypass = !bypass;
    } 
    else if (pinPressed == SwitchBtn_Pin) {
      if (clickCount == 1) {
        if (HAL_GPIO_ReadPin(SwitchBtn_GPIO_Port, SwitchBtn_Pin) == GPIO_PIN_RESET) {
          holdCount += 1;
          return;
        } else {
          if (holdCount < 2) {
            mute = !mute;         // single press or shold hold
          } else {
            forceuMag = true;     // long hold
          }
        }
      } 
      else if (clickCount > 1) {
        UserControlSwitch(&ctrl); // double press
      }
    }

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_TIM_Base_Stop_IT(&htim10);
    clickCount = 0;
    holdCount = 0;
    justPressed = false;
    TIM10->CNT = 0;
  }
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  
  MultiBufferRotate(&adcBuf);
  adcBuf.dmaDone = true;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc);
  // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  MultiBufferRotate(&adcBuf);
  adcBuf.dmaDone = true;
}



static void processData() {
  // load from ADC
  for (int i = 0; i < adcBuf.bufSize; i++) {
    LOAD_BUF(inBuf)[i] = INT16_TO_FLOAT( READY_RAW_BUF(adcBuf, 0)[i] );
  }
  adcBuf.dmaDone = false;
  MultiBufferRotate(&inBuf);
  
  if (mute) {
    memset(LOAD_BUF(outBuf), 0, FLOAT_HALF_FRAME);
    MultiBufferRotate(&outBuf);
  } else if (bypass) {
    memcpy(LOAD_BUF(outBuf), READY_BUF(inBuf, 1), FLOAT_HALF_FRAME);
    MultiBufferRotate(&outBuf);
  } else {

    // copy twice as fast!
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1,
      (uint32_t)(READY_BUF(inBuf, LEFT)),
      (uint32_t)(&temp1[0]),
      FLOAT_HALF_FRAME
    );
    HAL_DMA_Start(&hdma_memtomem_dma2_stream2,
      (uint32_t)(READY_BUF(inBuf, RIGHT)),
      (uint32_t)(&temp1[HALF_FRAME_SIZE]),
      FLOAT_HALF_FRAME
    );
    HAL_StatusTypeDef done1 = HAL_BUSY, done2 = HAL_BUSY;
    do {
      done1 = HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1, HAL_DMA_FULL_TRANSFER, 100);
      done2 = HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream2, HAL_DMA_FULL_TRANSFER, 100);
    } while ((done1 != HAL_OK) && (done2 != HAL_OK));
    // apply window
    arm_mult_f32(temp1, hann, LOAD_BUF(inFrameBuf), FULL_FRAME_SIZE);
    MultiBufferRotate(&inFrameBuf);

    // fft, mag
    memcpy(temp1, READY_BUF(inFrameBuf, NEW), FLOAT_FULL_FRAME);
    arm_rfft_fast_f32(&fft,
      temp1,
      LOAD_BUF(XBuf),
      false
    );
    MultiBufferRotate(&XBuf);
    arm_cmplx_mag_f32(READY_BUF(XBuf, NEW), LOAD_BUF(XmagBuf), HALF_FRAME_SIZE);
    MultiBufferRotate(&XmagBuf);
    
    // spectral subtraction
    arm_scale_f32(uMag, UserControlValue(&ctrl, SUBTRACTIONS), tempHalf1, HALF_FRAME_SIZE);
    arm_sub_f32(
      READY_BUF(XmagBuf, NEW),
      tempHalf1,
      tempHalf2,
      HALF_FRAME_SIZE
    );
    // half-wave retification
    arm_clip_f32(
      tempHalf2,
      LOAD_BUF(SmagBuf),
      0.0f,
      MAXFLOAT / 2,
      HALF_FRAME_SIZE
    );
    MultiBufferRotate(&SmagBuf);

    // average spectral SNR
    for (int i = 0; i < HALF_FRAME_SIZE; i++) {
      tempHalf1[i] = READY_BUF(SmagBuf, NEW)[i] / uMag[i];
      // CMSIS team said fp division cannot be accelerated
    }
    arm_mean_f32(tempHalf1, HALF_FRAME_SIZE, &meanRatio);
    logMeanRatio = 20 * log10f(meanRatio);
    
    // voice activity detection & coyote time
    threshold = UserControlValue(&ctrl, THRESHOLD);
    speechBefore = speech;
    if (logMeanRatio >= threshold) {
      speech = true;
      coolDown = UserControlValue(&ctrl, COYOTE);
      if (speechBefore == false) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }
    } else {
      if (coolDown > 0.0) {
        speech = true;
        coolDown -= 1.0f;
      } else {
        speech = false;
        coolDown = 0.0;
        if (speechBefore == true) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
      }
    }

    // attenuate during non-speech
    if (speech == false) {
      arm_scale_f32(
        READY_BUF(SmagBuf, NEW),
        UserControlValue(&ctrl, ATTENUATION),
        LOAD_BUF(SmagBuf),
        HALF_FRAME_SIZE
      );
    } else {  // apply volume
      arm_scale_f32(
        READY_BUF(SmagBuf, NEW),
        UserControlValue(&ctrl, VOLUME),
        LOAD_BUF(SmagBuf),
        HALF_FRAME_SIZE
      );
    }
    MultiBufferRotate(&SmagBuf);

    // update noise spectrum. average
    if (forceuMag == true) {
      speech = false;
      forceuMag = false;
    }
    if (speech == false) {
      arm_add_f32(
        READY_BUF(XmagBuf, NEW),
        READY_BUF(XmagBuf, PREV),
        tempHalf1,
        HALF_FRAME_SIZE
      );
      arm_add_f32(
        tempHalf1,
        READY_BUF(XmagBuf, OLD),
        tempHalf2,
        HALF_FRAME_SIZE
      );
      arm_scale_f32(tempHalf2, (1.0f / 3.0f), uMag, HALF_FRAME_SIZE);
    }

    // polar to rect
    for (int i = 0; i < HALF_FRAME_SIZE; i++) {
      temp1[i] = READY_BUF(SmagBuf, NEW)[i] / READY_BUF(XmagBuf, NEW)[i];
    }
    for (int i = 0; i < HALF_FRAME_SIZE; i++) {
      if (READY_BUF(XmagBuf, NEW)[i] != 0.0f) {
        temp2[2*i  ] = temp1[i] * READY_BUF(XBuf, NEW)[2*i];   // cos
        temp2[2*i+1] = temp1[i] * READY_BUF(XBuf, NEW)[2*i+1]; // sin
      } else {  // ideally should never happen with floats
        temp2[2*i  ] = 0.0f;
        temp2[2*i+1] = 0.0f;
      }
    }

    // ifft
    arm_rfft_fast_f32(&fft, temp2, LOAD_BUF(outFrameBuf), true);
    MultiBufferRotate(&outFrameBuf);

    // reconstruct, 50% overlap
    arm_add_f32(
      &(READY_BUF(outFrameBuf, RIGHT)[0]),
      &(READY_BUF(outFrameBuf, LEFT)[HALF_FRAME_SIZE]),
      LOAD_BUF(outBuf),
      HALF_FRAME_SIZE
    );
    MultiBufferRotate(&outBuf);
  }
  
  // push to DAC
  for (int i = 0; i < adcBuf.bufSize; i++) {
    LOAD_RAW_BUF(dacBuf)[i] = clampInt16(
      FLOAT_TO_INT16( READY_BUF(outBuf, 0)[i] ),
      0,
      ADC_MAX
    );
  }
  MultiBufferRotate(&dacBuf);
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
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 2 * HALF_FRAME_SIZE);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacData, 2 * HALF_FRAME_SIZE, DAC_ALIGN_12B_R);

  arm_rfft_fast_init_f32(&fft, FULL_FRAME_SIZE);
  arm_hanning_f32(hann, FULL_FRAME_SIZE);
  TIM2->CNT = CNT_MID;

  for (int i = 0; i < HALF_FRAME_SIZE; i++) {
    uMag[i] = 1.0f;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UserControlUpdate(&ctrl);
    if (adcBuf.dmaDone == true) {
      processData();
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
