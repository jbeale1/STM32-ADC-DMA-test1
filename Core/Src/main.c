/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ADC DMA readings on Nucleo STM32 L432KC, count signal peaks

  * 4x ovrsamp ADC @ 6.5 clocks of 32 MHz => 1.2308 MHz eff. sample rate, 307.7 Hz @ 4k buff
  * 166.92 Hz pulse rate x 8000 samples => 1.335Msps on DMA channel  (12/9/2023 jpb)
  *
  * DMA half,full @ 8k buff: 3ms high, 3ms low f=166.8 Hz  TDS-210 scope
  * each 8k buffer proc in 6 msec, make sure to use -Ofast flag to C++ compiler.
  * 80 MHz ADC clock
  *
  * https://www.st.com/resource/en/datasheet/stm32l432kc.pdf
  * STM32L432KC datasheet p.117/156: not all ADC inputs have the same speed!
      3. Fast channels are: PC0, PC1, PC2, PC3, PA0, PA1.
      4. Slow channels are: all ADC inputs except the fast channels.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>  // sqrt()
#include <stdio.h> // printf()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPU_RATE (80000000)   // CPU counter ticks per second
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ADC_BUFFER_SIZE 8000 // was 20k
#define SPEC_SIZE 512
#define PROC_BUF_SIZE (ADC_BUFFER_SIZE/2)
uint16_t adc_buffer[ADC_BUFFER_SIZE];  // DMA writes ADC data into this buffer

uint32_t specOut[SPEC_SIZE];
uint32_t pBuf[PROC_BUF_SIZE];
unsigned int pBp1 = 0;
unsigned int pBp2 = 2;

// uint16_t tBuf[ADC_BUFFER_SIZE/2];       // working buffer for operation & printout

uint32_t counter = 0;
uint32_t tick=0;      // geiger counter "count"
uint32_t lastTick=0;

uint32_t sMax = 0;
uint32_t sMin = 65535;

uint32_t sMaxTP = 0;

float avgVal = 511;  // 53 as of 6-Dec-2023 (4x ovrsamp)
float avgValFilt = 0.001;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int readingCount = 0;            // how many sets of ADC readings done
volatile int buf1Ready = false;  // Ping buffer is ready
volatile int buf2Ready = false;  // Pong buffer is ready

int pThreshold = 100;            // ADC amplitude threshold for finding rising edge

// Read a set of values from STM32 Nucleo L432KC board 12-bit ADC and get statistics
// Typical St.Dev. = 3.1 counts. Single-ended input from 1.5V AA (4095 counts = 3.3V)
// 14800 readings per second (SAMPLES=4k, no ADC oversampling)
// count   sps1      sps2      avg       min       max       st.dev
// 16.000  13675.214 13673.658 28240.963 28186.000 28297.000 15.218  (16x oversample)
// 10852.0 13675.2   13673.5   28252.0   28194.0   28317.0   14.5

// Process a Ping or Pong buffer looking for peaks
void procBuf(int bIndex) {
    int idxStart, idxEnd;

    if (bIndex==1) {
    	idxStart=0;
    	idxEnd = ADC_BUFFER_SIZE/2;
    } else {
    	idxStart=ADC_BUFFER_SIZE/2;
    	idxEnd = ADC_BUFFER_SIZE;
    }

    // int pThreshold = (132-avgMin);         // ADC thresh for rising edge (no oversamp)
    // int pThreshold = (510-avgMin);            // ADC thresh for rising edge (8x oversamp, <1 bitshft)
	sMax = 0;
	sMin = 65535;
	uint32_t x = 0;
	uint32_t tickOrig = tick;  // remember old value of tick counter
	uint32_t xTh = avgVal + 30;  // was 37 threshold for valid count

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // GPIO signal flag
	int j=0;
    for (int i = idxStart; i < idxEnd; i++) {  // copy over DMA mem into working buffer
      x = adc_buffer[i];
      pBuf[j++]=x;
      if (x > sMax) sMax = x;
    }

    for (int i=2; i<PROC_BUF_SIZE-3; i++)
    {
      if ((pBuf[i] > xTh) && (pBuf[i-1] < xTh) && (pBuf[i-2] < xTh)) {
		  tick++;
		  int e = (pBuf[i] + pBuf[i+1] + pBuf[i+2] + pBuf[i+3]) >> 6; // combine this and next sample as peak val
		  if (e >= SPEC_SIZE) {
			  e = (SPEC_SIZE-1);
		  }
		  specOut[e]++;
      }

      j++;  // index into pBuf[]
    }
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // GPIO signal flag

    if (tickOrig == tick) { // no new pulses found in this entire buffer section
    	avgVal = (avgVal * (1.0-avgValFilt)) + (sMax) * avgValFilt;
    //} else if ((tick - tickOrig) > 15) {
    }
    /* else if (sMax > 1000) {
        for (int i=0; i<PROC_BUF_SIZE; i++) {
        	printf("%d,%ld\n", i, pBuf[i]);
        }
    }
    */
	counter++;

	//sMaxTP = (sMax - 440);
	//if (sMaxTP < 0) sMaxTP = 0;
	//if (sMaxTP > 12) cps++;        // above threshold counts as a hit
	//printf("%04d,%d\n",sMaxTP,tick);
	//printf("%04ld,%04ld,%ld\n",sMax,(uint32_t)avgVal,tick);  // about 248 sps
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// enable core cycle counter per https://stackoverflow.com/questions/42747128/
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  for (int i=0;i<SPEC_SIZE;i++) {
	  specOut[i] = 0; // initialize spectrum array
  }

  //uint32_t msecPeriod = 60000;   // report time period in msec
  uint32_t msecPeriod = 10000;   // report time period in msec

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  uint32_t tLast = HAL_GetTick();
  unsigned int cycle=0;  // how many output cycles so far
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //sMaxT = 0;  // peak ADC value ever seen
	//sMinT = 65535;  // min ADC value ever seen

	while (!buf1Ready);
	buf1Ready = false;
	procBuf(1); // process first half ("ping") of buffer
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	while (!buf2Ready);
	buf2Ready = false;
	procBuf(2); // process second half ("pong") of buffer
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	uint32_t deltaT = HAL_GetTick() - tLast;  // msec since last readout
	// const int16_t loopSec = 224;                  // half-buffer-loops per second
	//if (counter > (loopSec * 60 )) {               // report CPM each minute
	//if (counter > (loopSec )) {               // report CPS each second
	if (deltaT >= msecPeriod) {
		//printf("%d,%d\n", counter,tick);
		printf("%d,%ld\n", cycle,tick);
		cycle++;
		counter = 0;
		tLast += msecPeriod;   // aim for one readout every second
		lastTick = tick;
		tick=0;

		for (int i=0;i<SPEC_SIZE/2;i++) {
		  printf("%d,%ld\n", i,specOut[i]);
		}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  buf1Ready = true;  // Ping buffer is ready
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  buf2Ready = true;  // Pong buffer is ready
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

}

// ================================================================

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
