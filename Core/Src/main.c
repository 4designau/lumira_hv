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
* reminder:
* char - int8_t - 8 bits
* short - int16_t - 16 bits
* int or long - int32_t - 32 bits
* long long - int64_t - 64 bits, suffix LL
* there is no suffix for char or short.
* whether you leave blank or add the suffix L, #define ends up being int16
*******************************************************************************
* the MCU has 5 programable exit from reset voltages:
*      VBORR4, VBORR3, VBORR2, VBORR1 and VPOR
*      3V, 2.68V, 2.38V, 2.18V, 1.98V
* the MCU has 5 programmable entry into reset voltages:
*      VBORF4, VBORF3, VBORF2, VBORF1 and VPDR
*      2.7V, 2.4V, 2.2V, 1.95V, 1.88V
* by default VPOR and VPDR apply.
* to enable the other levels, the BOR module must be enabled.
*******************************************************************************
* there are 960000, 48MHz cycles in 50Hz, or 480000 in a half cycle.
* there are 800000, 48MHz cycles in 60Hz, or 400000 in a half cycle.
* here is a list of common divisors of 480000 and 400000:
*
* divisor    period
*            (ns)
* 1            20.8333
* 2            41.6666
* 4            83.3333
* 5           104.1666
* 8           166.6666
* 10          208.3333
* 16          333.3333
* 20          416.6666
* 25          520.8333
* 32          666.6666
* 40          833.3333
* 50         1041.6666
* 64         1333.3333
* 80         1666.6666
* 100        2083.3333
* 125        2604.1666
* 128        2666.6666
* 160        3333.3333
* 200        4166.6666
* 250        5208.3333
* 320        6666.6666
* 400        8333.3333
* 500        1041.6666
* 625       13020.8333
* 640       13333.3333
* 800       16666.6666
* 1000      20833.3333
* 1250      26041.6666
* 1600      33333.3333
* 2000      41666.6666
* 2500      52083.3333
* 3125      65104.1666
* 3200      66666.6666
* 4000      83333.3333
* 5000     104166.6666
* 6250     130208.3333
* 8000     166666.6666
* 10000    208333.3333
* 20000    416666.6666
* 40000    833333.3333
* 80000   1666666.6666
********************************************************************************
* use these variable prefixes c = 8 bit
*                             s = 16 bit
*                             i = 32 bit
*                             l = 64 bit
*                             v = volatile
********************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SYSCLK_HZ (48000000UL)
//#define SAMPLE_PERIOD_US (100UL)
//#define SAMPLE_TIMER_ARR ((SYSCLK_HZ / 1000000UL) * SAMPLE_PERIOD_US - 1UL) /* 4800-1 to give 100us period */
//#define BAUD_RATE_U 115200UL // user defined baud rate bits per second, each byte takes 87us.
#define END_PRINT_CYCLES_ST 5000 // print period in sample time increments, at 83.3333us sample rate can pretty
                                 // much print one character per sample.
                                 // we want to accumulate 256 samples and then print something like 2000 x 2 characters.
                                 // We need 4256 sample intervals to finish doing that.
                                 // see notes below about the print logic.

#define END_BUFFER_CYCLES_ST 256
#define PRINT_BUFFER_SIZE 20     // only needs to hold two value at a time

//#define ADC_SEQ_LEN (7U)
//#define N_RMS (128U)
#define FIFTY_SIXTY_HZ_THRESHOLD_ST10 2200 // sample interval is 83.3333us,
                                           // this counts a full cycle.
                                           // there are 120 of these in one 50Hz half cycle and 100 in 60Hz.
                                           // unit is 10ths of a sample interval
#define FIFTY_HZ_COUNTS_ST 120 // number of sample intervals in a 50Hz half cycle
#define SIXTY_HZ_COUNTS_ST 100 // number of sample intervals in a 60Hz half cycle
#define FIFTY_HZ_DEFAULT_DUTY 120
#define SIXTY_HZ_DEFAULT_DUTY 100
#define DEAD_BAND_ST 60 // after we detect a zero crossing, do not look for another one until at least this many sample intervals
#define MAX_PERIOD_ADJUSTMENT_ST 1 // the maximum number of sample periods we are allowed to adjust the mains frequency by each half cycle

/* placeholders - adjust to your hardware scaling */
#define ADC_REF_VOLT (3.3)
#define ADC_MAX_COUNTS (4095)
#define NEUTRAL_ADC_COUNTS (2048)

#define ADC_NUM_CH 7U
// #define ADC_FRAMES 8U
// #define ADC_DATA_BUFFER_SIZE (ADC_NUM_CH * ADC_FRAMES)
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
#define VDDA_APPLI 3300U
#define TEST_VOLTAGE_mV 3000 // the MCU Vrefint is factory calibrated with this supply in milivolts
#define MSG_BUFFER_SIZE 2000 // has to be able to hold 256, 4 digit values, plus \r\n, that's 1536 characters.

// variables to implement the filters intended to remove offset from readings
// we want to attenuate 50Hz from a peak amplitude of 730 counts to 0.25 counts
// nominally 68s time constant when sample interval is 83us
#define ZERO_FILTER_MULTIPLIER_MINUS_ONE 131071
#define ZERO_FILTER_SHIFTS 17

// voltage and current filter constants
// woth shift of 2 delay is about 1 sample interval
// with shift of 3 delay is about 3 sample intervals and could probably get away with 2 sample intervals to make an estimate of slope
// sift of 4 is reasonably smooth output, but it introduces a delay of about 15 sample intervals
// a version with a sift 3 filter and look forward 4 voltage was tried but not better than shift 2 filter by its self
// a shift 2 filter seems to give a max zero crossing delay of about 4 sample intervals.
// additionaly we need to add protection for zero crossing bounce.

// use this setting for normal run mode
#define RUN_FILTER_MULTIPLIER_MINUS_ONE 3
#define RUN_FILTER_SHIFTS 2

// #define FILTER_MULTIPLIER_MINUS_ONE 15
// #define FILTER_SHIFTS 4

// #define FILTER_MULTIPLIER_MINUS_ONE 31
// #define FILTER_SHIFTS 5

// use this setting to pick up mains frequency
#define INIT_FILTER_MULTIPLIER_MINUS_ONE 63
#define INIT_FILTER_SHIFTS 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* State Variables only used in this file */
volatile uint8_t ucv_Stage;  // 3 is the first cycle that is ignored
                             // 2 means nothing aquired
                             // 1 means one rising edge on mains detected
                             // 0 means two rising edges on mains detected and we are ready to proceed
volatile uint8_t ucv_AState; // active state, 1 positive, 0 negative
volatile uint8_t ucv_DeadTime; // 1 after transition detected, 0 other times
                               // using dead time to set min half cycle duration as well as preventing contact bounce when frequency detecting
volatile uint8_t ucv_LeadingEdge; // 1 when set to leading edge, 0 when set to trailing edge
int16_t s_Nominal_Half_Period_ST; // remembers whether we are in a 50Hz or 60Hz system.
int16_t s_Half_Period_Counter_ST; // counts sample intervals in some current half cycle.

/* Variables only used in this file */
uint16_t us_voltageRawValue[ADC_NUM_CH];
int32_t i_ADCCurrentFilt; // not volatile because only used in ISR

/* Global state variables */
__IO uint8_t ucv_adcDmaTransferStatus; // 3 = buffer, 2 = print complete or wait, 1 = full transfer or print, 0 = half transfer, __IO is same as volatile
// __IO uint8_t ucv_AdcStatus = 1; // 1 sets ready to read temperature, 2 sets ready to read Vrefint
// __IO uint16_t usv_PrintCounter = 0; // when us_PrintCounter reaches PRINT_PERIOD_ST
                                                  // ISR internal values are copied to printable values
                                                  // Printing must finish before usv_PrintCounter
                                                  // finishes counting up again.
                                                  // DMA data processing needs to finish within one sample interval.
__IO uint16_t usvp_Period_Counter_ST10; // counts the period of some current cycle in 10ths of a sample interval.
	                                    // using increments of 10 because we want to filter this
	                                    // so increments of 10 gives more filter resolution
/* Global variables */
/* some variables are volatile because we want them to persist and be initialized at startup. */
/* others are volatile because they are copied out of the ISR to the print routine in main, those ones have a suffixe p as well as v */

__IO uint8_t ucv_switchState; // 1 on 0 off
__IO uint8_t ucv_switchStateRequest; // 1 on 0 off, the dynamic request veriable
__IO uint8_t ucv_SwitchCycleStateRequest; // 1 on 0 off, but can only be set at start of half cycle
__IO uint8_t ucv_WasLow;
__IO int16_t sv_Duty_ST; // duty measured in sample intervals

__IO uint16_t usvp_ADCTemperatureReading;
__IO uint16_t usvp_ADCVrefintReading;
__IO int16_t svp_ADCCurrent;
__IO int16_t svp_ADCVoltage;
__IO int16_t svp_ADCLoadVoltage;
__IO int16_t svp_ADCSenseVoltage;

__IO int32_t iv_CurrentZero;
__IO int32_t iv_AZero;
__IO int32_t iv_SAZero;
__IO int32_t iv_VrefintAv;
__IO int32_t iv_TempAv;
__IO int32_t iv_CurrentSQ;
__IO int32_t iv_ASQ; // active voltage
__IO int32_t iv_P; // power
__IO int64_t lv_E;

__IO uint16_t usv_Count;

__IO size_t sizev_len;
__IO int16_t sv_ValueBuffer1[((uint16_t)END_BUFFER_CYCLES_ST)];
__IO int16_t sv_ValueBuffer2[((uint16_t)END_BUFFER_CYCLES_ST)];

__IO uint8_t ucv_FilterState; // for filter use: 1 not first, 2 is first

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void USART1_Write(const char *s);
void ADC_Calibration_Start(void);
void ADC_DMA_Start(void);
void TIM_Base_Start(void);
void OutputProcessing(void);
void RightShiftFilter(int32_t *, int32_t *, int32_t, int32_t, int32_t);

// void AdcDmaTransferComplete_Callback(void) prototype in main.h
// void AdcDmaTransferError_Callback(void) //called from ISR, prototype in main.h
// void Error_Handler(void) // prototype in main.h
// void assert_failed(uint8_t *file, uint32_t line) // prototype in stm32_assert.h

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
  // uint32_t ui_startIndex = 0;
  uint16_t us_vrefint_cal = *VREFINT_CAL_ADDR; // the internal calibration value to calibrate the MCU supply
  uint32_t ui_main_temp;
  uint16_t us_AppSupply_mV; // measured on one device as being 3.35V.
  char cc_buf[PRINT_BUFFER_SIZE]; // has to be able to hold 1 signed 16 bit integer plus a \r and \n.
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SVC_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, 3);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 3);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* initialize measurements, do this before initializing peripherals because these variables get used in the ISRs */

  /* initialize mains state */
  ucv_Stage = ((uint8_t)2); // starting in stage 2 to reduce code, no need to start in stage 3
  ucv_AState = ((uint8_t)1); // choose a positive active line as the state to start
  ucv_DeadTime = 0; // 1 after transition detected for 1/4 cycle
  ucv_LeadingEdge = 1; // set to leading edge

  ucv_adcDmaTransferStatus = ((uint8_t)2); // 3 = buffer, 2 = print complete or wait, 1 = full transfer or print, 0 = half transfer
  usv_Count = ((uint16_t) END_PRINT_CYCLES_ST) - ((uint16_t) 1);
  sizev_len = 0;

  ucv_FilterState = 2; // first

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ADC_Calibration_Start();
  ADC_DMA_Start();
  TIM_Base_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // char cc_buf[MSG_BUFFER_SIZE];

  while (1)
  {
    /* print */
	if (ucv_adcDmaTransferStatus == 1) // // 3 = buffer, 2 = print complete or wait, 1 = full transfer or print, 0 = half transfer
    {
      // uint8_t st = ucv_adcDmaTransferStatus;

      // ui_startIndex = (st == 1) ? ADC_DATA_BUFFER_SIZE / 2 : 0;

      // snprintf(cc_buf, sizeof(cc_buf),
      //          "ADC (%s): ch0=%u ch1=%u ch2=%u ch3=%u ch4=%u\r\n",
      //          (st == 0) ? "HT" : "TC",
      //          ch0, ch1, ch2, ch3, ch4);

      // snprintf(cc_buf, sizeof(cc_buf),
      //        "Vref=%u Temp=%u current=%d av=%d sav=%d sv=%d\r\n",
      //        usvp_ADCVrefintReading, usvp_ADCTemperatureReading, svp_ADCCurrent, svp_ADCVoltage, svp_ADCLoadVoltage, svp_ADCSenseVoltage);

      // snprintf(cc_buf, sizeof(cc_buf),
      //        "Vref=%u Temp=%u current=%d av=%d sav=%d sv=%d period=%u\r\n",
      //        usvp_ADCVrefintReading, usvp_ADCTemperatureReading, svp_ADCCurrent, svp_ADCVoltage, svp_ADCLoadVoltage, svp_ADCSenseVoltage, usvp_Period_Counter_ST10);

      ui_main_temp = ((uint32_t)us_vrefint_cal) * ((uint32_t)TEST_VOLTAGE_mV);
      ui_main_temp = ui_main_temp / ((uint32_t)usvp_ADCVrefintReading);
      us_AppSupply_mV = ((uint16_t)ui_main_temp);

      // snprintf(cc_buf, sizeof(cc_buf),
      //         "Vdd(mV)=%u Temp=%u current=%d av=%d sav=%d sv=%d period=%u\r\n",
	  // 	    us_AppSupply_mV, usvp_ADCTemperatureReading, svp_ADCCurrent, svp_ADCVoltage, svp_ADCLoadVoltage, svp_ADCSenseVoltage, usvp_Period_Counter_ST10);

      //snprintf(cc_buf, sizeof(cc_buf),
      //        "Vdd(mV)=%u Temp=%u current=%d av=%d sav=%d sv=%d period=%u\r\n",
	  //	    us_AppSupply_mV, usvp_ADCTemperatureReading, svp_ADCCurrent, svp_ADCVoltage, svp_ADCLoadVoltage, svp_ADCSenseVoltage, usvp_Period_Counter_ST10);

      // if print state is set to PRINT, move values into print buffer, print buffer and then set the print state to WAIT

      for (uint16_t us_Index = 0; us_Index<END_BUFFER_CYCLES_ST ; us_Index++)
      {
    	  // load the buffer
          snprintf(cc_buf, sizeof(cc_buf),
                   "%d,%d\r\n", sv_ValueBuffer1[us_Index], sv_ValueBuffer2[us_Index]);
          // print the buffer
          USART1_Write(cc_buf);
      }

      // add one more carrage return line feed
      snprintf(cc_buf, sizeof(cc_buf),
               "\r\n");
      // print the buffer
      USART1_Write(cc_buf);

      ucv_adcDmaTransferStatus = 2; // means wait to start another print cycle
    }

    /* slow down printing so UART keeps up */
    // LL_mDelay(200); // 200 ms

    //******* stuff to do with input processing
    if (LL_GPIO_IsInputPinSet(GPIOA, Opto_Input_Pin)) // if high no input
    {
    	// do nothing
    	ucv_WasLow = 0;
    }
    else // if low there may be an input
    {
    	if (ucv_WasLow == ((uint8_t)1)) // there was already an input last sample interval as well
    	{
    	    if (ucv_switchStateRequest == ((uint8_t)1)) // output last requested on
            {
                if (ucv_switchState == ((uint8_t)1)) // and it is on
                {
            	    ucv_switchStateRequest = ((uint8_t)0); // request it off
                }
                else // it is off
                {
                	// do nothing
                }
            }
            else // output last requested off
            {
                if (ucv_switchState == ((uint8_t)1)) // and it is on
                {
                	// do nothing
                }
                else // it is off
                {
                	ucv_switchStateRequest = ((uint8_t)1); // request it on
                }
            }
        }
        else // first time this is low
        {
        	ucv_WasLow = ((uint8_t)1);
        }
    }
  } // end while

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  LL_RCC_SetHSIKERDiv(LL_RCC_HSIKER_DIV_2);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

/* USER CODE BEGIN 4 */
void ADC_Calibration_Start(void)
{
  __IO uint32_t wait_loop_index = 0U;
  __IO uint32_t backup_setting_adc_dma_transfer = 0U;

  // Check if ADC is disabled
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_EnableInternalRegulator(ADC1);
    // Delay for ADC Internal voltage regulator stabilisation
    wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2)) / 10);

    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Disable ADC DMA transfer request during calibration */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);

    /* Poll for ADC effectively calibrated */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (wait_loop_index-- == 0)
        {
          /* Time-out occurred */
        }
      }
    }
    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
  }
}

void ADC_DMA_Start(void)
{
  /* Configure the source and the destination address */
  /* extracts the address of the DMA buffer */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)us_voltageRawValue, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set the Data Length */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_NUM_CH);

  /* Enable DMA Transfer Interruption : Transfer complete, Half transfer, Error Transfer */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  /* Enable DMA Channel */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Enable the ADC and start the conversion*/
  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
  {
  }
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
}

void AdcDmaTransferComplete_Callback(void) //called from DMA ISR, this is the main sample interval routine
{
  static int16_t s_Period_Counter_ST10; // counts the period of some current cycle in 10ths of a sample interval.
	                                     // using increments of 10 because we want to filter this
	                                     // so increments of 10 gives more filter resolution
  static int32_t i_Period_st10; // the filtered value of mains period, always from positive going edge, in 10ths of a sample interval.

  /* get DMA transfered ADC values */
  // us_voltageRawValue[0]; // vrefint
  // us_voltageRawValue[1]; // temperature
  // us_voltageRawValue[2]; // current
  // us_voltageRawValue[3]; // neutral v
  // us_voltageRawValue[4]; // sens voltage
  // us_voltageRawValue[5]; // active voltage
  // us_voltageRawValue[6]; // load voltage
  int32_t i_ADCCurrent;
  int32_t i_ADCVoltage;
  int32_t i_ADCLoadVoltage;
  int32_t i_ADCSenseVoltage;
  int32_t i_ADCTemperatureReading;
  int32_t i_ADCVrefintReading;

  static int32_t i_ADCVoltageFilt;
  static int32_t i_ADCLoadVoltageFilt;
  static int32_t i_ADCSenseVoltageFilt;
  static int32_t i_ADCCurrentRemainder;
  static int32_t i_ADCVoltageRemainder;
  static int32_t i_ADCLoadVoltageRemainder;
  static int32_t i_ADCSenseVoltageRemainder;

  static int32_t i_FilterMultiplierToUse;
  static int32_t i_FilterShiftsToUse;

  static int32_t i_CurrentZero;
  static int32_t i_AZero;
  static int32_t i_SAZero;
  static int32_t i_CurrentRemainder;
  static int32_t i_ActiveRemainder;
  static int32_t i_SwitchedActiveRemainder;

  //*********
  // note below operations are performed on non volatile versions of variables
  // run period filter, not here but when each cycle is finished

  i_ADCCurrent = ((int32_t) us_voltageRawValue[2]) - ((int32_t) NEUTRAL_ADC_COUNTS);
  i_ADCVoltage = ((int32_t) us_voltageRawValue[5]) - ((int32_t) us_voltageRawValue[3]);
  i_ADCLoadVoltage = ((int32_t) us_voltageRawValue[6]) - ((int32_t) us_voltageRawValue[3]);
  i_ADCSenseVoltage = ((int32_t) us_voltageRawValue[4]) - ((int32_t) us_voltageRawValue[3]);
  i_ADCTemperatureReading = ((int32_t)us_voltageRawValue[1]);
  i_ADCVrefintReading = ((int32_t)us_voltageRawValue[0]);

  /* unfiltered output values */
  // run isquared filter ********
  iv_CurrentSQ = ((int32_t)i_ADCCurrent) * i_ADCCurrent;
  // run vsquared filter ********
  iv_ASQ = i_ADCVoltage * i_ADCVoltage;
  // run p filter ********
  iv_P = i_ADCCurrent * i_ADCVoltage;
  // run Vrefint filter ********
  iv_VrefintAv = i_ADCVrefintReading;
  // run Temperature filter ********
  iv_TempAv = i_ADCTemperatureReading;
  // acumulate energy
  lv_E = lv_E + ((int64_t)iv_P);

  /****************************************************/
  /* manage filtered values                           */
  /* so far only removing offset from 3 main channels */
  /****************************************************/
  if (ucv_FilterState == 2) // first sample interval
  {
	  ucv_FilterState = 1; // not first

      i_CurrentZero = 0;
      i_AZero = 0;
      i_SAZero = 0;
      i_CurrentRemainder = 0;
      i_ActiveRemainder = 0;
      i_SwitchedActiveRemainder = 0;
      i_ADCCurrentFilt = i_ADCCurrent;
      i_ADCVoltageFilt = i_ADCVoltage;
      i_ADCLoadVoltageFilt = i_ADCLoadVoltage;
      i_ADCSenseVoltageFilt = i_ADCSenseVoltage;
      i_ADCCurrentRemainder = 0;
      i_ADCVoltageRemainder = 0;
      i_ADCLoadVoltageRemainder = 0;
      i_ADCSenseVoltageRemainder = 0;
      i_FilterMultiplierToUse = INIT_FILTER_MULTIPLIER_MINUS_ONE;
      i_FilterShiftsToUse = INIT_FILTER_SHIFTS;

  }
  else if (ucv_FilterState == 1) // not first
  {
	  // run Izero filter
	  RightShiftFilter(&i_CurrentZero, &i_CurrentRemainder, i_ADCCurrent, ((int32_t)ZERO_FILTER_MULTIPLIER_MINUS_ONE), ((int32_t)ZERO_FILTER_SHIFTS));

	  // run AZero filter
	  RightShiftFilter(&i_AZero, &i_ActiveRemainder, i_ADCVoltage, ((int32_t)ZERO_FILTER_MULTIPLIER_MINUS_ONE), ((int32_t)ZERO_FILTER_SHIFTS));

	  // run SAzero filter
	  RightShiftFilter(&i_SAZero, &i_SwitchedActiveRemainder, i_ADCLoadVoltage, ((int32_t)ZERO_FILTER_MULTIPLIER_MINUS_ONE), ((int32_t)ZERO_FILTER_SHIFTS));

	  // we remove the zeros offsets that were derived from the raw adc readings,
	  // but now we used the adjusted adc readings - the ones without offset in them - to come up with filtered version of them
	  // adjust current readings
      i_ADCCurrent = i_ADCCurrent - i_CurrentZero;
      // adjust v reading
      i_ADCVoltage = i_ADCVoltage - i_AZero;
      // adjust sa reading
      i_ADCLoadVoltage = i_ADCLoadVoltage - i_SAZero;

	  // run I filter
      RightShiftFilter(&i_ADCCurrentFilt, &i_ADCCurrentRemainder, i_ADCCurrent, i_FilterMultiplierToUse, i_FilterShiftsToUse);

      // run A filter
	  RightShiftFilter(&i_ADCVoltageFilt, &i_ADCVoltageRemainder, i_ADCVoltage, i_FilterMultiplierToUse, i_FilterShiftsToUse);

      // run SA filter
	  RightShiftFilter(&i_ADCLoadVoltageFilt, &i_ADCLoadVoltageRemainder, i_ADCLoadVoltage, i_FilterMultiplierToUse, i_FilterShiftsToUse);

      // run Sense V filter
	  RightShiftFilter(&i_ADCSenseVoltageFilt, &i_ADCSenseVoltageRemainder, i_ADCSenseVoltage, i_FilterMultiplierToUse, i_FilterShiftsToUse);
  }


  /*   The State Machine  */
  /* so far implemented is leading edge switching */
  /* leading edge switching not fully implemented yet */
  /* to do leading edge properly, we should check the load current before turning off at the start of a cycle *******/
  if (ucv_Stage == ((uint8_t)2)) // first stage just looking for the first true neg to pos transition
  {
	  if (ucv_AState == ((uint8_t)1)) // mains was positive
	  {
		  /* test what state ********/
		  // LL_GPIO_SetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
          if (i_ADCVoltageFilt > 0) // active still positive
          {
               // do nothing
          }
          else // active changed to negative
          {
               ucv_AState = ((uint8_t)0);
          }
	  }
	  else // mains was negative
	  {
          if (i_ADCVoltageFilt > 0) // active changed to positive
          {
    		   /* test what state ********/
    		   // LL_GPIO_SetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
               ucv_AState = 1;
               ucv_Stage = 1; // progress to next stage
               ucv_DeadTime = 1;
               s_Period_Counter_ST10 = ((int16_t)10); // the sample interval that just started is the first
               s_Half_Period_Counter_ST = ((int16_t)1); // the first interval of the half cycle
               // don't need to count half cycles yet
          }
          else // active still negative
          {
               // do nothing
    	       /* test what state ********/
    	       // LL_GPIO_ResetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
          }
	  }
  }
  else if (ucv_Stage == ((uint8_t)1)) // first rising mains edge detected, now doing frequency selection
  {
	  if (ucv_AState == ((uint8_t)1)) // mains was positive
	  {
		  if (ucv_DeadTime == 1) // we are in the dead time
		  {
			  // increment counter
        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
	          if (s_Half_Period_Counter_ST >= ((int16_t) DEAD_BAND_ST)) // not in dead band any more
	          {
	        	  ucv_DeadTime = 0;
	          }
	          else // in dead band
	          {
	        	  // do nothing
	          }
		  }
		  else // not in dead time, can look for another edge
		  {
	          if (i_ADCVoltageFilt > 0) // active still positive
              {
	              // increment counter
	        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
              }
              else // active changed to negative
              {
        		  /* test what state ********/
        		  // LL_GPIO_ResetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
                  ucv_AState = ((uint8_t)0);
                  ucv_DeadTime = 1; // new dead time period
            	  s_Half_Period_Counter_ST = 1;
              }
	      }
    	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
	  }
	  else // mains was negative
	  {
		  if (ucv_DeadTime == 1) // we are in the dead time
		  {
			  // increment counters
        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
        	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
	          if (s_Half_Period_Counter_ST >= ((int16_t) DEAD_BAND_ST)) // not in dead band any more
	          {
	        	  ucv_DeadTime = 0;
	          }
		  }
		  else // not in dead time, can look for another edge
		  {
              if (i_ADCVoltageFilt > 0) // active changed to positive, uh oh, stage change
              {
        		  /* test what state ********/
        		  // LL_GPIO_SetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
            	  // end of stage 1, we now know the frequency
            	  // we are in a new period
            	  // how many sample ingtervals did we count in previous period
            	  if (s_Period_Counter_ST10 > ((int16_t)FIFTY_SIXTY_HZ_THRESHOLD_ST10)) // if 50Hz detected
            	  {
            		  s_Nominal_Half_Period_ST = ((int16_t) FIFTY_HZ_COUNTS_ST);
            		  sv_Duty_ST = ((int16_t) FIFTY_HZ_DEFAULT_DUTY); // set duty
            	  }
            	  else // 60Hz detected
            	  {
            		  s_Nominal_Half_Period_ST = ((int16_t) SIXTY_HZ_COUNTS_ST);
            		  sv_Duty_ST = ((int16_t) SIXTY_HZ_DEFAULT_DUTY); // set duty
            	  }
            	  // initialize the filtered period counter variable, i_Period_st10 is just for reporting, we don't use it in the state machine.
            	  i_Period_st10 = ((int32_t)s_Period_Counter_ST10);
            	  s_Period_Counter_ST10 = ((int16_t)10);
            	  s_Half_Period_Counter_ST = ((uint16_t)1);
                  ucv_AState = ((uint8_t)1);
                  ucv_Stage = 0; // progress to next stage
                  ucv_DeadTime = 1;
                  // turn the switch on or off, only do this here because we are actually already in the next stage
                  OutputProcessing();
                  // we had the mains filters set to a high filter value to get a reliable frequency
                  // but now that we have frequency, we need to dial back the filter to favour speed instead
                  // the active high low state will be out for a bit but it will correct its self within about 1s.
                  i_FilterMultiplierToUse = RUN_FILTER_MULTIPLIER_MINUS_ONE;
                  i_FilterShiftsToUse = RUN_FILTER_SHIFTS;
              }
              else // active still negative, no stage change
              {
            	  // increment counters
	         	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
	        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
              }
	      }
	  }
  }
  else // normal run mode
  {
	  if (ucv_AState == ((uint8_t)1)) // mains was positive
	  {
		  // increment counter to current one always because both positive and negative half cycle are part of cycle
    	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
		  if (ucv_DeadTime == 1) // we are in the dead time
		  {
              // increment counter to the current one
        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
			  // if not in dead band any more, note dead band extends for almost the entire half cycle
	          if (s_Half_Period_Counter_ST >= (s_Nominal_Half_Period_ST-((int16_t)MAX_PERIOD_ADJUSTMENT_ST)))
	          {
	        	  ucv_DeadTime = 0;
	          }
		  }
		  else // not in dead time, can look for another edge, and we may have gone beyond Nominal Half Period length
		  {
    	      if (i_ADCVoltageFilt > 0) // active still positive, still in positive half period
              {
    	    	  // check for max number of sample intervals
    	    	  // if half cycle extends beyond adjustment range, we want to force a stage change
    	    	  if (s_Half_Period_Counter_ST >= (s_Nominal_Half_Period_ST+((int16_t)MAX_PERIOD_ADJUSTMENT_ST))) // we have gone beyond max permissible half period count
	        	  {
    	    		  /* test what state ********/
    	    		  // LL_GPIO_ResetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
	        		  // we have gone beyond max half period counts, force new half cycle
    	        	  s_Half_Period_Counter_ST = ((uint16_t)1);
    	        	  ucv_AState = 0;
    	        	  ucv_DeadTime = 1;
	        	  }
	        	  else
	        	  {
	        		  // not last, keep counting
	                  // increment to current sample interval
	    	    	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + ((uint16_t)1);
    	    	  }
              }
              else // active changed to negative, so that's the end of the half period, we are now in the next half period
              {
        		  /* test what state ********/
        		  // LL_GPIO_ResetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
	        	  s_Half_Period_Counter_ST = ((uint16_t)1);
	        	  ucv_AState = 0;
	        	  ucv_DeadTime = 1;
              }
	      }
	  }
	  else // mains was negative
	  {
		  if (ucv_DeadTime == 1) // we are in the dead time
		  {
			  // increment counters
        	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + 1;
        	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
        	  // if not in dead band any more, note dead band extends for almost the entire half cycle
			  if (s_Half_Period_Counter_ST >= (s_Nominal_Half_Period_ST-((int16_t)MAX_PERIOD_ADJUSTMENT_ST))) // not in dead band any more
	          {
	        	  ucv_DeadTime = 0;
	          }
		  }
		  else // not in dead time, can look for another edge, and we may have gone beyond Nominal Half Period length
		  {
              if (i_ADCVoltageFilt > 0) // active changed to positive
              {
        		  /* test what state ********/
        		  // LL_GPIO_SetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
            	  // run the period filter *******
            	  i_Period_st10 = ((int32_t)s_Period_Counter_ST10);
            	  // start of a new cycle
    	    	  s_Half_Period_Counter_ST = ((uint16_t)1);
	        	  s_Period_Counter_ST10 = ((int16_t)10);
	        	  ucv_AState = 1;
	        	  ucv_DeadTime = 1;
              }
              else // active still negative, still in negative half period
              {
    	    	  // check for max number of sample intervals
    	    	  // if half cycle extends beyond adjustment range, we want to force a stage change
    	    	  if (s_Half_Period_Counter_ST >= (s_Nominal_Half_Period_ST+((int16_t)MAX_PERIOD_ADJUSTMENT_ST))) // we have gone beyond max permissible half period count
	        	  {
    	    		  /* test what state ********/
    	    		  // LL_GPIO_SetOutputPin(Opto_Output_GPIO_Port, Opto_Output_Pin);
	        		  // we have gone beyond max half period counts, force new half cycle
                	  // run the period filter *******
                	  i_Period_st10 = ((int32_t)s_Period_Counter_ST10);
                	  // start of a new cycle
        	    	  s_Half_Period_Counter_ST = ((uint16_t)1);
    	        	  s_Period_Counter_ST10 = ((int16_t)10);
    	        	  ucv_AState = 1;
    	        	  ucv_DeadTime = 1;
	        	  }
	        	  else
	        	  {
	        		  // not last, keep counting
	                  // increment to current sample interval
	    	    	  s_Half_Period_Counter_ST = s_Half_Period_Counter_ST + ((uint16_t)1);
	            	  s_Period_Counter_ST10 = s_Period_Counter_ST10 + ((int16_t)10);
    	    	  }
              }
		  }
	  }
	  /* check whether we need to turn on */
	  OutputProcessing();
  }

  // print logic is as follows:
  // in this ISR routine do this:
  // at start up print state is set to WAIT and count is set to END_PRINT_CYCLES
  // count is incremented every sample interval
  // when count gets to END_PRINT_CYCLES value, count is reset to zero and the print state is set to BUFFER
  // the print buffer is a printable string, it is not an array of values
  // when the print state is set to BUFFER, a new value is added to the print buffer and we check to see if count has reached END_BUFFER_CYCLE
  // when count reaches END_BUFFER_CYCLES, print state is changed to PRINT
  // in the main routine do this:
  // if print state is set to PRINT, print the print buffer and then set the print state to WAIT
  // notes:
  // count is continually counting sample intervals, if we want to accumulate 256 sample values of a variable, it will take 256 sample intervals
  // however, it will take time after that to print those values if we don't want to double buffer them
  // consequently ENT_PRINT_CYCLES has to allow enough time to accumulate the samples and print them to avoid over running the print buffer before printing has finished
  // this routine outputs a sequence of any nominated variable that changes from sample to sample
  // it does not control when in the mains cycle to start saving values and it does not save a continuous stream of values
  // but the values in each sample run are continuous
  // we want to use this to see what the waveforms look like for each ADC channel

  /* buffer values to print */
  // count
  usv_Count = usv_Count + ((uint16_t)1);
  // if count = big value, set count to 0, set state to buffer, reset len
  if (usv_Count == ((uint16_t) END_PRINT_CYCLES_ST)) // end of long count
  {
	  usv_Count = 0;
	  sizev_len = 0;
	  ucv_adcDmaTransferStatus = ((uint8_t)3); // 3 = buffer, 2 = print complete or wait, 1 = full transfer or print, 0 = half transfer
  }
  // if state is buffer
  //     add value to print buffer
  //     if count = small value, set state to print
  if (ucv_adcDmaTransferStatus == ((uint8_t)3)) // if buffering
  {
      // add a value to the print buffer
	  sv_ValueBuffer1[usv_Count] = ((int16_t)i_ADCCurrent); // active
	  sv_ValueBuffer2[usv_Count] = ((int16_t)i_ADCCurrentFilt); // neutral
	  if (usv_Count == (((uint16_t) END_BUFFER_CYCLES_ST) - ((uint16_t)1))) // if buffering finished
	  {
		  ucv_adcDmaTransferStatus = ((uint8_t)1); // finished buffering, set state to print
	  }
  }

  /* send string to printer */
  //if (usv_PrintCounter == ((uint8_t) PRINT_PERIOD_ST)) // if want to print this time
  //{
  //  svp_ADCCurrent = i_ADCCurrent;
  //  svp_ADCVoltage = i_ADCVoltage;
  //  svp_ADCLoadVoltage = i_ADCLoadVoltage;
  //  svp_ADCSenseVoltage = i_ADCSenseVoltage;
  //  usvp_ADCTemperatureReading = i_ADCTemperatureReading;
  //  usvp_ADCVrefintReading = i_ADCVrefintReading;
  //  ucv_adcDmaTransferStatus = 1; // 0 is half transfer, 1 is full transfer, 2 print complete
  //  usv_PrintCounter = 0;
  //  usvp_Period_Counter_ST10 = ((uint16_t)i_Period_st10);
  //}
  //else
  //{
  //  usv_PrintCounter = usv_PrintCounter + 1;
  //}

}

// this implements the Y(n)=(1-alpha)Y(n-1)+alphaX(n) single pole filter
// where alpha is a power of 2, thus avoiding having to do a divide
// we need to keep track of remainders to avoid the shift to a smaller magnitude bias that the right shift would otherwise introduce
void RightShiftFilter(int32_t *i_FilterObject, int32_t *i_RemainderObject, int32_t i_NewValue, int32_t i_FilterMultiplierMinusOne, int32_t i_Shift)
{
	int32_t i_RSFTemp1;
	int32_t i_RSFTemp2;
	int32_t i_Sign;
	int32_t i_Magnitude;

	// multiply-accumulate
	i_RSFTemp1 = (i_FilterMultiplierMinusOne * *i_FilterObject) + i_NewValue + *i_RemainderObject;
	// save sign
    i_Sign = (i_RSFTemp1 >= 0) ? 1 : -1;
	// save absolute value
	i_Magnitude = (i_RSFTemp1 < 0) ? -i_RSFTemp1 : i_RSFTemp1;
	// divide
	i_RSFTemp1 = i_Magnitude >> i_Shift;
	// get new remainder
	i_RSFTemp2 = i_Magnitude & ((1 << i_Shift) - 1); // picks out the low bits of Magnitude
	// restore sign
	*i_FilterObject = i_Sign * i_RSFTemp1;
    // restore remainder sign
	*i_RemainderObject = i_Sign * i_RSFTemp2;
}

void OutputProcessing(void)
{
	// when this routine is called the half period counter should have the number of the sample interval that we are about to enter
	// when it is 1 we are about to enter the first sample interval of the current half cycle
	// if it is 1 it means that we have just detected the polarity of the mains has changed state
	int16_t s_OPTemp;

    // clamp the half period counter
    if (s_Half_Period_Counter_ST > s_Nominal_Half_Period_ST)
    {
    	 s_OPTemp = s_Nominal_Half_Period_ST;
    }
    else
    {
    	 s_OPTemp = s_Half_Period_Counter_ST;
    }
    if (ucv_LeadingEdge == 1) // Leading edge. with leading edge turn on any time but only turn off after end of half cycle after current goes to zero.
    	                      // So we are permitted to turn off if the switch was only on because current was lingering from last cycle.
    	                      // And we can not turn off if the next on part of the cycle has started.
    	                      // If these two overlap then we wouldn't be able to tell why we are on because the current signal is very noisy.
    	                      // Consequently we need to make a decision at the start of the cycle whether this cycle is an on or an off cycle.
    {
    	if (s_OPTemp == 1) // at zero crossing
    	{
    	    if (ucv_switchStateRequest == ((uint8_t)1)) // if commanded on
    	    {
    	    	ucv_SwitchCycleStateRequest = 1;
    	    }
    	    else
    	    {
    	    	ucv_SwitchCycleStateRequest = 0;
    	    }
    	}
    	else // not at zero crossing
    	{
    		// cant change ucv_SwitchCycleStateRequest
    	}
        if (ucv_SwitchCycleStateRequest == 1) // if current cycle is a requested on cycle
	    {
        	if (ucv_switchState == 1) // if it is on
        	{
        	    if (s_OPTemp > (s_Nominal_Half_Period_ST - sv_Duty_ST)) // if it should be on in latter duty part of cycle
        	    {
	                // do nothing
        	    }
        	    else // it should be off leading (1-duty) part of cycle turn it off when we can
        	    {
                    if (ucv_AState == 1) // if in positive half cycle
                    {
                        if (i_ADCCurrentFilt > 0) // if current is greater than zero
                        {
                            // turn off
                            ucv_switchState = ((uint8_t)0);
	       	                LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
                        }
                        else // else current is not greater than zero
                        {
                            // do nothing
                        }
                    }
                    else // else in negative half cycle
                    {
                        if (i_ADCCurrentFilt < 0) // if current is less than zero
                        {
                            // turn off
                            ucv_switchState = ((uint8_t)0);
	      	                LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
                        }
                        else // else current is not less than zero
                    	{
                            // do nothing
                    	}
                    }
        	    }
        	}
        	else // it is off
        	{
        	    if (s_OPTemp > (s_Nominal_Half_Period_ST - sv_Duty_ST)) // if it should be on in latter duty part of cycle
        	    {
                    // turn it on
                    ucv_switchState = ((uint8_t)1);
                    LL_GPIO_SetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
        	    }
        	    else // else it should be off in leading (1-duty) part of cycle
        	    {
                    // do nothing
        	    }
        	}
	    }
	    else // if current cycle is a requested off cycle
        {
	        if (ucv_switchState == 1) // if it is on turn it off when we can
	    	{
      	    	// it doesn't matter whether we are in the duty or (1-duty) part of the cycle,
      	    	// if this is a requested off cycle, then we need to turn it off when we can.
	   		    if (ucv_AState == 1) // if in positive half cycle
	   		    {
	   	    		if (i_ADCCurrentFilt > 0) // if current is greater than zero
	   	    		{
                        // turn off
                        ucv_switchState = ((uint8_t)0);
                        LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
	  	    		}
	   	    		else // else current is not greater than zero
	   	    		{
                        // do nothing
	   	    		}
	   	    	}
	   	    	else // else in negative half cycle
	   	    	{
	   		       	if (i_ADCCurrentFilt < 0) // if current is less than zero
	   		     	{
                        // turn off
                        ucv_switchState = ((uint8_t)0);
   	  	                LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
	 	    		}
	   	    		else // else current is not less than zero
	   	    		{
                        // do nothing
	   	    		}
	   	    	}
   	    	}
	    	else // it is off
	    	{
   	      	    // do nothing
	    	}
        }
    }
    else // trailing edge. with trailing edge turn off any time but only turn on at start of half cycle
    {
    	// the below if about zero crossing and setting ucv_SwitchCycleStateRequest isn't needed or used in the trailing edge case
    	// but we have added it here so that it is in the correct state.
    	if (s_OPTemp == 1) // at zero crossing
    	{
    	    if (ucv_switchStateRequest == ((uint8_t)1)) // if commanded on
    	    {
    	    	ucv_SwitchCycleStateRequest = 1;
    	    }
    	    else
    	    {
    	    	ucv_SwitchCycleStateRequest = 0;
    	    }
    	}
    	else // not at zero crossing
    	{
    		// cant change ucv_SwitchCycleStateRequest
    	}
 	    if (ucv_switchStateRequest == ((uint8_t)1)) // if commanded on
        {
		    if (ucv_switchState == 1) // if it is on
		    {
	            if (s_OPTemp > sv_Duty_ST) // if it should be off in latter (1-duty) part of cycle
	            {
	                // turn off
                    ucv_switchState = ((uint8_t)0);
                    LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
	            }
	            else // else it should be on in leading duty part of cycle
	            {
                    // do nothing, it should be on
	            }
		    }
		    else // else it is off turn on when we can
		    {
	            if (s_OPTemp > sv_Duty_ST) // if it should be off in in latter (1-duty) part of cycle
	            {
                    // do nothing, it should be off
	            }
	            else // else it should be on in leading duty part of cycle
	            {
	        	    if (s_OPTemp == 1) // at zero crossing
	        	    {
                        // turn it on
                        ucv_switchState = ((uint8_t)1);
                        LL_GPIO_SetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
	        	    }
	        	    else // not at zero crossing
	        	    {
                        // do nothing
	        	    }
	            }
		    }
	    }
	    else // else commanded off
	    {
		    if (ucv_switchState == 1) // if it is on
		    {
	            // turn off
                ucv_switchState = ((uint8_t)0);
                LL_GPIO_ResetOutputPin(Light_Output_GPIO_Port, Light_Output_Pin);
		    }
		    else // else it is off
		    {
                // do nothing
		    }
	    }
    }
}


//void AdcDmaTransferHalf_Callback() //called from DMA ISR
//{
//  ucv_adcDmaTransferStatus = 0;
//}

void AdcDmaTransferError_Callback(void) //called from ISR
{
}

void TIM_Base_Start(void)
{
  LL_TIM_EnableCounter(TIM3);
}

void USART1_Write(const char *s)
{
  while (*s)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART1))
    {
    }
    LL_USART_TransmitData8(USART1, (uint8_t)*s++);
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
