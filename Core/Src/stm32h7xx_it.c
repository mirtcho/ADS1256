/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "stdint.h"
#include "stdbool.h"
#include "ads1256.h"
#include "stdio.h"

extern volatile bool drv_ready;
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
/* Vref=2,4891V    Gain =2*Vref/2^23  */
//#define ADC_GAIN 0.00000059173107147216796875
/* back calcl vfrom my Fluke183 Vref=2.48217V */
//#define ADC_GAIN 0.00000059179544449
/*used caclulated for Vref=2.48217 but corected to fluke sebastian */
#define ADC_GAIN 0.000000594783

#define V_SAMPLE_MIN 0.01
#define V_SAMPLE_MAX 5.00

/* ToDo clean up this section in one AnIn. structure */
static uint32_t adc_data;
double v_lpf=2.048645;  /*2.0474V measured with fluke tony */
volatile double v_min=3.0f;
volatile double v_max=1.0f;
volatile double v_pp=0.0f;
volatile double v_sample;
int16_t start_counter=0;
extern bool ads_initialized;
/* sample buffer data */
uint32_t skipped_samples=0;
int log_index=0;

/* Temperature vars */
double t_adc_ref_lpf=20.0;

double t_ext_ref_lpf=20.0;

static void analog_input_measurements()
{
	if ((v_sample>V_SAMPLE_MIN) &&(v_sample<V_SAMPLE_MAX))
	{
		  if (start_counter < 500)
		  {
			  /* LPF filter startup behavior */
			  start_counter++;
			  v_lpf+= 0.1*(v_sample-v_lpf);
		  }
		  else
		  {
			  v_lpf+= 0.001*(v_sample-v_lpf);
			  /* update statistics min/max */
			  if (v_lpf>v_max)
			  {
				  v_max=v_lpf;
				  v_pp=v_max-v_min;
			  }
			  if (v_lpf<v_min)
			  {
				  v_min=v_lpf;
				  v_pp=v_max-v_min;
			  }
			  /* fill sample buffer sample rate is 100s/sec -> 1sample/20sec */
			  if (skipped_samples++>=1000) //10sec
			  {
				  skipped_samples=0;
				  log_index++;
				  printf("Tadc= %f  Text_ref= %f   V[%d]= %lf \n\r",t_adc_ref_lpf,t_ext_ref_lpf,log_index,v_lpf);
			  }
		  }
	}
}

static void adc_temperature_measurements(float t_sample)
{
	t_adc_ref_lpf+= 0.1*(t_sample-t_adc_ref_lpf);
}

static void ext_temperature_measurements(float t_sample)
{
	t_ext_ref_lpf+= 0.1*(t_sample-t_ext_ref_lpf);
}


void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  drv_ready=true;
  if (ads_initialized)
  {
	  adc_data = ads_read_data();
	  adc_data = 0x1000000-adc_data; //invert sign to keep voltage positive

	  v_sample= ((int32_t)adc_data)*ADC_GAIN;
	  switch (ads_get_selected_adc_channel())
	  {
	  case ANALOG:
		  analog_input_measurements();
		  break;
	  case ADC_REF:
		  adc_temperature_measurements(get_ref_temp(ADC_REF,v_sample));
		  break;
	  case EXT_REF:
		  ext_temperature_measurements(get_ref_temp(EXT_REF,v_sample));
		  break;
	  case LAST_CHANNEL:
	  default:
		  break;
	  }
  }
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
