/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"
#include "sensor_driver.h"
#include "lora_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LPP_DATATYPE_HUMIDITY 0x68
#define LPP_DATATYPE_TEMPERATURE 0x67
#define LPP_DATATYPE_BAROMETER 0x73
#define LPP_DATATYPE_ADC 0x74
#define LPP_DATATYPE_PULSE 0x75

#define LORAWAN_APP_PORT           99;            /*LoRaWAN application port*/
#define LORAWAN_CONFIRMED_MSG      ENABLE         /*LoRaWAN confirmed messages*/

#define JOIN_MODE                  OTAA_JOIN_MODE   /*ABP_JOIN_MODE */ /*LoRaWan join methode*/

#define PULSE_COUNTER_COUNT 		1
#define PULSE_COEF                  4
#define PULSE_MINUTE                60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t pulse[PULSE_COUNTER_COUNT];
uint16_t calibrated_pulse[PULSE_COUNTER_COUNT];

TIM_HandleTypeDef *hPulseCycleTimer; //1sn
TIM_HandleTypeDef *hPulseCounter[PULSE_COUNTER_COUNT];
TIM_HandleTypeDef *hCollectCycleTimer; //5sn

static LoRaDriverParam_t LoRaDriverParam = {0, JOIN_MODE};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void PrepareSensorData(sSendDataBinary_t *SendDataBinary);
static LoRaDriverCallback_t LoRaDriverCallbacks = { PrepareSensorData, NULL };
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define min(a,b) (a) < (b) ? (a) : (b)
#define max(a,b) (a) > (b) ? (a) : (b)
#define abs(a) ((a) > 0 ? (a) : -(a))
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
  MX_LPUART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&hlpuart1);

  hmodem_uart = &huart1;

  hSendCycleTimer = &htim17; //15sn
  hPulseCycleTimer = &htim7; //1sn
  hPulseCounter[0] = &htim3;
  hCollectCycleTimer = &htim16; //5sn
  hProtectCycleTimer = &htim6; //18sn
  hADC1Timer = &htim1;
  hADC2Timer = &htim2;

  /* Context Initialization following the LoRa device modem Used*/
  Lora_Ctx_Init(&LoRaDriverCallbacks, &LoRaDriverParam);

  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  if (SensorDevicesInit() != COMPONENT_OK) {
	  printf("Sensors can not initialized! \r\n");

  } else {
	  printf("Sensors initialized! \r\n");
  }

  if (AnalogDevicesInit() != COMPONENT_OK) {
	  printf("ADCs can not initialized! \r\n");

  } else {
	  printf("ADCs initialized! \r\n");
  }

  //TIMERS
  start_timer(hPulseCycleTimer); //1sn timer
  start_timer(hCollectCycleTimer); //5sn timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("Starting \r\n");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Lora_fsm();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	BSP_LED_Toggle(LED_GREEN);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

	if( GPIO_Pin == LSM303AGR_INT_Pin )
	{
	  AXL_INT_received = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == hPulseCycleTimer) {
		
		for (int i = 0; i < PULSE_COUNTER_COUNT; i++) {
			HAL_TIM_Base_Stop(hPulseCounter[i]);
			pulse[i] = __HAL_TIM_GetCounter(hPulseCounter[i]);
			HAL_TIM_GenerateEvent(hPulseCounter[i], TIM_EVENTSOURCE_UPDATE);
			HAL_TIM_Base_Start(hPulseCounter[i]);
			//calibrated_pulse[i] = pulse[i] * (PULSE_MINUTE / PULSE_COEF);
			calibrated_pulse[i] = 0x1250;
		}

		return;
	}

	if (htim == hCollectCycleTimer) {
		
		if (CollectSensorData() != COMPONENT_OK) {
			printf("Can not get sensor data! \r\n");

		} else {
			printf("Collected sensor data :  temperature = %d, pressure = %d, humidity = %d \r\n",
					temperature, pressure, humidity);
		}

		if (CollectAnalogData() != COMPONENT_OK) {
			printf("Can not get analog data! \r\n");

		} else {

			printf("Collected analog data :  adc1[0] = %d, adc1[1] = %d, adc2[0] = %d \r\n",
			processed_adc1[0], processed_adc1[1], processed_adc2[0] );
		}
		return;
	} 

	if (htim == hSendCycleTimer) {
		
		Lora_OnNextSensorMeasureTimerEvt(NULL);

		return;
	}

	if (htim == hProtectCycleTimer) {
		
	//	Lora_OnJoinProtectTimerEvt(NULL); //disabled for now, should be reenabled later
	}
}

static void PrepareSensorData(sSendDataBinary_t *SendDataBinary)
{
	int index = 0;

//	SendDataBinary->Buffer[index++] = cchannel++;
/*	SendDataBinary->Buffer[index++] = LPP_DATATYPE_BAROMETER;
	SendDataBinary->Buffer[index++] = (pressure >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = pressure & 0xFF;
	SendDataBinary->Buffer[index++] = cchannel++;
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_TEMPERATURE;
	SendDataBinary->Buffer[index++] = (temperature >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = temperature & 0xFF;
	SendDataBinary->Buffer[index++] = cchannel++;
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_HUMIDITY;
	SendDataBinary->Buffer[index++] = humidity & 0xFF;
*/
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_ADC;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[0] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[1] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[1] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[2] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[2] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[3] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[3] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[4] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[4] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[5] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[5] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[0] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[1] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[1] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[2] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[2] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[3] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[3] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[4] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[4] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[5] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[5] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[6] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[6] & 0xFF;

	SendDataBinary->Buffer[index++] = LPP_DATATYPE_PULSE;
	SendDataBinary->Buffer[index++] = (calibrated_pulse[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_pulse[0] & 0xFF;


	SendDataBinary->DataSize = index;
	SendDataBinary->Port = LORAWAN_APP_PORT;
	SendDataBinary->Ack = !LORAWAN_CONFIRMED_MSG;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
