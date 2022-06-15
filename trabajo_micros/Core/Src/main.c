/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Luces.h"
#include "Toldo.h"

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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Interrupciones botones:
volatile int button_int = 0;
int boton_presionado = 0;
//Antirrebotes botones:
int debouncer(volatile int *button_int, GPIO_TypeDef *GPIO_port,
		uint16_t GPIO_number) {
	static uint8_t button_count = 0;
	static int counter = 0;

	if (*button_int == 1) {
		if (button_count == 0) {
			counter = HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick() - counter >= 20) {
			counter = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number) != 1) {
				button_count = 1;
			} else {
				button_count++;
			}
			if (button_count == 4) { //Periodo antirebotes
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}
//todos los pulsadores y finales de carrera
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2) { 			//2 3 4 corresponden a las luces
		button_int = 1;
		boton_presionado = 2;//OFF
	} else if (GPIO_Pin == GPIO_PIN_3) {
		button_int = 1;
		boton_presionado = 3;//ON
	} else if (GPIO_Pin == GPIO_PIN_4) {
		button_int = 1;
		boton_presionado = 4;//AUTO
	} else if (GPIO_Pin == GPIO_PIN_5) { 	//5 6 7 8 9 corresponden al toldo
		button_int = 1;
		boton_presionado = 5;//OFF
	} else if (GPIO_Pin == GPIO_PIN_6) {
		button_int = 1;
		boton_presionado = 6;//SUBIENDO
	} else if (GPIO_Pin == GPIO_PIN_7) {
		button_int = 1;
		boton_presionado = 7;//BAJANDO
	} else if (GPIO_Pin == GPIO_PIN_8) {
		button_int = 1;
		boton_presionado = 8;//FIN DE CARRERA ARRIBA
	} else if (GPIO_Pin == GPIO_PIN_9) {
		button_int = 1;
		boton_presionado = 9;//FIN DE CARRERA ABAJO
	} else if (GPIO_Pin == GPIO_PIN_10) { 	//10 11 12 corresponden al humidificador
		button_int = 1;
		boton_presionado = 10;//OFF
	} else if (GPIO_Pin == GPIO_PIN_11) {
		button_int = 1;
		boton_presionado = 11;//ON
	} else if (GPIO_Pin == GPIO_PIN_12) {
		button_int = 1;
		boton_presionado = 12;//AUTO
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */



  setLuces(0);
  cambiarEstadoToldo(0);
  HAL_TIM_Base_Start(&htim6);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (boton_presionado) {
	  	  		case 2:		//luces off
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_2)) {
	  	  				setLuces(0);
	  	  			}
	  	  			break;
	  	  		case 3:		//luces on
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_3)) {
	  	  				setLuces(1);
	  	  			}
	  	  			break;
	  	  		case 4:		//luces auto
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_4)) {
	  	  				setLuces(2);
	  	  			}
	  	  			break;
	  	  		case 5:		//toldo stop
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_5))  {
	  	  				cambiarEstadoToldo(0);
	  	  			}
	  	  			break;
	  	  		case 6:		//toldo subir
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_6)) {
	  	  				cambiarEstadoToldo(1);
	  	  			}
	  	  			break;
	  	  		case 7:		//toldo bajar
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_7)) {
	  	  				cambiarEstadoToldo(2);
	  	  			}
	  	  			break;
	  	  		case 8:		//toldo fin carrera arriba
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_8)) {
	  	  			  	cambiarEstadoToldo(3);
	  	  			}
	  	  			break;
	  	  		case 9:		//toldo fin carrera abajo
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_9)) {
	  	  			  	cambiarEstadoToldo(4);
	  	  			}
	  	  			break;
	  	  		case 7:		//toldo bajar
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_7)) {
	  	  			  	cambiarEstadoToldo(2);
	  	  			}
	  	  			break;
	  	  		case 8:		//toldo fin carrera arriba
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_8)) {
	  	  			  	cambiarEstadoToldo(3);
	  	  			}
	  	  			break;
	  	  		case 9:		//toldo fin carrera abajo
	  	  			if (debouncer(&button_int, GPIOA, GPIO_PIN_9)) {
	  	  			  	cambiarEstadoToldo(4);
	  	  			}
	  	  			break;
	  	  }
	  	  luces();
	  	  Humidificador();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
