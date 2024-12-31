/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static const uint8_t HIH7000_ADDRESS = 0x27 << 1;
static const uint8_t MICS_ADDRESS = 0b1110000 << 1;
static const uint8_t BMP_ADDRESS = 0x76 << 1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
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
	HAL_StatusTypeDef ret;
	uint8_t buf_data_temperatuur[4];
	uint8_t buf_string[200];
	uint8_t buf_transmit[3];
	int16_t val_temperatuur[2];
	float temp = 0;
	float hum = 0;
	int16_t temp_int = 0;
	uint16_t hum_int = 0;
	uint16_t lux_raw = 0;
	float lux = 0;

	uint8_t buf_data[3];
	uint8_t CO2_raw;
	uint8_t VOC_raw;
	float CO2;
	float VOC;

	uint8_t buf_transmit_pres[6];
	uint8_t buf_data_pres[6];

	uint8_t buf_NVM[30];

	uint32_t pres_raw;
	float pres;

	uint32_t temp_raw;

	float partial_data1_temp;
	float partial_data2_temp;

	float partial_data1_pres;
	float partial_data2_pres;
	float partial_data3_pres;
	float partial_data4_pres;
	float partial_out1;
	float partial_out2;

	uint16_t NVM_PAR_T1;
	float PAR_T1;

	uint16_t NVM_PAR_T2;
	float PAR_T2;

	int8_t NVM_PAR_T3;
	float PAR_T3;

	int16_t NVM_PAR_P1;
	float PAR_P1;

	int16_t NVM_PAR_P2;
	float PAR_P2;

	int8_t NVM_PAR_P3;
	float PAR_P3;

	int8_t NVM_PAR_P4;
	float PAR_P4;

	uint16_t NVM_PAR_P5;
	float PAR_P5;

	uint16_t NVM_PAR_P6;
	float PAR_P6;

	int8_t NVM_PAR_P7;
	float PAR_P7;

	int8_t NVM_PAR_P8;
	float PAR_P8;

	int16_t NVM_PAR_P9;
	float PAR_P9;

	int8_t NVM_PAR_P10;
	float PAR_P10;

	int8_t NVM_PAR_P11;
	float PAR_P11;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//
	// Lichtsensor
	//

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 20);
	lux_raw = HAL_ADC_GetValue(&hadc1);
	lux = (float)lux_raw / 4.095;


	// Luchtvochtigheid en temperatuursensor


	buf_transmit[0] = 0x00;
	buf_data_temperatuur[0] = 0;
	buf_data_temperatuur[1] = 0;
	buf_data_temperatuur[2] = 0;
	buf_data_temperatuur[3] = 0;

	val_temperatuur[0] = 0;
	val_temperatuur[1] = 0;

	// Stuur code naar sensor zodat deze weet wat er verwacht wordt
	ret = HAL_I2C_Master_Transmit(&hi2c1, HIH7000_ADDRESS, buf_transmit, 1, HAL_MAX_DELAY);

	// Wacht 250ms anders heeft HAL_Transmit niet genoeg tijd
	HAL_Delay(250);

	// Geen HAL_OK ontvangen = fout
	if (ret != HAL_OK) {
		strcpy((char*)buf_string, "Error met tempertuursensor Tx\r\n");
	} else {
		// Ontvang de vochtigheid en de temperatuur en sla deze op in de buffer
		ret = HAL_I2C_Master_Receive(&hi2c1, HIH7000_ADDRESS, buf_data_temperatuur, 4, HAL_MAX_DELAY);

		// Geen HAL_OK ontvangen = fout
		if (ret != HAL_OK) {
			strcpy((char*)buf_string, "Error met tempertuursensor Rx\r\n");
		} else {
			}

			// Vochtigheid berekenen (14 bits, unsigned)
			val_temperatuur[0] = ((buf_data_temperatuur[0]) << 8) | buf_data_temperatuur[1];
			hum = ((float)val_temperatuur[0] / 16382) * 100;

			// Temperatuur berekenen (14 bits, signed)
			val_temperatuur[1] = (buf_data_temperatuur[2] << 6) | (buf_data_temperatuur[3]);
			temp = ((float)val_temperatuur[1] / 16382) * 165 - 40;

			// Vochtigheid en temperatuur omzetten naar integers (en * 100 doen om 2 cijfers achter de komma te berekenen)
			hum_int = (uint16_t)(hum * 100);
			temp_int = (int16_t)(temp * 100);

			//
			// VOC en CO2 sensor
			//

			buf_transmit[0] = 0b000001100;

			//stuur code naar sensor zodat deze weet wat er verwacht wordt
			ret = HAL_I2C_Master_Transmit(&hi2c1, MICS_ADDRESS, buf_transmit, 1, HAL_MAX_DELAY);
			HAL_Delay(250);

			// geen HAL_OK onvangen = fout
			if ( ret != HAL_OK ) {
			  strcpy((char*)buf_string, "Error met CO2 sensor Tx\r\n");
			  } else {
				  // Ontvang alle data van de sensor en sla deze op in de buffer
				  ret = HAL_I2C_Master_Receive(&hi2c1, MICS_ADDRESS, buf_data, 2, HAL_MAX_DELAY);

				  // geen HAL_OK onvangen = fout
				  if ( ret != HAL_OK ) {
					  strcpy((char*)buf_string, "Error met CO2 sensor Rx\r\n");
				  } else {

					  // kopieer waarde van ruwe waarde in CO2_raw
					  CO2_raw = buf_data[1];
					  VOC_raw = buf_data[0];


					  // ruwe waarde converteren
					  CO2 = (CO2_raw - 13.0) * (1600.0 / 229.0) + 400.0;
					  VOC = (VOC_raw - 13.0) * (1000.0 / 229.0);

					  //
					  // druk
					  //

					  buf_transmit_pres[0] = 0x09;
					  buf_transmit_pres[1] = 0x08;
					  buf_transmit_pres[2] = 0x07;
					  buf_transmit_pres[3] = 0x06;
					  buf_transmit_pres[4] = 0x05;
					  buf_transmit_pres[5] = 0x04;

					  buf_data_pres[0] = 0x00;
					  buf_data_pres[1] = 0x00;
					  buf_data_pres[2] = 0x00;
					  buf_data_pres[3] = 0x00;
					  buf_data_pres[4] = 0x00;
					  buf_data_pres[5] = 0x00;

					  // open normal mode

					  HAL_I2C_Mem_Read(&hi2c1, BMP_ADDRESS, 0x31, 1, buf_NVM, 21, HAL_MAX_DELAY);

					  NVM_PAR_T1 = ((uint16_t)buf_NVM[1] << 8) | ((uint16_t)buf_NVM[0]);
					  NVM_PAR_T2 = ((uint16_t)buf_NVM[3] << 8) | ((uint16_t)buf_NVM[2]);
					  NVM_PAR_T3 = buf_NVM[4];
					  NVM_PAR_P1 = ((uint16_t)buf_NVM[6] << 8) | ((uint16_t)buf_NVM[5]);
					  NVM_PAR_P2 = ((uint16_t)buf_NVM[8] << 8) | ((uint16_t)buf_NVM[7]);
					  NVM_PAR_P3 = buf_NVM[9];
					  NVM_PAR_P4 = buf_NVM[10];
					  NVM_PAR_P5 = ((uint16_t)buf_NVM[12] << 8) | ((uint16_t)buf_NVM[11]);
					  NVM_PAR_P6 = ((uint16_t)buf_NVM[14] << 8) | ((uint16_t)buf_NVM[13]);
					  NVM_PAR_P7 = buf_NVM[15];
					  NVM_PAR_P8 = buf_NVM[16];
					  NVM_PAR_P9 = ((uint16_t)buf_NVM[18] << 8) | ((uint16_t)buf_NVM[17]);
					  NVM_PAR_P10 = buf_NVM[19];
					  NVM_PAR_P11 = buf_NVM[20];

					  PAR_T1 = (float)NVM_PAR_T1 * pow(2,8);       // 2^-8
					  PAR_T2 = (float)NVM_PAR_T2 / pow(2,30);      // 2^30
					  PAR_T3 = (float)NVM_PAR_T3 / pow(2,48);      // 2^48

					  PAR_P1 = ((float)NVM_PAR_P1 - 16384) / pow(2,20);      // (2^14) / (2^20)
					  PAR_P2 = ((float)NVM_PAR_P2 - 16384) / pow(2,29);      // (2^14) / (2^29)
					  PAR_P3 = (float)NVM_PAR_P3 / pow(2,32);      // 2^-32
					  PAR_P4 = (float)NVM_PAR_P4 / pow(2,37);      // 2^-7
					  PAR_P5 = (float)NVM_PAR_P5 * 8;              // 2^-3
					  PAR_P6 = (float)NVM_PAR_P6 / 64;             // 2^6
					  PAR_P7 = (float)NVM_PAR_P7 / 256;            // 2^8
					  PAR_P8 = (float)NVM_PAR_P8 / pow(2,15);      // 2^15
					  PAR_P9 = (float)NVM_PAR_P9 / pow(2,48);      // 2^48
					  PAR_P10 = (float)NVM_PAR_P10 / pow(2,48);    // 2^48
					  PAR_P11 = (float)NVM_PAR_P11 / pow(2,65);    // 2^65

					  uint8_t en_power = 0x33;
					  HAL_I2C_Mem_Write(&hi2c1, BMP_ADDRESS, 0x1B, 1, &en_power, 1, HAL_MAX_DELAY);

					  //stuur code naar sensor zodat deze weet dat hij de 6 bytes, die info over druk en temp bevatten, moet terugsturen
					  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP_ADDRESS, buf_transmit_pres, 6, HAL_MAX_DELAY);
					  HAL_Delay(250);

					  // geen HAL_OK onvangen = fout
					  if ( ret != HAL_OK ) {
						  strcpy((char*)buf_string, "Error Tx\r\n");
						  } else {
							  // Ontvang de 3 bytes van de sensor en sla deze op in de buffer
							  ret = HAL_I2C_Mem_Read(&hi2c1, BMP_ADDRESS, 0x04, 1, buf_data_pres, 6, HAL_MAX_DELAY);
							  HAL_Delay(250);
							  // geen HAL_OK onvangen = fout
							  if ( ret != HAL_OK ) {
								  strcpy((char*)buf_string, "Error Rx\r\n");
							  } else {

								  // plak de 3 bytes aan elkaar
								  pres_raw = ((uint32_t)buf_data_pres[3] << 16) | ((uint32_t)buf_data_pres[4] << 8) | ((uint32_t)buf_data_pres[5]);
								  temp_raw = ((uint32_t)buf_data_pres[0] << 16) | ((uint32_t)buf_data_pres[1] << 8) | ((uint32_t)buf_data_pres[2]);

								  // Gebruik de compensatiefunctie om de ruwe waarde te corrigeren

								  partial_data1_temp = ((float)temp_raw - PAR_T1);
								  partial_data2_temp = (float)(partial_data1_temp * PAR_T2);
								  float t_lin = partial_data2_temp + partial_data1_temp * partial_data1_temp * PAR_T3;

								  partial_data1_pres = PAR_P6 * t_lin;
								  partial_data2_pres = PAR_P7 * t_lin * t_lin;
								  partial_data3_pres = PAR_P8 * t_lin * t_lin * t_lin;
								  partial_out1 = PAR_P5 + partial_data1_pres + partial_data2_pres + partial_data3_pres;

								  partial_data1_pres = PAR_P2 * t_lin;
								  partial_data2_pres = PAR_P3 * t_lin * t_lin;
								  partial_data3_pres = PAR_P4 * t_lin * t_lin * t_lin;
								  partial_out2 = (float)pres_raw * (PAR_P1 + partial_data1_pres + partial_data2_pres + partial_data3_pres);

								  partial_data1_pres = (float)pres_raw * (float)pres_raw;
								  partial_data2_pres = PAR_P9 + PAR_P10 * t_lin;
								  partial_data3_pres = partial_data1_pres * partial_data2_pres;
								  partial_data4_pres = partial_data3_pres + ((float)pres_raw * (float)pres_raw * (float)pres_raw * PAR_P11);
								  pres = partial_out1 + partial_out2 + partial_data4_pres;


					  sprintf((char*)buf_string,
							"T:%u.%02u,RH:%u.%02u,CO2:%u,VOC:%u,L:%u,P:%u\r\n",
							(temp_int / 100), (temp_int % 100),
							(hum_int / 100), (hum_int % 100),
							(unsigned int)CO2, (unsigned int)VOC,
							(unsigned int)lux,
							(unsigned int)pres);
					}
				  }
	  			}

	  		// Buffer versturen
	  		HAL_UART_Transmit(&huart2, buf_string, strlen((char*)buf_string), HAL_MAX_DELAY);
	  		HAL_UART_Transmit(&huart1, buf_string, strlen((char*)buf_string), HAL_MAX_DELAY);

	  		// Wacht 0.25s
	  		HAL_Delay(250);
    /* USER CODE BEGIN 3 */
  }}}}
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
