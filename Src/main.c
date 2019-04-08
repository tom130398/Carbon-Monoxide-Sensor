
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include <math.h>

/* USER CODE BEGIN Includes */
//const uint16_t _AIRQ5_DATA_CHANNEL_CO = 0x6000;
//const uint16_t _AIRQ5_DATA_CHANNEL_NO2 = 0x4000;
//const uint16_t _AIRQ5_DATA_CHANNEL_NH3 = 0x5000;
//const uint8_t _AIRQ5_REG_POINTER_CONVERT = 0x00;
//const uint8_t _AIRQ5_REG_POINTER_CONFIG = 0x01;
//const uint16_t _AIRQ5_CONFIG_OS_SINGLE = 0x8000;
//const uint16_t _AIRQ5_CONFIG_PGA_2_048V = 0x0400;
//const uint16_t _AIRQ5_CONFIG_SINGLE_MODE = 0x0100;
//const uint16_t _AIRQ5_CONFIG_DATA_RATE_1600SPS = 0x0080;
//const uint16_t _AIRQ5_CONFIG_COMP_MODE_TRADITIONAL = 0x0000;
//const uint16_t _AIRQ5_CONFIG_COMP_POL_ACTIVE_LOW = 0x0000;
//const uint16_t _AIRQ5_CONFIG_COMP_LAT_NOT_LATCH = 0x0000;
//const uint16_t _AIRQ5_CONFIG_COMP_QUE_0CONV = 0x0003;
//
unsigned char start_delimeter = 0x7E;
unsigned char length_MSB = 0x00;
unsigned char length_LSB = 0x0C;
unsigned char frame_type = 0x01;		//Transmit Request Frame, API identifier
unsigned char frame_id = 0x01;		//Identifies data frame to enable respond frame, API Frame ID
unsigned char option = 0x00;
unsigned char destination_add_MSB = 0xAB;
unsigned char destination_add_LSB = 0x01;

unsigned char ADS1015_ADDRESS=0x48;			//1001000
unsigned char ADS1015_ADDRESS_write=0x90;	//10010000
unsigned char ADS1015_ADDRESS_read=0x91;	//10010001
unsigned char ADSwrite[6];
unsigned char received_data[6];
int16_t reading;
int CO;
const float voltageConv=2.048/2048 ;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void airq5_writeData(uint8_t reg, uint16_t _data)
//{
//    uint8_t writeReg[ 3 ];
//
//    writeReg[ 0 ] = reg;
//    writeReg[ 1 ] = _data >> 8;
//    writeReg[ 2 ] = _data & 0x00FF;
//
//    HAL_I2C_Master_Transmit(&hi2c1, address_for_write, writeReg, 1,100);
//}
//
//uint16_t airq5_readData(uint8_t reg)
//{
//    uint8_t writeReg[ 1 ];
//    uint8_t readReg[ 2 ];
//    uint16_t dataValue;
//
//    writeReg[ 0 ] = reg;
//
//    HAL_I2C_Master_Transmit(&hi2c1, address_for_write, writeReg, 1,100);
//    HAL_I2C_Master_Receive(&hi2c1, address_for_read, readReg, 2, 100);
//
//    dataValue = readReg[ 0 ];
//    dataValue = dataValue << 8;
//    dataValue = dataValue | readReg[ 1 ];
//    return dataValue;
//}
//
//void airq5_setConfiguration(uint16_t config)
//{
//    _dataConfig = config;
//}
//
//uint16_t airq5_readSensorData(uint16_t channel_data)
//{
//	uint16_t setConfig;
//	uint16_t getData;
//
//	setConfig = _dataConfig;
//	setConfig = setConfig | channel_data;
//    airq5_writeData(_AIRQ5_REG_POINTER_CONFIG, setConfig );
//    getData = airq5_readData( _AIRQ5_REG_POINTER_CONVERT );
//
//    getData = getData >> 4;
//
//    return getData;
//}
//
//void applicationInit()
//{
//    airq5_setConfiguration( _AIRQ5_CONFIG_OS_SINGLE |
//                            _AIRQ5_CONFIG_PGA_2_048V |
//                            _AIRQ5_CONFIG_SINGLE_MODE |
//                            _AIRQ5_CONFIG_DATA_RATE_1600SPS |
//                            _AIRQ5_CONFIG_COMP_MODE_TRADITIONAL |
//                            _AIRQ5_CONFIG_COMP_POL_ACTIVE_LOW |
//                            _AIRQ5_CONFIG_COMP_LAT_NOT_LATCH |
//                            _AIRQ5_CONFIG_COMP_QUE_0CONV );
//}
//
//uint8_t applicationTask()
//{
//      uint8_t CO_sensorData = airq5_readSensorData(_AIRQ5_DATA_CHANNEL_CO);
//      HAL_Delay( 200 );
//      return CO_sensorData;
//}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void send_to_xbee(char dataHexa[8]){
	int sum2 = 0x00;
	int sum1 = frame_type + frame_id + destination_add_MSB + destination_add_LSB + option;
	for (int i = 0; i < strlen(dataHexa); i++) {
		sum2 += dataHexa[i];
	}
	int sum = 0;
	sum = sum1 + sum2;
	unsigned char two_last_digit = sum & 0xFF;
	unsigned char checksum = 255 - two_last_digit;
	unsigned char message[16] = { start_delimeter, length_MSB, length_LSB,frame_type, frame_id, destination_add_MSB, destination_add_LSB,option, 0, 0, 0, 0, 0, 0, 0, checksum };
	for (int i = 0; i < 7; i++) {
		message[8 + i] = dataHexa[i];
	}
	HAL_UART_Transmit(&huart1, message, 16, 100);
	HAL_UART_Transmit(&huart2, message, 16, 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  //applicationInit();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  char data[8]="";
//	  uint8_t CO=applicationTask();
//	  sprintf(data,"CO=%d", CO);
//	  send_to_xbee(data);
//	  //HAL_UART_Transmit(&huart2, (uint16_t*)data, strlen(data), 100);
//	  HAL_Delay(1000);
  /* USER CODE END WHILE */
	  for(int i=0; i<4; i++){
		  char data[8]="";

		  ADSwrite[0]=0x01;
		  ADSwrite[1]=0xE5;	//11100101
		  ADSwrite[2]=0x83;	//10000011
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1015_ADDRESS_write, ADSwrite, 3, 100);
		  ADSwrite[0]=0x00;
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1015_ADDRESS_write, ADSwrite, 1, 100);
		  HAL_Delay(2000);
		  HAL_I2C_Master_Receive(&hi2c1, ADS1015_ADDRESS_read, received_data, 2, 100);
		  reading = ((received_data[0] << 8) | received_data[1]) >> 4;
		  //CO=reading*voltageConv;
		  if(CO < 1)
			  CO=1;
		  sprintf(data,"CO=%d", reading);
		  send_to_xbee(data);
	}
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
