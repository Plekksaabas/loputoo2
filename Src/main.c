/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "registers.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t tempUnitSelection[5], tempSourceSelection[5], configurationSettingsMode[5], configurationACCONLY[5], configurationMAGONLY[5], configurationGYROONLY[5], configurationACCMAG[5], configurationACCGYRO[5];
uint8_t configurationMAGGYRO[5], configurationAMG[5], configurationIMU[5], configurationCOMPASS[5], configurationM4G[5], configurationNDOF_FMC_OFF[5], configurationFusionModeNDOF[5];
uint8_t receivedDataUART[2], receivedDataUARTBuffer [32], sendDataReadInfo[4];
uint8_t transferUartBuffer[64];
int16_t acc_Z, acc_Y, acc_X = 0;
int isitworking, dataruined = 0;
int acc_Z_MSB, acc_Z_LSB, acc_Y_MSB, acc_Y_LSB, acc_X_MSB, acc_X_LSB, temperature, data = 0;
int acc_X_offset, acc_Y_offset, acc_Z_offset;
bool config_error = false;
bool accel_error = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	
	return ch;
}
void initVariables(){
	tempUnitSelection							[0] = UART_START_BYTE;
	tempUnitSelection							[1] = UART_WRITE;
	tempUnitSelection							[2] = UNIT_SEL_ADDR;
	tempUnitSelection							[3] = 0x01;
	tempUnitSelection							[4] = 0x00;
	
	tempSourceSelection       	  [0]  = UART_START_BYTE;
	tempSourceSelection     	    [1]  = UART_WRITE;
	tempSourceSelection       	  [2]  = TEMP_SOURCE_ADDR;
	tempSourceSelection     	    [3]  = 0x01;
	tempSourceSelection       	  [4]  = 0x00;	
	
	configurationSettingsMode 		[0] = UART_START_BYTE;
	configurationSettingsMode 		[1] = UART_WRITE;
	configurationSettingsMode 		[2] = OPR_MODE_ADDR;
	configurationSettingsMode 		[3] = 0x01;
	configurationSettingsMode 		[4] = 0x00;

	configurationACCONLY 					[0] = UART_START_BYTE;
	configurationACCONLY 					[1] = UART_WRITE;
	configurationACCONLY 					[2] = OPR_MODE_ADDR;
	configurationACCONLY 					[3] = 0x01;
	configurationACCONLY 					[4] = 0x01;

	configurationMAGONLY 					[0] = UART_START_BYTE;
	configurationMAGONLY 					[1] = UART_WRITE;
	configurationMAGONLY 					[2] = OPR_MODE_ADDR;
	configurationMAGONLY 					[3] = 0x01;
	configurationMAGONLY 					[4] = 0x02;

	configurationGYROONLY 				[0] = UART_START_BYTE;
	configurationGYROONLY 	 			[1] = UART_WRITE;
	configurationGYROONLY 	 			[2] = OPR_MODE_ADDR;
	configurationGYROONLY 	 			[3] = 0x01;
	configurationGYROONLY 	 			[4] = 0x03;

	configurationACCMAG 					[0] = UART_START_BYTE;
	configurationACCMAG 	 				[1] = UART_WRITE;
	configurationACCMAG 	 				[2] = OPR_MODE_ADDR;
	configurationACCMAG 	 				[3] = 0x01;
	configurationACCMAG 	 				[4] = 0x04;
	
	configurationACCGYRO 					[0] = UART_START_BYTE;
	configurationACCGYRO 	 				[1] = UART_WRITE;
	configurationACCGYRO 	 				[2] = OPR_MODE_ADDR;
	configurationACCGYRO 	 				[3] = 0x01;
	configurationACCGYRO 	 				[4] = 0x05;
	
	configurationMAGGYRO 					[0] = UART_START_BYTE;
	configurationMAGGYRO 	 				[1] = UART_WRITE;
	configurationMAGGYRO 	 				[2] = OPR_MODE_ADDR;
	configurationMAGGYRO 	 				[3] = 0x01;
	configurationMAGGYRO 	 				[4] = 0x06;

	configurationAMG 							[0] = UART_START_BYTE;
	configurationAMG 	 						[1] = UART_WRITE;
	configurationAMG 	 						[2] = OPR_MODE_ADDR;
	configurationAMG 	 						[3] = 0x01;
	configurationAMG 	 						[4] = 0x07;

	configurationIMU 							[0] = UART_START_BYTE;
	configurationIMU 	 						[1] = UART_WRITE;
	configurationIMU 	 						[2] = OPR_MODE_ADDR;
	configurationIMU 	 						[3] = 0x01;
	configurationIMU 	 						[4] = 0x08;

	configurationCOMPASS 					[0] = UART_START_BYTE;
	configurationCOMPASS 			 		[1] = UART_WRITE;
	configurationCOMPASS 			 		[2] = OPR_MODE_ADDR;
	configurationCOMPASS 			 		[3] = 0x01;
	configurationCOMPASS 			 		[4] = 0x09;

	configurationM4G 							[0] = UART_START_BYTE;
	configurationM4G 	 						[1] = UART_WRITE;
	configurationM4G 	 						[2] = OPR_MODE_ADDR;
	configurationM4G 	 						[3] = 0x01;
	configurationM4G 	 						[4] = 0x0A;
			
	configurationNDOF_FMC_OFF			[0] = UART_START_BYTE;
	configurationNDOF_FMC_OFF 	 	[1] = UART_WRITE;
	configurationNDOF_FMC_OFF 	 	[2] = OPR_MODE_ADDR;
	configurationNDOF_FMC_OFF 	 	[3] = 0x01;
	configurationNDOF_FMC_OFF 	 	[4] = 0x0B;
												
	configurationFusionModeNDOF 	[0] = UART_START_BYTE;
	configurationFusionModeNDOF 	[1] = UART_WRITE;
	configurationFusionModeNDOF 	[2] = OPR_MODE_ADDR;
	configurationFusionModeNDOF 	[3] = 0x01;
	configurationFusionModeNDOF 	[4] = 0x0C;
	
}
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){ 	
	byte_received = 1;
}
int getReadData(){

	byte_received = 0;
	data			= 0;
	int datareceive = 0;
	
	receivedDataUARTBuffer[0] = 0x00;
	receivedDataUARTBuffer[1] = 0x00;
	receivedDataUARTBuffer[2] = 0x00;

	while (byte_received == 0){
	if (datareceive == 0){
		HAL_UART_Receive_IT(&huart1, receivedDataUARTBuffer, 3);
		datareceive =1;
	}
	if (receivedDataUARTBuffer[0] == 0xEE && receivedDataUARTBuffer[1] != 0x00){
	  HAL_UART_Abort(&huart1);
		byte_received = 1;
		data = 999;
		return data;
	}
	if (byte_received == 1){
		data = receivedDataUARTBuffer[2];
	}

}
 
 return data;	
}
	


int getTemperature(){
	
	int data = 0;
	sendDataReadInfo			 [0] = UART_START_BYTE;
	sendDataReadInfo			 [1] = UART_READ;
	sendDataReadInfo			 [2] = TEMP_ADDR;
	sendDataReadInfo			 [3] = 0x01;
	
	HAL_UART_Transmit	(&huart1, sendDataReadInfo, 4, 200);
	
	data = getReadData();
	

	return data;
}
bool configurationSettings(UART_HandleTypeDef *huart, uint8_t *dataToSend, int howManyBytes, int delay) {
	byte_received   = 0;
	int datareceive = 0;
	bool data       = false;
	receivedDataUART [0] = 0x00;
	receivedDataUART [1] = 0x00;
	HAL_UART_Transmit(huart, dataToSend, howManyBytes, delay);
	while (byte_received == 0){
		if (datareceive == 0){
			HAL_UART_Receive_IT(huart, receivedDataUART, 2);
			datareceive =1;
		}
	}
	if (receivedDataUART[0] == 0xEE && receivedDataUART[1] != 0x01){
		HAL_UART_Abort(huart);
		data = true;
	}
	if (byte_received == 1 && receivedDataUART[1] != 0xEE && receivedDataUART[1] !=0x00 && receivedDataUART[1] !=0x03){
		data = false;
}
	
return data;
}
bool getAccData (){
	bool error = false;
	sendDataReadInfo			 [0] = UART_START_BYTE;
	sendDataReadInfo			 [1] = UART_READ;
	sendDataReadInfo			 [2] = ACCEL_DATA_X_LSB_ADDR;
	sendDataReadInfo			 [3] = 0x06;

	HAL_UART_Transmit(&huart1, sendDataReadInfo, 4,50);
	
	byte_received = 0;
	data			= 0;
	int datareceive = 0;
	
	receivedDataUARTBuffer[0] = 0x00;
	receivedDataUARTBuffer[1] = 0x00;
	receivedDataUARTBuffer[2] = 0x00;

	while (byte_received == 0){
	if (datareceive == 0){
		HAL_UART_Receive_IT(&huart1, receivedDataUARTBuffer, 8);
		datareceive = 1;
	}
	if (receivedDataUARTBuffer[0] == 0xEE && receivedDataUARTBuffer[1] != 0x00){
	  HAL_UART_Abort(&huart1);
		byte_received = 1;
		error = true;
		return error;
	}
	if (byte_received == 1){
		acc_X_LSB = receivedDataUARTBuffer[2];
		acc_X_MSB = receivedDataUARTBuffer[3];
		
		acc_Y_LSB = receivedDataUARTBuffer[4];
		acc_Y_MSB = receivedDataUARTBuffer[5];
		
		acc_Z_LSB = receivedDataUARTBuffer[6];
		acc_Z_MSB = receivedDataUARTBuffer[7];
	}

}
 
 return error;	

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	initVariables();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	//CONFIGURATION SETTINGS
	HAL_Delay(2000);
	if (config_error == false){
		config_error = configurationSettings(&huart1, configurationSettingsMode, 5, 200);
	}
	if (config_error == false){
		config_error = configurationSettings(&huart1, tempSourceSelection, 5, 200);
	}	
	if (config_error == false){
		config_error = configurationSettings(&huart1, tempUnitSelection, 5, 200);
	}	
	if (config_error == false){
	config_error = configurationSettings(&huart1, configurationACCONLY , 5, 200);
	}	
	HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	//temperature  = getTemperature();
	accel_error  = getAccData();
	dataruined = 0;
	
	if (isitworking == 100){
		acc_X_offset = acc_X;
		acc_Y_offset = acc_Y;
		acc_Z_offset = acc_Z;
	}
	
	
	if (accel_error == false ){
		acc_X = (acc_X_MSB << 8) | acc_X_LSB;	
		acc_Y = (acc_Y_MSB << 8) | acc_Y_LSB;
		acc_Z = (acc_Z_MSB << 8) | acc_Z_LSB;	
	}
  else {
		dataruined = 1;
		acc_X = 9999;
		acc_Y = 9999;
		acc_Z = 9999;
	}

		if (accel_error == false && isitworking > 150 ){
		acc_X = acc_X - acc_X_offset;
		acc_Y = acc_Y - acc_Y_offset;
		acc_Z = acc_Z - acc_Z_offset;
	}
	
	printf("X = %d, Y = %d, Z = %d  \n", acc_X , acc_Y, acc_Z );
	isitworking++;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
