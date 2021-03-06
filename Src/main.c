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
#include "stm32f4xx_it.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t CONFIG_temp_UnitSelection[5], CONFIG_temp_SourceSelection[5], CONFIG_Mode_Configuration[5], CONFIG_Mode_Accelerometer_Only[5], CONFIG_Mode_Magnetometer_Only[5], CONFIG_Mode_Gyroscope_Only[5], CONFIG_Mode_Accelerometer_Magnetometer[5], CONFIG_Mode_Accelerometer_Gyroscope[5];
uint8_t CONFIG_configurationPage_ID_0[5], CONFIG_configurationPage_ID_1[5], CONFIG_Mode_Magnetometer_Gyroscope[5], CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope[5], CONFIG_Mode_Inertial_Measurement_Unit[5], CONFIG_Mode_Compass[5], CONFIG_Mode_Magnet_For_Compass[5], CONFIG_Mode_NDOF_FMC_OFF[5], CONFIG_Mode_NDOF[5];
uint8_t CONFIG_register_Page_0[5], CONFIG_register_Page_1[5];
uint8_t receivedDataUART[2], receivedDataUARTBuffer [32], sendDataReadInfo[4], sendDataUARTBuffer[10];
uint8_t transferUartBuffer[64];
uint8_t readDataSingleReadBuffer[5], readDataSingleWriteBuffer[5], CONFIG_accel_RangeSelection[5];
uint8_t dataToSendToReadSingleRegister[5];
int16_t acc_1_Z, acc_1_Y, acc_1_X, acc_2_Z, acc_2_Y, acc_2_X = 0;
int isitworking, dataruined = 0;
uint8_t acc_1_Z_MSB, acc_1_Z_LSB, acc_1_Y_MSB, acc_1_Y_LSB, acc_1_X_MSB, acc_1_X_LSB;
uint8_t acc_2_Z_MSB, acc_2_Z_LSB, acc_2_Y_MSB, acc_2_Y_LSB, acc_2_X_MSB, acc_2_X_LSB;
int temperature, data = 0;
uint8_t OFFSET_Z_MSB, OFFSET_Z_LSB, OFFSET_Y_MSB, OFFSET_Y_LSB, OFFSET_X_MSB, OFFSET_X_LSB;
int acc_X_offset, acc_Y_offset, acc_Z_offset;
uint8_t acc_1_X_LSB_OFFSET, acc_1_X_MSB_OFFSET, acc_1_Y_LSB_OFFSET, acc_1_Y_MSB_OFFSET, acc_1_Z_LSB_OFFSET, acc_1_Z_MSB_OFFSET;
uint8_t acc_2_X_LSB_OFFSET, acc_2_X_MSB_OFFSET, acc_2_Y_LSB_OFFSET, acc_2_Y_MSB_OFFSET, acc_2_Z_LSB_OFFSET, acc_2_Z_MSB_OFFSET;
int howManySeconds;
int Data_getReadDataSingleRegister;
bool config_error = false;
bool accel_error = false;
bool configureAccelerometerOffset_ERROR = false;
int howManySecondsTheButtonHasBeenHeld = 0;
uint8_t readDataSingleReceiveBuffer[5];
int variable;
uint8_t variable_old[6];
uint8_t variable_new[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);

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
void initUARTBuffer(){
	for (int i = 0; i <= 10; i++){
		sendDataUARTBuffer      [i] = 0x00;
	}
	
	for (int i = 0; i<= 2; i++){
		receivedDataUART  [i] = 0x00;
	}	
}
bool setSingleRegister(UART_HandleTypeDef *huart, uint8_t dataToSend, uint8_t registerToSend ){
	bool error = false;
	int datareceive = 0;
  byte_received = 0;
	initUARTBuffer(); 
	
	sendDataUARTBuffer[0] = UART_START_BYTE;
	sendDataUARTBuffer[1] = UART_WRITE;
	sendDataUARTBuffer[2] = registerToSend;
	sendDataUARTBuffer[3] = 0x01;
	sendDataUARTBuffer[4] = dataToSend;
	
	HAL_UART_Transmit(huart, sendDataUARTBuffer, 5, 200);
    while (byte_received == 0){
        if (datareceive == 0){
            HAL_UART_Receive_IT(huart, receivedDataUART, 2);
            datareceive =1;
        }
    }
    if (receivedDataUART[0] == 0xEE && receivedDataUART[1] != 0x01){
        HAL_UART_Abort(huart);
        error = true;
    }
    if (byte_received == 1 && receivedDataUART[1] != 0xEE && receivedDataUART[1] !=0x00 && receivedDataUART[1] !=0x03){
        error = false;
    }
		
	return error;
}
int  readSingleRegister(UART_HandleTypeDef *huart, uint8_t registerToRead){
	byte_received = 0;
	int datareceive = 0;
	int data;
	
	dataToSendToReadSingleRegister [0] = UART_START_BYTE;
	dataToSendToReadSingleRegister [1] = UART_READ;
	dataToSendToReadSingleRegister [2] = registerToRead;
	dataToSendToReadSingleRegister [3] = 0x01;
	
	HAL_UART_Transmit(huart, dataToSendToReadSingleRegister, 4, 200);
	
	receivedDataUARTBuffer[0] = 0x00;
	receivedDataUARTBuffer[1] = 0x00;
	receivedDataUARTBuffer[2] = 0x00;

	while (byte_received == 0){
		if (datareceive == 0){
			HAL_UART_Receive_IT(huart, receivedDataUARTBuffer, 3);
			datareceive =1;
		}
		if (byte_received_time >= 1000){
			HAL_UART_Abort(huart);
			byte_received = 1;
			data = 999;
			return data;
		}
		if (receivedDataUARTBuffer[0] == 0xEE){
			HAL_UART_Abort(huart);
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
bool getWriteResonseData(UART_HandleTypeDef *huart){
	bool error      = false;
	byte_received   = 0;
	int datareceive = 0;
	
	receivedDataUART[0] = 0x00;
	receivedDataUART[1] = 0x00;

	while (byte_received == 0){
		if (datareceive == 0){
			HAL_UART_Receive_IT(huart, receivedDataUART, 2);
			datareceive = 1;
		}
	}
	if (receivedDataUART[0] == 0xEE && receivedDataUART[1] != 0x01){
		HAL_UART_Abort(huart);
		byte_received = 1;
		error = true;
		return error;
	}
 
 return error;	
}
bool setRegisterMapPage_0(UART_HandleTypeDef *huart){
	bool error             = false;
	
	sendDataUARTBuffer [0] = UART_START_BYTE;
	sendDataUARTBuffer [1] = UART_WRITE;
	sendDataUARTBuffer [2] = PAGE_ID_ADDR;
	sendDataUARTBuffer [3] = 0x01;
	sendDataUARTBuffer [4] = 0x00;
	
	HAL_UART_Transmit(huart, sendDataUARTBuffer, 5, 200);
	
	error = getWriteResonseData(huart);
	if (error == true){
		HAL_UART_Transmit(huart, sendDataUARTBuffer, 5, 200);
		error = getWriteResonseData(huart);
	}
	
	return error;
}
bool setRegisterMapPage_1(UART_HandleTypeDef *huart){
	bool error;
	
	sendDataUARTBuffer [0] = UART_START_BYTE;
	sendDataUARTBuffer [1] = UART_WRITE;
	sendDataUARTBuffer [2] = PAGE_ID_ADDR;
	sendDataUARTBuffer [3] = 0x01;
	sendDataUARTBuffer [4] = 0x01;
	
	HAL_UART_Transmit(huart, sendDataUARTBuffer, 5, 200);
		
	error = getWriteResonseData(huart);
	if (error == true){
		HAL_UART_Transmit(huart, sendDataUARTBuffer, 5, 200);
		error = getWriteResonseData(huart);
	}
	
	return error;
}
void initializeConfigurationVariables(){
	CONFIG_register_Page_0														[0] = UART_START_BYTE;
	CONFIG_register_Page_0														[1] = UART_WRITE;
	CONFIG_register_Page_0														[2] = PAGE_ID_ADDR;
	CONFIG_register_Page_0														[3] = 0x01;
	CONFIG_register_Page_0														[4] = 0x00;
	
	CONFIG_register_Page_1														[0] = UART_START_BYTE;
	CONFIG_register_Page_1														[1] = UART_WRITE;
	CONFIG_register_Page_1														[2] = PAGE_ID_ADDR;
	CONFIG_register_Page_1														[3] = 0x01;
	CONFIG_register_Page_1														[4] = 0x01;	
	
	CONFIG_temp_UnitSelection													[0] = UART_START_BYTE;
	CONFIG_temp_UnitSelection													[1] = UART_WRITE;
	CONFIG_temp_UnitSelection													[2] = UNIT_SEL_ADDR;
	CONFIG_temp_UnitSelection													[3] = 0x01;
	CONFIG_temp_UnitSelection													[4] = 0x00;
	
	CONFIG_configurationPage_ID_0											[0] = UART_START_BYTE;
	CONFIG_configurationPage_ID_0											[1] = UART_WRITE;
	CONFIG_configurationPage_ID_0											[2] = PAGE_ID_ADDR;
	CONFIG_configurationPage_ID_0											[3] = 0x01;
	CONFIG_configurationPage_ID_0											[4] = 0x00;
	
	CONFIG_configurationPage_ID_1											[0] = UART_START_BYTE;
	CONFIG_configurationPage_ID_1											[1] = UART_WRITE;
	CONFIG_configurationPage_ID_1											[2] = PAGE_ID_ADDR;
	CONFIG_configurationPage_ID_1											[3] = 0x01;
	CONFIG_configurationPage_ID_1											[4] = 0x01;

	CONFIG_accel_RangeSelection       								[0] = UART_START_BYTE;
	CONFIG_accel_RangeSelection												[1] = UART_WRITE;
	CONFIG_accel_RangeSelection   								    [2] = ACCEL_CONFIG_ADDR;
	CONFIG_accel_RangeSelection       								[3] = 0x01;
	CONFIG_accel_RangeSelection      							 		[4] = 0x12;
		
	CONFIG_temp_SourceSelection      							 	  [0]  = UART_START_BYTE;
	CONFIG_temp_SourceSelection     							    [1]  = UART_WRITE;
	CONFIG_temp_SourceSelection       	  						[2]  = TEMP_SOURCE_ADDR;
	CONFIG_temp_SourceSelection     	    						[3]  = 0x01;
	CONFIG_temp_SourceSelection       	  						[4]  = 0x00;	
	
	CONFIG_Mode_Configuration 												[0] = UART_START_BYTE;
	CONFIG_Mode_Configuration 												[1] = UART_WRITE;
	CONFIG_Mode_Configuration 												[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Configuration 												[3] = 0x01;
	CONFIG_Mode_Configuration 												[4] = 0x00;

	CONFIG_Mode_Accelerometer_Only 										[0] = UART_START_BYTE;
	CONFIG_Mode_Accelerometer_Only 										[1] = UART_WRITE;
	CONFIG_Mode_Accelerometer_Only 										[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Accelerometer_Only 										[3] = 0x01;
	CONFIG_Mode_Accelerometer_Only 										[4] = 0x01;

	CONFIG_Mode_Magnetometer_Only 										[0] = UART_START_BYTE;
	CONFIG_Mode_Magnetometer_Only 										[1] = UART_WRITE;
	CONFIG_Mode_Magnetometer_Only 										[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Magnetometer_Only 										[3] = 0x01;
	CONFIG_Mode_Magnetometer_Only 										[4] = 0x02;

	CONFIG_Mode_Gyroscope_Only 												[0] = UART_START_BYTE;
	CONFIG_Mode_Gyroscope_Only 	 											[1] = UART_WRITE;
	CONFIG_Mode_Gyroscope_Only 	 											[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Gyroscope_Only 	 											[3] = 0x01;
	CONFIG_Mode_Gyroscope_Only 	 											[4] = 0x03;

	CONFIG_Mode_Accelerometer_Magnetometer						[0] = UART_START_BYTE;
	CONFIG_Mode_Accelerometer_Magnetometer 						[1] = UART_WRITE;
	CONFIG_Mode_Accelerometer_Magnetometer 						[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Accelerometer_Magnetometer 						[3] = 0x01;
	CONFIG_Mode_Accelerometer_Magnetometer 						[4] = 0x04;
	
	CONFIG_Mode_Accelerometer_Gyroscope 							[0] = UART_START_BYTE;
	CONFIG_Mode_Accelerometer_Gyroscope 						 	[1] = UART_WRITE;
	CONFIG_Mode_Accelerometer_Gyroscope 	 						[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Accelerometer_Gyroscope 	 						[3] = 0x01;
	CONFIG_Mode_Accelerometer_Gyroscope						 	 	[4] = 0x05;
	
	CONFIG_Mode_Magnetometer_Gyroscope 								[0] = UART_START_BYTE;
	CONFIG_Mode_Magnetometer_Gyroscope 				 				[1] = UART_WRITE;
	CONFIG_Mode_Magnetometer_Gyroscope 				 				[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Magnetometer_Gyroscope 				 				[3] = 0x01;
	CONFIG_Mode_Magnetometer_Gyroscope			 	 				[4] = 0x06;

	CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope	[0] = UART_START_BYTE;
	CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope 	[1] = UART_WRITE;
	CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope 	[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope 	[3] = 0x01;
	CONFIG_Mode_Accelerometer_Magnetometer_Gyroscope 	[4] = 0x07;

	CONFIG_Mode_Inertial_Measurement_Unit 						[0] = UART_START_BYTE;
	CONFIG_Mode_Inertial_Measurement_Unit 	 					[1] = UART_WRITE;
	CONFIG_Mode_Inertial_Measurement_Unit 	 					[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Inertial_Measurement_Unit 	 					[3] = 0x01;
	CONFIG_Mode_Inertial_Measurement_Unit 	 					[4] = 0x08;

	CONFIG_Mode_Compass 															[0] = UART_START_BYTE;
	CONFIG_Mode_Compass 			 												[1] = UART_WRITE;
	CONFIG_Mode_Compass											 			 		[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Compass											 			 		[3] = 0x01;
	CONFIG_Mode_Compass											 			 		[4] = 0x09;

	CONFIG_Mode_Magnet_For_Compass			 							[0] = UART_START_BYTE;
	CONFIG_Mode_Magnet_For_Compass			 	 						[1] = UART_WRITE;
	CONFIG_Mode_Magnet_For_Compass			 	 						[2] = OPR_MODE_ADDR;
	CONFIG_Mode_Magnet_For_Compass			 	 						[3] = 0x01;
	CONFIG_Mode_Magnet_For_Compass			 	 						[4] = 0x0A;
			
	CONFIG_Mode_NDOF_FMC_OFF													[0] = UART_START_BYTE;
	CONFIG_Mode_NDOF_FMC_OFF 												 	[1] = UART_WRITE;
	CONFIG_Mode_NDOF_FMC_OFF											 	 	[2] = OPR_MODE_ADDR;
	CONFIG_Mode_NDOF_FMC_OFF											 	 	[3] = 0x01;
	CONFIG_Mode_NDOF_FMC_OFF											 	 	[4] = 0x0B;
												
	CONFIG_Mode_NDOF 																	[0] = UART_START_BYTE;
	CONFIG_Mode_NDOF																 	[1] = UART_WRITE;
	CONFIG_Mode_NDOF 																	[2] = OPR_MODE_ADDR;
	CONFIG_Mode_NDOF 																	[3] = 0x01;
	CONFIG_Mode_NDOF 																	[4] = 0x0C;
	
}
void printAccelData(){
	printf("1. X = %d, Y = %d, Z = %d || 2. X = X = %d, Y = %d, Z = %d \n", acc_1_X , acc_1_Y, acc_1_Z, acc_2_X, acc_2_Y, acc_2_Z);
}	
void accDataConversion(){
	acc_1_X = (acc_1_X_MSB << 8) | acc_1_X_LSB;	
	acc_1_Y = (acc_1_Y_MSB << 8) | acc_1_Y_LSB;
	acc_1_Z = (acc_1_Z_MSB << 8) | acc_1_Z_LSB;	
	
	acc_2_X = (acc_2_X_MSB << 8) | acc_2_X_LSB;	
	acc_2_Y = (acc_2_Y_MSB << 8) | acc_2_Y_LSB;
	acc_2_Z = (acc_2_Z_MSB << 8) | acc_2_Z_LSB;	
}
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){ 	
	byte_received = 1;
}

int  getReadData(UART_HandleTypeDef *huart){

	byte_received = 0;
	data			= 0;
	int datareceive = 0;
	
	receivedDataUARTBuffer[0] = 0x00;
	receivedDataUARTBuffer[1] = 0x00;
	receivedDataUARTBuffer[2] = 0x00;

	while (byte_received == 0){
		if (datareceive == 0){
			HAL_UART_Receive_IT(huart, receivedDataUARTBuffer, 3);
			datareceive =1;
		}
		if (receivedDataUARTBuffer[0] == 0xEE && receivedDataUARTBuffer[1] != 0x00){
			HAL_UART_Abort(huart);
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
	


int  getTemperature(){
	
	int data = 0;
	sendDataReadInfo			 [0] = UART_START_BYTE;
	sendDataReadInfo			 [1] = UART_READ;
	sendDataReadInfo			 [2] = TEMP_ADDR;
	sendDataReadInfo			 [3] = 0x01;
	
	HAL_UART_Transmit	(&huart1, sendDataReadInfo, 4, 200);
	
	data = getReadData(&huart1);
	

	return data;
}
bool sendConfigurationSettings(UART_HandleTypeDef *huart, uint8_t *dataToSend, int howManyBytes, int delay) {
    byte_received   = 0;
    int datareceive = 0;
    bool data       = false;
    receivedDataUART [0] = 0x00;
    receivedDataUART [1] = 0x00;
	  byte_received_time_wait = 0;
    HAL_UART_Transmit(huart, dataToSend, howManyBytes, delay);
    while (byte_received == 0){
        if (datareceive == 0){
          HAL_UART_Receive_IT(huart, receivedDataUART, 2);
          datareceive = 1;
					byte_received_time_wait = 1;
        }
				if (byte_received_time >= 2000 && byte_received == 0){
					HAL_UART_Abort(huart);
					byte_received = 1;
					byte_received_time_wait = 0;
					data = true;
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
bool getAccelerometerData (UART_HandleTypeDef *huart){
	HAL_UART_Abort(huart); 
	bool error = false;
 sendDataReadInfo    [0] = UART_START_BYTE;
 sendDataReadInfo    [1] = UART_READ;
 sendDataReadInfo    [2] = ACCEL_DATA_X_LSB_ADDR;
 sendDataReadInfo    [3] = 0x06;

 HAL_UART_Transmit(huart, sendDataReadInfo, 4,50);
 
 byte_received = 0;
 data   = 0;

 receivedDataUARTBuffer[0] = 0x00;
 receivedDataUARTBuffer[1] = 0x00;
 receivedDataUARTBuffer[2] = 0x00;

		HAL_UART_Receive_IT(huart, receivedDataUARTBuffer, 8);

	
while (byte_received == 0){
	if (receivedDataUARTBuffer[0] == 0xEE && receivedDataUARTBuffer[1] != 0x00){
		HAL_UART_Abort(huart);
		byte_received = 1;
		error = true;
		return error;
 }
}
if (byte_received == 1 && huart == &huart1){
	acc_1_X_LSB = receivedDataUARTBuffer[2];
	acc_1_X_MSB = receivedDataUARTBuffer[3];

	acc_1_Y_LSB = receivedDataUARTBuffer[4];
	acc_1_Y_MSB = receivedDataUARTBuffer[5];
  
	acc_1_Z_LSB = receivedDataUARTBuffer[6];
	acc_1_Z_MSB = receivedDataUARTBuffer[7];
 }

if (byte_received == 1 && huart == &huart3){
	acc_2_X_LSB = receivedDataUARTBuffer[2];
	acc_2_X_MSB = receivedDataUARTBuffer[3];

	acc_2_Y_LSB = receivedDataUARTBuffer[4];
	acc_2_Y_MSB = receivedDataUARTBuffer[5];
 
	acc_2_Z_LSB = receivedDataUARTBuffer[6];
	acc_2_Z_MSB = receivedDataUARTBuffer[7];
 } 
 
 
 return error;

}
void initConfigurationSettings_Acc1(){
	if (config_error == false){
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart1, CONFIG_Mode_Configuration, 5, 200);
		}
		if (config_error == false){
			config_error = setRegisterMapPage_0(&huart1);
		}
		if (config_error == false){
			acc_1_X_LSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_X_LSB_ADDR);
			acc_1_X_MSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_X_MSB_ADDR);
			
			acc_1_Y_LSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_Y_LSB_ADDR);
			acc_1_Y_MSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_Y_MSB_ADDR);
			
			acc_1_Z_LSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_Z_LSB_ADDR);
			acc_1_Z_MSB_OFFSET = readSingleRegister(&huart1, ACCEL_OFFSET_Z_MSB_ADDR);
		}
		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart1, CONFIG_configurationPage_ID_1, 5, 200);
		}		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart1, CONFIG_accel_RangeSelection, 5, 200);
		}
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart1, CONFIG_configurationPage_ID_0, 5, 200);
		}				
		if (config_error == false){
			config_error = setRegisterMapPage_1(&huart1);
		}
		data = readSingleRegister(&huart1, 0x08);
		if (config_error == false){
			config_error = setRegisterMapPage_0(&huart1);
		}		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart1, CONFIG_Mode_Accelerometer_Only , 5, 200);
		}
	}
}
void initConfigurationSettings_Acc2(){
	if (config_error == false){
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_Mode_Configuration, 5, 200);
		}
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_configurationPage_ID_0, 5, 200);
		}
		if (config_error == false){
			acc_1_X_LSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_X_LSB_ADDR);
			acc_1_X_MSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_X_MSB_ADDR);
			
			acc_1_Y_LSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_Y_LSB_ADDR);
			acc_1_Y_MSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_Y_MSB_ADDR);
			
			acc_1_Z_LSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_Z_LSB_ADDR);
			acc_1_Z_MSB_OFFSET = readSingleRegister(&huart3, ACCEL_OFFSET_Z_MSB_ADDR);
		}
		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_configurationPage_ID_1, 5, 200);
		}		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_accel_RangeSelection, 5, 200);
		}
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_configurationPage_ID_0, 5, 200);
		}				
		if (config_error == false){
			config_error = setRegisterMapPage_1(&huart3);
		}
		data = readSingleRegister(&huart3, 0x08);
		if (config_error == false){
			config_error = setRegisterMapPage_0(&huart3);
		}		
		if (config_error == false){
			config_error = sendConfigurationSettings(&huart3, CONFIG_Mode_Accelerometer_Only , 5, 200);
		}		
	}
}	
bool setAccelerometerOffset(UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	bool hasXDataBeenReceived = false;
	bool hasYDataBeenReceived = false;
	bool hasZDataBeenReceived = false;
  bool error = false;
	bool hasAccelerometerBeenConfigured = false;
	HAL_Delay(2000);
	buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	receivedDataUART[0] = 0x00;
	receivedDataUART[1] = 0x00;
	
	//<BUTTON TOGGLE>
	while (hasAccelerometerBeenConfigured == false){
		TIME_FreeUse = 0;		
		HAL_Delay(500);
		
		while (hasXDataBeenReceived == false){
			buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			
			if (TIME_FreeUse >= 1000){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				TIME_FreeUse = 0;
			}
			
			getAccelerometerData(huart);
			accDataConversion();
			printAccelData();
			
			if (buttonState == 0){
				OFFSET_X_LSB = acc_1_X_LSB;
				OFFSET_X_MSB = acc_1_X_MSB;
				hasXDataBeenReceived = true;
			}			
		}
		HAL_Delay(500);

		while (hasYDataBeenReceived == false){
			buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			
			if (TIME_FreeUse >= 400){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				TIME_FreeUse = 0;
			}
			
			getAccelerometerData(huart);
			accDataConversion();
			printAccelData();			
			
			if (buttonState == 0){
				OFFSET_Y_LSB = acc_1_Y_LSB;
				OFFSET_Y_MSB = acc_1_Y_MSB;
				hasYDataBeenReceived = true;
			}		
		}
		HAL_Delay(500);
		
		while (hasZDataBeenReceived == false){
			buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			
			if (TIME_FreeUse >= 100){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				TIME_FreeUse = 0;
			}
			
			getAccelerometerData(huart);
			accDataConversion();
			printAccelData();		
			
			if (buttonState == 0){
				OFFSET_Z_LSB = acc_1_Z_LSB;
				OFFSET_Z_MSB = acc_1_Z_MSB;
				hasZDataBeenReceived = true;
			}		
		}	

		if (hasXDataBeenReceived == true && hasYDataBeenReceived == true && hasZDataBeenReceived == true){
			hasAccelerometerBeenConfigured = true;
		}
	
	}
	
	//</BUTTON TOGGLE>
	
	error = setRegisterMapPage_0(&huart1);
	error = setSingleRegister(&huart1, 0x00, OPR_MODE_ADDR);
	
	sendDataUARTBuffer[0] = UART_START_BYTE;
	sendDataUARTBuffer[1] = UART_WRITE;
	sendDataUARTBuffer[2] = ACCEL_OFFSET_X_LSB_ADDR;
	sendDataUARTBuffer[3] = 0x06;
	sendDataUARTBuffer[4] = OFFSET_X_LSB;
	sendDataUARTBuffer[5] = OFFSET_X_MSB;
	sendDataUARTBuffer[6] = OFFSET_Y_LSB;
	sendDataUARTBuffer[7] = OFFSET_Y_MSB;
	sendDataUARTBuffer[8] = OFFSET_Z_LSB;
	sendDataUARTBuffer[9] = OFFSET_Z_MSB;
	
	error = sendConfigurationSettings(huart, sendDataUARTBuffer, 10, 200);
	HAL_Delay(1000);
	error = sendConfigurationSettings(huart, CONFIG_Mode_Accelerometer_Only, 5, 200);
	HAL_Delay(1000);
	
	
	return error;
}










/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	initializeConfigurationVariables();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	//CONFIGURATION SETTINGS
	HAL_Delay(1000);
	initConfigurationSettings_Acc1();
	initConfigurationSettings_Acc2();
	HAL_Delay(500);
	
	
	//If there was an error during configuration, LD2 will be lit.
	if (config_error == true){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  howManyMilliSeconds = 0;
	howManyMilliSecondsTheButtonHasBeenHeld = 0;
	
	while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	//temperature  = getTemperature();
	accel_error  = getAccelerometerData(&huart1);
	if (accel_error == false){
		accel_error = getAccelerometerData(&huart3);
	}
		
	dataruined = 0;
	
	if (accel_error == false ){
		accDataConversion();
	}
  else {
		dataruined = 1;
	}

	buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	
	printAccelData();
	isitworking++;
  
	howManySeconds = (howManyMilliSeconds / 1000);
	
  howManySecondsTheButtonHasBeenHeld = 	howManyMilliSecondsTheButtonHasBeenHeld / 1000;
	
	if (howManySecondsTheButtonHasBeenHeld >= 5){		
		configureAccelerometerOffset_ERROR = setAccelerometerOffset(&huart1);	
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
