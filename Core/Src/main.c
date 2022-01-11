/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include "../lib/uart.h"
#include "../lib/addresses.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t flag = BOOTLOADER; //результат прохождения этапов прошивке

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("rcc->csr = %lu\n", RCC->CSR);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    if (*(__IO uint32_t*)BOOTLOADER_FLAG_ADDRESS == BOOTLOADER) { //если в памяти 'BOOTLOADER', то начинаем процесс прошивки 08018C00
    	uint8_t value[46]; //значение, считанное с uart
    	uint8_t dataLength = dataCreate((char*)value, BOOTLOADER, 0); //готовим данные для отмашки к началу передачи прошивки от esp
    	//	  uint8_t lineLength;
    	//		  uint8_t dataSense;
		size_t	currentAddress = 0x08000000, //адрес, в который пишем сейчас. нужен главным образом для хранения расширенного адреса
			currentMinAddress = 0x0, //адрес начала блока стертой памяти (в которую сейчас пишем)
			currentMaxAddress = 0x0; //конец блока стертой памяти
    	HEXData hex; //структура для расшифрованного пакета c hex-данных
    	union { //структура для записи данных в память (для HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data))
    		uint8_t *buf8;
    		uint32_t* buf32;
    	} hexBuf;
    	hexBuf.buf8 = hex.buf; //вносим адрес буфера с данными
    	static uint32_t PAGEError; // = 0xFFFFFFFF;
    	FLASH_EraseInitTypeDef EraseInitStruct; //структура для стирания блока памяти
    	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; //стираем постранично
    	EraseInitStruct.NbPages = 1; //по одной странице за раз
    	HAL_FLASH_Unlock();
    	EraseInitStruct.PageAddress = currentMinAddress; //стираем первую страницу
    	if (HAL_UART_Transmit(&huart1, value, dataLength, 200) == HAL_OK) { //отправляем запрос на начало передачи прошивки
    		uint8_t counter = 0x5;
        	while ((HAL_GPIO_ReadPin(GPIOC, B1_Pin) != GPIO_PIN_RESET) //ждем нажатия кнопки, мигая диодом
        			&& (--counter)) {
        		printf("counter");
        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        		HAL_Delay(1000);
        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        		HAL_Delay(200);
        	}
    		printf("\n end counter");
        	if (counter) { //если нажата кнопка, мигаем и переходим к прошивке
       			counter = 0xf;
       			for (int i = 0; i < 5; i++) {
       	    		printf("\ncounter 2");
       				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
           			HAL_Delay(50);
           			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
           			HAL_Delay(10);
       			}
       			flag = BOOTLOADER;
        	} else {
        		flag = FLASH_DONE;
        	}
    		printf("\n end counter 2");

    		while ((HAL_UART_Receive(&huart1, value, 1, 10000) == HAL_OK) //читаем первый байт пакета
    				&& (--counter)
					&& (flag != FLASH_DONE)
					&& (flag != CRITICAL_ERROR)) {
    			//printf("\nwhile ((HAL_UART_Receive(&huart1, value, 1, 2000) == HAL_OK)");
    			if (value[0] == ':') { //ищем начало hex-строки
    				//printf("\nif (value[0] == ':')");
    				if (HAL_UART_Receive(&huart1, &value[1], 2, 10000) == HAL_OK) {//читаем длину hex-строки
    					//printf("\nif (HAL_UART_Receive(&huart1, &value[1], 2, 2000) == HAL_OK) {//читаем длину hex-строки");
    					dataLength = chars2int((char*)&value[1]); //if ((dataLength = chars2int((char*)&value[1])) != DATA_ERROR) {
        					//printf("\nif ((dataLength = chars2int((char*)&value[1])) != DATA_ERROR) {");
    						if (HAL_UART_Receive(&huart1, &value[3], ((dataLength << 1) + 8), 10000) == HAL_OK) { //читаем hex-строку
    							//printf("\nif (HAL_UART_Receive(&huart1, &value[3], ((dataLength << 1) + 8), 2000) == HAL_OK) { //читаем hex-строку");
    							if (dataHEXDecrypt32BitBuf(&hex, (char*)value, dataLength) != DATA_ERROR) { //далее исходим из уверенности, что данные доставлены без повреждений
    	    						//printf("\nif (dataHEXDecrypt(&hex, (char*)value, dataLength) != DATA_ERROR) {");
    	    						switch (hex.dataType) {
    	    						case 0 : { //пишем данные
    	    							//printf("\ncase 0 :\n");
    	    							//for (int i=0; i<dataLength; i++) printf("%x", hex.buf[i]);
    	    							//printf("\n");
    	    							currentAddress = (currentAddress & 0xffff0000) + hex.address;
    	    							//printf("\ncurrentAddress=%x; currentAddress + dataLength=%x; upA=%x; ", currentAddress, (currentAddress + dataLength), upperAddress);
    	    							if ((currentAddress >= FIRMWARE_START_ADDRESS) && ((currentAddress + dataLength) < UPPER_ADDRESS)) { //проверяем лежит ли полученный пакет в пределах допустимой для записи памяти
    	    								//printf("\ncurrA=%x; currentMinAddress=%x; currentMaxAddress=%x; ", currentAddress, currentMinAddress, currentMaxAddress);
    	    								if (currentAddress < currentMinAddress) {//|| (currentAddress > currentMaxAddress)) { //данные идут не последовательно
    	    									printf("--------error------------ if ((currentAddress < currentMinAddress)) {");
    	    									flag = CRITICAL_ERROR;
    	    									break;
    										} else {
    											if (currentAddress > currentMaxAddress) {
        											currentMinAddress = ((currentAddress >> 10) << 10); //адрес страницы для стирания - содержит currentAddress
        											currentMaxAddress = currentMinAddress + PAGE_SIZE;
        											EraseInitStruct.PageAddress = currentMinAddress; //стираем страницу
        											printf("\n!!!HAL_FLASHEx_Erase currA=%x; currentMinAddress=%x; currentMaxAddress=%x; ", currentAddress, currentMinAddress, currentMaxAddress);
        											if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
        												flag = CRITICAL_ERROR;
        												break;
        											}
    											}
    											if ((currentAddress + dataLength) >= currentMaxAddress) { //выходят данные для записи за пределы стертой страницы?
    												currentMinAddress = ((currentAddress >> 10) + 1) << 10; //адрес страницы для стирания - содержит (currentAddress + dataLength)
    												currentMaxAddress = currentMinAddress + PAGE_SIZE;
    												EraseInitStruct.PageAddress = currentMinAddress; //стираем страницу
    												printf("\n***HAL_FLASHEx_Erase currA=%x; currentMinAddress=%x; currentMaxAddress=%x; ", currentAddress, currentMinAddress, currentMaxAddress);
    												if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
    													flag = CRITICAL_ERROR;
    													break;
    												}
    											}
    										}
    	    								//если дошди до этого места без ошибок, начинаем запись
    	    								for (uint8_t i = 0; i < hex.dataSize; i+=4) {
    	    									if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (currentAddress + i), *hexBuf.buf32) != HAL_OK) {
    	    										flag = CRITICAL_ERROR;
    									  			break;
    									  		}
    	    									hexBuf.buf32++;
    	    								}
    	    								//printf("\nHAL_FLASH_Program from=%x to=%x; size=%d;", currentAddress, currentAddress+dataLength, dataLength);
        	    			        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        	    			        		HAL_Delay(10);
        	    			        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    	    								counter = 0xf;
	    	    							flag = DATA_OK;
  		    	    						hexBuf.buf8 = hex.buf;
    	    							} else { //if ((currentAddress >= lowerAddress) && ((currentAddress + dataLength) < upperAddress)) { //проверяем лежит ли полученный пакет в пределах допустимой для записи памяти
    	    								flag = CRITICAL_ERROR;
    	    							}
    	    							break;
    	    						} //case 0 : { //пишем данные
    	    						case 4 : { //пишем расширенный адрес
    	    							currentAddress = hex.buf[0];
    	    							currentAddress <<= 8;
    	    							currentAddress += hex.buf[1];
    	    							currentAddress <<= 16;
    	    							printf("\ncase : 4; currA = %x", currentAddress);
    	    							counter = 0xf;
    	    							flag = DATA_OK;
    	    							break;
    	    						}
    	    						case 1 : { //конец файла, завершаем запись
    	    							printf("\ncase 1 :");
										EraseInitStruct.PageAddress = BOOTLOADER_FLAG_ADDRESS; //стираем страницу
										printf("\n***HAL_FLASHEx_Erase currA=%x; currentMinAddress=%x; currentMaxAddress=%x; ", currentAddress, currentMinAddress, currentMaxAddress);
										if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
											flag = CRITICAL_ERROR;
											break;
										}
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						        		HAL_Delay(2000);
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
						        		HAL_Delay(1000);
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						        		HAL_Delay(2000);
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
						        		HAL_Delay(1000);
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
						        		HAL_Delay(2000);
						        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    	    							flag = FLASH_DONE;
    	    							break;
    	    						}
    	    						case 5 : {
    	    							printf("\ncase : 5");
    	    							counter = 0xf;
    	    							flag = DATA_OK;
    	    							break;
    	    						}
    	    						} //switch (hex.dataType) {
    							} else { //if (dataHEXDecrypt32BitBuf(&hex, (char*)value, dataLength) != DATA_ERROR) { //далее исходим из уверенности, что данные доставлены без повреждений
    	    						printf("\n---error--- if (dataHEXDecrypt(&hex, (char*)value, dataLength) != DATA_ERROR) {");
    	    						flag = DATA_ERROR;
    							}
    						} else { //if (HAL_UART_Receive(&huart1, &value[3], ((dataLength << 1) + 8), 10000) == HAL_OK) { //читаем hex-строку
        						printf("\n---error--- if (HAL_UART_Receive(&huart1, &value[3], ((dataLength << 1) + 8), 2000) == HAL_OK) { //читаем hex-строку");
        						flag = DATA_ERROR;
    						}
    				} else { //if (HAL_UART_Receive(&huart1, &value[1], 2, 2000) == HAL_OK) {//читаем длину hex-строки
						printf("\n---error--- if (HAL_UART_Receive(&huart1, &value[1], 2, 2000) == HAL_OK) {//читаем длину hex-строки");
    					flag = DATA_ERROR;
    				}
    				//printf("\nsend flag = %u\n", flag);
					//for (int i=0; i<dataLength; i++) printf("%x", hex.buf[i]);
					//printf("\n");
    				dataLength = dataCreate((char*)value, flag, 0);
    				HAL_UART_Transmit(&huart1, value, dataLength, 200);
    			} //if (value[0] == ':') { //ищем начало hex-строки
    		} //while ((HAL_UART_Receive(&huart1, value, 1, 10000) == HAL_OK) //читаем первый байт пакета
    	} //if (HAL_UART_Transmit(&huart1, value, dataLength, 200) == HAL_OK) { //отправляем запрос на начало передачи прошивки
    	HAL_FLASH_Lock();
    } //if (*(__IO uint32_t*)0x08018C00 == 9) { //если в памяти '9', то начинаем процесс прошивки
	HAL_Delay(1000);
    if (flag != CRITICAL_ERROR) {
    	void (*GoToFirmware)(void);
		GoToFirmware = (void (*)(void)) *((volatile uint32_t*)(FIRMWARE_START_ADDRESS + 4));
		HAL_DeInit();
		__disable_irq();
		__set_MSP(*((volatile uint32_t*)FIRMWARE_START_ADDRESS));
		GoToFirmware();
	}


  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 1200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len) {
	//int i = 1;
	for (int i = 0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
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
