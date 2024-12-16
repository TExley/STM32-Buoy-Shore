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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "nrf24.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum transmit_size
{
	SIZE_REQUIRED_DATA_ONLY = 5,
	SIZE_SPECTRA_DATA_ONLY = 9,
	SIZE_VALIDATION_DATA = 10, // required + 5 validation parameters
	SIZE_SPECTRA_DATA = 13, // required data + other 8 spectra data
	SIZE_ALL_DATA = 18,
} transmit_size;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXIMUM_PRINT_TIMEOUT (uint32_t) 100 // Maximum time-out to wait for any print ACK
#define MAX_PRINT_LENGTH (uint32_t) 100 // Maximum lpuart1 serial data buffer length
#define MAX_PAYLOAD_PRINT_TIMEOUT (uint32_t) 60000 // Maximum time-out to wait without receiving data before printing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t NRF24_ADDR[] = { 'B', 'o', 'y', '0', '1' }; // The TX address
const uint8_t NRF24_ADDR_SIZE = 5; // Must in the range 3 - 5
const uint8_t NRF24_CHANNEL = 1; // Transmits on the frequency of 2400Mhz + this

const uint8_t NRF24_HEADER_CHECK = 0b11111111;
const uint8_t NRF24_BODY_CHECK = 0b0;
const uint8_t NRF24_CHECK_BIT = 3;

const char* data_names[SIZE_ALL_DATA] = {"Rhq\0", "Phih\0", "gamma2\0", "gamma3\0", "lambda\0", "r1\0", "a1\0", "r2\0", "a2\0", "C11m\0", "C22m\0", "C33m\0", "C23m\0", "Q12m\0", "C12m\0", "Q13m\0", "C13m\0", "Q23m\0"};
float** data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void serial_print(const char* buffer);
void print_data(uint8_t data_size, uint16_t sample_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[MAX_PRINT_LENGTH];
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	nRF24_HAL_Init(&hspi1);

	serial_print("\r\n\r\n\r\n\r\n\r\nNew Session\r\n");
	nRF24_Init();
	if (!nRF24_Check())
	{
		serial_print("Could not find nRF24.\r\n");
		Error_Handler();
	}
	serial_print("Initialized nRF24.\r\n");

	nRF24_SetRFChannel(NRF24_CHANNEL); // set RF channel to 2424MHz
	nRF24_SetDataRate(nRF24_DR_1Mbps); // 2Mbit/s data rate
	nRF24_SetCRCScheme(nRF24_CRC_2byte); // 1-byte CRC scheme
	nRF24_SetAddrWidth(NRF24_ADDR_SIZE); // address width is 5 bytes
	nRF24_SetAddr(nRF24_PIPE1, NRF24_ADDR); // program pipe address
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32); // enable RX pipe#1 with Auto-ACK: enabled, payload length: 10 bytes
	nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power for Auto-ACK, good choice - same power level as on transmitter
	nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
	nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t nRF24_payload[32]; // buffer for payload
	uint8_t payload_length; // variable to store a length of received payload

	uint16_t sample_size;
	transmit_size data_size;

	uint32_t data_col_start, data_col_end, data_proc_end, transmit_start_time;
	bool transmitting = false;

	//uint32_t *bits = (uint32_t *) &(data_outf[i][n + j]);
	//sprintf(str, "%hu %hu %hu\r\n", nRF24_payload[0], nRF24_payload[1], nRF24_payload[2]);
	//serial_print(str);
	// NRF24_HEADER_CHECK
	nRF24_ClearIRQFlags(); // clear any pending IRQ bits
	nRF24_CE_H; // start receiving
	while (1) {
    	//
    	// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
    	//
    	// This is far from best solution, but it's ok for testing purposes
    	// More smart way is to use the IRQ pin :)
    	//
    	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
    		// Get a payload from the transceiver
    		nRF24_ReadPayload(nRF24_payload, &payload_length);

    		// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();

			if (nRF24_payload[NRF24_CHECK_BIT] == NRF24_BODY_CHECK && transmitting) // 3
			{
				uint8_t payload_size = nRF24_payload[0] & 0b00000111;
				uint16_t start_ind;
				memcpy(&start_ind, nRF24_payload + 1, sizeof(uint16_t)); // 1, 2

				if (start_ind + payload_size > sample_size) // Ignore packet with oob data
					continue;

				uint8_t data_ind = nRF24_payload[0] & 0b11111000 >> 3;
				memcpy(data[data_ind] + start_ind, nRF24_payload + sizeof(uint32_t), payload_size); // 4 -> 31

				if (start_ind + payload_size == sample_size)
				{
					sprintf(str, "Transmition %d%% complete.", (uint16_t) (data_ind + 1) * 100 / data_size);
					if (data_ind + 1 == data_size) // If transmission is done
					{
						print_data(data_size, sample_size);
						transmitting = false;
					}
				}

			} else if (nRF24_payload[NRF24_CHECK_BIT] == NRF24_HEADER_CHECK) // 3
			{
				if (transmitting)
				{
					print_data(data_size, sample_size);
					transmitting = false;
				}

				data_size = nRF24_payload[0]; // 0
				if (!(data_size == SIZE_REQUIRED_DATA_ONLY || data_size == SIZE_SPECTRA_DATA_ONLY
					|| data_size == SIZE_VALIDATION_DATA || data_size == SIZE_SPECTRA_DATA || data_size == SIZE_ALL_DATA))
					continue; // Ingore if invalid size in header

				memcpy(&sample_size, nRF24_payload + sizeof(uint8_t), sizeof(uint16_t)); // 1, 2
				memcpy(&data_col_start, nRF24_payload + sizeof(uint32_t), sizeof(uint32_t)); // 4, 5, 6, 7
				memcpy(&data_col_end, nRF24_payload + sizeof(uint32_t) * 2, sizeof(uint32_t)); // 8, 9, 10, 11
				memcpy(&data_proc_end, nRF24_payload + sizeof(uint32_t) * 3, sizeof(uint32_t)); // 12, 13, 14, 15
				// 16 -> 31 bits unused in header

				data = (float**) malloc(sizeof(float) * data_size);
				for (int i = 0; i < data_size; i++)
					data[i] = (float*) calloc(sizeof(float), sample_size);

				serial_print("Receiving data from Bouy01.\r\n");
				sprintf(str, "Receiving %u arrays of length %u.\r\n", data_size, sample_size);
				serial_print(str);

				transmit_start_time = HAL_GetTick();
				transmitting = true;
			}
			// If checkbit is wrong we discard the packet
    	} else if (transmitting && HAL_GetTick() - transmit_start_time >= MAX_PAYLOAD_PRINT_TIMEOUT)
    	{
    		print_data(data_size, sample_size);
    		transmitting = false;
    	}

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|nRF24_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : nRF24_CSN_Pin */
  GPIO_InitStruct.Pin = nRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF24_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF24_CE_Pin */
  GPIO_InitStruct.Pin = nRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF24_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF24_IRQ_Pin */
  GPIO_InitStruct.Pin = nRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void serial_print(const char* buffer)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), MAXIMUM_PRINT_TIMEOUT);
}

void print_data(uint8_t data_size, uint16_t sample_size)
{
	uint8_t data_name_offset = (data_size == SIZE_VALIDATION_DATA || data_size == SIZE_ALL_DATA)
		? 0 : SIZE_VALIDATION_DATA - SIZE_REQUIRED_DATA_ONLY;

	for (int i = 0; i < data_size; i++)
	{
		sprintf(str, "%s, ", data_names[data_name_offset + i]);
		serial_print(str);
	}
	serial_print("\r\n");

	for (int i = 0; i < sample_size; i++)
	{
		for (int j = 0; j < data_size; j++)
		{
			sprintf(str, "%f, ", data[j][i]);
			serial_print(str);
		}
		serial_print("\r\n");
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
