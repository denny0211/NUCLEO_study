/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void move(char *Data);
void BlutoothOutput();
void USBOutput();

volatile uint8_t rx2ch;
volatile uint8_t rx3ch;
volatile char USBInPutData[50];
volatile char BlutoothInPutData[50];

volatile int OutPutSW = 0;

volatile int USSIN = 0;

volatile int Rspeed = 0, Lspeed = 0;

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM7_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_UART_Receive_IT(&huart3, (uint8_t*) &rx3ch, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx2ch, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	BlutoothOutput();
	USBOutput();
	while (1) {
		if (OutPutSW > 0) {
			if (OutPutSW == 1) move(BlutoothInPutData); //ex)R500L600
			else if (OutPutSW == 2) move(USBInPutData);

			BlutoothOutput();
			USBOutput();
			OutPutSW = 0;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void BlutoothOutput() {
	char temp[50] = { 0, };
	int i = 0;
	if (OutPutSW == 0)
		sprintf(temp, "%start main\r\n");
	else if (OutPutSW == 1)
		sprintf(temp, "Blutooth In Put Motor Data - R:%d, L:%d\r\n", Rspeed,
				Lspeed);
	else if (OutPutSW == 2)
		sprintf(temp, "USB In Put Motor Data - R:%d, L:%d\r\n", Rspeed, Lspeed);
	else if (OutPutSW == 3)
		sprintf(temp, "%d cm\r\n", USSIN);

	while (temp[i] != '\0') {
		HAL_UART_Transmit(&huart2, &temp[i], 1, 10);
		i++;
	}
}
void USBOutput() {
	char temp[50] = { 0, };
	int i = 0;
	if (OutPutSW == 0)
		sprintf(temp, "%start main2\r\n");
	else if (OutPutSW == 1)
		sprintf(temp, "Blutooth In Put Motor Data - R:%d, L:%d\r\n", Rspeed,
				Lspeed);
	else if (OutPutSW == 2)
		sprintf(temp, "USB In Put Motor Data - R:%d, L:%d\r\n", Rspeed, Lspeed);
	else if (OutPutSW == 3)
		sprintf(temp, "%d cm\r\n", USSIN);

	while (temp[i] != '\0') {
		HAL_UART_Transmit(&huart3, &temp[i], 1, 10);
		i++;
	}
}

void move(char *Data) {

	char speedsw = '\0';
	int L = 0, R = 0;
	char CRspeed[5] = { '\0', };
	char CLspeed[5] = { '\0', };

	for (int s = 0; s < 10; s++) {

		if (Data[s] == 'L') {
			speedsw = 'L';
			s++;
		} else if (Data[s] == 'R') {
			speedsw = 'R';
			s++;
		} else if (Data[s] == '\0') {
			speedsw = '\0';
		}

		if (speedsw == 'L')
			CLspeed[L++] = Data[s];
		else if (speedsw == 'R')
			CRspeed[R++] = Data[s];
	}

	TIM3->CCR4 = TIM3->CCR3 = TIM4->CCR4 = TIM4->CCR3 = 0;

	(R) ? (Rspeed = atoi(CRspeed)) : (Rspeed = 0);
	(L) ? (Lspeed = atoi(CLspeed)) : (Lspeed = 0);

	(Rspeed > 0) ? (TIM3->CCR3 = Rspeed) : (TIM3->CCR4 = -Rspeed);
	(Lspeed > 0) ? (TIM4->CCR3 = Lspeed) : (TIM4->CCR4 = -Lspeed);

	memset(USBInPutData,0,50);
	memset(BlutoothInPutData,0,50);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static int U2 = 0;
	static int U3 = 0;
	BlutoothInPutData[U2] = rx2ch;
	USBInPutData[U3] = rx3ch;

	if (huart->Instance == USART2) {
		if ((BlutoothInPutData[U2] == '\n')
				|| (BlutoothInPutData[U2] == '\r')) {
			BlutoothInPutData[U2] = '\0';
			OutPutSW = 1;
			U2 = 0;
		} else {
			U2++;
		}
	}

	if (huart->Instance == USART3) {
		if ((USBInPutData[U3] == '\n') || (USBInPutData[U3] == '\r')) {
			USBInPutData[U3] = '\0';
			OutPutSW = 2;
			U3 = 0;
		} else {
			U3++;
		}
	}

	HAL_UART_Receive_IT(&huart3, &rx3ch, 1);
	HAL_UART_Receive_IT(&huart2, &rx2ch, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//Ultra Sonic Sensor -> USS 초음?��?��?���?????? ?��?���?????? ?�� 길어?�� ?���???????��만�?
	if (htim->Instance == TIM7) {

		static int TimeSW = 5;
		static int long USSOutTime = 0;
		static int long USSInTime = 0;

		if (TimeSW == 0) {
			if (USSOutTime < 5) {
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 1);
			} else {
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, 0);
				TimeSW = 1;
			}
		} else if (TimeSW == 1) {
			if (USSOutTime > 100000) {
				USSIN = USSInTime * 17 / 1000;
				if (USSIN < 7) {
					OutPutSW = 3;
					TIM3->CCR4 = TIM3->CCR3 = TIM4->CCR4 = TIM4->CCR3 = 0;
				}
				USSInTime = USSOutTime = 0;
				TimeSW = 0;
			}
		}
		USSOutTime++;

		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0))
			USSInTime++;

	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
