/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ws2812b/ws2812b.h"
#include "led_array.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->RxXferSize > 5)
	{
		//

	}
}

extern volatile int lastRXtck;

volatile int tck=0;

void HAL_SYSTICK_Callback(void)
{
	tck++;
}


/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */

	uint8_t rxbuffer[1000] = {0};

	  uint8_t Nachricht[] = {"BlaBlubb"};



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */

  LED_Init();

  LED_fill(0,0,0);
  //setPixel(0,0,30,30,30);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  LED_clear();
	  LED_start();

	  HAL_Delay(100);
	  //LED_writeText("XgXgXgXGXGXGXG",0,255,0,0);

//	  LED_setBlk(0,72,20,20,20);
//	  LED_start();
//	  HAL_Delay(500);
//	  LED_setBlk(0,72,50,0,0);
//	  LED_start();
//	  HAL_Delay(500);
//	  LED_setBlk(0,72,0,50,0);
//	  LED_start();
//	  HAL_Delay(500);
//	  LED_setBlk(0,72,0,0,50);
//	  LED_start();
//	  HAL_Delay(500);

//
//	  LED_clear();
//	  LED_start();

	  //HAL_UART_Receive_IT(&huart1,rxbuffer,5);



	  HAL_UART_Transmit_IT(&huart1, Nachricht, sizeof(Nachricht)); //Überträgt eine Nachricht per USART
	  HAL_UART_Receive_IT(&huart1, rxbuffer, 100); //Empfängt eine Nachricht per USART
	  if (tck-lastRXtck < 100) // ist was gekommen
	  {
		  //machen

	  }

//	  HAL_Delay(500);
	  LED_runText("Franz jagt im komplett verwahrlosten Auto Quer durch Bayern.1einself",128,0,50);
//	  char t[] = {'H','a','l','l','o',' ','J',96+31,'r','g','!'};
//	  LED_runText(t,255,0,0);
//	  LED_writeChar('T',72-7,255,0,0);
	  HAL_Delay(2000);
//
//	  LED_writeChar('T',72-14,0,255,0);
//	  HAL_Delay(1000);
//
//	  LED_writeChar('T',72-21,0,0,255);
//	  HAL_Delay(3000);

	  LED_writeText("Franz jagt im komplett verwahrlosten Auto Quer durch Bayern.1einself",128,0,50);
	  LED_start();
	  HAL_Delay(3000);
//	  clearArr();
	  //LED_writeText("ABCDEFG",0,0,0,255);
//	  if(do_it) visHandle();
//	  HAL_Delay(500);
//
//	  for (int i =1; i< 32;i++)
//	  {
//		  setBlk(i-1,1,0,0,0);
//		  setBlk(i,1,100,0,0);
//		  visHandle();
//		  HAL_Delay(20);
//	  }
//	  for (int i =32; i>0;i--)
//	  {
//		  setBlk(i,1,0,0,0);
//		  setBlk(i+1,1,100,0,0);
//		  visHandle();
//		  HAL_Delay(20);
//	  }
//	  setLBuffer(100,0,0);
//	  visHandle();
//	  HAL_Delay(500);
//	  setLBuffer(0,100,0);
//	  visHandle();
//	  HAL_Delay(500);
//	  setLBuffer(0,0,100);
//	  visHandle();
//	  HAL_Delay(500);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
