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
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "ws2812b/ws2812b.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// define array of Pixel-Data
// RGB Frame buffers 8 buffers for 8 lines with 72 LEDs each.
uint8_t frameBuffer[8][3*WS2812B_NUMBER_OF_LEDS];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
int writeChar(char c, uint8_t x, uint8_t r, uint8_t g, uint8_t b);
void writeText(char* text, uint8_t r, uint8_t g, uint8_t b);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


#include "fonts/homespun_font.h"
#define FONTWIDTH 7
#define FONTOFFSET 32


void writeText(char* text, uint8_t r, uint8_t g, uint8_t b)
{
	char* ptr = text;
	int x = 0;
	do
	{
		x += writeChar(*ptr, x, r, g, b);
		x++; // spacing between chars
		ptr++;
	}
	while( *ptr !=0);
}

// return x position
int writeChar(char c, uint8_t x, uint8_t r, uint8_t g, uint8_t b)
{
	int idx = c-FONTOFFSET;
	if (idx < 0)
		return x;
	int n = 0;
	for (int i = 0; i < FONTWIDTH; i++)
	{
		uint8_t xi = x+i;
		uint8_t col = font[idx][i];

		if( col != 0)
		{
			n ++; // increment n for every used column to determine width of letter

			for (int j = 0; j < 8; j++)
			{
				if(col & (0x01 << j)) // down is left
					//setPixel(x+i,j,r,g,b);
					setPixel(xi,7-j,r,g,b); // upside down
			}
		}
	}

	return n;
}



void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
	frameBuffer[y][x*3]   = r;
	frameBuffer[y][x*3+1] = g;
	frameBuffer[y][x*3+2] = b;
}

void setLBuffer(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t* ptr = &frameBuffer[0][0];
	for (int i=0; i < sizeof (frameBuffer); i+=3)
	{
		ptr[i]   = r;
		ptr[i+1] = g;
		ptr[i+2] = b;
	}
}

void clearArr(void)
{
	setLBuffer(0,0,0);
}


void setBlk(uint8_t x, uint8_t l, uint8_t r, uint8_t g, uint8_t b)
{
	for (int i=x; i < (x+l); i++)
	{
		for (int y=0; y < 8; y++)
			{
				setPixel(i,y,r,g,b);
			}
	}
}



void visHandle()
{

	if(ws2812b.transferComplete)
	{
		// Update your framebuffer here or swap buffers


		// Signal that buffer is changed and transfer new data
		ws2812b.startTransfer = 1;
		ws2812b_handle();
	}
}

void visInit()
{
	for(int i = 0; i< WS2812_BUFFER_COUNT; i++)
	{
		// Set output channel/pin, GPIO_PIN_0 = 0, for GPIO_PIN_5 = 5 - this has to correspond to WS2812B_PINS
		ws2812b.item[i].channel = i;
		// Your RGB frame buffer
		ws2812b.item[i].frameBufferPointer = frameBuffer[i];
		// RAW size of frame buffer
		ws2812b.item[i].frameBufferSize = sizeof(frameBuffer[i]);
	}

	ws2812b_init();
}


volatile int32_t do_it = 1;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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

  visInit();

  setLBuffer(0,0,0);
  //setPixel(0,0,30,30,30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  clearArr();
	  writeText("Hi",255,0,0);
	  if(do_it) visHandle();
	  HAL_Delay(500);
	  clearArr();
	  writeText("Joerg",0,0,255);
	  if(do_it) visHandle();
	  HAL_Delay(500);

	  for (int i =1; i< 32;i++)
	  {
		  setBlk(i-1,1,0,0,0);
		  setBlk(i,1,100,0,0);
		  visHandle();
		  HAL_Delay(20);
	  }
	  for (int i =32; i>0;i--)
	  {
		  setBlk(i,1,0,0,0);
		  setBlk(i+1,1,100,0,0);
		  visHandle();
		  HAL_Delay(20);
	  }
	  setLBuffer(100,0,0);
	  visHandle();
	  HAL_Delay(1500);
	  setLBuffer(0,100,0);
	  visHandle();
	  HAL_Delay(1500);
	  setLBuffer(0,0,100);
	  visHandle();
	  HAL_Delay(1500);

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
