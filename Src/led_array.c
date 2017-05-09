/*
 * led_array.c
 *
 *  Created on: 09.05.2017
 *      Author: fabian
 */

#include "stm32f1xx_hal.h"
#include "ws2812b/ws2812b.h"
#include "fonts/homespun_font.h"
#include "led_array.h"

#define FONTWIDTH 7
#define FONTOFFSET 32

// define array of Pixel-Data
// RGB Frame buffers 8 buffers for 8 lines with 72 LEDs each.
uint8_t frameBuffer[8][3*WS2812B_NUMBER_OF_LEDS];

// return pixel length
int LED_writeText(char* text, int x, uint8_t r, uint8_t g, uint8_t b)
{
	char* ptr = text;
	do
	{
		x += LED_writeChar(*ptr, x, r, g, b);

		ptr++;
	}
	while( *ptr !=0);
	return x;
}

void LED_runText(char* text, uint8_t r, uint8_t g, uint8_t b)
{
	int pxlen= LED_writeText(text,0,r,g,b); // just get pixel length of text
	LED_fill(0,0,0); // erase again !

	for (int i = WS2812B_NUMBER_OF_LEDS; i>-pxlen;i-- )
	{
		LED_writeText(text,i,r,g,b);
		LED_start();
		HAL_Delay(60);
	}
}

// return x position
int LED_writeChar(char c, int x, uint8_t r, uint8_t g, uint8_t b)
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
			//n ++; // increment n for every used column to determine width of letter
			n = i+1; // increment n for every used column to determine width of letter
		}
		for (int j = 0; j < 8; j++)
		{
			if(col & (0x01 << j)) // down is left
				//setPixel(x+i,j,r,g,b);
				LED_setPixel(xi,7-j,r,g,b); // upside down
			else
				LED_setPixel(xi,7-j,10,10,10); // upside down
		}
	}
	for (int j = 0; j < 8; j++) // tailing space
	{
			LED_setPixel(x+n,7-j,0,20,0);
	}
	//visHandle();
	//HAL_Delay(5);
	return n+1;
}



void LED_setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if( x < 0 || x > WS2812B_NUMBER_OF_LEDS-1) return;
	if( y < 0 || y > 7) return;
	frameBuffer[y][x*3]   = r;
	frameBuffer[y][x*3+1] = g;
	frameBuffer[y][x*3+2] = b;
}

void LED_fill(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t* ptr = &frameBuffer[0][0];
	for (int i=0; i < sizeof (frameBuffer); i+=3)
	{
		ptr[i]   = r;
		ptr[i+1] = g;
		ptr[i+2] = b;
	}
}

void LED_clear(void)
{
	LED_fill(0,0,0);
}


void LED_setBlk(int x, int l, uint8_t r, uint8_t g, uint8_t b)
{
	for (int i=x; i < (x+l); i++)
	{
		for (int y=0; y < 8; y++)
			{
				LED_setPixel(i,y,r,g,b);
			}
	}
}



void LED_start(void)
{

	if(ws2812b.transferComplete)
	{
		// Update your framebuffer here or swap buffers


		// Signal that buffer is changed and transfer new data
		ws2812b.startTransfer = 1;
		ws2812b_handle();
	}
}

void LED_Init()
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