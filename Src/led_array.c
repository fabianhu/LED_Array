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

typedef struct RgbColor
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RgbColor;

typedef struct HsvColor
{
	uint8_t h;
	uint8_t s;
	uint8_t v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv);
HsvColor RgbToHsv(RgbColor rgb);


uint8_t bk_r = 0;
uint8_t bk_g = 0;
uint8_t bk_b = 0;

// define array of Pixel-Data
// RGB Frame buffers 8 buffers for 8 lines with 72 LEDs each.

framebuf_t frameBuffer;

void LED_writeText(char* text, uint8_t r, uint8_t g, uint8_t b)
{

	LED_writeText_int(frameBuffer,text,0,r,g,b);
}


// return pixel length
int LED_writeText_int(framebuf_t buffer, char* text, int x, uint8_t r, uint8_t g, uint8_t b)
{
	int z=x;
	char* ptr = text;
	do
	{
		z += LED_writeChar(buffer,*ptr, z, r, g, b);
		ptr++;
	}
	while( *ptr !=0);
	return z;
}

void LED_runText(char* text, uint8_t r, uint8_t g, uint8_t b)
{
	int pxlen= LED_writeText_int(NULL,text,0,0,0,0); // just get pixel length of text
	LED_fill(0,0,0); // erase again !

	for (int i = WS2812B_NUMBER_OF_LEDS; i>-pxlen;i-- )
	{
		LED_clear();
		LED_writeText_int(frameBuffer,text,i,r,g,b);
		LED_start();
		HAL_Delay(1000/60);
	}
}

// return x position
int LED_writeChar(framebuf_t buffer, char c, int x, uint8_t r, uint8_t g, uint8_t b)
{
	int idx = c-FONTOFFSET;

	if (idx < 0)
		return 0; // fixme

	int i = 0;
	for (i = 0; i < FONTWIDTH; i++)
	{

		uint8_t col = font[idx][i];

		if(col == 0) break;

		for (int j = 0; j < 8; j++)
		{
			if(col & (0x01 << j)) // down is left
				LED_setPixel(buffer, x+i,7-j,r,g,b); // upside down
			else
				LED_setPixel(buffer, x+i,7-j,bk_r,bk_g,bk_b); // upside down
		}

	}
	for (int j = 0; j < 8; j++) // tailing space
	{
			LED_setPixel(buffer, x+i,7-j,bk_r,bk_g,bk_b);
	}
	return i+1;
}



void LED_setPixel(framebuf_t buffer, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if(buffer == NULL) return;
	if( x < 0 || x > WS2812B_NUMBER_OF_LEDS-1) return;
	if( y < 0 || y > 7) return;

	buffer[y][x*3]   = r;
	buffer[y][x*3+1] = g;
	buffer[y][x*3+2] = b;
}

void LED_setRainbowPixel(uint8_t *buffer[], int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if( x < 0 || x > WS2812B_NUMBER_OF_LEDS-1) return;
	if( y < 0 || y > 7) return;

	HsvColor hsv;
	RgbColor rgb;

	rgb.r = r;
	rgb.g = g;
	rgb.b = b;

	hsv = RgbToHsv(rgb);
	hsv.h = x*10;
	rgb = HsvToRgb(hsv);

	r= rgb.r;
	g= rgb.g;
	b= rgb.b;


	buffer[y][x*3]   = r;
	buffer[y][x*3+1] = g;
	buffer[y][x*3+2] = b;
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


//void LED_setBlk(int x, int l, uint8_t r, uint8_t g, uint8_t b)
//{
//	for (int i=x; i < (x+l); i++)
//	{
//		for (int y=0; y < 8; y++)
//			{
//				LED_setPixel(i,y,r,g,b);
//			}
//	}
//}



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




RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    uint8_t region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    uint8_t rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * (int)(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}
