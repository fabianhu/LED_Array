/*
 * led_array.h
 *
 *  Created on: 09.05.2017
 *      Author: fabian
 */

#ifndef LED_ARRAY_H_
#define LED_ARRAY_H_

typedef uint8_t framebuf_t [8][3*WS2812B_NUMBER_OF_LEDS];

void LED_setPixel(framebuf_t buffer, int x, int y, uint8_t r, uint8_t g, uint8_t b);
void LED_setRainbowPixel(framebuf_t buffer, int x, int y, uint8_t r, uint8_t g, uint8_t b);
int LED_writeChar(framebuf_t buffer, char c, int x, uint8_t r, uint8_t g, uint8_t b);
int LED_writeText_int(framebuf_t buffer, char* text, int x, uint8_t r, uint8_t g, uint8_t b);
void LED_writeText(char* text, uint8_t r, uint8_t g, uint8_t b);
void LED_runText(char* text, uint8_t r, uint8_t g, uint8_t b);
void LED_start(void);
void LED_fill(uint8_t r, uint8_t g, uint8_t b);
void LED_setBlk(int x, int l, uint8_t r, uint8_t g, uint8_t b);
void LED_Init(void);
void LED_clear(void);




#endif /* LED_ARRAY_H_ */
