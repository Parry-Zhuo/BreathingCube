/**
 * @file Ws212bDriver.h
 * @brief Driver for controlling WS2812B LEDs using PWM and DMA.
 *
 * This driver generates a PWM signal to communicate with WS2812B LEDs in NZR communication mode.
 * The PWM signal is designed to have a frequency of 800 kHz with specific duty cycles to represent
 * the digital '0' and '1' bits required by the WS2812B protocol.
 *
 * ## Clock Configuration and PWM Generation
 *
 * - Desired PWM frequency: 800 kHz (Period = 1.25 µs)
 * - Clock frequency (f_clk): 64 MHz
 * - to send 0: Send PWM T0L - 0.85us , T0H - 0.4us
 * - to send 1: Send PWM T1L - 0.45us , T1H - 0.8us
 * - reset set LOW at 0.5us > T > 280us
 *
 *
 *
 * To achieve the desired frequency, the prescaler (PSC) and auto-reload register (ARR) values
 * are calculated as follows:
 *
 * - The timer frequency is determined by the formula:
 *   - Timer Frequency = Clock Frequency / [(PSC + 1) * (ARR + 1)]
 * - Given the clock frequency of 64 MHz and the desired PWM frequency of 800 kHz:
 *   - The optimal values are:
 *     - PSC (Prescaler) = 0
 *     - ARR (Auto-Reload Register) = 79
 *
 * These values ensure the PWM signal operates at an exact frequency of 800 kHz.
 *
 * These values ensure that the PWM frequency is precisely 800 kHz.
 *
 * ## Duty Cycle Configuration
 *
 * - The WS2812B protocol requires specific duty cycles to represent '0' and '1':
 *   - '0' bit: ~33.3% duty cycle (1/3)
 *   - '1' bit: ~66.6% duty cycle (2/3)
 * - The corresponding capture/compare register (CCR) values for these duty cycles are:
 *   - CCR for 1/3 duty cycle: 27
 *   - CCR for 2/3 duty cycle: 53
 *
 * These settings allow the driver to generate the correct PWM signals to control the LEDs.
 *
 * @note This driver uses DMA1 for efficient data transfer to the LEDs.
 *
 * @date Created on: Jun 12, 2024
 * @author Parry
 */
#ifndef WS2812B_WS212BDRIVER_H_
#define WS2812B_WS212BDRIVER_H_

#include <stdio.h>
#include <stdint.h>

/* How do we initialize this part?.
 * 1. Clear all bits. Does not appear to utilize anything other than 24 bits for RGB. Alongside reset.
 *
 */

//data structure that holds RGB values to represent a color
typedef struct
{
	uint8_t g;
	uint8_t r;
	uint8_t b;
}rgb_color;

typedef struct
{
	uint16_t g[8];
	uint16_t r[8];
	uint16_t b[8];
}pixel;

/* To turn on and off LED's we must generate a PWM with certain frequency
 * 1. We will generate a PWM signal with period of
 * *
 * But before this we will need to know how to send a bit. How do we send 1's and 0's?
 *

 *
 * Total period is 1.25us
 *
 * To get ON.
 *
 * */


#define _onDutyCycle 53
#define _offDutyCycle 27
#define _numOfLed 8
#define _numOfWLedBits (_numOfLed * TIM16->ARR) + TIM16->ARR




//this function will send send a low signal to all LED's to turn them off
void resetAllLed(pixel* leds);
/**
 * @brief sets a single LED "index" away from "leds" with the color "rgc_color" while not changing
 * anything else
 *
 * @param[in,out] leds Pointer to the array of pixel structures, each representing an LED.
 *                     The function modifies the color of the LED at the specified index.
 * @param[in] rgb The RGB color value to set for the specified LED. This structure contains
 *                the red, green, and blue components of the color.
 * @param[in] index The index of the LED in the array to update. The index starts from 0
 *                  and must be within the valid range of the pixel array.
 *
 * @note Ensure that the index is within the bounds of the pixel array to avoid out-of-bounds access.
 * @note The pixel array may be linked to a DMA buffer, which will transmit the updated data to the LEDs.
 */
void setSinglePixels(pixel * leds,rgb_color rgb,int index);//changes pixel values, which are tied to DMA which is tied to an output!
void setAllLed(pixel* leds, rgb_color rgb,float brightness);
void debugPixel(pixel * ledVal,rgb_color rgb,int index);
rgb_color changeBrightnessRGB(rgb_color rgb, double offset,double scale);
void setBrightnessRGB(rgb_color * rgb, int red, int green, int blue);

void generatePWMBuffers(uint16_t* brightnessArray, uint16_t numOfLeds, pixel* pwmBuffer);

//so we are using DMA1.


#endif /* WS2812B_WS212BDRIVER_H_ */
