/*
 * Ws212bDriver.cpp
 *
 *  Created on: Jun 12, 2024
 *      Author: Parry
 *
 * This is a driver for the WS212B. Which are LED's that operate on a ASYNCHRONOUS protocol
 * Code was written by Parry with inspiration taken from https://www.youtube.com/watch?v=4-bzawdOCQs
 *
 *
 * How does this program work?
 * How do we initialize this LED?
 */

#include <Ws2812bDriver.h>

void resetAllLed(pixel* leds){
	pixel * currentLed = leds;
	for(uint16_t index = 0; index < _numOfLed; index++ ){
		//i'm a little confused as to how to use pointers. How to connect that with index.
		for(int x = 0; x< _numOfLed; x++){
			currentLed->r[x] = _offDutyCycle;
			currentLed->g[x] = _offDutyCycle;
			currentLed->b[x] = _offDutyCycle;
		}
		currentLed++;
	}
}
rgb_color changeBrightnessRGB(rgb_color rgb, double offset, double scale){

	rgb.r = rgb.r/scale + offset;
	rgb.g = rgb.g/scale + offset;
	rgb.b = rgb.b/scale + offset;

	if(rgb.r == 0){
		rgb.r = 5;
	}else if(rgb.g == 0){
		rgb.g = 5;
	}else if(rgb.b == 0){
		rgb.b = 5;
	}


	return rgb;
}
void setBrightnessRGB(rgb_color * rgb, int red, int green, int blue){
	rgb->r = red;
	rgb->b = blue;
	rgb->g = green;
}
/*The goal of implementation of this is to set the PWM to a certain frequency.*/
void setSinglePixels(pixel * leds,rgb_color rgb,int index){
	pixel * currentLed = leds +  index;//this will be at the position of LED we want to set
	//we need to iterate over the RGB values and put them into current values as PWM signals.
	//How do we get an integer, from 0-255 to be represented as a binary number,

    for(int x = 0; x < 7; x++) {
         // Set the duty cycle based on the current bit of the red channel
         if((rgb.r >> x) & 0x01) {
             currentLed->r[7-x] = _onDutyCycle;
         } else {
             currentLed->r[7-x] = _offDutyCycle;
         }

         // Set the duty cycle based on the current bit of the green channel
         if((rgb.g >> x) & 0x01) {
             currentLed->g[7-x] = _onDutyCycle;
         } else {
             currentLed->g[7-x] = _offDutyCycle;
         }

         // Set the duty cycle based on the current bit of the blue channel
         if((rgb.b >> x) & 0x01) {
             currentLed->b[7-x] = _onDutyCycle;
         } else {
             currentLed->b[7-x] = _offDutyCycle;
         }
    }
}
/**
 * @brief Sets the color and brightness for an array of LEDs.
 *
 * This function updates each LED in the provided array with the specified
 * RGB color and brightness. It scales the RGB values by the brightness level
 * and converts these values into PWM duty cycles to control the LED output.
 * The function iterates over all LEDs in the array and applies these changes.
 *
 * @param[in,out] leds       Pointer to the array of LEDs to update.
 * @param[in]     rgb        The RGB color to set for each LED.
 * @param[in]     numOfLeds  The number of LEDs in the array.
 * @param[in]     brightness The brightness level (0.0 to 1.0) to apply to each LED.
 *
 * @note This function assumes each LED is represented by a `pixel` structure
 *       with individual red, green, and blue channels.
 */
void setAllLed(pixel* leds, rgb_color rgb, float brightness) {
    pixel *currentLed = leds;  // Pointer to the current LED in the array

    for(uint16_t index = 0; index < _numOfLed-1; index++) {  // Iterate over each LED in the array
        // Apply brightness scaling to RGB values
//        currentLed->r *= (uint16_t)(brightness);
//        currentLed->g *= (uint16_t)(brightness);
//        currentLed->b *= (uint16_t)(brightness);

        // Convert RGB values to PWM duty cycles for each bit (7 bits per color channel)
        for(int x = 0; x < 7; x++) {
            // Set the duty cycle based on the current bit of the red channel
            if((rgb.r >> x) & 0x01) {
                currentLed->r[7-x] = _onDutyCycle;
            } else {
                currentLed->r[7-x] = _offDutyCycle;
            }

            // Set the duty cycle based on the current bit of the green channel
            if((rgb.g >> x) & 0x01) {
                currentLed->g[7-x] = _onDutyCycle;
            } else {
                currentLed->g[7-x] = _offDutyCycle;
            }

            // Set the duty cycle based on the current bit of the blue channel
            if((rgb.b >> x) & 0x01) {
                currentLed->b[7-x] = _onDutyCycle;
            } else {
                currentLed->b[7-x] = _offDutyCycle;
            }
        }

        currentLed++;  // Move to the next LED in the array
    }
}


void debugPixel(pixel * ledVal,rgb_color rgb,int index){
	pixel * currentLed = ledVal +  index;//this will be at the position of LED we want to set
	//we need to iterate over the RGB values and put them into current values as PWM signals.
	//How do we get an integer, from 0-255 to be represented as a binary number,
	for(int x = 0; x< 4;x++){
		if((rgb.r >> x) & 0x01){
			currentLed->r[7-x] = _onDutyCycle;
		}else{
			currentLed->r[7-x] = _onDutyCycle;
		}
		if((rgb.g >> x) & 0x01){
			currentLed->g[7-x] = _onDutyCycle;
		}else{
			currentLed->g[7-x] = _onDutyCycle;
		}
		if((rgb.b >> x) & 0x01){
			currentLed->b[7-x] = _onDutyCycle;
		}else{
			currentLed->b[7-x] = _onDutyCycle;
		}
	}
	for(int x = 4; x< 7;x++){
		if((rgb.r >> x) & 0x01){
			currentLed->r[7-x] = _offDutyCycle;
		}else{
			currentLed->r[7-x] = _offDutyCycle;
		}
		if((rgb.g >> x) & 0x01){
			currentLed->g[7-x] = _offDutyCycle;
		}else{
			currentLed->g[7-x] = _offDutyCycle;
		}
		if((rgb.b >> x) & 0x01){
			currentLed->b[7-x] = _offDutyCycle;
		}else{
			currentLed->b[7-x] = _offDutyCycle;
		}
	}

}

void generatePWMBuffers(uint16_t* brightnessArray, uint16_t numOfLeds, pixel* pwmBuffer) {
    // Temporary rgb_color structure to store RGB values
    rgb_color rgb;

    // Iterate through each LED
    for (uint16_t i = 0; i < numOfLeds; i++) {
        // Set the RGB values based on the brightness level
        setBrightnessRGB(&rgb, brightnessArray[i], brightnessArray[i], brightnessArray[i]);

        // Update the PWM buffer for the current LED using setSinglePixels
        setSinglePixels(&pwmBuffer[i], rgb, i);
    }
}

