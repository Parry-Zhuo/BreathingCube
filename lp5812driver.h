#ifndef LP5812_H
#define LP5812_H

#include "main.h"

// LP5812 I2C Address
#define LP5812_I2C_ADDRESS 0x38

// LP5812 Registers
#define CHIP_EN_REG     0x000
#define LED_MODE_REG    0x002
#define CMD_UPDATE_REG  0x010
#define LED_EN_REG      0x020
#define MANUAL_DC_REG   0x030
#define MANUAL_PWM_REG  0x040


/**
 * @brief  Initializes the LP5812 LED driver and configures it for direct drive mode.
 * 
 * This function 
 * 1. enable the chip
 * 2. configure LED drive mode
 * 3. set current and PWM values for all LEDs. 
 * 
 * @retval int 
 *         - 1: Initialization successful.
 *         - 0: Initialization failed.
 */
int LP5812_Init(void);



#endif // LP5812_H