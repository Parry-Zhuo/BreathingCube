#include "lp5812.h"

int LP5812_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;

    // Step 1: Enable the chip
    data = 0x01; // Enable chip
        status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), &data, 1, 100);
        if (status != HAL_OK) {
        return 0; // Failure
    }
    // Step 2: Set LED drive mode to Direct Drive Mode
    // data = 0x00; // Direct drive mode
    // status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), &data, 1, 100);
    // if (status != HAL_OK) {
    //     return 0; // Failure
    // }
    // // Step 3: Send Update Command
    // data = 0x55; // Update command
    // status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), &data, 1, 100);
    // if (status != HAL_OK) {
    //     return 0; // Failure
    // }
    // // Step 4: Enable all LEDs
    // data = 0x0F; // Enable all LEDs
    // status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), &data, 1, 100);
    // if (status != HAL_OK) {
    //     return 0; // Failure
    // }
    // // Step 5: Set peak current for all LEDs (12.75 mA)
    // uint8_t current[4] = {0x7F, 0x7F, 0x7F, 0x7F}; // Current values
    // status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), current, 4, 100);
    // if (status != HAL_OK) {
    //     return 0; // Failure
    // }
    // // Step 6: Set 50% PWM duty cycle for all LEDs
    // uint8_t pwm[4] = {0x7F, 0x7F, 0x7F, 0x7F}; // PWM values
    // status = HAL_I2C_Master_Transmit(&hi2c1, (LP5812_I2C_ADDRESS << 1), pwm, 4, 100);
    // if (status != HAL_OK) {
    //     return 0; // Failure
    // }
    return 1; // Success
}