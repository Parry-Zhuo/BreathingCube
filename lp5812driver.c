#include "stm32f4xx_hal.h"  // Update based on your STM32 family

#define LP5812_BROADCAST_ADDR  0xC0  // LP5812 broadcast I2C address (7-bit shifted to left)
#define CHIP_EN_REG            0x00  // Register to enable the device
#define LED_MODE_REG           0x02  // Register to set LED mode
#define CMD_UPDATE_REG         0x10  // Command register for update
#define CONFIG_ERR_STATUS_REG  0x300 // Register to check configuration status
#define LED_EN1_REG            0x20  // Register to enable LEDs 0-7
#define LED_EN2_REG            0x21  // Register to enable LEDs 8-11
#define MANUAL_DC_START_REG    0x34  // Start of Manual_DC registers
#define MANUAL_PWM_START_REG   0x44  // Start of Manual_PWM registers

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef LP5812_WriteRegister(uint16_t reg, uint8_t value) {

    // WRITE COMMAND

    // 1. Communicate device address +2 MSB for register address
    // 2. Acknowledge + 0, which is the write bit
    // 3. Transmit the rest of the REGISTER ADDRESSES + write bit
    uint8_t i2c_addr = LP5812_BROADCAST_ADDR | ((reg >> 8) & 0x03);
    uint8_t data[2];
    data[0] = reg & 0xFF;  // Low byte of register address
    data[1] = value;       // Data to write
    
    return HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, data, 2, 100);
}

uint8_t LP5812_ReadRegister(uint16_t reg) {

    // READ COMMAND:
    // 1. Start condition + device address + write bit
    // 2. Register address + read bit
    // 3. Restart
    // 4. Register write address
    // 5. Read the byte of data.
    // 6. Acknowledge if you want to continue receiving more data.
    // or Not acknowledge if you want to stop receiving data.
    // 7. Finally we have the stop condition.
    HAL_StatusTypeDef status;

    // Step 1: Prepare the device address with RA9 and RA8 for Write (W = 0)
    uint8_t i2c_addr_write = LP5812_BASE_ADDR | ((reg >> 8) & 0x03);  // Combine base address + RA9/RA8

    // Step 2: Prepare the register address (low byte)
    uint8_t reg_addr = reg & 0xFF;

    // Step 3: Send the Register Address in Write Mode
    status = HAL_I2C_Master_Transmit(&hi2c1, i2c_addr_write, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        return status;  // Return error if transmission fails
    }

    // Step 4: Prepare the device address with RA9 and RA8 for Read (R = 1)
    uint8_t i2c_addr_read = i2c_addr_write | 0x01;  // Set Read bit (R = 1)

    // Step 5: Read the Data Byte
    status = HAL_I2C_Master_Receive(&hi2c1, i2c_addr_read, pValue, 1, 100);
    return status;  // Return the status of the operation
}

int LP5812_Init(void) {
    HAL_StatusTypeDef status;

    // 1. Enable the device
    status = LP5812_WriteRegister(CHIP_EN_REG, 0x01);
    if (status != HAL_OK) return -1;

    // 2. Set LED drive mode to 4-scan
    status = LP5812_WriteRegister(LED_MODE_REG, 0x40);
    if (status != HAL_OK) return -1;

    // 3. Send update command
    status = LP5812_WriteRegister(CMD_UPDATE_REG, 0x55);
    if (status != HAL_OK) return -1;

    // 4. Check config error status
    uint8_t config_status = LP5812_ReadRegister(CONFIG_ERR_STATUS_REG);
    if (config_status != 0x00) {
        return -2;  // Error: Configuration not proper
    }

    // 5. Enable all 12 LEDs
    status = LP5812_WriteRegister(LED_EN1_REG, 0xF0);  // Enable LEDs 0–7
    if (status != HAL_OK) return -1;
    status = LP5812_WriteRegister(LED_EN2_REG, 0xFF);  // Enable LEDs 8–11
    if (status != HAL_OK) return -1;

    // 6. Set 12.75mA peak current for all LEDs in manual mode
    for (uint16_t reg = MANUAL_DC_START_REG; reg <= (MANUAL_DC_START_REG + 11); reg++) {
        status = LP5812_WriteRegister(reg, 0x7F);  // 0x7F = ~12.75mA
        if (status != HAL_OK) return -1;
    }

    // 7. Set 50% PWM duty cycle for all 12 LEDs
    for (uint16_t reg = MANUAL_PWM_START_REG; reg <= (MANUAL_PWM_START_REG + 11); reg++) {
        status = LP5812_WriteRegister(reg, 0x7F);  // 0x7F = 50% duty cycle
        if (status != HAL_OK) return -1;
    }

    // 8. Indicate success via GPIO pin
    HAL_GPIO_WritePin(GPIOA, Indication_light_Pin, GPIO_PIN_SET);

    return 0;  // Success
}
