#include "stm32f4xx_hal.h"  // Update based on your STM32 family

extern I2C_HandleTypeDef hi2c1;
/**
 * @brief Writes a value to a specific register in the LP5812 via I2C.
 *
 * The LP5812 operates with a 5-bit device address and a 10-bit register address. 
 * The device address incorporates the upper two bits (RA9 and RA8) of the register 
 * address. The remaining 8 bits of the register address, along with the value to 
 * write, are sent as data in the I2C transaction.
 *
 * The steps for a write operation are:
 * 1. Transmit the device address + RA9/RA8 (MSB of the register address).
 * 2. Transmit the remaining 8 bits of the register address (low byte).
 * 3. Transmit the data byte (value to be written).
 *
 * @param[in] reg 10-bit register address (significant bits: RA9-RA0).
 * @param[in] value 8-bit value to be written to the register.
 * @return HAL_StatusTypeDef Status of the I2C transmission (HAL_OK if successful).
 */
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
/**
 * @brief Reads a single register from the LP5812 device over I2C.
 *
 * The LP5812 uses a 5-bit device address and a 10-bit register address for I2C communication. 
 * This function performs the read operation by sending the register address in write mode, 
 * followed by reading the data from the register in read mode.
 *
 * The steps are:
 * 1. Start condition + device address + write bit.
 * 2. Send the register address.
 * 3. Restart condition.
 * 4. Device address + read bit.
 * 5. Receive the byte of data.
 * 6. Acknowledge or not acknowledge to stop receiving data.
 * 7. Stop condition.
 *
 * @param[in] reg 16-bit register address to be read (10 bits significant).
 * @return Status of the read operation (HAL_OK if successful, error code otherwise).
 */
uint8_t LP5812_ReadRegister(uint16_t reg) {

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

    ///Enable the device
    status = LP5812_WriteRegister(Chip_Enable_Register, 0x01);
    if (status != HAL_OK) return -1;

    ///Limit MAX current = 25.5mA
    status = LP5812_WriteRegister(Dev_Config0_Register, 0X00);  // Enable LEDs 0–7
    if (status != HAL_OK) return -1;

    ///Set LED drive mode to 4-scan
    status = LP5812_WriteRegister(Dev_Config1_Register, 0x40);
    if (status != HAL_OK) return -1;

    ///lsd_threshold = 0.65Vcc, shut off if cathode voltage surpasses set amount. Also enabled short and open fault
    status = LP5812_WriteRegister(Dev_Config12_Register, 0x0F);
    if (status != HAL_OK) return -1;

    ///Send update command(applies Dev_Config0_Register->Dev_Config10_Register)
    status = LP5812_WriteRegister(Update_CMD_REG, 0x55);
    if (status != HAL_OK) return -1;

    ///Check config error status. See if any fault had been tripped
    uint8_t config_status = LP5812_ReadRegister(TSD_CONFIG_STATUS);
    if (config_status != 0x00) {
        return -2;  // Error: Configuration not proper
    }


    ///Enable all 12 LEDs, A0,A1,A2.... Disabling LED0,LED1,....
    status = LP5812_WriteRegister(LED_EN1_REG, 0xF0);  // Enable LEDs 0–7
    if (status != HAL_OK) return -1;
    status = LP5812_WriteRegister(LED_EN2_REG, 0xFF);  // Enable LEDs 8–11
    if (status != HAL_OK) return -1;

    /** 
     * Set current limit for specific LED's. LED MAX ALLOWABLE CURRENT according to datasheet= 20mA
     * I_led = I_max(Register Value/255).
     * ~12.75mA = 25.5(127.5/255), write 0x7F
     * This takes place over 0X34 -> 0X3F. 12 LED's
     * **/
    uint8_t peakCurrent = 0x7F;
    for (uint16_t reg = MANUAL_DC_START_REG; reg < (MANUAL_DC_START_REG + 12); reg++) {
        status = LP5812_WriteRegister(reg, peakCurrent); 
        if (status != HAL_OK) return -1;
    }

    ///Set 80% PWM duty cycle for all 12 LEDs(0xCC = 80%)
    for (uint16_t reg = MANUAL_PWM_START_REG; reg <= (MANUAL_PWM_START_REG + 11); reg++) {
        status = LP5812_WriteRegister(reg, 0xCC); 
        if (status != HAL_OK) return -1;
    }

    ///Indicate success via GPIO pin
    HAL_GPIO_WritePin(GPIOA, Indication_light_Pin, GPIO_PIN_SET);

    return 0;  // Success
}
