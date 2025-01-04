#include "lp5812driver.h"
#include "stm32wbxx_hal.h" // Replace 'fxx' with your specific STM32 series
#include <string.h>


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
HAL_StatusTypeDef LP5812_BurstWrite(uint16_t startReg, uint8_t *values, uint16_t dataSize) {
    // Calculate the device address based on the starting register
    uint8_t i2c_addr = LP5812_BROADCAST_ADDR | ((startReg >> 8) & 0x03);

    // Create a buffer for the register address and data
    uint8_t dataBuffer[dataSize + 1];
    dataBuffer[0] = startReg & 0xFF;  // Low byte of the starting register address

    // Copy the values to the data buffer after the register address
    memcpy(&dataBuffer[1], values, dataSize);

    // Transmit the data in a single I2C transaction
    return HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, dataBuffer, dataSize + 1, 100);
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
HAL_StatusTypeDef LP5812_ReadRegister(uint16_t reg, uint8_t *pValue) {

    HAL_StatusTypeDef status;

    // Step 1: Prepare the device address with RA9 and RA8 for Write (W = 0)
    uint8_t i2c_addr_write = LP5812_BROADCAST_ADDR | ((reg >> 8) & 0x03);  // Combine base address + RA9/RA8

    // Step 2: Prepare the register address (low byte)
    uint8_t reg_addr = reg & 0xFF;

    // Step 3: Send the Register Address in Write Mode
    status = HAL_I2C_Master_Transmit(&hi2c1, i2c_addr_write, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        return status;  // Return error if transmission fails
    }
    // HAL_Delay(1);  // Small delay to stabilize the I²C bus

    // Step 4: Prepare the device address with RA9 and RA8 for Read (R = 1)
    uint8_t i2c_addr_read = i2c_addr_write | 0x01;  // Set Read bit (R = 1)

    // Step 5: Read the Data Byte
    
    status = HAL_I2C_Master_Receive(&hi2c1, i2c_addr_read, pValue, 1, 100);
    return status;  // Return the status of the operation
}

HAL_StatusTypeDef LP5812_DetectFault(void){
    HAL_StatusTypeDef fault_status;
    HAL_GPIO_WritePin(GPIOA, Indication_light_Pin, GPIO_PIN_RESET);
    // Step 1: Read the TSD_CONFIG_STATUS register (0x1A)
    uint8_t faultRegisters = 0;
    fault_status = LP5812_ReadRegister(TSD_CONFIG_STATUS,&faultRegisters);//default 
    if(faultRegisters & (0x03) != 0x00){
        // return fault_status;
    }
    fault_status = LP5812_ReadRegister(LOD_STATUS1, &faultRegisters);
    if(faultRegisters & ~(0x0F) != 0x00){
        // return fault_status;
    }
    fault_status = LP5812_ReadRegister(LOD_STATUS2, &faultRegisters);
    if(faultRegisters != 0x00){
        // return fault_status;
    }
    fault_status = LP5812_ReadRegister(LSD_STATUS1, &faultRegisters);
    if(faultRegisters != 0x00){
        // return fault_status;
    }
    fault_status = LP5812_ReadRegister(LSD_STATUS2, &faultRegisters);
    if(faultRegisters != 0x00){
        // return fault_status;
    }
    // HAL_GPIO_WritePin(GPIOA, Indication_light_Pin, GPIO_PIN_SET);
    return  HAL_OK;
}

HAL_StatusTypeDef LP5812_ClearFaults(void) {
    HAL_StatusTypeDef status;
    uint8_t fault_clear_value = 0x07;  // 0b00000111: Set bits 2, 1, 0 to clear all faults

    // Write to the Fault_Clear Register (0x22) to clear all faults
    status = LP5812_WriteRegister(Fault_CLR_REG, fault_clear_value);
    if (status != HAL_OK) {
        return status;  // Return error if write operation fails
    }

    // Verify that faults are cleared by reading the register back
    uint8_t read_value;
    status = LP5812_ReadRegister(Fault_CLR_REG, &read_value);
    if (status != HAL_OK) {
        return status;  // Return error if read operation fails
    }

    // Ensure all faults are cleared (register should read back 0x00)
    if (read_value != 0x00) {
        return HAL_ERROR;  // Fault clearing failed
    }

    return HAL_OK;  // Faults cleared successfully
}
int LP5812_Init_Manual(void) {
    HAL_StatusTypeDef status;

    ///Enable the device
    status = LP5812_WriteRegister(Chip_Enable_Register, 0x01);
    if (status != HAL_OK) return -1;

    // ///Limit MAX current = 25.5mA
    // status = LP5812_WriteRegister(Dev_Config0_Register, 0X00);  // Enable LEDs 0–7
    // if (status != HAL_OK) return -1;

    // ///Set LED drive mode to 4-scan
    // status = LP5812_WriteRegister(Dev_Config1_Register, 0x40);
    // if (status != HAL_OK) return -1;
    uint8_t data[] = {0x00, 0x40};
    status = LP5812_BurstWrite(Dev_Config0_Register,data, sizeof(data));
    if (status != HAL_OK) return -1;


    ///lsd_threshold = 0.65Vcc, shut off if cathode voltage surpasses set amount. Also enabled short and open fault
    status = LP5812_WriteRegister(Dev_Config12_Register, 0x0F);
    if (status != HAL_OK) return -1;



    ///Send update command(applies Dev_Config0_Register->Dev_Config10_Register)
    status = LP5812_WriteRegister(Update_CMD_REG, 0x55);
    if (status != HAL_OK) return -1;

    ///Check config error status. See if any fault had been tripped
    uint8_t config_register;
    uint8_t config_status = LP5812_ReadRegister(TSD_CONFIG_STATUS,&config_register);
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
//    uint8_t peakCurrent = 0x7F;
    uint8_t peakCurrent = 0xFF;
    for (uint16_t reg = Manual_DC_START+4; reg < (Manual_DC_START + 16); reg++) {
        status = LP5812_WriteRegister(reg, peakCurrent);
        if (status != HAL_OK) return -1;
    }

    //Set 80% PWM duty cycle for all 12 LEDs(0xCC = 80%)
	    uint8_t ledPWM_DutyCycle = 0x10;
	    for (uint16_t reg = Manual_PWM_START; reg <= (Manual_PWM_START + 11); reg++) {
	        status = LP5812_WriteRegister(reg, ledPWM_DutyCycle);
	        if (status != HAL_OK) return -1;
	    }

    ///Indicate success via GPIO pin
    

    return 0;  // Success
}

void FadeLEDs_Manual(uint16_t startRegister, uint16_t resolution, int delayTime) {
    // Gradually increase brightness (fade-in)
    uint8_t data[3];
    for (uint16_t pwm = 0; pwm <= 0xFF; pwm += resolution) {
        data[0] = pwm;  // LED 1
        data[1] = pwm;  // LED 2
        data[2] = pwm;  // LED 3

        LP5812_BurstWrite(startRegister, data, sizeof(data));  // Burst write PWM values
        HAL_Delay(delayTime);  // Delay for smooth effect
    }

    // Gradually decrease brightness (fade-out)
    for (int16_t pwm = 0xFF; pwm >= 0; pwm -= resolution) {
        data[0] = pwm;  // LED 1
        data[1] = pwm;  // LED 2
        data[2] = pwm;  // LED 3

        LP5812_BurstWrite(startRegister, data, sizeof(data));  // Burst write PWM values
        HAL_Delay(delayTime);  // Delay for smooth effect
    }
}
void fadeAlternatingColours_manual(){

    uint8_t data[3];  // Buffer for burst writing PWM values
    uint16_t resolution = 0x05;  // Step size for PWM changes
    int delayTime = 75;  
    //WHITE
    FadeLEDs_Manual(Manual_PWM_START + 0x04, resolution, delayTime);
    //BLUE
    FadeLEDs_Manual(Manual_PWM_START + 0x07, resolution, delayTime);
    //GREEN
    FadeLEDs_Manual(Manual_PWM_START + 0x10, resolution, delayTime);
    //RED
    FadeLEDs_Manual(Manual_PWM_START + 0x13, resolution, delayTime);
    
}
int LP5812_Init_Autonomous(void) {
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


    // ///Sets order of scanning. May be the reason why SC
    // status = LP5812_WriteRegister(Dev_Config2_Register, 0x40);
    // if (status != HAL_OK) return -1;

    ///We are enabling LED's to be AUTONOMOUS
    status = LP5812_WriteRegister(Dev_Config3_Register, 0XF0);
    if (status != HAL_OK) return -1;

    status = LP5812_WriteRegister(Dev_Config4_Register, 0XFF);
    if (status != HAL_OK) return -1;
    //EXPONENTIAL PWM DIMMER
    status = LP5812_WriteRegister(Dev_Config5_Register, 0XF0);
    if (status != HAL_OK) return -1;

    //EXPONENTIAL PWM DIMMER
    status = LP5812_WriteRegister(Dev_Config6_Register, 0XFF);
    if (status != HAL_OK) return -1;



    ///lsd_threshold = 0.65Vcc, shut off if cathode voltage surpasses set amount. Also enabled short and open fault
    status = LP5812_WriteRegister(Dev_Config12_Register, 0x0F);
    if (status != HAL_OK) return -1;

    ///Send update command(applies Dev_Config0_Register->Dev_Config10_Register to chip)
    status = LP5812_WriteRegister(Update_CMD_REG, 0x55);
    if (status != HAL_OK) return -1;

    ///Check config error status. See if any fault had been tripped
    uint8_t config_register;
    uint8_t config_status = LP5812_ReadRegister(TSD_CONFIG_STATUS,&config_register);
    if (config_status != 0x00) {
        return -2;  // Error: Configuration not proper
    }


    ///Enable all 12 LEDs, A0,A1,A2.... Disabling LED0,LED1,....
    status = LP5812_WriteRegister(LED_EN1_REG, 0xF0);  // Enable LEDs 0–7
    if (status != HAL_OK) return -1;
    status = LP5812_WriteRegister(LED_EN2_REG, 0xFF);  // Enable LEDs 8–11
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

    uint8_t values[16];         // Array to hold 16 copies of peakCurrent
    for (int i = 0; i < 16; i++) {
        values[i] = peakCurrent;
    }
    status = LP5812_BurstWrite(Manual_PWM_START+4, values,16);
    if (status != HAL_OK) return -1;

    return 0;  // Success
    
}

HAL_StatusTypeDef AEU_SET(uint8_t LED_NUM,//LED NUM 0,1,3
             //uint8_t AEU_NUM, //AEU1/2/3
             uint8_t PWM1,//0-255
             uint8_t PWM2,
             uint8_t PWM3,
             uint8_t PWM4,
             uint8_t PWM5,
             uint8_t T1,//0-15 from 0-8s
             uint8_t T2,
             uint8_t T3,
             uint8_t T4,
             uint8_t PT){//playback time 0h = 0 time, 1h = 1 time, 2h = 2 times, 3h = infinite times
        
    HAL_StatusTypeDef status;

    // Auto_Pause, set to 0x00 -> 0 seconds
    status = LP5812_WriteRegister(LEDA0_Pause_Time + (LED_NUM * 26), 0x00); 
    if (status != HAL_OK) return -1;
    //Playback times 0x0F -> infinite times
    status = LP5812_WriteRegister(LEDA0_Playback_Time + (LED_NUM * 26), 0x0F); 
    if (status != HAL_OK) return -1;
    //Setting animation Engine PWM values 
    uint8_t AEU1_PWM_Values[] = { PWM1,PWM2,PWM3,PWM4,PWM5};//{ 0x00,0x80,0xFF,0x80,0x00};
    status = LP5812_BurstWrite(LEDA0_AEU1_PWM1 + (LED_NUM * 26),AEU1_PWM_Values,5);
    if (status != HAL_OK) return -1;
    //time for breathe in/out (1.07*2) secs each
    uint8_t breathingInOutTime[] = { (T2<<2)| (T1 & 0x0F), (T4<<2)| (T3 & 0x0F)};//{ 0x06,0x06, 0x06,0x06};
    status = LP5812_BurstWrite(LEDA0_AEU1_SLOPE_TIME12 + (LED_NUM * 26),breathingInOutTime,2);
    if (status != HAL_OK) return -1;
    status = LP5812_WriteRegister(LEDA0_AEU1_PT1+ (LED_NUM * 26),PT);
    if (status != HAL_OK) return -1;

}

HAL_StatusTypeDef AEU_SET_AllWhite() {
    HAL_StatusTypeDef status;
    // Configure LED0 (A0)
    status = AEU_SET(0,  // LED_NUM: 0 (A0)
                     0x00, 0x40, 0x80, 0xC0, 0xFF,  // PWM values for fade-in/out
                     0x06, 0x06, 0x06, 0x06,        // Slope times
                     0x03);                         // Infinite playback
    if (status != HAL_OK) {
        return status;  // Return error if configuration fails for LED0
    }
    // Configure LED1 (A1)
    status = AEU_SET(1,  // LED_NUM: 1 (A1)
                     0x00, 0x40, 0x80, 0xC0, 0xFF,  // PWM values for fade-in/out
                     0x06, 0x06, 0x06, 0x06,        // Slope times
                     0x03);                         // Infinite playback
    if (status != HAL_OK) {
        return status;  // Return error if configuration fails for LED1
    }
    // Configure LED2 (A2)
    status = AEU_SET(2,  // LED_NUM: 2 (A2)
                     0x00, 0x40, 0x80, 0xC0, 0xFF,  // PWM values for fade-in/out
                     0x06, 0x06, 0x06, 0x06,        // Slope times
                     0x03);                         // Infinite playback
    if (status != HAL_OK) {
        return status;  // Return error if configuration fails for LED2
    }
    return HAL_OK;  // Return success if all LEDs are configured correctly
}
