/*
 * ICM20948Driver.cpp
 *
 *  Created on: Dec 22, 2024
 *      Author: Parry
 */


// ICM20948_driver.c

#include "ICM20948Driver.h"


extern I2C_HandleTypeDef hi2c1;

uint16_t _ICM20948_DeviceAddress = ICM20948_I2C_ADDR0;


//// Read a single register
HAL_StatusTypeDef ICM20948_ReadReg(uint8_t reg, uint8_t *data) {
    // Send the register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, _ICM20948_DeviceAddress, &reg, 1, 100);
    if (status != HAL_OK) {
        return status; // Return error if the transmit fails
    }

    // Receive the data from the specified register
    status = HAL_I2C_Master_Receive(&hi2c1, _ICM20948_DeviceAddress | 0x01, data, 1, 100);
    return status;
}
// Write a single register
HAL_StatusTypeDef ICM20948_WriteReg(uint8_t reg, uint8_t value) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
    return HAL_I2C_Master_Transmit(&hi2c1, _ICM20948_DeviceAddress, data, 2, 100);
}

HAL_StatusTypeDef ICM20948_SetBank(uint8_t bank) {

	//find register bank select. Address 0x7F, VALUE 0x0(0,1,2,3) for 4 banks respectively
	uint8_t bankSelectReg = 0x7f;

    uint8_t regValue = (bank & 0x03); // Only bits 4-5 are used for bank selection
    return HAL_I2C_Master_Transmit(&hi2c1, _ICM20948_DeviceAddress, (uint8_t[]){bankSelectReg, regValue}, 2, 100);
}

// Initialize the ICM20948 sensor
HAL_StatusTypeDef ICM20948_Init() {

//    select BANK0
    if(ICM20948_SetBank(0) !=HAL_OK){
    	return HAL_ERROR;
    }
// 	Check WHO_AM_I register( 0xEA)
//    uint8_t who_am_i = 0;
//    if (ICM20948_ReadReg(WHO_AM_I, &who_am_i) != HAL_OK) {
//        return HAL_ERROR;
//    }

//    // Wake up the sensor by clearing the sleep bit
    //0X01<<7 to reset Internal registers to default
    //(001-101) bit (0-3) automatically chooses best CLOCK source.
    if (ICM20948_WriteReg(PWR_MGMT_1, 0xc1) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100);
    //exit from sleep mode
    if (ICM20948_WriteReg(PWR_MGMT_1, 0x01) != HAL_OK) {
        return HAL_ERROR;
    }
    //all data is sampled synchronously.
    if (ICM20948_WriteReg(ODR_ALIGN_EN, 0x01) != HAL_OK) {
        return HAL_ERROR;
    }
    //Sample rate divider = 0 for GYRO, page 59. Maximum sampling rate
    if (ICM20948_WriteReg(GYRO_SMPLRT_DIV, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }
    // gyroscope range set and enable digital LP filter, page 59
    if (ICM20948_WriteReg(GYRO_CONFIG_1, ((GYRO_RANGE_VALUE << 1) | 0X01)) != HAL_OK) { // Example: +/-8g
        return HAL_ERROR;
    }
    //Acceleration configuration, sample rate divider = 0 Maximum sampling rate and LP filter,page 63
    if (ICM20948_WriteReg(ACCEL_SMPLRT_DIV_1, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ICM20948_WriteReg( ACCEL_SMPLRT_DIV_2, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }

    //read the USER CONTROL REGISTER, then write 1 at 6th spot. To enable I2C 0x20
    uint8_t tempData;
    if (ICM20948_ReadReg(USER_CTRL, &tempData) != HAL_OK) {
        return HAL_ERROR;
    }
    tempData |= 0x20;
	if (ICM20948_WriteReg(USER_CTRL, tempData) != HAL_OK) {
		return HAL_ERROR;
	}
	ICM20948_SetBank(0);//the values that hold sensor data is in dataBANK0

    return HAL_OK;
}
//10:01


// Read accelerometer data
HAL_StatusTypeDef ICM20948_ReadAccel(icm_20948_data* gyroData) {
    uint8_t rawData[6];
    if (HAL_I2C_Mem_Read(&hi2c1,_ICM20948_DeviceAddress, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine high and low bytes
    gyroData->x_accel = (int16_t)((rawData[0] << 8) | rawData[1])/ACCEL_RANGE_VALUE; // X-axis
    gyroData->y_accel = (int16_t)((rawData[2] << 8) | rawData[3])/ACCEL_RANGE_VALUE; // Y-axis
    gyroData->z_accel = (int16_t)((rawData[4] << 8) | rawData[5])/ACCEL_RANGE_VALUE; // Z-axis

    return HAL_OK;
}
//
//// Read gyroscope data
HAL_StatusTypeDef ICM20948_ReadGyro(icm_20948_data* gyroData) {
    uint8_t rawData[6];
//    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_I2C_Mem_Read(&hi2c1, _ICM20948_DeviceAddress, GYRO_XOUT_H , I2C_MEMADD_SIZE_8BIT, rawData, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine high and low bytes
    gyroData->x_gyro = (int16_t)((rawData[0] << 8) | rawData[1]); // X-axis
    gyroData->y_gyro  = (int16_t)((rawData[2] << 8) | rawData[3]); // Y-axis
    gyroData->z_gyro  = (int16_t)((rawData[4] << 8) | rawData[5]); // Z-axis

    return HAL_OK;
}

