
// ICM20948_driver.h

#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "main.h"
#include "stm32wbxx_hal.h" // Adjust based on your STM32 series

typedef struct{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
}icm_20948_data;


#define ICM20948_I2C_ADDR0 0x68<<1//1101000//AD0 = 0
#define ICM20948_I2C_ADDR1 0x69<<1//1101001//AD0 = 1



//#define IMU_SPI              hspi1
//#define IMU_CS_PORT          SPI1_CS_GPIO_Port
//#define IMU_CS_PIN           SPI1_CS_Pin
#define GYRO_RANGE_VALUE     _gyro_1000dps
#define ACCEL_RANGE_VALUE    _accel_4g

// Register Map (Add more as needed)
#define ODR_ALIGN_EN       0x09
#define WHO_AM_I           0x00
#define PWR_MGMT_1         0x06
#define USER_CTRL          0x03
#define GYRO_SMPLRT_DIV    0x00
#define GYRO_CONFIG_1      0x01
#define GYRO_CONFIG_2      0x02

#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x10
#define ACCEL_CONFIG       0x14

#define ACCEL_XOUT_H       0x2d
#define ACCEL_XOUT_L       0x2e
#define ACCEL_YOUT_H       0x2f
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0xf31
#define ACCEL_ZOUT_L       0x32

#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38

typedef enum
{
    _gyro_250dps,
    _gyro_500dps,
    _gyro_1000dps,
    _gyro_2000dps
} gyro_range;

typedef enum
{
    _accel_2g,
    _accel_4g,
    _accel_8g,
    _accel_16g
} accel_range;

typedef enum
{
    _b0 = 0,
    _b1 = 1 << 4,
    _b2 = 2 << 4,
    _b4 = 3 << 4
} user_bank;

// Function Prototypes
HAL_StatusTypeDef ICM20948_Init();
HAL_StatusTypeDef ICM20948_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ICM20948_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ICM20948_ReadAccel(icm_20948_data* gyroData);
HAL_StatusTypeDef ICM20948_ReadGyro(icm_20948_data *gyroData);
HAL_StatusTypeDef ICM20948_read_data(icm_20948_data * data);
#endif // ICM20948_DRIVER_H

