#ifndef LP5812_H
#define LP5812_H
/**
 * @file led_control.h
 * @brief Header file for controlling LEDs with LP5812.
 * @author Parry
 * @date 2024-12-18
 */

#include "main.h"


#define LP5812_BROADCAST_ADDR  0x1B <<3  ///1B<<2 = 6C LP5812 broadcast I2C address (7-bit shifted to left)
/***************************************************************************//**
 *Register Address
 ******************************************************************************/
/* Device Configuration Register Address*/
#define Chip_Enable_Register               0x000
#define Dev_Config0_Register               0x001//Max Current Sink Current & Boost Output Voltage
#define Dev_Config1_Register               0x002//Direct Connected Outputs(Only effective when configured to Mix Drive Mode) & LED Configuration & PWM Dimming Frequency Setting
#define Dev_Config2_Register               0x003//Scan Line FET Number in Matrix Mode
#define Dev_Config3_Register               0x004//LED Autonomous Control Enable
#define Dev_Config4_Register               0x005//LED Autonomous Control Enable
#define Dev_Config5_Register               0x006//LED Exponential Dimming Curve Enable
#define Dev_Config6_Register               0x007//LED Exponential Dimming Curve Enable
#define Dev_Config7_Register               0x008//PWM Phase Align Method
#define Dev_Config8_Register               0x009//PWM Phase Align Method
#define Dev_Config9_Register               0x00A//PWM Phase Align Method
#define Dev_Config10_Register              0x00B//PWM Phase Align Method
#define Dev_Config11_Register              0x00C//Line Change Time & Vsync setting
#define Dev_Config12_Register              0x00D//LSD Fault Comparator Threshold & Action When LSD/LOD & Clamp Behavior setting

/*Command registers*/
#define Update_CMD_REG                     0x010 //Write 55h to send this command
#define Start_CMD_REG                      0x011 //Write FFh to send this command
#define Stop_CMD_REG                       0x012 //Write AAh to send this command
#define Pause_CMD_REG                      0x013 //Write 33h to send this command
#define Continue_CMD_REG                   0x014 //Write CCh to send this command

/*LED_EN Registers*/
#define LED_EN1                            0x020
#define LED_EN2                            0x021

/*FAULT_CLEAR REGISTERS*/
#define Fault_CLR_REG                      0x022

/*RESET REGISTERS*/
#define RESET_REG                          0x023


/*MANUAL_DC REGISTERS*/
/*LED0/LED1/LED2/LED3/LED_A0/LED_A1/LED_A2/LED_B0/LED_B1/LED_B2/LED_C0/LED_C1/LED_C2/LED_D0/LED_D1/LED_D2/ */
#define Manual_DC_GAP                      0x001
#define Manual_DC_START                    0x030


/*MANUAL_PWM REGISTERS*/
/*LED0/LED1/LED2/LED3/LED_A0/LED_A1/LED_A2/LED_B0/LED_B1/LED_B2/LED_C0/LED_C1/LED_C2/LED_D0/LED_D1/LED_D2/ */
#define Manual_PWM_GAP                      0x001
#define Manual_PWM_START                    0x040


/*MANUAL_DC REGISTERS*/
/*LED0/LED1/LED2/LED3/LED_A0/LED_A1/LED_A2/LED_B0/LED_B1/LED_B2/LED_C0/LED_C1/LED_C2/LED_D0/LED_D1/LED_D2/ */
#define Auto_DC_GAP                         0x001
#define Auto_DC_START                       0x050


/*AUTONOMOUS CONTROL REGISTERS*/
/*LED0/LED1/LED2/LED3/LED_A0/LED_A1/LED_A2/LED_B0/LED_B1/LED_B2/LED_C0/LED_C1/LED_C2/LED_D0/LED_D1/LED_D2/ */
#define LED_AEU_GAP                         0x01A
#define AEU_GAP                             0x008

#define LEDA0_Pause_Time                     0x0E8
#define LEDA0_Playback_Time                  0x0E9
#define LEDA0_AEU1_PWM1                      0x0EA
#define LEDA0_AEU1_PWM2                      0x0EB
#define LEDA0_AEU1_PWM3                      0x0EC
#define LEDA0_AEU1_PWM4                      0x0ED
#define LEDA0_AEU1_PWM5                      0x0EE
#define LEDA0_AEU1_SLOPE_TIME12               0x0EF
#define LEDA0_AEU1_SLOPE_TIME34               0x0F0
#define LEDA0_AEU1_PT1                       0x0F1
#define LEDA0_AEU2_PWM1                      0x0F2
#define LEDA0_AEU2_PWM2                      0x0F3
#define LEDA0_AEU2_PWM3                      0x0F4
#define LEDA0_AEU2_PWM4                      0x0F5
#define LEDA0_AEU2_PWM5                      0x0F6
#define LEDA0_AEU2_SLOPE_TIME1               0x0F7
#define LEDA0_AEU2_SLOPE_TIME2               0x0F8
#define LEDA0_AEU2_PT1                       0x0F9
#define LEDA0_AEU3_PWM1                      0x0FA
#define LEDA0_AEU3_PWM2                      0x0FB
#define LEDA0_AEU3_PWM3                      0x0FC
#define LEDA0_AEU3_PWM4                      0x0FD
#define LEDA0_AEU3_PWM5                      0x0FE
#define LEDA0_AEU3_SLOPE_TIME1               0x0FF
#define LEDA0_AEU3_SLOPE_TIME2               0x100
#define LEDA0_AEU3_PT1                       0x101


/* FLAG Register Address*/
#define TSD_CONFIG_STATUS                   0x300
#define LOD_STATUS1                         0x301
#define LOD_STATUS2                         0x302
#define LSD_STATUS1                         0x303
#define LSD_STATUS2                         0x304

/* TEST Register Address*/
#define OTP_CONFIG_Register                 0x352
#define SRAM_CONFIG_Register                0x353
#define CLOCK_GATING_EN_Register            0x354


/***************************************************************************//**
 *Register Value
 ******************************************************************************/
#define Chip_Disable                       0x00
#define Chip_Enable                        0x01


#define LOD_Clear_EN                       0x01
#define LSD_Clear_EN                       0x02
#define Reset_En                           0x66


#define LED0                               0x00
#define LED1                               0x01
#define LED2                               0x02
#define LED3                               0x03
#define LED_A0                             0x04
#define LED_A1                             0x05
#define LED_A2                             0x06
#define LED_B0                             0x07
#define LED_B1                             0x08
#define LED_B2                             0x09
#define LED_C0                             0x0A
#define LED_C1                             0x0B
#define LED_C2                             0x0C
#define LED_D0                             0x0D
#define LED_D1                             0x0E
#define LED_D2                             0x0F

#define LED_A                              0x04
#define LED_B                              0x07
#define LED_C                              0x0A
#define LED_D                              0x0D

#define Update_CMD_Value                   0x55 //Write 55h to send this command
#define Start_CMD_Value                    0xFF //Write FFh to send this command
#define Stop_CMD_Value                     0xAA //Write AAh to send this command
#define Pause_CMD_Value                    0x33 //Write 33h to send this command
#define Continue_CMD_Value                 0xCC //Write CCh to send this command

#define AEU1                               0x00
#define AEU2                               0x01
#define AEU3                               0x02

/***************************************************************************//**
 *Register Dev_config0 Value
 ******************************************************************************/
#define MAX_CURRENT_25_5                   0x0 //default
#define MAX_CURRENT_51                     0x1

#define BOOST_VOUT_3                       0x00 //default
#define BOOST_VOUT_3_3                     0x03
#define BOOST_VOUT_3_5                     0x05
#define BOOST_VOUT_4                       0x0A
#define BOOST_VOUT_5                       0x14
#define BOOST_VOUT_5_5                     0x1F



/***************************************************************************//**
 *Register Dev_config1 Value
 ******************************************************************************/
#define PWM_FREQ_24K                       0x0 //default
#define PWM_FREQ_12K                       0x1

#define LED_CONFIG_DIRECT                  0x0 //default
#define LED_CONFIG_1SCAN                   0x1
#define LED_CONFIG_2SCAN                   0x2
#define LED_CONFIG_3SCAN                   0x3
#define LED_CONFIG_4SCAN                   0x4
#define LED_CONFIG_MIX_1SCAN               0x5
#define LED_CONFIG_MIX_2SCAN               0x6
#define LED_CONFIG_MIX_3SCAN               0x7

#define MIX_SEL_LED_NONE                   0x0 //default
#define MIX_SEL_LED_OUT0                   0x1
#define MIX_SEL_LED_OUT1                   0x2
#define MIX_SEL_LED_OUT2                   0x4
#define MIX_SEL_LED_OUT3                   0x8


/***************************************************************************//**
 *Register Dev_config2 Value
 ******************************************************************************/
#define SCAN_ORDER_OUT0                   0x0
#define SCAN_ORDER_OUT1                   0x1
#define SCAN_ORDER_OUT2                   0x2
#define SCAN_ORDER_OUT3                   0x3

#define LED_EN1_REG            0x20  // Register to enable LEDs 0-7
#define LED_EN2_REG            0x21  // Register to enable LEDs 8-11



HAL_StatusTypeDef LP5812_WriteRegister(uint16_t reg, uint8_t value);
HAL_StatusTypeDef LP5812_BurstWrite(uint16_t startReg, uint8_t *values, uint16_t dataSize) ;
HAL_StatusTypeDef LP5812_ReadRegister(uint16_t reg, uint8_t *pValue);
/**
 * @brief Initializes the LP5812 LED driver.
 *
 * This function configures the LP5812 for LED control, including enabling
 * the device and setting default configurations.
 *
 * @param[in] address I2C address of the LP5812.
 * @return Status of the initialization (0 = success, non-zero = error).
 */
int LP5812_Init_Manual(void);
int LP5812_Init_Autonomous(void);
int LP5812_BreathingAnimation(void);
int LP5812_stopAnimation(void);
int LP5812_manualControl(void);
HAL_StatusTypeDef LP5812_DetectFault(void);
HAL_StatusTypeDef LP5812_ClearFaults(void);
void FadeLEDs_Manual(uint16_t startRegister, uint16_t resolution, int delayTime);
void fadeAlternatingColours_manual();
HAL_StatusTypeDef AEU_SET(uint8_t LED_NUM,
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
             uint8_t PT//playback time 0h = 0 time, 1h = 1 time, 2h = 2 times, 3h = infinite times
             );
HAL_StatusTypeDef AEU_SET_AllWhite();
HAL_StatusTypeDef LP5812_AnimationCommand(uint8_t command);

#endif // LP5812_H
