#ifndef LP5812_H
#define LP5812_H

#include "main.h"


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

#define LED0_Pause_Time                     0x080
#define LED0_Playback_Time                  0x081
#define LED0_AEU1_PWM1                      0x082
#define LED0_AEU1_PWM2                      0x083
#define LED0_AEU1_PWM3                      0x084
#define LED0_AEU1_PWM4                      0x085
#define LED0_AEU1_PWM5                      0x086
#define LED0_AEU1_SlOPE_TIME1               0x087
#define LED0_AEU1_SlOPE_TIME2               0x088
#define LED0_AEU1_PT1                       0x089
#define LED0_AEU2_PWM1                      0x08A
#define LED0_AEU2_PWM2                      0x08B
#define LED0_AEU2_PWM3                      0x08C
#define LED0_AEU2_PWM4                      0x08D
#define LED0_AEU2_PWM5                      0x08E
#define LED0_AEU2_SlOPE_TIME1               0x08F
#define LED0_AEU2_SlOPE_TIME2               0x090
#define LED0_AEU2_PT1                       0x091
#define LED0_AEU3_PWM1                      0x092
#define LED0_AEU3_PWM2                      0x093
#define LED0_AEU3_PWM3                      0x094
#define LED0_AEU3_PWM4                      0x095
#define LED0_AEU3_PWM5                      0x096
#define LED0_AEU3_SlOPE_TIME1               0x097
#define LED0_AEU3_SlOPE_TIME2               0x098
#define LED0_AEU3_PT1                       0x099

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
int LP5812_BreathingAnimation(void);
int LP5812_stopAnimation(void);
int LP5812_manualControl(void);


#endif // LP5812_H