cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32WB55xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32WBxx_HAL_Driver/Inc
    ../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32WBxx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32wbxx_it.c
    ../../Core/Src/stm32wbxx_hal_msp.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_adc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_rcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_ipcc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_lptim.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pcd_ex.c
    ../../Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_usb.c
    ../../Core/Src/system_stm32wbxx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32wb55xx_cm4.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


