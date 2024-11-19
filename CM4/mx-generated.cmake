# File automatically-generated by STM32CubeMX - Do not modify

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE 
	CORE_CM4 
	USE_HAL_DRIVER 
	STM32H745xx
    $<$<CONFIG:Debug>:DEBUG>
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ./Core/Inc
    ../Drivers/STM32H7xx_HAL_Driver/Inc
    ../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ../Drivers/CMSIS/Include
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ./Core/Src/main.c
    ./Core/Src/stm32h7xx_it.c
    ./Core/Src/stm32h7xx_hal_msp.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    ../Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c
    ./Core/Src/sysmem.c
    ./Core/Src/syscalls.c
    ./Core/Startup/startup_stm32h745xx_CM4.s
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} 
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()
