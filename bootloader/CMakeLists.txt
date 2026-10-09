cmake_minimum_required(VERSION 3.22)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
project(Guided_dart_bootloader C ASM)

set(APP_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(CMAKE_C_STANDARD 11)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork
    -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections
    -fdata-sections -fno-common -Wall -Wextra -Wshadow
    -Wdouble-promotion -Os)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork
    -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--gc-sections
    -T${CMAKE_CURRENT_LIST_DIR}/bootloader.ld)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${APP_DIR}/Core/Inc
    ${APP_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${APP_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${APP_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${APP_DIR}/Drivers/CMSIS/Include ${APP_DIR}/component)
add_definitions(-DSTM32G431xx -DUSE_HAL_DRIVER -DBOOTLOADER_BUILD)

file(GLOB HAL_SOURCES "${APP_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/*.c")
set(SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/boot_main.c
    ${CMAKE_CURRENT_LIST_DIR}/src/boot_external_storage.c
    ${CMAKE_CURRENT_LIST_DIR}/src/boot_msp.c
    ${CMAKE_CURRENT_LIST_DIR}/src/boot_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/src/boot_isr.c
    ${APP_DIR}/Core/Src/system_stm32g4xx.c
    ${APP_DIR}/Core/Startup/startup_stm32g431cbux.s
    ${APP_DIR}/component/lora_protocol.c
    ${APP_DIR}/component/external_flash.c
    ${APP_DIR}/Core/Src/gpio.c
    ${APP_DIR}/Core/Src/spi.c ${HAL_SOURCES})

add_executable(${PROJECT_NAME}.elf ${SOURCES})
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)
