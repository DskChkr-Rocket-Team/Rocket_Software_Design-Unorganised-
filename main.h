/*
 * main.h - Flight Computer Main Header
 * STM32H750VBT6 Flight Computer
 * All sensors, data logging, pyro control
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Pin Definitions */

/* SPI2 - Sensors */
#define SPI2_SCK_PIN        GPIO_PIN_13
#define SPI2_SCK_PORT       GPIOB
#define SPI2_MISO_PIN       GPIO_PIN_14
#define SPI2_MISO_PORT      GPIOB
#define SPI2_MOSI_PIN       GPIO_PIN_15
#define SPI2_MOSI_PORT      GPIOB

/* BMP388 Barometer */
#define BMP388_CS_PIN       GPIO_PIN_0
#define BMP388_CS_PORT      GPIOB
#define BMP388_INT_PIN      GPIO_PIN_1
#define BMP388_INT_PORT     GPIOB

/* BMI088 IMU (24g) */
#define BMI088_ACCEL_CS_PIN     GPIO_PIN_12
#define BMI088_ACCEL_CS_PORT    GPIOB
#define BMI088_GYRO_CS_PIN      GPIO_PIN_2
#define BMI088_GYRO_CS_PORT     GPIOC
#define BMI088_INT1_PIN         GPIO_PIN_0
#define BMI088_INT1_PORT        GPIOA
#define BMI088_INT2_PIN         GPIO_PIN_1
#define BMI088_INT2_PORT        GPIOA

/* MMC5983MA Magnetometer */
#define MMC5983_CS_PIN      GPIO_PIN_12
#define MMC5983_CS_PORT     GPIOB
#define MMC5983_INT_PIN     GPIO_PIN_15
#define MMC5983_INT_PORT    GPIOC

/* LSM6DSVTR IMU (16g) */
#define LSM6_CS_PIN         GPIO_PIN_14
#define LSM6_CS_PORT        GPIOD
#define LSM6_INT1_PIN       GPIO_PIN_12
#define LSM6_INT1_PORT      GPIOD

/* H3LIS331DL High-G Accelerometer (400g) */
#define H3LIS_CS_PIN        GPIO_PIN_14
#define H3LIS_CS_PORT       GPIOE
#define H3LIS_INT1_PIN      GPIO_PIN_15
#define H3LIS_INT1_PORT     GPIOD

/* GPS UART - USART2 */
#define GPS_UART_TX_PIN     GPIO_PIN_5
#define GPS_UART_TX_PORT    GPIOD
#define GPS_UART_RX_PIN     GPIO_PIN_6
#define GPS_UART_RX_PORT    GPIOD
#define GPS_RESET_PIN       GPIO_PIN_4
#define GPS_RESET_PORT      GPIOD
#define GPS_PPS_PIN         GPIO_PIN_7
#define GPS_PPS_PORT        GPIOD

/* SDMMC - SD Card */
#define SDMMC_D0_PIN        GPIO_PIN_8
#define SDMMC_D0_PORT       GPIOC
#define SDMMC_D1_PIN        GPIO_PIN_9
#define SDMMC_D1_PORT       GPIOC
#define SDMMC_D2_PIN        GPIO_PIN_10
#define SDMMC_D2_PORT       GPIOC
#define SDMMC_D3_PIN        GPIO_PIN_11
#define SDMMC_D3_PORT       GPIOC
#define SDMMC_CK_PIN        GPIO_PIN_12
#define SDMMC_CK_PORT       GPIOC
#define SDMMC_CMD_PIN       GPIO_PIN_2
#define SDMMC_CMD_PORT      GPIOD

/* Pyro Channels */
#define PYRO1_PIN           GPIO_PIN_0
#define PYRO1_PORT          GPIOE
#define PYRO2_PIN           GPIO_PIN_1
#define PYRO2_PORT          GPIOE
#define PYRO3_PIN           GPIO_PIN_2
#define PYRO3_PORT          GPIOE
#define PYRO4_PIN           GPIO_PIN_3
#define PYRO4_PORT          GPIOE
#define PYRO5_PIN           GPIO_PIN_4
#define PYRO5_PORT          GPIOE
#define PYRO6_PIN           GPIO_PIN_5
#define PYRO6_PORT          GPIOE

/* Safety & Status */
#define ARM_SWITCH_PIN      GPIO_PIN_6
#define ARM_SWITCH_PORT     GPIOE
#define ARM_LED_PIN         GPIO_PIN_7
#define ARM_LED_PORT        GPIOE
#define BUZZER_PIN          GPIO_PIN_8
#define BUZZER_PORT         GPIOE

/* Status LEDs */
#define LED_POWER_PIN       GPIO_PIN_9
#define LED_POWER_PORT      GPIOE
#define LED_GPS_PIN         GPIO_PIN_10
#define LED_GPS_PORT        GPIOE
#define LED_ARMED_PIN       GPIO_PIN_11
#define LED_ARMED_PORT      GPIOE
#define LED_ERROR_PIN       GPIO_PIN_12
#define LED_ERROR_PORT      GPIOE
#define LED_LOGGING_PIN     GPIO_PIN_13
#define LED_LOGGING_PORT    GPIOE

/* ADC - Battery & Pyro Continuity */
#define BATTERY_VOLT_PIN    GPIO_PIN_5
#define BATTERY_VOLT_PORT   GPIOA
#define PYRO1_CONT_PIN      GPIO_PIN_0
#define PYRO1_CONT_PORT     GPIOA
#define PYRO2_CONT_PIN      GPIO_PIN_1
#define PYRO2_CONT_PORT     GPIOA
#define PYRO3_CONT_PIN      GPIO_PIN_4
#define PYRO3_CONT_PORT     GPIOA

/* Constants -----------------------------------------------------------------*/
#define FLIGHT_COMPUTER_VERSION "1.0.0"
#define MAX_FLIGHT_TIME_MS      300000  // 5 minutes max flight
#define LOG_RATE_HZ             200     // 200 Hz logging
#define SENSOR_RATE_HZ          1000    // 1 kHz sensor polling

/* Macros --------------------------------------------------------------------*/
#define CS_LOW(port, pin)   HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define CS_HIGH(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */