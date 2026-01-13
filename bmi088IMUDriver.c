/*
 * bmi088.h - BMI088 6-Axis IMU Driver (24g Accelerometer + 2000°/s Gyro)
 * Bosch BMI088 - High performance IMU with separate accel/gyro dies
 */

#ifndef __BMI088_H
#define __BMI088_H

#include "main.h"

/* BMI088 Accelerometer Registers */
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_DATA_START       0x12
#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

/* BMI088 Gyroscope Registers */
#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_DATA_START      0x02
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14

/* Chip IDs */
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

/* Accel Ranges */
#define BMI088_ACC_RANGE_3G         0x00
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_ACC_RANGE_12G        0x02
#define BMI088_ACC_RANGE_24G        0x03

/* Gyro Ranges */
#define BMI088_GYRO_RANGE_2000      0x00
#define BMI088_GYRO_RANGE_1000      0x01
#define BMI088_GYRO_RANGE_500       0x02
#define BMI088_GYRO_RANGE_250       0x03
#define BMI088_GYRO_RANGE_125       0x04

/* Data structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    
    // Raw data
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    int16_t raw_temp;
    
    // Calibrated data (SI units)
    float accel_x_mps2, accel_y_mps2, accel_z_mps2;  // m/s²
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;        // °/s
    float temperature_C;
    
    // Configuration
    float accel_scale;
    float gyro_scale;
    
    // Status
    bool initialized;
    uint32_t last_update_ms;
    uint32_t error_count;
} BMI088_t;

/* Function Prototypes */
HAL_StatusTypeDef BMI088_Init(BMI088_t *bmi, SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef BMI088_ReadAccel(BMI088_t *bmi);
HAL_StatusTypeDef BMI088_ReadGyro(BMI088_t *bmi);
HAL_StatusTypeDef BMI088_Update(BMI088_t *bmi);

#endif /* __BMI088_H */

/*
 * bmi088.c - Implementation
 */
 
#include "bmi088.h"

/* CS Control */
static void BMI088_Accel_CS_Low(void) {
    CS_LOW(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN);
}

static void BMI088_Accel_CS_High(void) {
    CS_HIGH(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN);
}

static void BMI088_Gyro_CS_Low(void) {
    CS_LOW(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
}

static void BMI088_Gyro_CS_High(void) {
    CS_HIGH(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
}

/* Initialization */
HAL_StatusTypeDef BMI088_Init(BMI088_t *bmi, SPI_HandleTypeDef *hspi) {
    uint8_t chip_id;
    
    bmi->hspi = hspi;
    bmi->initialized = false;
    
    // Initialize accelerometer
    // Read chip ID
    BMI088_Accel_CS_Low();
    uint8_t reg = BMI088_ACC_CHIP_ID | 0x80;
    HAL_SPI_Transmit(hspi, &reg, 1, 100);
    HAL_SPI_Receive(hspi, &chip_id, 1, 100);
    BMI088_Accel_CS_High();
    
    if (chip_id != BMI088_ACC_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }
    
    // Configure accelerometer
    // Set range to ±24g
    uint8_t acc_range[] = {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G};
    BMI088_Accel_CS_Low();
    HAL_SPI_Transmit(hspi, acc_range, 2, 100);
    BMI088_Accel_CS_High();
    
    // Set bandwidth to 1 kHz
    uint8_t acc_conf[] = {BMI088_ACC_CONF, 0xAC};  // 1600 Hz ODR, Normal mode
    BMI088_Accel_CS_Low();
    HAL_SPI_Transmit(hspi, acc_conf, 2, 100);
    BMI088_Accel_CS_High();
    
    // Power on accelerometer
    uint8_t pwr_ctrl[] = {BMI088_ACC_PWR_CTRL, 0x04};
    BMI088_Accel_CS_Low();
    HAL_SPI_Transmit(hspi, pwr_ctrl, 2, 100);
    BMI088_Accel_CS_High();
    
    HAL_Delay(5);
    
    // Initialize gyroscope
    BMI088_Gyro_CS_Low();
    reg = BMI088_GYRO_CHIP_ID | 0x80;
    HAL_SPI_Transmit(hspi, &reg, 1, 100);
    HAL_SPI_Receive(hspi, &chip_id, 1, 100);
    BMI088_Gyro_CS_High();
    
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }
    
    // Set gyro range to ±2000°/s
    uint8_t gyro_range[] = {BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000};
    BMI088_Gyro_CS_Low();
    HAL_SPI_Transmit(hspi, gyro_range, 2, 100);
    BMI088_Gyro_CS_High();
    
    // Set gyro bandwidth to 1 kHz
    uint8_t gyro_bw[] = {BMI088_GYRO_BANDWIDTH, 0x01};  // 1000 Hz ODR
    BMI088_Gyro_CS_Low();
    HAL_SPI_Transmit(hspi, gyro_bw, 2, 100);
    BMI088_Gyro_CS_High();
    
    // Set scaling factors
    bmi->accel_scale = 24.0f * 9.81f / 32768.0f;  // ±24g range
    bmi->gyro_scale = 2000.0f / 32768.0f;         // ±2000°/s range
    
    bmi->initialized = true;
    return HAL_OK;
}

/* Read Accelerometer Data */
HAL_StatusTypeDef BMI088_ReadAccel(BMI088_t *bmi) {
    uint8_t data[6];
    
    BMI088_Accel_CS_Low();
    uint8_t reg = BMI088_ACC_DATA_START | 0x80;
    HAL_SPI_Transmit(bmi->hspi, &reg, 1, 100);
    uint8_t dummy;
    HAL_SPI_Receive(bmi->hspi, &dummy, 1, 100);  // Dummy read for accel
    HAL_SPI_Receive(bmi->hspi, data, 6, 100);
    BMI088_Accel_CS_High();
    
    // Parse 16-bit signed data (LSB first)
    bmi->raw_accel_x = (int16_t)((data[1] << 8) | data[0]);
    bmi->raw_accel_y = (int16_t)((data[3] << 8) | data[2]);
    bmi->raw_accel_z = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to m/s²
    bmi->accel_x_mps2 = bmi->raw_accel_x * bmi->accel_scale;
    bmi->accel_y_mps2 = bmi->raw_accel_y * bmi->accel_scale;
    bmi->accel_z_mps2 = bmi->raw_accel_z * bmi->accel_scale;
    
    return HAL_OK;
}

/* Read Gyroscope Data */
HAL_StatusTypeDef BMI088_ReadGyro(BMI088_t *bmi) {
    uint8_t data[6];
    
    BMI088_Gyro_CS_Low();
    uint8_t reg = BMI088_GYRO_DATA_START | 0x80;
    HAL_SPI_Transmit(bmi->hspi, &reg, 1, 100);
    HAL_SPI_Receive(bmi->hspi, data, 6, 100);
    BMI088_Gyro_CS_High();
    
    // Parse 16-bit signed data (LSB first)
    bmi->raw_gyro_x = (int16_t)((data[1] << 8) | data[0]);
    bmi->raw_gyro_y = (int16_t)((data[3] << 8) | data[2]);
    bmi->raw_gyro_z = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to °/s
    bmi->gyro_x_dps = bmi->raw_gyro_x * bmi->gyro_scale;
    bmi->gyro_y_dps = bmi->raw_gyro_y * bmi->gyro_scale;
    bmi->gyro_z_dps = bmi->raw_gyro_z * bmi->gyro_scale;
    
    return HAL_OK;
}

/* Update Both Sensors */
HAL_StatusTypeDef BMI088_Update(BMI088_t *bmi) {
    if (!bmi->initialized) {
        return HAL_ERROR;
    }
    
    if (BMI088_ReadAccel(bmi) != HAL_OK) {
        bmi->error_count++;
        return HAL_ERROR;
    }
    
    if (BMI088_ReadGyro(bmi) != HAL_OK) {
        bmi->error_count++;
        return HAL_ERROR;
    }
    
    bmi->last_update_ms = HAL_GetTick();
    return HAL_OK;
}