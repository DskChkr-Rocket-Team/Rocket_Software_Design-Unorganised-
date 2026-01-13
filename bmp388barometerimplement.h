/*
 * bmp388.c - BMP388 Barometric Pressure Sensor Driver Implementation
 */

#include "bmp388.h"
#include <math.h>

/* Private Variables ---------------------------------------------------------*/
static float prev_altitude = 0;

/* Private Function Prototypes -----------------------------------------------*/
static void BMP388_CS_Low(void);
static void BMP388_CS_High(void);
static float BMP388_CompensateTemperature(BMP388_t *bmp);
static float BMP388_CompensatePressure(BMP388_t *bmp, float t_lin);

/* CS Control ----------------------------------------------------------------*/
static void BMP388_CS_Low(void) {
    CS_LOW(BMP388_CS_PORT, BMP388_CS_PIN);
}

static void BMP388_CS_High(void) {
    CS_HIGH(BMP388_CS_PORT, BMP388_CS_PIN);
}

/* Initialization ------------------------------------------------------------*/
HAL_StatusTypeDef BMP388_Init(BMP388_t *bmp, SPI_HandleTypeDef *hspi) {
    uint8_t chip_id;
    
    bmp->hspi = hspi;
    bmp->initialized = false;
    bmp->calibrated = false;
    bmp->error_count = 0;
    
    // Verify chip ID
    if (BMP388_ReadChipID(bmp, &chip_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (chip_id != BMP388_CHIP_ID) {
        return HAL_ERROR;
    }
    
    // Soft reset
    if (BMP388_Reset(bmp) != HAL_OK) {
        return HAL_ERROR;
    }
    
    HAL_Delay(10);
    
    // Read calibration data
    if (BMP388_ReadCalibData(bmp) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Configure sensor
    // Enable pressure and temperature measurement
    if (BMP388_WriteRegister(bmp, BMP388_REG_PWR_CTRL, 
        BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Set oversampling: 8x pressure, 1x temperature
    uint8_t osr = (BMP388_OSR_x8 << 0) | (BMP388_OSR_x1 << 3);
    if (BMP388_WriteRegister(bmp, BMP388_REG_OSR, osr) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Set ODR to 200 Hz
    if (BMP388_WriteRegister(bmp, BMP388_REG_ODR, BMP388_ODR_200_HZ) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Set IIR filter coefficient to 3
    if (BMP388_WriteRegister(bmp, BMP388_REG_CONFIG, BMP388_IIR_FILTER_3) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Set normal mode (continuous measurement)
    if (BMP388_SetMode(bmp, BMP388_PWR_MODE_NORMAL) != HAL_OK) {
        return HAL_ERROR;
    }
    
    HAL_Delay(50);
    
    bmp->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_Reset(BMP388_t *bmp) {
    return BMP388_WriteRegister(bmp, BMP388_REG_CHIP_ID, BMP388_SOFT_RESET);
}

HAL_StatusTypeDef BMP388_ReadChipID(BMP388_t *bmp, uint8_t *chip_id) {
    return BMP388_ReadRegister(bmp, BMP388_REG_CHIP_ID, chip_id, 1);
}

/* Configuration -------------------------------------------------------------*/
HAL_StatusTypeDef BMP388_SetMode(BMP388_t *bmp, uint8_t mode) {
    uint8_t pwr_ctrl;
    
    if (BMP388_ReadRegister(bmp, BMP388_REG_PWR_CTRL, &pwr_ctrl, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    
    pwr_ctrl = (pwr_ctrl & 0xCF) | mode;
    return BMP388_WriteRegister(bmp, BMP388_REG_PWR_CTRL, pwr_ctrl);
}

/* Calibration ---------------------------------------------------------------*/
HAL_StatusTypeDef BMP388_ReadCalibData(BMP388_t *bmp) {
    uint8_t calib_data[21];
    
    if (BMP388_ReadRegister(bmp, BMP388_REG_CALIB_DATA, calib_data, 21) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Parse calibration coefficients
    bmp->calib.T1 = (uint16_t)(calib_data[1] << 8) | calib_data[0];
    bmp->calib.T2 = (uint16_t)(calib_data[3] << 8) | calib_data[2];
    bmp->calib.T3 = (int8_t)calib_data[4];
    
    bmp->calib.P1 = (int16_t)(calib_data[6] << 8) | calib_data[5];
    bmp->calib.P2 = (int16_t)(calib_data[8] << 8) | calib_data[7];
    bmp->calib.P3 = (int8_t)calib_data[9];
    bmp->calib.P4 = (int8_t)calib_data[10];
    bmp->calib.P5 = (uint16_t)(calib_data[12] << 8) | calib_data[11];
    bmp->calib.P6 = (uint16_t)(calib_data[14] << 8) | calib_data[13];
    bmp->calib.P7 = (int8_t)calib_data[15];
    bmp->calib.P8 = (int8_t)calib_data[16];
    bmp->calib.P9 = (int16_t)(calib_data[18] << 8) | calib_data[17];
    bmp->calib.P10 = (int8_t)calib_data[19];
    bmp->calib.P11 = (int8_t)calib_data[20];
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_CalibrateGroundLevel(BMP388_t *bmp, uint16_t samples) {
    float sum = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (BMP388_Update(bmp) != HAL_OK) {
            return HAL_ERROR;
        }
        sum += bmp->pressure_Pa;
        HAL_Delay(10);
    }
    
    bmp->ground_pressure_Pa = sum / samples;
    bmp->ground_altitude_m = 0.0f;
    bmp->calibrated = true;
    prev_altitude = 0;
    
    return HAL_OK;
}

/* Data Reading --------------------------------------------------------------*/
HAL_StatusTypeDef BMP388_ReadRawData(BMP388_t *bmp) {
    uint8_t data[6];
    
    if (BMP388_ReadRegister(bmp, BMP388_REG_DATA_0, data, 6) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Parse pressure (24-bit)
    bmp->raw_pressure = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    
    // Parse temperature (24-bit)
    bmp->raw_temperature = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    
    return HAL_OK;
}

/* Temperature Compensation */
static float BMP388_CompensateTemperature(BMP388_t *bmp) {
    float partial_data1 = (float)(bmp->raw_temperature - (256.0f * bmp->calib.T1));
    float partial_data2 = bmp->calib.T2 * partial_data1;
    float partial_data3 = partial_data1 * partial_data1;
    float partial_data4 = partial_data3 * bmp->calib.T3;
    float t_lin = partial_data2 + partial_data4;
    
    bmp->temperature_C = t_lin / 100.0f;
    return t_lin;
}

/* Pressure Compensation */
static float BMP388_CompensatePressure(BMP388_t *bmp, float t_lin) {
    float partial_data1 = bmp->calib.P6 * t_lin;
    float partial_data2 = bmp->calib.P7 * (t_lin * t_lin);
    float partial_data3 = bmp->calib.P8 * (t_lin * t_lin * t_lin);
    float partial_out1 = bmp->calib.P5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = bmp->calib.P2 * t_lin;
    partial_data2 = bmp->calib.P3 * (t_lin * t_lin);
    partial_data3 = bmp->calib.P4 * (t_lin * t_lin * t_lin);
    float partial_out2 = bmp->raw_pressure * (bmp->calib.P1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = bmp->raw_pressure * bmp->raw_pressure;
    partial_data2 = bmp->calib.P9 + (bmp->calib.P10 * t_lin);
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + (bmp->raw_pressure * bmp->raw_pressure * bmp->raw_pressure) * bmp->calib.P11;
    
    bmp->pressure_Pa = partial_out1 + partial_out2 + partial_data4;
    return bmp->pressure_Pa;
}

HAL_StatusTypeDef BMP388_CompensateData(BMP388_t *bmp) {
    float t_lin = BMP388_CompensateTemperature(bmp);
    BMP388_CompensatePressure(bmp, t_lin);
    
    if (bmp->calibrated) {
        bmp->altitude_m = BMP388_PressureToAltitude(bmp->pressure_Pa, bmp->ground_pressure_Pa);
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_Update(BMP388_t *bmp) {
    if (!bmp->initialized) {
        return HAL_ERROR;
    }
    
    if (BMP388_ReadRawData(bmp) != HAL_OK) {
        bmp->error_count++;
        return HAL_ERROR;
    }
    
    if (BMP388_CompensateData(bmp) != HAL_OK) {
        bmp->error_count++;
        return HAL_ERROR;
    }
    
    bmp->last_update_ms = HAL_GetTick();
    return HAL_OK;
}

/* Altitude Calculation ------------------------------------------------------*/
float BMP388_PressureToAltitude(float pressure_Pa, float sea_level_Pa) {
    return 44330.0f * (1.0f - powf(pressure_Pa / sea_level_Pa, 0.1903f));
}

float BMP388_GetAltitudeAGL(BMP388_t *bmp) {
    if (!bmp->calibrated) {
        return 0.0f;
    }
    return bmp->altitude_m;
}

float BMP388_GetVerticalVelocity(BMP388_t *bmp, float dt) {
    if (!bmp->calibrated || dt <= 0) {
        return 0.0f;
    }
    
    float velocity = (bmp->altitude_m - prev_altitude) / dt;
    prev_altitude = bmp->altitude_m;
    
    return velocity;
}

/* Low-level SPI -------------------------------------------------------------*/
HAL_StatusTypeDef BMP388_ReadRegister(BMP388_t *bmp, uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef status;
    
    reg |= 0x80;  // Read bit
    
    BMP388_CS_Low();
    status = HAL_SPI_Transmit(bmp->hspi, &reg, 1, 100);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(bmp->hspi, data, len, 100);
    }
    BMP388_CS_High();
    
    return status;
}

HAL_StatusTypeDef BMP388_WriteRegister(BMP388_t *bmp, uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status;
    uint8_t tx[2] = {reg & 0x7F, data};  // Clear read bit
    
    BMP388_CS_Low();
    status = HAL_SPI_Transmit(bmp->hspi, tx, 2, 100);
    BMP388_CS_High();
    
    return status;
}