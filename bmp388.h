/*
 * bmp388.h - BMP388 Barometric Pressure Sensor Driver
 * Bosch BMP388 - High precision barometer
 * Interface: SPI
 */

#ifndef __BMP388_H
#define __BMP388_H

#include "main.h"

/* BMP388 Register Addresses -------------------------------------------------*/
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_ERR          0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA_0       0x04    // Pressure data (LSB first)
#define BMP388_REG_DATA_1       0x05
#define BMP388_REG_DATA_2       0x06
#define BMP388_REG_DATA_3       0x07    // Temperature data
#define BMP388_REG_DATA_4       0x08
#define BMP388_REG_DATA_5       0x09
#define BMP388_REG_SENSORTIME_0 0x0C
#define BMP388_REG_EVENT        0x10
#define BMP388_REG_INT_STATUS   0x11
#define BMP388_REG_INT_CTRL     0x19
#define BMP388_REG_IF_CONF      0x1A
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB_DATA   0x31    // Calibration coefficients start

/* BMP388 Constants ----------------------------------------------------------*/
#define BMP388_CHIP_ID          0x50
#define BMP388_SOFT_RESET       0xB6

/* Power Control */
#define BMP388_PWR_PRESS_EN     (1 << 0)
#define BMP388_PWR_TEMP_EN      (1 << 1)
#define BMP388_PWR_MODE_SLEEP   (0 << 4)
#define BMP388_PWR_MODE_FORCED  (1 << 4)
#define BMP388_PWR_MODE_NORMAL  (3 << 4)

/* Output Data Rate (ODR) */
#define BMP388_ODR_200_HZ       0x00
#define BMP388_ODR_100_HZ       0x01
#define BMP388_ODR_50_HZ        0x02
#define BMP388_ODR_25_HZ        0x03

/* IIR Filter Coefficient */
#define BMP388_IIR_FILTER_0     (0 << 1)    // Bypass
#define BMP388_IIR_FILTER_1     (1 << 1)
#define BMP388_IIR_FILTER_3     (2 << 1)
#define BMP388_IIR_FILTER_7     (3 << 1)
#define BMP388_IIR_FILTER_15    (4 << 1)
#define BMP388_IIR_FILTER_31    (5 << 1)
#define BMP388_IIR_FILTER_63    (6 << 1)
#define BMP388_IIR_FILTER_127   (7 << 1)

/* Oversampling Rates */
#define BMP388_OSR_x1           0x00
#define BMP388_OSR_x2           0x01
#define BMP388_OSR_x4           0x02
#define BMP388_OSR_x8           0x03
#define BMP388_OSR_x16          0x04
#define BMP388_OSR_x32          0x05

/* Data Structures -----------------------------------------------------------*/

/* Calibration coefficients */
typedef struct {
    uint16_t T1;
    uint16_t T2;
    int8_t   T3;
    int16_t  P1;
    int16_t  P2;
    int8_t   P3;
    int8_t   P4;
    uint16_t P5;
    uint16_t P6;
    int8_t   P7;
    int8_t   P8;
    int16_t  P9;
    int8_t   P10;
    int8_t   P11;
} BMP388_CalibData_t;

/* BMP388 data structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    
    // Calibration data
    BMP388_CalibData_t calib;
    
    // Raw data
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    
    // Compensated data
    float temperature_C;
    float pressure_Pa;
    float altitude_m;
    
    // Ground reference
    float ground_pressure_Pa;
    float ground_altitude_m;
    
    // Status
    bool initialized;
    bool calibrated;
    uint32_t last_update_ms;
    uint32_t error_count;
} BMP388_t;

/* Function Prototypes -------------------------------------------------------*/

/* Initialization */
HAL_StatusTypeDef BMP388_Init(BMP388_t *bmp, SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef BMP388_Reset(BMP388_t *bmp);
HAL_StatusTypeDef BMP388_ReadChipID(BMP388_t *bmp, uint8_t *chip_id);

/* Configuration */
HAL_StatusTypeDef BMP388_SetMode(BMP388_t *bmp, uint8_t mode);
HAL_StatusTypeDef BMP388_SetOSR(BMP388_t *bmp, uint8_t press_osr, uint8_t temp_osr);
HAL_StatusTypeDef BMP388_SetODR(BMP388_t *bmp, uint8_t odr);
HAL_StatusTypeDef BMP388_SetIIRFilter(BMP388_t *bmp, uint8_t filter);

/* Data Reading */
HAL_StatusTypeDef BMP388_ReadRawData(BMP388_t *bmp);
HAL_StatusTypeDef BMP388_CompensateData(BMP388_t *bmp);
HAL_StatusTypeDef BMP388_Update(BMP388_t *bmp);

/* Calibration */
HAL_StatusTypeDef BMP388_ReadCalibData(BMP388_t *bmp);
HAL_StatusTypeDef BMP388_CalibrateGroundLevel(BMP388_t *bmp, uint16_t samples);

/* Altitude Calculation */
float BMP388_PressureToAltitude(float pressure_Pa, float sea_level_Pa);
float BMP388_GetAltitudeAGL(BMP388_t *bmp);
float BMP388_GetVerticalVelocity(BMP388_t *bmp, float dt);

/* Low-level SPI */
HAL_StatusTypeDef BMP388_ReadRegister(BMP388_t *bmp, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef BMP388_WriteRegister(BMP388_t *bmp, uint8_t reg, uint8_t data);

#endif /* __BMP388_H */