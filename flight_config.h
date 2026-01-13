/*
 * flight_config.h - Flight Computer Configuration
 * Adjust these values for your specific rocket
 */

#ifndef __FLIGHT_CONFIG_H
#define __FLIGHT_CONFIG_H

/* Rocket Configuration ------------------------------------------------------*/
#define ROCKET_NAME             "TestRocket_01"
#define LAUNCH_SITE             "BothellWA"
#define EXPECTED_APOGEE_M       1000.0f     // Expected apogee in meters

/* Sensor Configuration ------------------------------------------------------*/

/* BMP388 Barometer */
#define BMP388_ENABLED          1
#define BMP388_SAMPLE_RATE_HZ   200
#define BMP388_OSR_PRESSURE     8           // Oversampling: 8x
#define BMP388_OSR_TEMP         1           // Oversampling: 1x
#define BMP388_IIR_FILTER       3           // IIR filter coefficient

/* BMI088 IMU (Primary - 24g) */
#define BMI088_ENABLED          1
#define BMI088_SAMPLE_RATE_HZ   1000
#define BMI088_ACCEL_RANGE      24          // ±24g
#define BMI088_GYRO_RANGE       2000        // ±2000°/s

/* MMC5983MA Magnetometer */
#define MMC5983_ENABLED         1
#define MMC5983_SAMPLE_RATE_HZ  100
#define MMC5983_AUTO_SR         1           // Auto SET/RESET

/* LSM6DSVTR IMU (Secondary - 16g) */
#define LSM6_ENABLED            1
#define LSM6_SAMPLE_RATE_HZ     500
#define LSM6_ACCEL_RANGE        16          // ±16g
#define LSM6_GYRO_RANGE         2000        // ±2000°/s

/* H3LIS331DL High-G Accelerometer (400g) */
#define H3LIS_ENABLED           1
#define H3LIS_SAMPLE_RATE_HZ    1000
#define H3LIS_RANGE             400         // ±400g

/* GPS */
#define GPS_ENABLED             1
#define GPS_BAUD_RATE           38400       // M10Q default
#define GPS_UPDATE_RATE_HZ      10

/* Data Logging --------------------------------------------------------------*/
#define LOGGING_ENABLED         1
#define LOG_BUFFER_SIZE         512         // Log buffer in KB
#define LOG_FILE_PREFIX         "FLIGHT_"
#define LOG_FILE_EXTENSION      ".csv"

/* Flight State Machine ------------------------------------------------------*/

/* Launch Detection */
#define LAUNCH_ACCEL_THRESHOLD_G    3.0f    // 3G acceleration = launch
#define LAUNCH_ACCEL_DURATION_MS    100     // Must sustain for 100ms

/* Apogee Detection */
#define APOGEE_VELOCITY_THRESHOLD   -2.0f   // -2 m/s = descending
#define APOGEE_ACCEL_THRESHOLD      -5.0f   // -5 m/s² = freefall
#define APOGEE_DETECTION_DELAY_MS   200     // Confirmation delay

/* Main Chute Deployment */
#define MAIN_DEPLOY_ALTITUDE_AGL_M  300.0f  // Deploy at 300m AGL
#define MAIN_DEPLOY_VELOCITY_MAX    -20.0f  // Don't deploy if falling faster

/* Landing Detection */
#define LANDING_ALTITUDE_CHANGE_M   2.0f    // <2m change in 5 seconds
#define LANDING_ACCEL_THRESHOLD     1.2f    // ~1g stable

/* Pyrotechnic Control -------------------------------------------------------*/
#define PYRO_ENABLED            1

/* Channel Assignments */
#define PYRO_DROGUE_CHANNEL     1           // Drogue parachute
#define PYRO_DROGUE_BACKUP      2           // Drogue backup
#define PYRO_MAIN_CHANNEL       3           // Main parachute
#define PYRO_MAIN_BACKUP        4           // Main backup
#define PYRO_SEPARATION_CHANNEL 5           // Stage separation
#define PYRO_SPARE_CHANNEL      6           // Spare

/* Firing Parameters */
#define PYRO_FIRE_DURATION_MS   500         // 500ms pulse
#define PYRO_MIN_CONTINUITY_OHM 0.5f        // Min e-match resistance
#define PYRO_MAX_CONTINUITY_OHM 5.0f        // Max e-match resistance

/* Safety Interlocks */
#define REQUIRE_HW_ARM_SWITCH   1           // Hardware arming required
#define REQUIRE_GPS_LOCK        0           // GPS lock required (optional)
#define REQUIRE_CONTINUITY      1           // E-match continuity required
#define MIN_PAD_TIME_SEC        10          // Min time on pad before arm

/* Battery Monitoring --------------------------------------------------------*/
#define BATTERY_CELLS           4           // 4S LiPo (14.8V nominal)
#define BATTERY_VOLTAGE_MIN     12.0f       // Minimum safe voltage
#define BATTERY_VOLTAGE_WARN    13.0f       // Warning voltage
#define BATTERY_VOLTAGE_SCALE   5.7f        // ADC scaling factor

/* Buzzer & LED Patterns -----------------------------------------------------*/
#define BUZZER_POWER_ON_BEEPS       2
#define BUZZER_GPS_LOCK_BEEPS       3
#define BUZZER_ARMED_BEEPS          5
#define BUZZER_ERROR_CONTINUOUS     1
#define BUZZER_LANDING_PATTERN      1       // Continuous for recovery

#define LED_BLINK_RATE_NORMAL_MS    1000
#define LED_BLINK_RATE_FAST_MS      250
#define LED_ARMED_BLINK_MS          500

/* Failsafes -----------------------------------------------------------------*/
#define ENABLE_WATCHDOG         1
#define WATCHDOG_TIMEOUT_MS     1000

#define MAX_APOGEE_TIME_S       60          // Max time to apogee
#define FORCE_MAIN_DEPLOY_S     120         // Force main at 2 minutes

/* Debug & Testing -----------------------------------------------------------*/
#define DEBUG_UART_ENABLED      1           // Enable debug output
#define DEBUG_BAUD_RATE         115200
#define TEST_MODE_ENABLED       0           // Disable pyros in test mode
#define VERBOSE_LOGGING         1           // Extra log details

#endif /* __FLIGHT_CONFIG_H */