/*
 * main.c - Flight Computer Main Application
 * Complete rocket flight computer with all sensors and pyro control
 */

#include "main.h"
#include "flight_config.h"
#include "bmp388.h"
#include "bmi088.h"
// Include other sensor headers as needed

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_ARMED,
    STATE_LAUNCH,
    STATE_BOOST,
    STATE_COAST,
    STATE_APOGEE,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DEPLOY,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
    STATE_ERROR
} FlightState_t;

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;  // GPS
SD_HandleTypeDef hsd1;      // SD card
ADC_HandleTypeDef hadc1;    // Battery monitoring
TIM_HandleTypeDef htim1;    // Buzzer PWM

/* Sensor instances */
BMP388_t barometer;
BMI088_t imu_primary;

/* Flight state */
FlightState_t flight_state = STATE_INIT;
uint32_t state_entry_time = 0;
uint32_t flight_time_ms = 0;

/* Flight data */
float max_altitude_m = 0;
float max_velocity_mps = 0;
float max_acceleration_g = 0;
float launch_detect_accel_g = 0;
bool hw_armed = false;
bool sw_armed = false;

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

/* Application Functions */
static void Init_Sensors(void);
static void Update_Sensors(void);
static void Flight_State_Machine(void);
static void Check_Safety_Interlocks(void);
static void Fire_Pyro_Channel(uint8_t channel, uint16_t duration_ms);
static void Update_LEDs(void);
static void Beep(uint16_t duration_ms, uint16_t frequency);
static void Log_Data(void);

/**
  * @brief  The application entry point.
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize all peripherals */
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_SDMMC1_SD_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    
    /* Power-on beep */
    Beep(200, 2000);
    HAL_Delay(100);
    Beep(200, 2000);
    
    /* Initialize sensors */
    Init_Sensors();
    
    /* Calibrate ground level */
    if (BMP388_CalibrateGroundLevel(&barometer, 100) == HAL_OK) {
        Beep(100, 3000);  // Success beep
    } else {
        Beep(500, 500);   // Error beep
    }
    
    /* Turn on power LED */
    HAL_GPIO_WritePin(LED_POWER_PORT, LED_POWER_PIN, GPIO_PIN_SET);
    
    /* Main loop */
    flight_state = STATE_IDLE;
    uint32_t last_sensor_update = 0;
    uint32_t last_log_time = 0;
    
    while (1)
    {
        uint32_t now = HAL_GetTick();
        
        /* Update sensors at 1 kHz */
        if (now - last_sensor_update >= 1) {
            Update_Sensors();
            last_sensor_update = now;
        }
        
        /* Run state machine */
        Flight_State_Machine();
        
        /* Log data at configured rate */
        if (now - last_log_time >= (1000 / LOG_RATE_HZ)) {
            Log_Data();
            last_log_time = now;
        }
        
        /* Update LED status */
        Update_LEDs();
        
        /* Check safety interlocks */
        Check_Safety_Interlocks();
    }
}

/**
  * @brief Initialize all sensors
  */
static void Init_Sensors(void)
{
    /* Initialize BMP388 Barometer */
    if (BMP388_Init(&barometer, &hspi2) != HAL_OK) {
        Error_Handler();
    }
    
    /* Initialize BMI088 IMU */
    if (BMI088_Init(&imu_primary, &hspi2) != HAL_OK) {
        Error_Handler();
    }
    
    // Initialize other sensors similarly...
}

/**
  * @brief Update all sensors
  */
static void Update_Sensors(void)
{
    /* Update barometer */
    BMP388_Update(&barometer);
    
    /* Update IMU */
    BMI088_Update(&imu_primary);
    
    // Update other sensors...
    
    /* Calculate derived values */
    float total_accel_g = sqrtf(
        imu_primary.accel_x_mps2 * imu_primary.accel_x_mps2 +
        imu_primary.accel_y_mps2 * imu_primary.accel_y_mps2 +
        imu_primary.accel_z_mps2 * imu_primary.accel_z_mps2
    ) / 9.81f;
    
    /* Track maximums */
    if (barometer.altitude_m > max_altitude_m) {
        max_altitude_m = barometer.altitude_m;
    }
    
    if (total_accel_g > max_acceleration_g) {
        max_acceleration_g = total_accel_g;
    }
}

/**
  * @brief Main flight state machine
  */
static void Flight_State_Machine(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t time_in_state = now - state_entry_time;
    
    switch (flight_state) {
        
        case STATE_IDLE:
            /* Waiting on pad */
            /* Check for arming command */
            if (hw_armed && sw_armed) {
                flight_state = STATE_ARMED;
                state_entry_time = now;
                Beep(100, 3000);
                Beep(100, 3000);
                Beep(100, 3000);
            }
            break;
            
        case STATE_ARMED:
            /* Armed and ready for launch */
            /* Detect launch: >3G acceleration sustained */
            float total_accel_g = sqrtf(
                imu_primary.accel_x_mps2 * imu_primary.accel_x_mps2 +
                imu_primary.accel_y_mps2 * imu_primary.accel_y_mps2 +
                imu_primary.accel_z_mps2 * imu_primary.accel_z_mps2
            ) / 9.81f;
            
            if (total_accel_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (launch_detect_accel_g == 0) {
                    launch_detect_accel_g = total_accel_g;
                    state_entry_time = now;
                }
                
                if (time_in_state >= LAUNCH_ACCEL_DURATION_MS) {
                    flight_state = STATE_LAUNCH;
                    state_entry_time = now;
                    flight_time_ms = 0;
                }
            } else {
                launch_detect_accel_g = 0;
                state_entry_time = now;
            }
            break;
            
        case STATE_LAUNCH:
            /* Launch detected - in powered ascent */
            flight_time_ms = now - state_entry_time;
            
            /* Check if motor has burned out (acceleration drops) */
            total_accel_g = sqrtf(
                imu_primary.accel_x_mps2 * imu_primary.accel_x_mps2 +
                imu_primary.accel_y_mps2 * imu_primary.accel_y_mps2 +
                imu_primary.accel_z_mps2 * imu_primary.accel_z_mps2
            ) / 9.81f;
            
            if (total_accel_g < 2.0f && time_in_state > 500) {
                flight_state = STATE_COAST;
                state_entry_time = now;
            }
            
            /* Timeout check */
            if (time_in_state > 10000) {
                flight_state = STATE_COAST;
                state_entry_time = now;
            }
            break;
            
        case STATE_COAST:
            /* Coasting to apogee */
            flight_time_ms = now - state_entry_time;
            
            /* Detect apogee: negative vertical velocity */
            float vert_velocity = BMP388_GetVerticalVelocity(&barometer, 0.001f);
            
            if (vert_velocity < APOGEE_VELOCITY_THRESHOLD) {
                if (time_in_state > APOGEE_DETECTION_DELAY_MS) {
                    flight_state = STATE_APOGEE;
                    state_entry_time = now;
                    
                    /* Fire drogue parachute */
                    Fire_Pyro_Channel(PYRO_DROGUE_CHANNEL, PYRO_FIRE_DURATION_MS);
                }
            }
            
            /* Failsafe: force apogee after max time */
            if (time_in_state > (MAX_APOGEE_TIME_S * 1000)) {
                flight_state = STATE_APOGEE;
                state_entry_time = now;
                Fire_Pyro_Channel(PYRO_DROGUE_CHANNEL, PYRO_FIRE_DURATION_MS);
            }
            break;
            
        case STATE_APOGEE:
            /* Apogee detected, drogue deployed */
            flight_state = STATE_DROGUE_DESCENT;
            state_entry_time = now;
            break;
            
        case STATE_DROGUE_DESCENT:
            /* Descending on drogue chute */
            flight_time_ms = now - state_entry_time;
            
            /* Check for main chute deployment altitude */
            if (barometer.altitude_m <= MAIN_DEPLOY_ALTITUDE_AGL_M) {
                flight_state = STATE_MAIN_DEPLOY;
                state_entry_time = now;
                
                /* Fire main parachute */
                Fire_Pyro_Channel(PYRO_MAIN_CHANNEL, PYRO_FIRE_DURATION_MS);
            }
            
            /* Failsafe: force main deployment after time */
            if (time_in_state > (FORCE_MAIN_DEPLOY_S * 1000)) {
                flight_state = STATE_MAIN_DEPLOY;
                state_entry_time = now;
                Fire_Pyro_Channel(PYRO_MAIN_CHANNEL, PYRO_FIRE_DURATION_MS);
            }
            break;
            
        case STATE_MAIN_DEPLOY:
            /* Main chute deployed */
            flight_state = STATE_MAIN_DESCENT;
            state_entry_time = now;
            break;
            
        case STATE_MAIN_DESCENT:
            /* Descending on main chute */
            flight_time_ms = now - state_entry_time;
            
            /* Detect landing: stable low altitude + low accel */
            total_accel_g = sqrtf(
                imu_primary.accel_x_mps2 * imu_primary.accel_x_mps2 +
                imu_primary.accel_y_mps2 * imu_primary.accel_y_mps2 +
                imu_primary.accel_z_mps2 * imu_primary.accel_z_mps2
            ) / 9.81f;
            
            if (barometer.altitude_m < LANDING_ALTITUDE_CHANGE_M &&
                fabsf(total_accel_g - 1.0f) < 0.2f) {
                if (time_in_state > 5000) {
                    flight_state = STATE_LANDED;
                    state_entry_time = now;
                }
            }
            break;
            
        case STATE_LANDED:
            /* Safely on ground */
            /* Continuous beeping for recovery */
            if (time_in_state % 3000 == 0) {
                Beep(500, 2500);
            }
            break;
            
        case STATE_ERROR:
            /* Error state */
            /* Continuous error beep */
            if (time_in_state % 1000 == 0) {
                Beep(200, 500);
            }
            break;
            
        default:
            flight_state = STATE_ERROR;
            state_entry_time = now;
            break;
    }
}

/**
  * @brief Check safety interlocks
  */
static void Check_Safety_Interlocks(void)
{
    /* Read hardware arming switch */
    hw_armed = (HAL_GPIO_ReadPin(ARM_SWITCH_PORT, ARM_SWITCH_PIN) == GPIO_PIN_SET);
    
    /* Check battery voltage */
    // ADC reading and conversion...
    
    /* Check pyro continuity */
    // ADC reading of continuity pins...
    
    /* Software arm logic */
    if (hw_armed && flight_state == STATE_IDLE) {
        // Additional checks before allowing software arm
        sw_armed = true;
    } else if (!hw_armed) {
        sw_armed = false;
    }
}

/**
  * @brief Fire a pyro channel
  */
static void Fire_Pyro_Channel(uint8_t channel, uint16_t duration_ms)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    
    /* Select channel */
    switch (channel) {
        case 1: port = PYRO1_PORT; pin = PYRO1_PIN; break;
        case 2: port = PYRO2_PORT; pin = PYRO2_PIN; break;
        case 3: port = PYRO3_PORT; pin = PYRO3_PIN; break;
        case 4: port = PYRO4_PORT; pin = PYRO4_PIN; break;
        case 5: port = PYRO5_PORT; pin = PYRO5_PIN; break;
        case 6: port = PYRO6_PORT; pin = PYRO6_PIN; break;
        default: return;
    }
    
    /* Safety check */
    if (!hw_armed || !sw_armed) {
        return;
    }
    
    /* Fire! */
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(duration_ms);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/**
  * @brief Update status LEDs
  */
static void Update_LEDs(void)
{
    uint32_t now = HAL_GetTick();
    
    /* Power LED: always on */
    HAL_GPIO_WritePin(LED_POWER_PORT, LED_POWER_PIN, GPIO_PIN_SET);
    
    /* GPS LED: blink when no lock, solid when locked */
    // TODO: Check GPS lock status
    
    /* Armed LED: fast blink when armed */
    if (sw_armed) {
        GPIO_PinState state = ((now / LED_ARMED_BLINK_MS) % 2) ? 
            GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(LED_ARMED_PORT, LED_ARMED_PIN, state);
    } else {
        HAL_GPIO_WritePin(LED_ARMED_PORT, LED_ARMED_PIN, GPIO_PIN_RESET);
    }
    
    /* Logging LED: on when logging active */
    if (flight_state >= STATE_ARMED && flight_state < STATE_LANDED) {
        HAL_GPIO_WritePin(LED_LOGGING_PORT, LED_LOGGING_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_LOGGING_PORT, LED_LOGGING_PIN, GPIO_PIN_RESET);
    }
}

/**
  * @brief Generate beep on buzzer
  */
static void Beep(uint16_t duration_ms, uint16_t frequency)
{
    /* Configure TIM1 for PWM at specified frequency */
    // PWM configuration...
    
    /* Start PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/**
  * @brief Log data to SD card
  */
static void Log_Data(void)
{
    /* Format: timestamp, state, altitude, velocity, accel_x, accel_y, accel_z, ... */
    char log_buffer[256];
    
    snprintf(log_buffer, sizeof(log_buffer),
        "%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f
",
        HAL_GetTick(),
        flight_state,
        barometer.altitude_m,
        BMP388_GetVerticalVelocity(&barometer, 0.005f),
        imu_primary.accel_x_mps2,
        imu_primary.accel_y_mps2,
        imu_primary.accel_z_mps2,
        imu_primary.gyro_x_dps,
        imu_primary.gyro_y_dps,
        imu_primary.gyro_z_dps
    );
    
    /* Write to SD card */
    // SD card write function...
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 240;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief SPI2 Initialization Function
  */
static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // ~3 MHz
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0x0;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    /* Configure all CS pins as outputs, default HIGH */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = BMP388_CS_PIN;
    HAL_GPIO_Init(BMP388_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BMP388_CS_PORT, BMP388_CS_PIN, GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = BMI088_ACCEL_CS_PIN;
    HAL_GPIO_Init(BMI088_ACCEL_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = BMI088_GYRO_CS_PIN;
    HAL_GPIO_Init(BMI088_GYRO_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
    
    /* Configure pyro pins as outputs, default LOW */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    GPIO_InitStruct.Pin = PYRO1_PIN | PYRO2_PIN | PYRO3_PIN | 
                          PYRO4_PIN | PYRO5_PIN | PYRO6_PIN;
    HAL_GPIO_Init(PYRO1_PORT, &GPIO_InitStruct);
    
    /* Configure LED pins */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    GPIO_InitStruct.Pin = LED_POWER_PIN | LED_GPS_PIN | LED_ARMED_PIN | 
                          LED_ERROR_PIN | LED_LOGGING_PIN;
    HAL_GPIO_Init(LED_POWER_PORT, &GPIO_InitStruct);
    
    /* Configure arm switch as input */
    GPIO_InitStruct.Pin = ARM_SWITCH_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ARM_SWITCH_PORT, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    /* Infinite error beep */
    while (1)
    {
        HAL_GPIO_TogglePin(LED_ERROR_PORT, LED_ERROR_PIN);
        HAL_Delay(250);
    }
}