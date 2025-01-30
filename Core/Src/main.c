/**
  *****************************************************************************************
  * @file           : main.c
  * @brief          : Logic for oxidiser flow through injector for hybrid rocket motor
  * @author         : Atharva Kulkarni 
  *****************************************************************************************
**/  
 
/***** Includes *****************/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

/******* Initialize Handles *******/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
osThreadId defaultTaskHandle;
osThreadId defaultTaskHandle2;

/******* Function Definitions *****/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void start_sensor_read(void const * argument);
int ReadPressureSensor(int sensorIndex);
int ReadTemperatureSensor(int sensorIndex);

/************ Definitions **********/
#define PRESSURE_SENSOR_COUNT 6
#define TEMPERATURE_SENSOR_COUNT 12
#define FUSION_THRESHOLD 1000   
volatile uint8_t fused_value_flag = 0;

/* Pressure Sensors I2C Addresses */
const uint16_t pressureSensorI2CAddresses[PRESSURE_SENSOR_COUNT] = {
    0x40, 
    0x41, 
    0x42, 
    0x43, 
    0x44, 
    0x45  
};

/* Temperature Sensors I2C Addresses */
const uint16_t temperatureSensorI2CAddresses[TEMPERATURE_SENSOR_COUNT] = {
    0x46,  
    0x47,  
    0x48,  
    0x49,  
    0x4A,  
    0x4B,  
    0x4C,  
    0x4D,  
    0x4E,  
    0x4F,  
    0x50,  
    0x51   
};

/* Mutex for thread-safe flag access */
osMutexId flagMutexHandle;
osMutexDef(flagMutex);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(sensor_read, start_sensor_read, osPriorityNormal, 0, 128);
  defaultTaskHandle2 = osThreadCreate(osThread(sensor_read), NULL);

  osKernelStart();

  /* Should never go into forever while */
  while (1)
  {
    Error_Handler();
  }
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void const * argument)
{
    // Initialize the valve state
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    for(;;)
    {
        uint8_t flag_status = 0;

        // **Check the Flag Safely**
        osMutexWait(flagMutexHandle, osWaitForever);
        flag_status = fused_value_flag;
        osMutexRelease(flagMutexHandle);

        // **Control the Valve Based on the Flag**
        if (flag_status)
        {
            // Turn the valve on
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        }
        else
        {
            // Turn the valve off
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        }

        osDelay(50);
    }
}

/**
  * @brief  Function implementing reading from sensors.
  * @param  argument: Not used
  * @retval None
  */
void start_sensor_read(void const * argument)
{
    uint16_t pressure_values[PRESSURE_SENSOR_COUNT] = {0};
    uint16_t temperature_values[TEMPERATURE_SENSOR_COUNT] = {0};
    uint32_t fused_value = 0;

    // Initialize the mutex
    flagMutexHandle = osMutexCreate(osMutex(flagMutex));

    for(;;)
    {
        fused_value = 0;

        // **Read Pressure Sensors**
        for (int i = 0; i < PRESSURE_SENSOR_COUNT; i++)
        {
            // Replace with actual sensor reading
            pressure_values[i] = ReadPressureSensor(i);
            fused_value += pressure_values[i];
        }

        // **Read Temperature Sensors**
        for (int i = 0; i < TEMPERATURE_SENSOR_COUNT; i++)
        {
            // Replace with actual sensor reading
            temperature_values[i] = ReadTemperatureSensor(i);
            fused_value += temperature_values[i];
        }

        // **Data Fusion and Flag Setting**
        if (fused_value > FUSION_THRESHOLD)
        {
            // Lock mutex before modifying the flag
            osMutexWait(flagMutexHandle, osWaitForever);
            fused_value_flag = 1;
            osMutexRelease(flagMutexHandle);

            // If using semaphore
            // osSemaphoreRelease(flagSemHandle);
        }
        else
        {
            osMutexWait(flagMutexHandle, osWaitForever);
            fused_value_flag = 0;
            osMutexRelease(flagMutexHandle);

            // If using semaphore
            // osSemaphoreWait(flagSemHandle, 0);
        }

        // **Optional Debugging Indicator**
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        osDelay(100);
    }

    // Not necessary to terminate the thread in an infinite loop
    // osThreadTerminate(NULL);
}






int ReadPressureSensor(int sensorIndex)
{
    if (sensorIndex < 0 || sensorIndex >= PRESSURE_SENSOR_COUNT)
        return 0;

    uint16_t sensorValue = 0;
    uint8_t i2cBuffer[2] = {0};

    // **Read 2 bytes from the I2C device**
    if (HAL_I2C_Master_Receive(&hi2c1, pressureSensorI2CAddresses[sensorIndex] << 1, i2cBuffer, 2, HAL_MAX_DELAY) == HAL_OK)
    {
        // Combine the two bytes into a 16-bit value
        sensorValue = (i2cBuffer[0] << 8) | i2cBuffer[1];
    }
    else
    {
        // Handle I2C communication error
        Error_Handler();
    }

    return sensorValue;
}

int ReadTemperatureSensor(int sensorIndex)
{
    if (sensorIndex < 0 || sensorIndex >= PRESSURE_SENSOR_COUNT)
        return 0;

    uint16_t sensorValue = 0;
    uint8_t i2cBuffer[2] = {0};

    // **Read 2 bytes from the I2C device**
    if (HAL_I2C_Master_Receive(&hi2c1, pressureSensorI2CAddresses[sensorIndex] << 1, i2cBuffer, 2, HAL_MAX_DELAY) == HAL_OK)
    {
        // Combine the two bytes into a 16-bit value
        sensorValue = (i2cBuffer[0] << 8) | i2cBuffer[1];
    }
    else
    {
        // Handle I2C communication error
        Error_Handler();
    }

    return sensorValue;
}
/***** Initialize Communication Lines **************/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    Error_Handler();
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    Error_Handler();

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    Error_Handler();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM6) 
    HAL_IncTick();

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    printf("Error, in Error Handler");
  }
}