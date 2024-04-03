/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/* USER CODE BEGIN 1 */
// Define the L3GD20's WHO_AM_I register address and the expected value
		#define WHO_AM_I_REG 0x0F
		#define WHO_AM_I_VALUE 0xD4
		#define L3GD20_ADDRESS 0x6B << 1 // The device's 7-bit address shifted left with the R/W bit clear
/* GPIO Initialization */
void MX_GPIO_Init(void)
{
    /* Enable GPIOB and GPIOC clocks */
    RCC->AHBENR |= (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN);

    /* I2C2 SDA and SCL configuration (PB11 and PB13) */
    GPIOB->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13); // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1); // Set to alternate function mode
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13); // Set to open-drain
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR13); // Set to high speed
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR13); // No pull-up, no pull-down
    GPIOB->AFR[1] |= (4 << (4 * (11 - 8))) | (4 << (4 * (13 - 8))); // Alternate function 4 (I2C2)

    /* Additional GPIO configuration for L3GD20 (PB14 and PC0) */
    GPIOB->MODER &= ~GPIO_MODER_MODER14; // Clear mode bits for PB14
    GPIOB->MODER |= GPIO_MODER_MODER14_0; // Set PB14 to output mode
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14; // Set PB14 to push-pull output
    GPIOB->ODR |= GPIO_ODR_14; // Set PB14 high

    GPIOC->MODER &= ~GPIO_MODER_MODER0; // Clear mode bits for PC0
    GPIOC->MODER |= GPIO_MODER_MODER0_0; // Set PC0 to output mode
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0; // Set PC0 to push-pull output
    GPIOC->ODR |= GPIO_ODR_0; // Set PC0 high
		
		// Add LED initialization (example for an LED on PC8)
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable clock for GPIOC
		GPIOC->MODER &= ~GPIO_MODER_MODER8; // Clear mode
		GPIOC->MODER |= GPIO_MODER_MODER8_0; // Set to output mode
		GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; // Push-pull
		GPIOC->ODR &= ~GPIO_ODR_8; // Ensure LED is off
}

/* I2C2 Initialization */
void MX_I2C2_Init(void)
{
    /* Enable the I2C2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    /* I2C2 Timing configuration for 100 kHz on a 48MHz SYSCLK */
    /* This presumes the SYSCLK frequency is 48 MHz; adjust as needed */
    I2C2->TIMINGR = (uint32_t)0x10420F13; // Calculated for 48MHz clock and 100kHz I2C speed

    /* Enable the I2C2 peripheral */
    I2C2->CR1 |= I2C_CR1_PE;
}

/* Main program */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C2_Init();

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE BEGIN 3 */

				

				uint8_t readValue;

				// Start I2C communication
				I2C2->CR2 = L3GD20_ADDRESS | (1 << 16); // Set the slave address and the number of bytes to transmit
				I2C2->CR2 |= I2C_CR2_START; // Generate the start condition

				// Wait until the transmit buffer is empty (TXIS flag) or NACKF flag is set
				while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF));

				// Check if a NACK was received
				if (I2C2->ISR & I2C_ISR_NACKF)
				{
						// Handle error and possibly reset the I2C bus
				}

				// Write the WHO_AM_I register address to the transmit data register
				I2C2->TXDR = WHO_AM_I_REG & I2C_TXDR_TXDATA;

				// Wait until the transfer is complete (TC flag)
				while (!(I2C2->ISR & I2C_ISR_TC));

				// Now read from the WHO_AM_I register
				I2C2->CR2 = L3GD20_ADDRESS | I2C_CR2_RD_WRN | (1 << 16); // Set the slave address and number of bytes to read
				I2C2->CR2 |= I2C_CR2_START; // Generate the repeated start condition

				// Wait until the receive buffer is not empty (RXNE flag)
				while (!(I2C2->ISR & I2C_ISR_RXNE));

				// Read the value from the receive data register
				readValue = (uint8_t)(I2C2->RXDR & I2C_RXDR_RXDATA);

				// Validate the read value
				if (readValue == WHO_AM_I_VALUE)
				{
						// Correct WHO_AM_I value received
						// Turn on LED
						GPIOC->ODR |= GPIO_ODR_8;
				}
				else
				{
						// Incorrect WHO_AM_I value received
						// Turn off LED
						GPIOC->ODR &= ~GPIO_ODR_8;
				}

				// Generate the stop condition
				I2C2->CR2 |= I2C_CR2_STOP;

				/* USER CODE END 3 */

    }
}

/* USER CODE END 1 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
