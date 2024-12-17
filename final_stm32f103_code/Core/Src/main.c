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
#define IMAGE_SIZE 64
#define IMAGE_DATA_SIZE 4096
#define MAX_PWM_BITS 3
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t lastButtonPressTime = 0;
uint32_t lastRotationTime = 0;
uint8_t tapCount = 0;
const uint32_t debounceDelay = 100;
const uint32_t tapTimeout = 500;
int8_t encoderDirection = 0;
uint8_t lastStateA = 0;
const char* command = "";
uint8_t imageBuffer[IMAGE_SIZE][IMAGE_SIZE];
uint32_t rowAddressValues[32];
uint32_t bsrrLookupTable[64];
uint8_t pixelLookupTable[MAX_PWM_BITS][256];  // 3 PWM bits x 256 pixel values
const int pwmBitsRed = 3;
const int pwmBitsGreen = 3;
const int pwmBitsBlue = 2;
const int maxPwmBits = 3; // Maximum number of bits among R, G, B
int baseDelay = 200;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void handleButtonAction(void);
void togglePlayPause(void);
void skipToNextSong(void);
void skipToPrevSong(void);
void adjustVolume(void);
void updateDisplay(void);
void init_buf(void);
void initRowAddressValues(void);
void initBsrrLookupTable(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Interrupt handler for both rotation and button press
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTime = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_0)  // Pins A of the encoder
    {

    	//Debounce Rotation
    	if ((currentTime - lastRotationTime) > debounceDelay)
    	{
			// Read the state of both encoder pins
			uint8_t stateA = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);  // Encoder pin A
			uint8_t stateB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);  // Encoder pin B

			// Determine rotation direction
			if (stateA != lastStateA){
				if (stateA != stateB) {
					encoderDirection = 1;  // Clockwise (CW)
				} else {
					encoderDirection = -1;  // Counterclockwise (CCW)
				}

				// Adjust the volume based on the rotation
				adjustVolume();
				encoderDirection = 0;
				lastRotationTime = currentTime;
			}
			lastStateA = stateA;
    	}

    }

    // Handle button press (connected to GPIO_PIN_4)
    if (GPIO_Pin == GPIO_PIN_4)  // Button press on the encoder
    {
		// Debounce the button press
		if ((currentTime - lastButtonPressTime) > debounceDelay)
		{
			// Register the button press and start counting taps
			tapCount++;

			// Update last button press time
			lastButtonPressTime = currentTime;
		}
   }
}

//UART Interrupt handler for receiving image data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Restart UART reception
        HAL_UART_Receive_IT(&huart1, (uint8_t*)imageBuffer, IMAGE_DATA_SIZE);
    }
}


// Function to determine what action to perform based on tap count
void handleButtonAction(void)
{
	if (tapCount == 1){
		togglePlayPause();
	}
	else if (tapCount == 2){
		skipToNextSong();
	}
	else if (tapCount == 3){
		skipToPrevSong();
	}
	else{
	}
}

// Volume adjustment based on rotary encoder direction
void adjustVolume(void)
{
    if (encoderDirection > 0)
    {
        // Clockwise rotation: Increase volume
    	command = "raiseVolume\n";
    }
    else if (encoderDirection < 0)
    {
        // Counterclockwise rotation: Decrease volume
        command = "lowerVolume\n";
    }
    else{
    	return;
    }
	// Send 'command' to ESP32 via UART (hlpuart1)
	HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
}

// Action for single tap (play/pause)
void togglePlayPause(void)
{
    // Code to send play/pause command to Spotify
	command = "togglePlay\n";
	// Send 'command' to ESP32 via UART (hlpuart1)
	HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
}

// Action for double tap (next song)
void skipToNextSong(void)
{
    // Code to send next song command to Spotify
	command = "nextSong\n";
	// Send 'command' to ESP32 via UART (hlpuart1)
	(HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY) != HAL_OK);
}

// Action for triple tap (previous song)
void skipToPrevSong(void)
{
    // Code to send previous song command to Spotify
	command = "previousSong\n";
	// Send 'command' to ESP32 via UART (hlpuart1)
	(HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY) != HAL_OK);
}

// Function to set a pixel at (x, y) with RGB332 data
void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    // Ensure coordinates are within bounds
    if (x < 0 || x >= IMAGE_SIZE || y < 0 || y >= IMAGE_SIZE) return;

    // Limit color components to their respective bit sizes
    r &= 0x07; // 3 bits for Red
    g &= 0x07; // 3 bits for Green
    b &= 0x03; // 2 bits for Blue

    // Combine RGB components into a single byte
    uint8_t pixelData = r | (g << 3) | (b << 6);

    // Store the pixel data
    imageBuffer[y][x] = pixelData;
}


void init_buf(){
    for (int y = 0; y < 32; y++) {
        for (int x = 0; x < IMAGE_SIZE; x++) {
            setPixel(x, y, 0x00, 0x07, 0x00); // Green 
        }
    }
    for (int y = 32; y < IMAGE_SIZE; y++) {
        for (int x = 0; x < IMAGE_SIZE; x++) {
            setPixel(x, y, 0x00, 0x00, 0x00); // Black 
        }
    }
}


void initRowAddressValues(void) {
    for (int row = 0; row < 32; row++) {
        uint32_t value = 0;

        // A line (PB7)
        if (row & 0x01) value |= (1 << 7);    // Set A
        else            value |= (1 << (7+16)); // Reset A

        // B line (PB8)
        if (row & 0x02) value |= (1 << 8);
        else            value |= (1 << (8+16));

        // C line (PB9)
        if (row & 0x04) value |= (1 << 9);
        else            value |= (1 << (9+16));

        // D line (PB10)
        if (row & 0x08) value |= (1 << 10);
        else            value |= (1 << (10+16));

        // E line (PB6)
        if (row & 0x10) value |= (1 << 6);
        else            value |= (1 << (6+16));

        rowAddressValues[row] = value;
    }
}


void initBsrrLookupTable(void) {
    for (int i = 0; i < 64; i++) {
        uint32_t bsrrValue = 0;

        int r1 = (i >> 0) & 1;
        int g1 = (i >> 1) & 1;
        int b1 = (i >> 2) & 1;
        int r2 = (i >> 3) & 1;
        int g2 = (i >> 4) & 1;
        int b2 = (i >> 5) & 1;

        // R1 = PB0
        bsrrValue |= r1 ? (1 << 0) : (1 << (0+16));
        // G1 = PB1
        bsrrValue |= g1 ? (1 << 1) : (1 << (1+16));
        // B1 = PB2
        bsrrValue |= b1 ? (1 << 2) : (1 << (2+16));
        // R2 = PB3
        bsrrValue |= r2 ? (1 << 3) : (1 << (3+16));
        // G2 = PB4
        bsrrValue |= g2 ? (1 << 4) : (1 << (4+16));
        // B2 = PB5
        bsrrValue |= b2 ? (1 << 5) : (1 << (5+16));

        bsrrLookupTable[i] = bsrrValue;
    }
}

void initPixelLookupTable() {
    for (int pwmBit = 0; pwmBit < MAX_PWM_BITS; pwmBit++) {
        uint8_t bitMaskRed = 1 << (2 - pwmBit);
        uint8_t bitMaskGreen = 1 << (2 - pwmBit);
        uint8_t bitMaskBlue = 1 << (1 - pwmBit);  // Since Blue has only 2 bits

        for (int pixelValue = 0; pixelValue < 256; pixelValue++) {
            uint8_t r = pixelValue & 0x07;         // 3 bits for Red
            uint8_t g = (pixelValue >> 3) & 0x07;  // 3 bits for Green
            uint8_t b = (pixelValue >> 6) & 0x03;  // 2 bits for Blue

            uint8_t r_on = (r & bitMaskRed) ? 1 : 0;
            uint8_t g_on = (g & bitMaskGreen) ? 1 : 0;
            uint8_t b_on = (b & bitMaskBlue) ? 1 : 0;

            uint8_t pixelBits = (r_on << 0) | (g_on << 1) | (b_on << 2);

            pixelLookupTable[pwmBit][pixelValue] = pixelBits;
        }
    }
}

void updateDisplay(void) {
    for (int pwmBit = 0; pwmBit < maxPwmBits; pwmBit++) {
        int delay = baseDelay << (maxPwmBits - 1 - pwmBit);
        uint8_t* lookupTable = pixelLookupTable[pwmBit];

        for (int row = 0; row < 32; row++) {
            // Set Row Address Lines
            GPIOB->BSRR = rowAddressValues[row];

            uint8_t* rowPtr1 = imageBuffer[row];
            uint8_t* rowPtr2 = imageBuffer[row + 32];

            for (int col = 0; col < 64; col++) {
                uint8_t pixel1 = rowPtr1[col];
                uint8_t pixel2 = rowPtr2[col];

                uint8_t pixelBits1 = lookupTable[pixel1];
                uint8_t pixelBits2 = lookupTable[pixel2];

                uint8_t pixelBits = pixelBits1 | (pixelBits2 << 3);

                // Set pixel colors using lookup table
                GPIOB->BSRR = bsrrLookupTable[pixelBits];

                // Toggle Clock (CLK = PB11)
                GPIOB->BSRR = (1 << 11);        // Set CLK high
                GPIOB->BSRR = (1 << (11+16));   // Set CLK low
            }

            // Latch Data (LAT = PB12)
            GPIOB->BSRR = (1 << 12);         // LAT high
            GPIOB->BSRR = (1 << (12+16));    // LAT low

            // Enable Display (OE = PB13 active low)
            GPIOB->BSRR = (1 << (13+16));    // OE low (output enabled)

            // Delay proportional to the bit weight
            for (volatile int i = 0; i < delay; i++) {
                __NOP();
            }

            // Disable Display (OE = PB13)
            GPIOB->BSRR = (1 << 13);         // OE high (output disabled)
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  HAL_Delay(500);
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)imageBuffer, IMAGE_DATA_SIZE);
  init_buf();
  initRowAddressValues();
  initBsrrLookupTable();
  initPixelLookupTable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      updateDisplay();
      // Check if the tap timeout has elapsed after the first tap
      if ((tapCount > 0) && (HAL_GetTick() - lastButtonPressTime) > tapTimeout)
      {
          // Call the function to handle the number of taps detected
          handleButtonAction();

          // Reset tap count for the next sequence
          tapCount = 0;
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
