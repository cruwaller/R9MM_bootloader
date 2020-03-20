/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
#if TARGET_R9M
#include "stk500.h"
#else
#include "xmodem.h"
#endif
#include "flash.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART_Init(void);
/*static void delay(int x) {
    for (int i = 0; i < x; i++)
        for (int j = 0; j < 500; j++)
            __asm__("nop");
}
*/

/* Private user code ---------------------------------------------------------*/

void led_red_state_set(const GPIO_PinState state) {
#ifdef LED_RED_Pin
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, state);
#else
  (void)state;
#endif
}

void led_green_state_set(const GPIO_PinState state) {
#ifdef LED_GRN_Pin
  HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, state);
#else
  (void)state;
#endif
}

#if TARGET_R9MM
static void boot_code(void) {
  int i, ctr;

  led_red_state_set(0);
  led_green_state_set(1);
  /* Send welcome message on startup. */
  uart_transmit_str(
      (uint8_t *)"\n\r==========================================\n\r");
  uart_transmit_str((uint8_t *)"UART Bootloader for ExpressLRS R9mm\n\r");
  uart_transmit_str(
      (uint8_t *)"https://github.com/AlessandroAU/ExpressLRS\n\r");
  uart_transmit_str(
      (uint8_t *)"==========================================\n\r\n\r");

  /* If the button is pressed, then jump to the user application,
   * otherwise stay in the bootloader. */
  uart_transmit_str(
      (uint8_t
           *)"Send '2bl', 'bbb' or hold down button to begin bootloader\n\r");

  led_red_state_set(1);
  led_green_state_set(0);
  HAL_Delay(100);
  led_red_state_set(1);
  led_green_state_set(1);

  uint8_t header[6] = {0, 0, 0, 0, 0, 0};

  uart_receive(header, 5u);

  bool BLrequested = false;
  /* Search for magic strings */
  if (strstr((char *)header, "2bl") || strstr((char *)header, "bbb")) {
    BLrequested = true;
  } else {
#ifdef BTN_Pin
    /* Read button a few times to make sure we pressed it and we are not
     * sampling noise */
    ctr = 0;
    for (i = 0; i < 16; i++) {
      /* Button is active low */
      if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET)
        ctr++;
      HAL_Delay(10);
    }
    /* if >75% of our samples is 'pressed', we assume it indeed was */
    if (ctr > 12) {
      uart_transmit_str((uint8_t *)"Detected button press\n\r");
      BLrequested = true;
    }
#endif /* BTN_Pin */
  }

  if (BLrequested == true) {
    /* BL was requested, GRN led on */
    led_red_state_set(0);
    led_green_state_set(1);
  } else {
    /* BL was not requested, RED led on. Use app will soon use the LED's for
     * it's own purpose, thus if RED stays on there is an error */
    led_red_state_set(1);
    led_green_state_set(0);
    uart_transmit_str((uint8_t *)"Jumping to user application...\n\r");
    flash_jump_to_app();
  }

  /* Infinite loop */
  while (1) {

    /* Turn on the green LED to indicate, that we are in bootloader mode.*/
    // HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    /* Ask for new data and start the Xmodem protocol. */
    // uart_transmit_str((uint8_t*)"Please send a new binary file with Xmodem
    // protocol to update the firmware.\n\r");
    xmodem_receive();
    /* We only exit the xmodem protocol, if there are any errors.
     * In that case, notify the user and start over. */
    uart_transmit_str((uint8_t *)"\n\rFailed... Please try again.\n\r");
  }
}

#elif TARGET_R9M

static void boot_code(void) {
  /* Infinite loop */
  while (1) {
    /* Check if update is requested */
    if (stk500_check() < 0) {
      flash_jump_to_app();
    }
  }
}

#endif /* TARGET_R9MM */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART_Init();

  boot_code();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief UART Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART_Init(void) {
#if UART_NUM == 1
  huart1.Instance = USART1;
#elif UART_NUM == 2
  huart1.Instance = USART1;
#elif UART_NUM == 3
  huart1.Instance = USART3;
#endif
  huart1.Init.BaudRate = UART_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
#if defined(BTN_Pin) || defined(LED_GNR_Pin) || defined(LED_RED_Pin)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
#endif

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

#ifdef BTN_Pin
  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);
#endif

#ifdef LED_GRN_Pin
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GRN_Pin */
  GPIO_InitStruct.Pin = LED_GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GRN_GPIO_Port, &GPIO_InitStruct);
#endif

#ifdef LED_RED_Pin
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);
#endif
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state
   */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
