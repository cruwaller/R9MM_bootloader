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
#include <string.h>

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "uart.h"
#include "flash.h"
#if XMODEM
#include "xmodem.h"
#elif STK500
#include "stk500.h"
#elif FRSKY
#include "frsky.h"
#else
#error "Upload protocol not defined!"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define BUTTON_NEW_BOUNCE 1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#if defined(BUTTON_PIN)
void *btn_port;
uint16_t btn_pin;
#else
#define btn_port BTN_GPIO_Port
#define btn_pin BTN_Pin
#endif
#if defined(LED_RED_PIN)
void *led_red_port;
uint16_t led_red_pin;
#else
#define led_red_port LED_RED_GPIO_Port
#define led_red_pin LED_RED_Pin
#endif
#if defined(LED_GREEN_PIN)
void *led_green_port;
uint16_t led_green_pin;
#else
#define led_green_port LED_GRN_GPIO_Port
#define led_green_pin LED_GRN_Pin
#endif
#if defined(DUPLEX_PIN)
void *duplex_port;
uint16_t duplex_pin;
#else
#define duplex_port DUPLEX_Port
#define duplex_pin DUPLEX_Pin
#endif

#if GPIO_USE_LL
#define BTN_READ() LL_GPIO_IsInputPinSet(btn_port, btn_pin)
#else
#define BTN_READ() HAL_GPIO_ReadPin(btn_port, btn_pin)
#endif
#ifndef BUTTON_INVERTED
#define BUTTON_INVERTED   1
#endif // BUTTON_INVERTED

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* Private user code ---------------------------------------------------------*/

void gpio_port_pin_get(uint32_t io, void ** port, uint16_t * pin)
{
  *pin = IO_GET_PIN(io);
  io = IO_GET_PORT(io);
  *port = (void*)((uintptr_t)GPIOA_BASE + (io * 0x0400UL));

  // Enable gpio clock
  switch(io) {
    case 0:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case 1:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
#ifdef __HAL_RCC_GPIOC_CLK_ENABLE
    case 2:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOD_CLK_ENABLE
    case 3:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOE_CLK_ENABLE
    case 4:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
#endif
#ifdef __HAL_RCC_GPIOF_CLK_ENABLE
    case 5:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
#endif
    default:
      break;
    }
}

void led_state_set(uint32_t state)
{
  uint32_t val;
  switch (state) {
  case LED_BOOTING:
    val = 0x00ffff;
    break;
  case LED_FLASHING:
    val = 0x00ffff;
    break;
  case LED_FLASHING_ALT:
    val = 0x00ff00;
    break;
  case LED_STARTING:
    val = 0xffff00;
    break;
  case LED_OFF:
  default:
    val = 0x0;
  };

#if defined(LED_RED)
  GPIO_WritePin(led_red_port, led_red_pin, !!(uint8_t)val);
#endif
#if defined(LED_GREEN)
  GPIO_WritePin(led_green_port, led_green_pin, !!(uint8_t)(val >> 8));
#endif
  ws2812_set_color((uint8_t)(val), (uint8_t)(val >> 8), (uint8_t)(val >> 16));
}

void duplex_state_set(const enum duplex_state state)
{
#if TARGET_R9M
  GPIO_WritePin(duplex_port, duplex_pin, (state == DUPLEX_TX));
#else
  (void)state;
#endif
}

#if XMODEM
static void boot_code(void)
{
  bool BLrequested = false;
  uint8_t header[6] = {0, 0, 0, 0, 0, 0};

  /* Send welcome message on startup. */
  uart_transmit_str((uint8_t *)"\n\r=========================\n\r");
  uart_transmit_str((uint8_t *)"  ExpressLRS bootloader\n\r");
  uart_transmit_str((uint8_t *)"=========================\n\r");
#if defined(MCU_TYPE)
  uart_transmit_str((uint8_t *)BUILD_MCU_TYPE(MCU_TYPE));
  uart_transmit_str((uint8_t *)"\n\r");
#endif
  /* If the button is pressed, then jump to the user application,
   * otherwise stay in the bootloader. */
  uart_transmit_str((uint8_t *)"Send '2bl', 'bbb' or hold down button\n\r");

#if defined(BUTTON) && BUTTON_NEW_BOUNCE
  uint32_t btn_val = BTN_READ();
#endif

  /* Wait input from UART */
  uart_receive(header, 5u);

  /* Search for magic strings */
  if (strstr((char *)header, "2bl") || strstr((char *)header, "bbb"))
  {
    BLrequested = true;
  }
#if defined(BUTTON)
#if BUTTON_NEW_BOUNCE
  // Debounce check (1sec)
  else if (BTN_READ() == btn_val && (btn_val ^ BUTTON_INVERTED) == GPIO_PIN_SET) {
    // Button value is still same and is pressed
    BLrequested = true;
  }
#else // !BUTTON_NEW_BOUNCE
  // Wait button press to access bootloader
  if (!BLrequested) // Check UART if not button not pressed
    for (int i = 0; i < 2; i++)
    {
      if ((BTN_READ() ^ BUTTON_INVERTED) == GPIO_PIN_SET)
      {
        HAL_Delay(100); // wait debounce
        if ((BTN_READ() ^ BUTTON_INVERTED) == GPIO_PIN_SET)
        {
          // Button still pressed
          BLrequested = true;
          break;
        }
      }
      else
      {
        HAL_Delay(50);
      }
    }
#endif /* BUTTON_NEW_BOUNCE */
#endif /* BUTTON */

  if (!BLrequested)
  {
    /* BL was not requested, RED led on. Use app will soon use the LED's for
     * it's own purpose, thus if RED stays on there is an error */
    // uart_transmit_str((uint8_t *)"Start app\n\r");
    flash_jump_to_app();
  }

  /* Infinite loop */
  while (1)
  {
    xmodem_receive();
    /* We only exit the xmodem protocol, if there are any errors.
     * In that case, notify the user and start over. */
    //uart_transmit_str((uint8_t *)"\n\rFailed... Please try again.\n\r");
  }
}

#else // !XMODEM

#define BOOT_WAIT 300 // ms

static uint32_t boot_end_time;

int8_t timer_end(void)
{
  //return 0;
  return (HAL_GetTick() > boot_end_time);
}

static void boot_code(void)
{
  boot_end_time = HAL_GetTick() + BOOT_WAIT;

  /* Infinite loop */
  while (1)
  {
    /* Check if update is requested */
    if (
#if STK500
        stk500_check() < 0
#else
        frsky_check() < 0
#endif
    )
    {
      flash_jump_to_app();
    }
  }
}

#endif /* XMODEM */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* MCU Configuration---------------------------------------------*/

  /* Make sure the vectors are set correctly */
  SCB->VTOR = BL_FLASH_START;

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  __enable_irq();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  uart_init();

  led_state_set(LED_BOOTING);
  boot_code();
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitTypeDef));
  memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitTypeDef));

#if defined(STM32L0xx) || defined(STM32L4xx)
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
#if defined(STM32L4xx)
  RCC_OscInitStruct.PLL.PLLM = 1;  // 16MHz
  RCC_OscInitStruct.PLL.PLLN = 10; // 10 * 16MHz
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // 160MHz / 2
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; // 160MHz / 7
#if defined(STM32L432xx)
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4; // 160MHz / 4
#else // !STM32L432xx
  /* STM32L433 */
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // 160MHz / 2
#endif // STM32L432xx

#else // !STM32L4xx
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
#endif // STM32L4xx
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
#if defined(STM32L4xx)
  uint32_t flash_latency = FLASH_LATENCY_4;
#else // !STM32L4xx
  uint32_t flash_latency = FLASH_LATENCY_1;
#endif // STM32L4xx
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK)
    Error_Handler();

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();

  HAL_ResumeTick();

#elif defined(STM32L1xx)
#warning "Clock setup is missing! Should use HSI!"

#elif defined(STM32F1) || defined(STM32F3xx)

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
#if defined(STM32F3xx)
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; // 8MHz / DIV2 = 4MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 16 * 4MHz = 64MHz
#else
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // 8MHz / DIV2 = 4MHz
  //RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12; // 12 * 4MHz = 48MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 16 * 4MHz = 64MHz
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

#if defined(STM32F3xx)
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
#endif
#endif

  SystemCoreClockUpdate();
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
#if defined(BTN_Pin)     || defined(LED_GRN_Pin) || defined(LED_RED_Pin)   || \
    defined(DUPLEX_Pin)  || defined(BUTTON_PIN)  || defined(LED_GREEN_PIN) || \
    defined(LED_RED_PIN) || defined(DUPLEX_PIN)
#if !GPIO_USE_LL
  GPIO_InitTypeDef GPIO_InitStruct;
#endif
#endif

#ifdef BUTTON
#if defined(BUTTON_PIN)
  gpio_port_pin_get(IO_CREATE(BUTTON_PIN), &btn_port, &btn_pin);
#endif
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(btn_port, btn_pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(btn_port, btn_pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(btn_port, btn_pin,
                     BUTTON_INVERTED ? LL_GPIO_PULL_UP : LL_GPIO_PULL_DOWN);
#else // GPIO_USE_LL
  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = btn_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = BUTTON_INVERTED ? GPIO_PULLUP : GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(btn_port, &GPIO_InitStruct);
#endif // GPIO_USE_LL
#endif // BUTTON

#ifdef LED_GRN
#if defined(LED_GREEN_PIN)
  gpio_port_pin_get(IO_CREATE(LED_GREEN_PIN), &led_green_port, &led_green_pin);
#endif
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(led_green_port, led_green_pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(led_green_port, led_green_pin,
                           LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(led_green_port, led_green_pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_ResetOutputPin(led_green_port, led_green_pin);
#else // GPIO_USE_LL
  GPIO_InitStruct.Pin = led_green_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_green_port, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_green_port, led_green_pin, GPIO_PIN_RESET);
#endif // GPIO_USE_LL
#endif // LED_GRN

#ifdef LED_RED
#if defined(LED_RED_PIN)
  gpio_port_pin_get(IO_CREATE(LED_RED_PIN), &led_red_port, &led_red_pin);
#endif
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(led_red_port, led_red_pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(led_red_port, led_red_pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(led_red_port, led_red_pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_ResetOutputPin(led_red_port, led_red_pin);
#else // GPIO_USE_LL
  GPIO_InitStruct.Pin = led_red_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_red_port, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_red_port, led_red_pin, GPIO_PIN_RESET);
#endif // GPIO_USE_LL
#endif // LED_RED

#if TARGET_R9M
#if defined(DUPLEX_PIN)
  gpio_port_pin_get(IO_CREATE(DUPLEX_PIN), &duplex_port, &duplex_pin);
#endif
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(duplex_port, duplex_pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(duplex_port, duplex_pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(duplex_port, duplex_pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_ResetOutputPin(duplex_port, duplex_pin);
#else // GPIO_USE_LL
  GPIO_InitStruct.Pin = duplex_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(duplex_port, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(duplex_port, duplex_pin, GPIO_PIN_RESET);
#endif // GPIO_USE_LL
#endif // TARGET_R9M

  ws2812_init();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state
   */
  while (1)
    ;
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
