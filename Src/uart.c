/**
 * @file    uart.c
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module is a layer between the HAL UART functions and my Xmodem protocol.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "uart.h"
#include "main.h"
#include <string.h>

#if USART_USE_LL
USART_TypeDef *UART_handle;
#if TARGET_GHOST_RX_V1_2 || TARGET_R9SLIM_PLUS
USART_TypeDef *UART_handle_tx;
#define UART_TX_HANDLE UART_handle_tx
#else
#define UART_TX_HANDLE UART_handle
#endif
#else // !USART_USE_LL
static UART_HandleTypeDef huart1;
#if TARGET_GHOST_RX_V1_2
static UART_HandleTypeDef huart_tx;
#define UART_TX_HANDLE huart_tx
#else
#define UART_TX_HANDLE huart1
#endif
#endif // USART_USE_LL

#ifndef UART_BAUD
#define UART_BAUD 420000
#endif

#if defined(DEBUG_UART) && defined(STM32F1)
#if (DEBUG_UART == UART_NUM)
#error "Same uart cannot be used for debug and comminucation!"
#endif

UART_HandleTypeDef debug;

void debug_init(void)
{
  /* Configure GPIO pins */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* RX pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TX pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  __HAL_RCC_USART1_CLK_ENABLE();

  // R9M - UART on external pin stripe
  debug.Instance = USART1;
  debug.Init.BaudRate = 57600;
  debug.Init.WordLength = UART_WORDLENGTH_8B;
  debug.Init.StopBits = UART_STOPBITS_1;
  debug.Init.Parity = UART_PARITY_NONE;
  debug.Init.Mode = UART_MODE_TX;
  debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  debug.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&debug) != HAL_OK)
  {
    Error_Handler();
  }
}

void debug_send(uint8_t data)
{
  /* Make available the UART module. */
  if (HAL_UART_STATE_TIMEOUT == HAL_UART_GetState(&debug))
  {
    HAL_UART_Abort(&debug);
  }
  HAL_UART_Transmit(&debug, &data, 1u, UART_TIMEOUT);
}

#endif

/**
 * @brief   Receives data from UART.
 * @param   *data: Array to save the received data.
 * @param   length:  Size of the data.
 * @return  status: Report about the success of the receiving.
 */
uart_status uart_receive(uint8_t *data, uint16_t length)
{
  return uart_receive_timeout(data, length, UART_TIMEOUT);
}

uart_status uart_receive_timeout(uint8_t *data, uint16_t length, uint16_t timeout)
{
#if HALF_DUPLEX
#if USART_USE_LL
  UART_handle->CR1 = USART_CR1_UE | USART_CR1_RE;
#else
  HAL_HalfDuplex_EnableReceiver(&huart1);
#endif
#endif

#if USART_USE_LL
  uint32_t tickstart = HAL_GetTick();
  while (length--) {
    while (!LL_USART_IsActiveFlag_RXNE(UART_handle)) {
      /* Check for the Timeout */
      if (timeout != HAL_MAX_DELAY) {
        if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
          return UART_ERROR;
        }
      }
    }
    *data++ = (uint8_t)LL_USART_ReceiveData8(UART_handle);
  }
  return UART_OK;
#else
  if (HAL_OK == HAL_UART_Receive(&huart1, data, length, timeout))
  {
    return UART_OK;
  }
  return UART_ERROR;
#endif
}

/**
 * @brief   Transmits a string to UART.
 * @param   *data: Array of the data.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_str(uint8_t *data)
{
  uint32_t length = 0u;
  /* Calculate the length. */
  while ('\0' != data[length]) {
    length++;
  }
  return uart_transmit_bytes(data, length);
}

/**
 * @brief   Transmits a single char to UART.
 * @param   *data: The char.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_ch(uint8_t data)
{
  return uart_transmit_bytes(&data, 1u);
}

uart_status uart_transmit_bytes(uint8_t *data, uint32_t len)
{
  uart_status status = UART_ERROR;

  duplex_state_set(DUPLEX_TX);
#if HALF_DUPLEX
#if USART_USE_LL
  UART_TX_HANDLE->CR1 = USART_CR1_UE | USART_CR1_TE;
#else
  HAL_HalfDuplex_EnableTransmitter(&UART_TX_HANDLE);
#endif
#endif

#if USART_USE_LL
  while (len--) {
    LL_USART_TransmitData8(UART_TX_HANDLE, *data++);
    while (!LL_USART_IsActiveFlag_TXE(UART_TX_HANDLE))
      ;
  }
  while (!LL_USART_IsActiveFlag_TC(UART_TX_HANDLE))
    ;
  status = UART_OK;
#else
  if (HAL_OK == HAL_UART_Transmit(&UART_TX_HANDLE, data, len, UART_TIMEOUT)) {
    status = UART_OK;
  }
#endif
  duplex_state_set(DUPLEX_RX);
  return status;
}

#if USART_USE_LL
static void usart_hw_init(USART_TypeDef *USARTx, uint32_t dir) {
  uint32_t pclk = SystemCoreClock / 2;
#if defined(STM32F1)
  LL_USART_SetBaudRate(USARTx, pclk, UART_BAUD);
#else
  LL_USART_SetBaudRate(USARTx, pclk, LL_USART_OVERSAMPLING_16, UART_BAUD);
#endif
  LL_USART_ConfigAsyncMode(USARTx);
  USARTx->CR1 = USART_CR1_UE | dir; //| USART_CR1_RE | USART_CR1_TE;
}
#endif // USART_USE_LL

/**
 * @brief UART Initialization Function
 * @param None
 * @retval None
 */
void uart_init(void)
{
#if !GPIO_USE_LL
  GPIO_InitTypeDef GPIO_InitStruct;
#endif

#if !USART_USE_LL
  memset(&huart1, 0, sizeof(huart1));
#endif

#if defined(DEBUG_UART) && defined(STM32F1)
  debug_init();
#endif

#if TARGET_GHOST_RX_V1_2
  // RX = PB6 [USART1]
  // TX = PA2 [USART2]

  /* Reset RX UART */
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* Reset TX UART */
  __HAL_RCC_USART2_FORCE_RESET();
  __HAL_RCC_USART2_RELEASE_RESET();
  __HAL_RCC_USART2_CLK_ENABLE();

  gpio_port_clock((uint32_t)GPIOB);
  gpio_port_clock((uint32_t)GPIOA);
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  //LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL); // needed?
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
#else // !GPIO_USE_LL
  /* Init RX pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Init TX pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif // GPIO_USE_LL

#if USART_USE_LL
  usart_hw_init(USART2, USART_CR1_TE); // TX, half duplex
  LL_USART_EnableHalfDuplex(USART2);
  UART_TX_HANDLE = USART2;
  usart_hw_init(USART1, USART_CR1_RE); // RX, half duplex
  LL_USART_EnableHalfDuplex(USART1);
  UART_handle = USART1;
#else // !USART_USE_LL
  /* Init TX UART */
  huart_tx.Instance = USART2;
  huart_tx.Init.BaudRate = UART_BAUD;
  huart_tx.Init.WordLength = UART_WORDLENGTH_8B;
  huart_tx.Init.StopBits = UART_STOPBITS_1;
  huart_tx.Init.Parity = UART_PARITY_NONE;
  huart_tx.Init.Mode = UART_MODE_TX;
  huart_tx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_tx.Init.OverSampling = UART_OVERSAMPLING_16;
  huart_tx.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  if (HAL_HalfDuplex_Init(&huart_tx) != HAL_OK) {
    Error_Handler();
  }

  /* Init RX UART */
  memcpy(&huart1.Init, &huart_tx.Init, sizeof(huart_tx.Init));
  huart1.Instance = USART1;
  huart1.Init.Mode = UART_MODE_RX;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
#endif // USART_USE_LL

#elif TARGET_R9SLIM_PLUS // !TARGET_GHOST_RX_V1_2

  // RX = PB11 [USART3]
  // TX = PA9 [USART1]

  /* Reset RX UART */
  __HAL_RCC_USART3_FORCE_RESET();
  __HAL_RCC_USART3_RELEASE_RESET();
  __HAL_RCC_USART3_CLK_ENABLE();

  /* Reset TX UART */
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* UART RX pin config */
  gpio_port_clock((uint32_t)GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* UART TX pin config */
  gpio_port_clock((uint32_t)GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  usart_hw_init(USART1, USART_CR1_TE); // TX, half duplex
  UART_TX_HANDLE = USART1;
  usart_hw_init(USART3, USART_CR1_RE); // RX, half duplex
  UART_handle = USART3;

#else //!TARGET_GHOST_RX_V1_2 && !TARGET_R9SLIM_PLUS
  USART_TypeDef * uart_ptr;
  GPIO_TypeDef *gpio_ptr;
  uint32_t pin_rx, pin_tx;

#if UART_NUM == 1
  uart_ptr = USART1;
#if UART_AFIO == 1
  gpio_ptr = GPIOB;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_7 : GPIO_PIN_7);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_6 : GPIO_PIN_6);
#elif UART_AFIO == 2
  gpio_ptr = GPIOC;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_5 : GPIO_PIN_5);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_4 : GPIO_PIN_4);
#else // !UART_AFIO
  gpio_ptr = GPIOA;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_9 : GPIO_PIN_9);
#endif

#elif UART_NUM == 2
  uart_ptr = USART2;
#if UART_AFIO == 1
  /* JTAG pins. Need remapping! */
  gpio_ptr = GPIOA;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_15 : GPIO_PIN_15);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_14 : GPIO_PIN_14);
#elif UART_AFIO == 2
  /* JTAG pins. Need remapping! */
  gpio_ptr = GPIOB;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_4 : GPIO_PIN_4);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3);
#else // !UART_AFIO
  gpio_ptr = GPIOA;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_2 : GPIO_PIN_2);
#endif

#elif UART_NUM == 3 && defined(USART3)
  uart_ptr = USART3;

#if UART_AFIO == 1
  gpio_ptr = GPIOB;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_8 : GPIO_PIN_8);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_9 : GPIO_PIN_9);
#elif UART_AFIO == 2
  gpio_ptr = GPIOC;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_11 : GPIO_PIN_11);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
#else // !UART_AFIO
  gpio_ptr = GPIOB;
  pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_11 : GPIO_PIN_11);
  pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
#endif

#else
#error "Invalid UART config"
#endif

  if (uart_ptr == USART1) {
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();
    __HAL_RCC_USART1_CLK_ENABLE();
  } else if (uart_ptr == USART2) {
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    __HAL_RCC_USART2_CLK_ENABLE();
  }
#if defined(USART3)
  else if (uart_ptr == USART3) {
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
    __HAL_RCC_USART3_CLK_ENABLE();
  }
#endif

  gpio_port_clock((uint32_t)gpio_ptr);

#if GPIO_USE_LL
  /* RX pin */
#if defined(STM32L0xx)
  LL_GPIO_SetPinMode(gpio_ptr, pin_rx, LL_GPIO_MODE_ALTERNATE);
  if (uart_ptr == USART1 && pin_rx == LL_GPIO_PIN_7) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_rx, LL_GPIO_AF_0);
  } else if (pin_rx <= LL_GPIO_PIN_7) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_rx, LL_GPIO_AF_4);
  } else {
    LL_GPIO_SetAFPin_8_15(gpio_ptr, pin_rx, LL_GPIO_AF_4);
  }
#elif defined(STM32L4xx) || defined(STM32F3xx)
  LL_GPIO_SetPinMode(gpio_ptr, pin_rx, LL_GPIO_MODE_ALTERNATE);
  if (pin_rx <= LL_GPIO_PIN_7) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_rx, LL_GPIO_AF_7);
  } else {
    LL_GPIO_SetAFPin_8_15(gpio_ptr, pin_rx, LL_GPIO_AF_7);
  }
#else
  LL_GPIO_SetPinMode(gpio_ptr, pin_rx, LL_GPIO_MODE_INPUT);
#endif
  //LL_GPIO_SetPinSpeed(gpio_ptr, pin_rx, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(gpio_ptr, pin_rx, LL_GPIO_PULL_UP);

  /* TX pin */
  LL_GPIO_SetPinMode(gpio_ptr, pin_tx, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(gpio_ptr, pin_tx, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(gpio_ptr, pin_tx, LL_GPIO_PULL_UP);
#if defined(STM32L0xx)
  if (uart_ptr == USART1 && pin_tx == LL_GPIO_PIN_6) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_tx, LL_GPIO_AF_0);
  } else if (pin_tx <= LL_GPIO_PIN_7) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_tx, LL_GPIO_AF_4);
  } else {
    LL_GPIO_SetAFPin_8_15(gpio_ptr, pin_tx, LL_GPIO_AF_4);
  }
#elif defined(STM32L4xx) || defined(STM32F3xx)
  if (pin_tx <= LL_GPIO_PIN_7) {
    LL_GPIO_SetAFPin_0_7(gpio_ptr, pin_tx, LL_GPIO_AF_7);
  } else {
    LL_GPIO_SetAFPin_8_15(gpio_ptr, pin_tx, LL_GPIO_AF_7);
  }
#endif

#else // !GPIO_USE_LL
  /* RX pin */
  GPIO_InitStruct.Pin = pin_rx;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(STM32L0xx)
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  if (uart_ptr == USART1 && pin_rx == GPIO_PIN_7) {
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  } else {
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  }
#elif defined(STM32L4xx) || defined(STM32F3xx)
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
#else
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#endif
  HAL_GPIO_Init(gpio_ptr, &GPIO_InitStruct);

  /* TX pin */
  GPIO_InitStruct.Pin = pin_tx;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(STM32L0xx)
  if (uart_ptr == USART1 && pin_tx == GPIO_PIN_6) {
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  } else {
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  }
#elif defined(STM32L4xx) || defined(STM32F3xx)
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
#endif
  HAL_GPIO_Init(gpio_ptr, &GPIO_InitStruct);
#endif // GPIO_USE_LL

  duplex_state_set(DUPLEX_RX);

#if USART_USE_LL
  UART_handle = uart_ptr;
  usart_hw_init(uart_ptr, (USART_CR1_TE | USART_CR1_RE));
#else // USART_USE_LL
  huart1.Instance = uart_ptr;
  huart1.Init.BaudRate = UART_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
#if HALF_DUPLEX
  huart1.Init.Mode = UART_MODE_RX;
#else
  huart1.Init.Mode = UART_MODE_TX_RX;
#endif
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#endif // USART_USE_LL
#endif // TARGET_GHOST_RX_V1_2
}
