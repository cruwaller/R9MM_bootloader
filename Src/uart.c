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

#define USART_USE_RX_ISR 1

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

#if USART_USE_RX_ISR
#define USART_CR1_FLAGS (USART_CR1_UE | LL_USART_CR1_RXNEIE)
#else
#define USART_CR1_FLAGS (USART_CR1_UE)
#endif

enum
{
  DUPLEX_RX,
  DUPLEX_TX,
};

static uint8_t uart_half_duplex;
static void *duplex_port;
static uint32_t duplex_pin;

void duplex_setup_pin(int32_t pin)
{
  duplex_port = NULL;
  if (pin < 0)
    return;

  gpio_port_pin_get(pin, &duplex_port, &duplex_pin);
  gpio_port_clock((uint32_t)duplex_port);
#if GPIO_USE_LL
  LL_GPIO_SetPinMode(duplex_port, duplex_pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(duplex_port, duplex_pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(duplex_port, duplex_pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_ResetOutputPin(duplex_port, duplex_pin);
#else // GPIO_USE_LL
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = duplex_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(duplex_port, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(duplex_port, duplex_pin, GPIO_PIN_RESET);
#endif // GPIO_USE_LL
}

#ifndef DUPLEX_INVERTED
#define DUPLEX_INVERTED 0
#endif // DUPLEX_INVERTED

void duplex_state_set(const uint8_t state)
{
  if (uart_half_duplex) {
    switch (state) {
      case DUPLEX_TX:
#if USART_USE_LL
        UART_TX_HANDLE->CR1 = USART_CR1_FLAGS | USART_CR1_TE;
#else
      HAL_HalfDuplex_EnableTransmitter(&UART_TX_HANDLE);
#endif
        break;
      case DUPLEX_RX:
#if USART_USE_LL
        UART_TX_HANDLE->CR1 = USART_CR1_FLAGS | USART_CR1_RE;
#else
        HAL_HalfDuplex_EnableReceiver(&UART_TX_HANDLE);
#endif
        break;
      default:
        break;
    }
  }
  if (duplex_port) {
    GPIO_WritePin(duplex_port, duplex_pin, ((state == DUPLEX_TX) ^ DUPLEX_INVERTED));
  }
}

// **************************************************
#if USART_USE_RX_ISR

uint8_t rx_head, rx_tail;
uint8_t rx_buffer[256];

int rx_buffer_available(void)
{
    uint8_t head = read_u8(&rx_head), tail = read_u8(&rx_tail);
    return (uint8_t)(head - tail);
}

int rx_buffer_read(void)
{
    if (!rx_buffer_available())
        return -1;
    uint8_t tail = read_u8(&rx_tail);
    write_u8(&rx_tail, tail+1);
    return rx_buffer[tail++];
}

// ***********************

void USARTx_IRQ_handler(USART_TypeDef * uart)
{
  /* Check for RX data */
  if (LL_USART_IsActiveFlag_ORE(uart) || LL_USART_IsActiveFlag_RXNE(uart)) {
    uint8_t next = rx_head;
    uint8_t data = (uint8_t)LL_USART_ReceiveData8(uart);
    if ((next + 1) != rx_tail) {
      rx_buffer[next] = data;
      rx_head = next + 1;
    }
  }
}


void USART1_IRQHandler(void)
{
  USARTx_IRQ_handler(USART1);
}
void USART2_IRQHandler(void)
{
  USARTx_IRQ_handler(USART2);
}
#if defined(USART3)
void USART3_IRQHandler(void)
{
  USARTx_IRQ_handler(USART3);
}
#endif

#endif // USART_USE_RX_ISR

// **************************************************


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
#if USART_USE_LL
#if !USART_USE_RX_ISR
  LL_USART_ClearFlag_ORE(UART_handle);
  LL_USART_ClearFlag_NE(UART_handle);
  LL_USART_ClearFlag_FE(UART_handle);

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
#else
  uint32_t tickstart = HAL_GetTick();
  while (length--) {
    while (!rx_buffer_available()) {
      /* Check for the Timeout */
      if (timeout != HAL_MAX_DELAY) {
        if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
          return UART_ERROR;
        }
      }
    }
    *data++ = (uint8_t)rx_buffer_read();
  }
#endif

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
uart_status uart_transmit_str(char *data)
{
  uint32_t length = 0u;
  /* Calculate the length. */
  while ('\0' != data[length]) {
    length++;
  }
  return uart_transmit_bytes((uint8_t *)data, length);
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
  uart_status status = UART_OK;
  duplex_state_set(DUPLEX_TX);
#if USART_USE_LL
  while (len--) {
    LL_USART_TransmitData8(UART_TX_HANDLE, *data++);
    while (!LL_USART_IsActiveFlag_TXE(UART_TX_HANDLE))
      ;
  }
  while (!LL_USART_IsActiveFlag_TC(UART_TX_HANDLE))
    ;
#else
  if (HAL_OK != HAL_UART_Transmit(&UART_TX_HANDLE, data, len, UART_TIMEOUT)) {
    status = UART_ERROR;
  }
#endif
  duplex_state_set(DUPLEX_RX);
  return status;
}

#if USART_USE_LL
static void usart_hw_init(USART_TypeDef *USARTx, uint32_t baud, uint32_t dir, uint8_t halfduplex) {
  uint32_t pclk = SystemCoreClock / 2;
#if defined(STM32F1)
  LL_USART_SetBaudRate(USARTx, pclk, baud);
#else
  LL_USART_SetBaudRate(USARTx, pclk, LL_USART_OVERSAMPLING_16, baud);
#endif
  LL_USART_ConfigAsyncMode(USARTx);
  if (halfduplex)
    LL_USART_EnableHalfDuplex(USARTx);
  USARTx->CR1 = USART_CR1_FLAGS | dir;
  uart_half_duplex = halfduplex;

#if USART_USE_RX_ISR
  if (USARTx == USART1) {
    NVIC_SetPriority(USART1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(USART1_IRQn);
#ifdef USART2
  } else if (USARTx == USART2) {
    NVIC_SetPriority(USART2_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(USART2_IRQn);
#endif // USART2
#ifdef USART3
  } else if (USARTx == USART3) {
    NVIC_SetPriority(USART3_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(USART3_IRQn);
#endif // USART3
  }
#endif // USART_USE_RX_ISR
}
#endif // USART_USE_LL

static void uart_reset(USART_TypeDef * uart_ptr)
{
  if (!uart_ptr) return;

  uart_ptr->CR1 = 0;
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
}

/**
 * @brief UART Initialization Function
 * @param None
 * @retval None
 */
void uart_init(uint32_t baud, uint32_t uart_idx, uint32_t afio, int32_t duplexpin, uint8_t halfduplex)
{
#if !GPIO_USE_LL
  GPIO_InitTypeDef GPIO_InitStruct;
#endif

#if !USART_USE_LL
  memset(&huart1, 0, sizeof(huart1));
#endif

#if TARGET_GHOST_RX_V1_2
  (void)duplexpin;
  (void)halfduplex;
  // RX = PB6 [USART1]
  // TX = PA2 [USART2]

  /* Reset RX UART */
  uart_reset(USART1);
  /* Reset TX UART */
  uart_reset(USART2);

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
  usart_hw_init(USART2, baud, USART_CR1_TE, 1); // TX, half duplex
  UART_TX_HANDLE = USART2;
  usart_hw_init(USART1, baud, USART_CR1_RE, 1); // RX, half duplex
  UART_handle = USART1;
#else // !USART_USE_LL
  /* Init TX UART */
  huart_tx.Instance = USART2;
  huart_tx.Init.BaudRate = baud;
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
  (void)duplexpin;
  (void)halfduplex;

  // RX = PB11 [USART3]
  // TX = PA9 [USART1]

  /* Reset RX UART */
  uart_reset(USART3);
  /* Reset TX UART */
  uart_reset(USART1);

  /* UART RX pin config */
  gpio_port_clock((uint32_t)GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* UART TX pin config */
  gpio_port_clock((uint32_t)GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  usart_hw_init(USART1, baud, USART_CR1_TE, 0); // TX, half duplex
  UART_TX_HANDLE = USART1;
  usart_hw_init(USART3, baud, USART_CR1_RE, 0); // RX, half duplex
  UART_handle = USART3;

#else //!TARGET_GHOST_RX_V1_2 && !TARGET_R9SLIM_PLUS

  USART_TypeDef * uart_ptr;
  GPIO_TypeDef *gpio_ptr;
  uint32_t pin_rx, pin_tx, dir = USART_CR1_TE;

  switch (uart_idx) {
    case 1: {
      uart_ptr = USART1;
      switch (afio) {
        case 1:
          gpio_ptr = GPIOB;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_7 : GPIO_PIN_7);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_6 : GPIO_PIN_6);
          break;
        case 2:
          gpio_ptr = GPIOC;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_5 : GPIO_PIN_5);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_4 : GPIO_PIN_4);
          break;
        default:
          gpio_ptr = GPIOA;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_9  : GPIO_PIN_9);
          break;
      }
      break;
    }

    case 2: {
      uart_ptr = USART2;
      switch (afio) {
        case 1:
          /* JTAG pins. Need remapping! */
          gpio_ptr = GPIOA;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_15 : GPIO_PIN_15);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_14 : GPIO_PIN_14);
          break;
        case 2:
          /* JTAG pins. Need remapping! */
          gpio_ptr = GPIOB;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_4 : GPIO_PIN_4);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3);
          break;
        default:
          gpio_ptr = GPIOA;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_2 : GPIO_PIN_2);
          break;
      }
      break;
    }

#if defined(USART3)
    case 3: {
      uart_ptr = USART3;
      switch (afio) {
        case 1:
          gpio_ptr = GPIOB;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_8 : GPIO_PIN_8);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_9 : GPIO_PIN_9);
          break;
        case 2:
          gpio_ptr = GPIOC;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_11 : GPIO_PIN_11);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
          break;
        default:
          gpio_ptr = GPIOB;
          pin_rx = (GPIO_USE_LL ? LL_GPIO_PIN_11 : GPIO_PIN_11);
          pin_tx = (GPIO_USE_LL ? LL_GPIO_PIN_10 : GPIO_PIN_10);
          break;
      }
      break;
    }
#endif // defined(USART3)

    default:
      Error_Handler();
      return;
  }

  uart_reset(uart_ptr);
  gpio_port_clock((uint32_t)gpio_ptr);

  if (!halfduplex) {
    dir = USART_CR1_RE | USART_CR1_TE;

    /* RX pin */
#if GPIO_USE_LL
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
  }

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

  duplex_setup_pin(duplexpin);
  duplex_state_set(DUPLEX_RX);

#if USART_USE_LL
  UART_handle = uart_ptr;
  usart_hw_init(uart_ptr, baud, dir, halfduplex);
#else // USART_USE_LL
  huart1.Instance = uart_ptr;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  if (halfduplex)
    huart1.Init.Mode = UART_MODE_RX;
  else
    huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#endif // USART_USE_LL
#endif // TARGET_GHOST_RX_V1_2
}

void uart_deinit(void)
{
  if (UART_handle)
    uart_reset(UART_handle);
  if (UART_TX_HANDLE && UART_handle != UART_TX_HANDLE)
    uart_reset(UART_TX_HANDLE);
}
