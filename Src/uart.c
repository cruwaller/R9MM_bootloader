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

#if defined(STM32L4xx)
// FIXME: UART ISR causes xmodem failure
#define USART_USE_RX_ISR 0
#else
#define USART_USE_RX_ISR 1
#endif

USART_TypeDef *UART_handle_rx, *UART_handle_tx;

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
static struct gpio_pin duplex_pin;

void duplex_setup_pin(int32_t pin)
{
  duplex_pin.reg = NULL;
  if (pin < 0)
    return;
  duplex_pin = GPIO_Setup(pin, GPIO_OUTPUT, -1);
}

#ifndef DUPLEX_INVERTED
#define DUPLEX_INVERTED 0
#endif // DUPLEX_INVERTED

void duplex_state_set(const uint8_t state)
{
  if (uart_half_duplex || duplex_pin.reg) {
    switch (state) {
      case DUPLEX_TX:
        UART_handle_tx->CR1 = USART_CR1_FLAGS | USART_CR1_TE;
        break;
      case DUPLEX_RX:
        UART_handle_tx->CR1 = USART_CR1_FLAGS | USART_CR1_RE;
        break;
      default:
        break;
    }
  }
  if (duplex_pin.reg) {
    GPIO_Write(duplex_pin, ((state == DUPLEX_TX) ^ DUPLEX_INVERTED));
  }
}

void usart_pin_config(uint32_t pin, uint8_t isrx)
{
#if defined(STM32L0xx)
  uint32_t fn = GPIO_FUNCTION(4);
  if (pin == GPIO('B', 6) || pin == GPIO('B', 7)) {
    // USART1 is AF0
    fn = GPIO_FUNCTION(0);
  }
#else
  uint32_t fn = GPIO_FUNCTION(7);
#endif
  GPIO_Setup(pin, fn, isrx);
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
  uint32_t tickstart;
#if !USART_USE_RX_ISR
  LL_USART_ClearFlag_ORE(UART_handle_rx);
  LL_USART_ClearFlag_NE(UART_handle_rx);
  LL_USART_ClearFlag_FE(UART_handle_rx);

  while (length--) {
    tickstart = HAL_GetTick();
    while (!LL_USART_IsActiveFlag_RXNE(UART_handle_rx)) {
      /* Check for the Timeout */
      if (timeout != HAL_MAX_DELAY) {
        if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
          return UART_ERROR;
        }
      }
    }
    *data++ = (uint8_t)LL_USART_ReceiveData8(UART_handle_rx);
  }
#else
  while (length--) {
    tickstart = HAL_GetTick();
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
  while (len--) {
    while (!LL_USART_IsActiveFlag_TXE(UART_handle_tx))
      ;
    LL_USART_TransmitData8(UART_handle_tx, *data++);
  }
  while (!LL_USART_IsActiveFlag_TXE(UART_handle_tx))
    ;
  while (!LL_USART_IsActiveFlag_TC(UART_handle_tx))
    ;
  duplex_state_set(DUPLEX_RX);
  return status;
}

#if USART_USE_RX_ISR
static IRQn_Type usart_get_irq(USART_TypeDef *USARTx)
{
  if (USARTx == USART1) {
    return USART1_IRQn;
  } else if (USARTx == USART2) {
    return USART2_IRQn;
#if defined(USART3)
  } else if (USARTx == USART3) {
    return USART3_IRQn;
#endif // USART3
  }
  return 0;
}
#endif

static void uart_reset(USART_TypeDef * uart_ptr)
{
  if (!uart_ptr) return;

  uart_ptr->CR1 = 0;

#if USART_USE_RX_ISR
  IRQn_Type irq = usart_get_irq(uart_ptr);
  NVIC_DisableIRQ(irq);
#endif

  if (uart_ptr == USART1) {
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();
    __HAL_RCC_USART1_CLK_ENABLE();
  } else if (uart_ptr == USART2) {
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    __HAL_RCC_USART2_CLK_ENABLE();
#if defined(USART3)
  } else if (uart_ptr == USART3) {
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
    __HAL_RCC_USART3_CLK_ENABLE();
#endif
  }
}

static void usart_hw_init(USART_TypeDef *USARTx, uint32_t baud, uint32_t dir, uint8_t halfduplex) {
  uint32_t pclk = SystemCoreClock / 2;

  /* Reset UART peripheral */
  uart_reset(USARTx);

#if defined(STM32F1)
  LL_USART_SetBaudRate(USARTx, pclk, baud);
#else
  LL_USART_SetBaudRate(USARTx, pclk, LL_USART_OVERSAMPLING_16, baud);
#endif
  LL_USART_ConfigAsyncMode(USARTx);
  if (halfduplex)
    LL_USART_EnableHalfDuplex(USARTx);
  USARTx->CR1 = USART_CR1_FLAGS | dir;

  if (dir & USART_CR1_TE)
    uart_half_duplex = halfduplex;

  /* Store handles */
  if (dir == USART_CR1_TE)
    UART_handle_tx = USARTx;
  else if (dir == USART_CR1_RE)
    UART_handle_rx = USARTx;
  else
    UART_handle_rx = UART_handle_tx = USARTx;

#if USART_USE_RX_ISR
  IRQn_Type irq = usart_get_irq(USARTx);
  NVIC_SetPriority(irq,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
  NVIC_EnableIRQ(irq);
#endif // USART_USE_RX_ISR
}

static uint8_t uart_valid_pin_tx(uint32_t pin)
{
  switch (pin) {
    case GPIO('A', 2):
    case GPIO('A', 9):
    case GPIO('A', 14):
    case GPIO('B', 3):
    case GPIO('B', 6):
    case GPIO('B', 9):
    case GPIO('B', 10):
    case GPIO('C', 4):
    case GPIO('C', 10):
      return 1;
  }
  return 0;
}

static USART_TypeDef * uart_peripheral_get(uint32_t pin)
{
  switch (pin) {
    case GPIO('A', 9):
    case GPIO('A', 10):
    case GPIO('B', 6):
    case GPIO('B', 7):
    case GPIO('C', 4):
    case GPIO('C', 5):
      return USART1;
    case GPIO('A', 2):
    case GPIO('A', 3):
    case GPIO('A', 14):
    case GPIO('A', 15):
    case GPIO('B', 3):
    case GPIO('B', 4):
      return USART2;
#ifdef USART3
    case GPIO('B', 8):
    case GPIO('B', 9):
    case GPIO('B', 10):
    case GPIO('B', 11):
    case GPIO('C', 10):
    case GPIO('C', 11):
      return USART3;
#endif // USART3
  }
  return NULL;
}


/**
 * @brief UART Initialization Function
 * @param None
 * @retval None
 */
void uart_init(uint32_t baud, uint32_t pin_rx, uint32_t pin_tx, int32_t duplexpin)
{
  uint8_t halfduplex;
#if 0
  switch (uart_idx) {
    case 1: {
      switch (afio) {
        case 1:
          pin_rx = GPIO('B', 7);
          pin_tx = GPIO('B', 6);
          break;
        case 2:
          pin_rx = GPIO('C', 5);
          pin_tx = GPIO('C', 4);
          break;
        default:
          pin_rx = GPIO('A', 10);
          pin_tx = GPIO('A', 9);
          break;
      }
      break;
    }

    case 2: {
      switch (afio) {
        case 1:
          /* JTAG pins. Need remapping! */
          pin_rx = GPIO('A', 15);
          pin_tx = GPIO('A', 14);
          break;
        case 2:
          /* JTAG pins. Need remapping! */
          pin_rx = GPIO('B', 4);
          pin_tx = GPIO('B', 3);
          break;
        default:
          pin_rx = GPIO('A', 3);
          pin_tx = GPIO('A', 2);
          break;
      }
      break;
    }

#if defined(USART3)
    case 3: {
      switch (afio) {
        case 1:
          pin_rx = GPIO('B', 8);
          pin_tx = GPIO('B', 9);
          break;
        case 2:
          pin_rx = GPIO('C', 11);
          pin_tx = GPIO('C', 10);
          break;
        default:
          pin_rx = GPIO('B', 11);
          pin_tx = GPIO('B', 10);
          break;
      }
      break;
    }
#endif // defined(USART3)

    default:
      Error_Handler();
      return;
  }

  /* Skip RX pin if half duplex */
  if (halfduplex)
    pin_rx = 0;
#endif

  USART_TypeDef * uart_ptr = uart_peripheral_get(pin_tx);
  uint32_t dir = USART_CR1_RE | USART_CR1_TE;
  halfduplex = 0;

  if (!uart_ptr) {
    Error_Handler();
  }

  if (pin_rx) {
    USART_TypeDef * uart_ptr_rx = uart_peripheral_get(pin_rx);
    if (uart_ptr_rx && uart_ptr_rx != uart_ptr) {
      /* RX USART peripheral config */
      usart_hw_init(uart_ptr_rx, baud, USART_CR1_RE, uart_valid_pin_tx(pin_rx));
      dir = USART_CR1_TE;
      halfduplex = 1;
    }
    /* RX pin */
    usart_pin_config(pin_rx, 1);
  } else {
    halfduplex = 1; // No RX pin == half duplex
  }
  /* TX USART peripheral config */
  usart_hw_init(uart_ptr, baud, dir, halfduplex);
  /* TX pin */
  usart_pin_config(pin_tx, 0);

  /* Duplex pin */
  duplex_setup_pin(duplexpin);
  /* Enable RX by default */
  duplex_state_set(DUPLEX_RX);
}

void uart_deinit(void)
{
  if (UART_handle_rx)
    uart_reset(UART_handle_rx);
  if (UART_handle_rx != UART_handle_tx)
    uart_reset(UART_handle_tx);
}
