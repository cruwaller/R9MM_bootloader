/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L0xx
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_usart.h"
#elif defined(STM32L1xx)
#include "stm32l1xx.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_usart.h"
#elif defined(STM32F1)
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#elif defined(STM32F3xx)
#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#elif defined(STM32L4xx)
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_usart.h"
#else
#error "Not supported CPU type!"
#endif

#define STRINGIFY(str) #str
#define BUILD_MCU_TYPE(type) STRINGIFY(BL_TYPE: type)

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

enum duplex_state
{
  DUPLEX_RX,
  DUPLEX_TX,
};

enum {
  A = 'A',
  B = 'B',
  C = 'C',
  D = 'D',
  E = 'E',
  F = 'F'
};

#define IO_CREATE_IMPL(port, pin) ((port * 16) + pin)
#define IO_CREATE(...)  IO_CREATE_IMPL(__VA_ARGS__)
#define IO_GET_PORT(_p) (((_p) / 16) - 'A')
#define IO_GET_PIN(_p)  ((_p) % 16)

enum led_states
{
  LED_OFF,
  LED_BOOTING,
  LED_FLASHING,
  LED_FLASHING_ALT,
  LED_STARTING,
};

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void led_state_set(uint32_t state);
void duplex_state_set(const enum duplex_state state);
int8_t timer_end(void);

void gpio_port_pin_get(uint32_t io, void ** port, uint32_t * pin);
void gpio_port_clock(uint32_t port);

#if GPIO_USE_LL
/*static inline void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t state)
{
  if (state)
    LL_GPIO_SetOutputPin(GPIOx, pin);
  else
    LL_GPIO_ResetOutputPin(GPIOx, pin);
}*/
#define GPIO_WritePin(port, pin, _state) \
  (_state) ? LL_GPIO_SetOutputPin(port, pin) : \
  LL_GPIO_ResetOutputPin(port, pin)
#else
#define GPIO_WritePin(...) HAL_GPIO_WritePin(__VA_ARGS__)
#endif

/* Private defines -----------------------------------------------------------*/
#if !defined(XMODEM) && !STK500 && !FRSKY
#define XMODEM 1 // Default is XMODEM protocol
#endif

#ifdef BUTTON
#if TARGET_R9MX
#define BTN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_0 : GPIO_PIN_0)
#define BTN_GPIO_Port GPIOB
#elif TARGET_R9MM
#define BTN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_13 : GPIO_PIN_13)
#define BTN_GPIO_Port GPIOC
#elif TARGET_SX1280_RX_Nano_v05
#define BTN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3)
#define BTN_GPIO_Port GPIOA
#elif TARGET_GHOST_RX_V1_2
#define BTN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_12 : GPIO_PIN_12)
#define BTN_GPIO_Port GPIOA
#endif
#endif /* BUTTON */

#ifdef LED_GRN
#if TARGET_R9MM
#define LED_GRN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3)
#define LED_GRN_GPIO_Port GPIOB
#elif TARGET_R9M
#define LED_GRN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_12 : GPIO_PIN_12)
#define LED_GRN_GPIO_Port GPIOA
#elif TARGET_R9MX
#define LED_GRN_Pin (GPIO_USE_LL ? LL_GPIO_PIN_3 : GPIO_PIN_3)
#define LED_GRN_GPIO_Port GPIOB
#endif
#endif /* LED_GRN */

#ifdef LED_RED
#if TARGET_R9MM
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_1 : GPIO_PIN_1)
#define LED_RED_GPIO_Port GPIOC
#elif TARGET_R9M
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_11 : GPIO_PIN_11)
#define LED_RED_GPIO_Port GPIOA
#elif TARGET_RHF76
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_4 : GPIO_PIN_4)
#define LED_RED_GPIO_Port GPIOB
#elif TARGET_RAK4200
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_12 : GPIO_PIN_12)
#define LED_RED_GPIO_Port GPIOA
#elif TARGET_RAK811

#elif TARGET_SX1280_RX_2020_v02
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_15 : GPIO_PIN_15)
#define LED_RED_GPIO_Port GPIOB
#elif TARGET_R9MX
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_2 : GPIO_PIN_2)
#define LED_RED_GPIO_Port GPIOB
#elif TARGET_SX1280_RX_Nano_v05
#define LED_RED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_5 : GPIO_PIN_5)
#define LED_RED_GPIO_Port GPIOB
#elif TARGET_GHOST_RX_V1_2
#define WS2812_LED_Pin (GPIO_USE_LL ? LL_GPIO_PIN_7 : GPIO_PIN_7)
#define WS2812_LED_Port GPIOA
#endif
#endif /* LED_RED */

#if TARGET_R9M
#define DUPLEX_Pin (GPIO_USE_LL ? LL_GPIO_PIN_5 : GPIO_PIN_5)
#define DUPLEX_Port GPIOA
#endif /* TARGET_R9M */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
