#ifndef __LED_H_
#define __LED_H_

#include <stdint.h>

#if defined(WS2812_LED_PIN) || (defined(WS2812_LED_Port) && defined(WS2812_LED_Pin))
void ws2812_init(void);
void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b);
#else
#define ws2812_init()
#define ws2812_set_color(...)
#endif
#endif /* __LED_H_ */
