#ifndef __LED_H_
#define __LED_H_

#include <stdint.h>

void ws2812_init(void);
void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b);

#endif /* __LED_H_ */
