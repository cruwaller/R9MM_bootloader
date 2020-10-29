#include "led.h"
#include "main.h"

#if defined(WS2812_LED_PIN) || (defined(WS2812_LED_Port) && defined(WS2812_LED_Pin))
#if defined(WS2812_LED_PIN)
static void *ws2812_port;
static uint16_t ws2812_pin;
#else  // !WS2812_LED_PIN
#define ws2812_port WS2812_LED_Port
#define ws2812_pin WS2812_LED_Pin
#endif // WS2812_LED_PIN

#if GPIO_USE_LL
#define GPIO_WRITE_LOW(_state)  LL_GPIO_ResetOutputPin(ws2812_port, ws2812_pin)
#define GPIO_WRITE_HIGH(_state) LL_GPIO_SetOutputPin(ws2812_port, ws2812_pin)
#else // GPIO_USE_LL
#define GPIO_WRITE_LOW(_state) \
  HAL_GPIO_WritePin(ws2812_port, ws2812_pin, GPIO_PIN_RESET)
#define GPIO_WRITE_HIGH(_state) \
  HAL_GPIO_WritePin(ws2812_port, ws2812_pin, GPIO_PIN_SET)
#endif // GPIO_USE_LL


#define WS2812_DELAY_LONG() \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP();

#define WS2812_DELAY_SHORT() \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP();


static inline void ws2812_send_1(void)
{
    GPIO_WRITE_HIGH();
    WS2812_DELAY_LONG();
    GPIO_WRITE_LOW();
    WS2812_DELAY_SHORT();
}

static inline void ws2812_send_0(void)
{
    GPIO_WRITE_HIGH();
    WS2812_DELAY_SHORT();
    GPIO_WRITE_LOW();
    WS2812_DELAY_LONG();
}

static uint32_t bitReverse(uint8_t input)
{
    uint8_t r = input; // r will be reversed bits of v; first get LSB of v
    uint8_t s = 8 - 1; // extra shift needed at end

    for (input >>= 1; input; input >>= 1) {
        r <<= 1;
        r |= input & 1;
        s--;
    }
    r <<= s; // shift when input's highest bits are zero
    return r;
}

static void ws2812_send_color(uint8_t const *const RGB) // takes RGB data
{
    uint32_t LedColourData =
        bitReverse(RGB[1]) +        // Green
        (bitReverse(RGB[0]) << 8) + // Red
        (bitReverse(RGB[2]) << 16); // Blue
    uint8_t bits = 24;
    while (bits--) {
        (LedColourData & 0x1) ? ws2812_send_1() : ws2812_send_0();
        LedColourData >>= 1;
    }
}

void ws2812_init(void)
{
#if defined(WS2812_LED_PIN)
    gpio_port_pin_get(CREATE_IO(WS2812_LED_PIN), &ws2812_port, &ws2812_pin);
#endif
#if GPIO_USE_LL
    LL_GPIO_SetPinMode(ws2812_port, ws2812_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(ws2812_port, ws2812_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(ws2812_port, ws2812_pin, LL_GPIO_SPEED_FREQ_HIGH);
    //LL_GPIO_SetPinPull(ws2812_port, ws2812_pin, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(ws2812_port, ws2812_pin);
#else // !GPIO_USE_LL
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_port_pin_get(CREATE_IO(WS2812_LED_PIN), &ws2812_port, &ws2812_pin);
    GPIO_InitStruct.Pin = ws2812_pin;
    HAL_GPIO_Init(ws2812_port, &GPIO_InitStruct);
#endif // GPIO_USE_LL
}

void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b)
{
    uint8_t data[3] = {r, g, b};
    ws2812_send_color(data);
}

#else
void ws2812_init(void) {}
void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b) {}
#endif /* WS2812_RED */
