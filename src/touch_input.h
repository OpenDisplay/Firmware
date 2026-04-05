#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include <cstdint>

void initTouchInput(void);
void processTouchInput(void);
bool touch_input_gpio_is_touch_int(uint8_t pin);

#endif
