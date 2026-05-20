#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include <cstdint>

void initTouchInput(void);
void processTouchInput(void);
bool touch_input_gpio_is_touch_int(uint8_t pin);
/** Suspend GT911 polling during EPD refresh (nested calls OK). */
void touchSuspendForEpdRefresh(void);
/** Resume touch after EPD refresh; re-inits I2C for active controllers. */
void touchResumeAfterEpdRefresh(void);

#endif
