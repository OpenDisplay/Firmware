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
/**
 * Drive the suspend counter to 0 unconditionally, resuming touch. Idempotent.
 *
 * For session teardown (abortToKnownState), where the balanced suspend/resume
 * pairing cannot be relied on: a teardown routed through the partial path bypasses
 * cleanupDirectWriteState() -- the only place that clears directWriteTouchSuspended
 * -- and leaves the counter stuck above zero, so touch never comes back for the
 * rest of the boot. Not for the refresh brackets, which stay balanced.
 */
void touchForceResume(void);

#endif
