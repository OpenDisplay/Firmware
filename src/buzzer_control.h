#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include <stdint.h>

void initPassiveBuzzers(void);
void handleBuzzerActivate(uint8_t* data, uint16_t len);

#endif
