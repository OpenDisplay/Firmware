#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef OPND_SEEED_TCON_BUSY_TIMEOUT_MS
#define OPND_SEEED_TCON_BUSY_TIMEOUT_MS 15000u
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool opnd_seeed_tcon_busy_timed_out;
void opnd_seeed_tcon_busy_timeout_reset(void);

extern int8_t opnd_seeed_runtime_sclk;
extern int8_t opnd_seeed_runtime_miso;
extern int8_t opnd_seeed_runtime_mosi;
extern int8_t opnd_seeed_runtime_cs;
extern int8_t opnd_seeed_runtime_rst;
extern int8_t opnd_seeed_runtime_busy;
extern int8_t opnd_seeed_runtime_tft_enable;
extern int8_t opnd_seeed_runtime_ite_enable;

#ifdef __cplusplus
}
#endif
