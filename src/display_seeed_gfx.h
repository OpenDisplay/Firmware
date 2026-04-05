#ifndef DISPLAY_SEEED_GFX_H
#define DISPLAY_SEEED_GFX_H

#include <stdint.h>
#include <stdbool.h>

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)

#ifdef __cplusplus
extern "C" {
#endif

struct DisplayConfig;
struct SystemConfig;

void opendisplay_seeed_gfx_load_pins_from_display(const struct DisplayConfig* d, const struct SystemConfig* sys, uint16_t panel_ic_type);
void opnd_seeed_tcon_busy_timeout_reset(void);
bool opnd_seeed_tcon_busy_timeout_occurred(void);

#ifdef __cplusplus
}
#endif

void seeed_gfx_prepare_hardware(void);
void seeed_gfx_epaper_begin(void);
void seeed_gfx_full_update(void);
bool seeed_gfx_wait_refresh(int timeout_sec);
void seeed_gfx_sleep_after_refresh(void);

void seeed_gfx_boot_write_row(uint16_t y, const uint8_t* row, unsigned pitch);
void seeed_gfx_boot_skip_planes(void);

void seeed_gfx_direct_write_reset(void);
void seeed_gfx_direct_write_chunk(const uint8_t* data, uint32_t len);
/** refresh_mode: 0 = REFRESH_FULL (two GC16 passes, less ghosting), 1 = REFRESH_FAST (single pass). */
void seeed_gfx_direct_refresh(int refresh_mode);
void seeed_gfx_direct_sleep(void);

#endif

#endif
