#ifndef DISPLAY_FASTEPD_H
#define DISPLAY_FASTEPD_H

#include <stdint.h>
#include <stdbool.h>

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_FASTEPD)

#ifdef __cplusplus
extern "C" {
#endif

struct DisplayConfig;
struct SystemConfig;

void opendisplay_fastepd_load_pins_from_display(const struct DisplayConfig* d, const struct SystemConfig* sys, uint16_t panel_ic_type);
bool fastepd_init_failed(void);

#ifdef __cplusplus
}
#endif

void fastepd_prepare_hardware(void);
void fastepd_epaper_begin(void);
void fastepd_full_update(void);
bool fastepd_wait_refresh(int timeout_sec);
void fastepd_sleep_after_refresh(void);

void fastepd_boot_write_row(uint16_t y, const uint8_t* row, unsigned pitch);
void fastepd_boot_skip_planes(void);

void fastepd_direct_write_reset(void);
void fastepd_direct_write_chunk(const uint8_t* data, uint32_t len);
/** refresh_mode: 0 = REFRESH_FULL, 1 = REFRESH_FAST, 2 = REFRESH_PARTIAL. */
void fastepd_direct_refresh(int refresh_mode);
void fastepd_direct_sleep(void);
/** Clear the hw-init flag so the next push runs a full initIT8951() instead of
 *  wake()-ing a power-cycled IT8951 TCON. Call whenever the panel rail is cut. */
void fastepd_mark_hw_deinitialized(void);

void fastepd_partial_prepare(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
bool fastepd_partial_write_chunk(const uint8_t* data, uint32_t len);
bool fastepd_partial_refresh(int refresh_mode);

#endif

#endif
