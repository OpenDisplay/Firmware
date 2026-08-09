#ifndef SPLIT_PANEL_H
#define SPLIT_PANEL_H

#include <stdint.h>
#include <stdbool.h>

// Dual-controller ("split buffer") e-paper panels — a single glass driven by two
// controllers on separate chip-selects, each owning one half of every row.
// Currently the Seeed reTerminal E1004's 13.3" 1200x1600 Spectra6 (T133A01) and
// the 8.1" 1024x576 Spectra6.
//
// Selection is entirely at RUNTIME: splitPanelUsed() tests BBEP_SPLIT_BUFFER on
// bbep.iFlags, which bbepSetPanelType() fills in from the panel table for whatever
// panel_ic_type the device config names. There is no build flag — this code is
// compiled into every target, and a device that is not configured for a split
// panel simply never enters it. The predecessor of this file was gated on
// BBEP_T133A01, a *bb_epaper* define rather than a firmware one, so repinning the
// library silently compiled the whole panel away while the build still succeeded.
//
// BUFFERLESS. Image bytes go straight from the transport to SPI; nothing here
// allocates a framebuffer. bb_epaper's own dual-controller writer
// (bbepWriteImage4bppDual, reached via bbepWritePlane) requires a 960 KB
// ucScreen, so this drives the data phase directly from the library's CS
// primitives instead — cs_mode = CMD_CS_NONE, manual chip-select, one DTM per
// controller, chunked bbepWriteData. That is the same idiom bbepWriteImage4bppDual
// uses internally; only the byte source differs.
//
// The other three phases need no help: the panel's init sequence carries
// CMD_CS1/CMD_CS1_CS2 opcodes, and bbepRefresh()/bbepSleep() broadcast DRF/POFF to
// both controllers off the BBEP_SPLIT_BUFFER flag whether or not a buffer exists.
//
// The limengdu bb_epaper fork exported bbepStartDataStream/WriteDataStreamByte/
// EndDataStream for exactly this; upstream removed them, which is what this
// module replaces.

/** True when the configured panel is dual-controller. Valid once bbepSetPanelType() has run. */
bool splitPanelUsed(void);

/** CS2 GPIO from the device config (displays[0].cs_pin_2), defaulting to 2. */
uint8_t splitPanelCs2Pin(void);

/**
 * Cold bring-up: set CS2, then bbepInitIO(), which sends the panel's init
 * sequence — whose embedded CMD_CS1_CS2 opcodes need CS2 already configured, hence
 * the ordering.
 */
void splitPanelInitIo(void);

/**
 * Validate config geometry against the panel. Call from
 * configureBbepPanelGeometry() after bbepSetPanelType(). Sets the flag read by
 * splitPanelGeometryOk().
 */
void splitPanelConfigureGeometry(void);

/** True when config dimensions and colour scheme match the panel. */
bool splitPanelGeometryOk(void);

/**
 * Open the frame: wait out any live refresh, take manual control of both chip
 * selects, and send the first DTM to the primary controller. Every exit path owes
 * a splitPanelCloseFrame() — leaving cs_mode at CMD_CS_NONE makes every
 * subsequent library write silently reach no controller at all.
 */
bool splitPanelBeginFrame(void);

/**
 * Stream image bytes in OD_COLOR_SCHEME_BWGBRY_SPLIT wire order — the entire left
 * half-plane (h rows of native_width/4 bytes) followed by the entire right
 * half-plane. Crossing to the second controller happens here, at the half
 * boundary. Bytes past the end of the frame are dropped and the frame is flagged.
 */
void splitPanelSinkBytes(const uint8_t* data, uint32_t len);

/**
 * Release both chip selects and restore cs_mode. Idempotent, and safe to call on
 * teardown paths that never opened a frame. Returns true only when a complete,
 * unfaulted frame was streamed — callers refreshing the panel should treat false
 * as "do not refresh", so a short or overrun frame cannot reach the glass as a
 * successful update.
 */
bool splitPanelCloseFrame(void);

/**
 * Power both controllers down after a refresh has completed. bb_epaper's C
 * bbepRefresh() issues DRF and returns without waiting or powering off (only the
 * C++ BBEPAPER::refresh wrapper sends POFF, and this firmware does not use it),
 * so without this the HV rails stay up for the whole keep-alive window.
 */
void splitPanelPowerOff(void);

#endif
