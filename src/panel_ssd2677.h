#ifndef PANEL_SSD2677_H
#define PANEL_SSD2677_H

#include <stdint.h>

// reTerminal Sticky panel auto-detection and the SSD2677 ("3.97 V2") drive path.
//
// The Sticky is moving from an SSD1677 panel to an SSD2677 panel of the same size
// and pixel count. The config keeps panel_ic_type 69/70 on both: the difference
// is resolved here at runtime, so clients see an unchanged 800x480 mono/4-gray
// device and the resolved variant is never written back to the config.
//
// The probe is a one-byte read of register 0x70 (0x07 = SSD2677). Nothing else
// distinguishes the panels, and bb_epaper's SPI has no read line, so it is
// bit-banged on the panel pins while the SPI peripheral is released.

// Park the Sticky's MicroSD, which shares SPI2 with the panel. No-op on other boards.
void panelStickySdPark();

// Decide which panel is attached. Requires the panel rail to be on. Cheap no-op
// after the first conclusive result, and on any config this does not apply to.
void panelVariantResolve();

// True once the probe has identified an SSD2677.
bool panelSsd2677Active();

// Call after bbepSetPanelType(): replaces the placeholder init data on `bbep`.
void panelSsd2677ApplyOverrides();

// Call after the init sequence, before any image data: selects the waveform for
// the current panel temperature. The controller needs this before the stream.
void panelSsd2677AfterInit();

// Call once BUSY has cleared after a refresh: drops the high-voltage rails.
void panelSsd2677AfterRefresh();

// Image-path replacements for bbepStartWrite/bbepWriteData/bbepRefresh. On any
// panel but the SSD2677 these forward to bb_epaper unchanged.
void epdStartWrite(int plane);
void epdWriteData(const uint8_t* data, int len);
void epdStartRefresh(int mode);

#endif
