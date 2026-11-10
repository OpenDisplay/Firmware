#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

// Both require globalConfig.display_count > 0. The caller must check this.
// Both dereference displays[0] unconditionally.
bool writeBootScreenWithQr();
bool showLowBatteryScreen();

#endif
