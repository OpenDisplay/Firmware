#pragma once

// Soft battery-latch / power-button support for devices that gate their own
// power rail through a MCU-controlled MOSFET (XTEINK X4: GPIO13 holds the
// battery latch closed, GPIO3 is the active-low power button).
//
// All functions compile to no-ops unless OPENDISPLAY_XTEINK_X4 is defined, so
// callers in shared code need no per-board guards.

// Assert the keep-alive latch at boot so the device stays powered on battery.
// Safe to call on every boot, including timer-wake from deep sleep.
void powerLatchBegin();

// Poll the hardware power button; perform a clean power-off on a long press.
// Cheap; intended to be called from the existing button-polling path.
void powerButtonPoll();

// Keep the latch closed across OpenDisplay's timer-wake deep sleep so the
// device retains power and can wake itself. Call immediately before
// esp_deep_sleep_start().
void powerLatchHoldForSleep();
