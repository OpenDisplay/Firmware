#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
#include "OpenDisplay/opendisplay_runtime_pins.h"
#endif
#ifdef TFT_BUSY
#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
    if (opnd_seeed_runtime_busy >= 0) pinMode(opnd_seeed_runtime_busy, INPUT);
#else
    pinMode(TFT_BUSY, INPUT);
#endif
#endif
#ifdef TFT_ENABLE
#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
    if (opnd_seeed_runtime_tft_enable >= 0) {
        pinMode(opnd_seeed_runtime_tft_enable, OUTPUT);
        digitalWrite(opnd_seeed_runtime_tft_enable, HIGH);
    }
#else
    pinMode(TFT_ENABLE, OUTPUT);
    digitalWrite(TFT_ENABLE, HIGH);
#endif
#endif
#ifdef ITE_ENABLE
#if defined(OPENDISPLAY_SEEED_GFX_RUNTIME_PINS)
    if (opnd_seeed_runtime_ite_enable >= 0) {
        pinMode(opnd_seeed_runtime_ite_enable, OUTPUT);
        digitalWrite(opnd_seeed_runtime_ite_enable, HIGH);
    }
#else
    pinMode(ITE_ENABLE, OUTPUT);
    digitalWrite(ITE_ENABLE, HIGH);
#endif
#endif
    hostTconInitFast();  
