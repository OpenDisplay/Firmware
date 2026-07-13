#include "buzzer_control.h"
#include "buzzer_hw.h"
#include "structs.h"
#include <Arduino.h>
#include <string.h>

extern struct GlobalConfig globalConfig;
void sendResponse(uint8_t* response, uint16_t len);

static_assert(sizeof(PassiveBuzzerConfig) == 32, "PassiveBuzzerConfig must be 32 bytes");

// Playable frequency window (old 400 Hz / 12 000 Hz limits, now centi-Hz).
static constexpr uint32_t kBuzzerFreqMinCentiHz = 40000u;      // 400.00 Hz
static constexpr uint32_t kBuzzerFreqMaxCentiHz = 1200000u;    // 12 000.00 Hz
static constexpr uint8_t kBuzzerDurationUnitMs = 5u;
static constexpr uint32_t kBuzzerInterPatternGapMs = 20u;
static constexpr uint32_t kBuzzerMaxTotalMs = 30000u;

// Quarter-tone frequency table in centi-Hz (Hz * 100):
//   kBuzzerCentiHzTable[idx] = round(100 * 13.75 * 2^(idx/24)), entry [0] = 0.
// 24 quarter-tones/octave anchored at A-1 = 13.75 Hz; idx 120 = A4 = 440.00 Hz
// exact. uint32 (not float) because ESP32-C3/C6 are soft-float and both PWM
// backends want integer math; max 2 171 433 << 2^32. Entry 0 = nNone sentinel.
//
// Generated -- do not hand-edit. Regenerate with:
//   python3 -c "print(',\n'.join(', '.join(f'{round(100*13.75*2**(i/24)) if i else 0:8d}u' for i in range(r, r+8)) for r in range(0, 256, 8)))"
static constexpr uint32_t kBuzzerCentiHzTable[256] = {
           0u,     1415u,     1457u,     1499u,     1543u,     1589u,     1635u,     1683u,
        1732u,     1783u,     1835u,     1889u,     1945u,     2002u,     2060u,     2121u,
        2183u,     2247u,     2312u,     2380u,     2450u,     2522u,     2596u,     2672u,
        2750u,     2831u,     2914u,     2999u,     3087u,     3177u,     3270u,     3366u,
        3465u,     3566u,     3671u,     3778u,     3889u,     4003u,     4120u,     4241u,
        4365u,     4493u,     4625u,     4760u,     4900u,     5044u,     5191u,     5343u,
        5500u,     5661u,     5827u,     5998u,     6174u,     6354u,     6541u,     6732u,
        6930u,     7133u,     7342u,     7557u,     7778u,     8006u,     8241u,     8482u,
        8731u,     8987u,     9250u,     9521u,     9800u,    10087u,    10383u,    10687u,
       11000u,    11322u,    11654u,    11996u,    12347u,    12709u,    13081u,    13465u,
       13859u,    14265u,    14683u,    15113u,    15556u,    16012u,    16481u,    16964u,
       17461u,    17973u,    18500u,    19042u,    19600u,    20174u,    20765u,    21374u,
       22000u,    22645u,    23308u,    23991u,    24694u,    25418u,    26163u,    26929u,
       27718u,    28530u,    29366u,    30227u,    31113u,    32024u,    32963u,    33929u,
       34923u,    35946u,    36999u,    38084u,    39200u,    40348u,    41530u,    42747u,
       44000u,    45289u,    46616u,    47982u,    49388u,    50836u,    52325u,    53858u,
       55437u,    57061u,    58733u,    60454u,    62225u,    64049u,    65926u,    67857u,
       69846u,    71892u,    73999u,    76167u,    78399u,    80696u,    83061u,    85495u,
       88000u,    90579u,    93233u,    95965u,    98777u,   101671u,   104650u,   107717u,
      110873u,   114122u,   117466u,   120908u,   124451u,   128097u,   131851u,   135715u,
      139691u,   143785u,   147998u,   152334u,   156798u,   161393u,   166122u,   170990u,
      176000u,   181157u,   186466u,   191929u,   197553u,   203342u,   209300u,   215433u,
      221746u,   228244u,   234932u,   241816u,   248902u,   256195u,   263702u,   271429u,
      279383u,   287569u,   295996u,   304669u,   313596u,   322785u,   332244u,   341979u,
      352000u,   362314u,   372931u,   383859u,   395107u,   406684u,   418601u,   430867u,
      443492u,   456488u,   469864u,   483632u,   497803u,   512390u,   527404u,   542858u,
      558765u,   575138u,   591991u,   609338u,   627193u,   645571u,   664488u,   683958u,
      704000u,   724629u,   745862u,   767717u,   790213u,   813368u,   837202u,   861734u,
      886984u,   912975u,   939727u,   967263u,   995606u,  1024780u,  1054808u,  1085716u,
     1117530u,  1150276u,  1183982u,  1218675u,  1254385u,  1291142u,  1328975u,  1367917u,
     1408000u,  1449258u,  1491724u,  1535435u,  1580427u,  1626737u,  1674404u,  1723467u,
     1773969u,  1825950u,  1879455u,  1934527u,  1991213u,  2049560u,  2109616u,  2171433u
};

// Playable index bounds derived from the table (C++11: recursive constexpr, no
// loops). kBuzzerMinNoteIdx = first idx >= min freq, kBuzzerMaxNoteIdx = last
// idx <= max freq. They compute to 117 (403.48 Hz) and 234 (11 839.82 Hz).
static constexpr uint8_t buzzer_scan_min(uint16_t i) {
    return (i >= 256) ? 1u
         : (kBuzzerCentiHzTable[i] >= kBuzzerFreqMinCentiHz) ? (uint8_t)i
         : buzzer_scan_min((uint16_t)(i + 1));
}
static constexpr uint8_t buzzer_scan_max(uint16_t i) {
    return (i == 0) ? 255u
         : (kBuzzerCentiHzTable[i] <= kBuzzerFreqMaxCentiHz) ? (uint8_t)i
         : buzzer_scan_max((uint16_t)(i - 1));
}
static constexpr uint8_t kBuzzerMinNoteIdx = buzzer_scan_min(1);
static constexpr uint8_t kBuzzerMaxNoteIdx = buzzer_scan_max(255);
static_assert(kBuzzerMinNoteIdx == 117, "kBuzzerMinNoteIdx must derive to 117");
static_assert(kBuzzerMaxNoteIdx == 234, "kBuzzerMaxNoteIdx must derive to 234");

// Octave-fold an out-of-range index into the playable window, preserving pitch
// class (shift +/-24 = one octave at a time). nNone passes through unchanged.
static uint8_t buzzer_fold_index(uint8_t idx) {
    if (idx == 0) {
        return 0;
    }
    uint16_t i = idx;
    while (i < kBuzzerMinNoteIdx) {
        i += 24;                         // below range: up an octave
    }
    while (i > kBuzzerMaxNoteIdx) {
        i -= 24;                         // above range: down an octave
    }
    if (i < kBuzzerMinNoteIdx) {
        i = kBuzzerMinNoteIdx;           // defensive: degenerate (< 1 octave) range
    }
    return (uint8_t)i;
}

// Single lookup point: fold then read the table.
static uint32_t buzzer_index_to_centihz(uint8_t idx) {
    return kBuzzerCentiHzTable[buzzer_fold_index(idx)];
}

static void buzzer_set_enable(const PassiveBuzzerConfig* b, bool on) {
    if (b->enable_pin == 0xFF) {
        return;
    }
    bool high = on;
    if ((b->flags & BUZZER_FLAG_ENABLE_ACTIVE_HIGH) == 0) {
        high = !high;
    }
    digitalWrite(b->enable_pin, high ? HIGH : LOW);
}

static void buzzer_drive_off(const PassiveBuzzerConfig* b) {
    pinMode(b->drive_pin, OUTPUT);
    digitalWrite(b->drive_pin, LOW);
}

// --- Non-blocking playback state machine (modeled on s_led/processLedFlash) ---
// Own copy of the melody: ESP32 recycles the incoming command buffer the moment
// the handler returns, so the payload MUST be copied before playback.

typedef enum {
    BZ_PHASE_STEP = 0,    // ready to emit step si of the current pattern
    BZ_PHASE_GAP,         // waiting the inter-pattern gap
} buzzer_phase_t;

static struct {
    bool active;
    PassiveBuzzerConfig* b;
    uint8_t melody[256];      // copied payload: [inst][outer][pattern_count][patterns...]
    uint16_t melody_len;
    uint8_t outer;            // total outer repeats
    uint8_t rep;              // current repeat (0-based)
    uint8_t pattern_count;
    uint8_t pi;               // current pattern (0-based)
    uint16_t poff;            // cursor into melody
    uint8_t nsteps;           // steps in the current pattern
    uint8_t si;               // current step (0-based)
    buzzer_phase_t phase;
    bool tone_on;             // a tone is currently sounding on drive_pin
    uint32_t step_until_ms;   // wait deadline for the current step/gap
    uint32_t play_start_ms;   // for the 5 s total cap
} s_buzzer;

static void buzzer_stop_internal(void) {
    if (s_buzzer.b != nullptr) {
        if (s_buzzer.tone_on) {
            buzzer_hw_tone_stop(s_buzzer.b->drive_pin);
        }
        buzzer_set_enable(s_buzzer.b, false);
        buzzer_drive_off(s_buzzer.b);
    }
    memset(&s_buzzer, 0, sizeof(s_buzzer));
}

// Emit the next scheduled item (tone / rest / gap) and return once a timed wait
// is scheduled, or finish. The currently-playing tone (if any) is stopped by
// the caller (buzzerService) before this runs; the handler calls it with none.
static void buzzer_run(void) {
    uint32_t now = millis();
    for (;;) {
        if (!s_buzzer.active) {
            return;
        }
        // Global 5 s cap.
        if ((uint32_t)(now - s_buzzer.play_start_ms) >= kBuzzerMaxTotalMs) {
            buzzer_stop_internal();
            return;
        }

        switch (s_buzzer.phase) {
            case BZ_PHASE_STEP:
                if (s_buzzer.si < s_buzzer.nsteps) {
                    uint8_t fidx = s_buzzer.melody[s_buzzer.poff];
                    uint8_t dunit = s_buzzer.melody[s_buzzer.poff + 1];
                    s_buzzer.poff += 2;
                    s_buzzer.si++;

                    uint32_t ms = (uint32_t)dunit * (uint32_t)kBuzzerDurationUnitMs;
                    uint32_t remaining = kBuzzerMaxTotalMs - (now - s_buzzer.play_start_ms);
                    if (ms > remaining) {
                        ms = remaining;
                    }
                    if (ms == 0) {
                        continue;    // zero-length step: no-op, on to the next
                    }

                    // Enable stays asserted for the step's duration even on a
                    // rest (nNone), preserving melody rhythm.
                    buzzer_set_enable(s_buzzer.b, true);
                    uint32_t centihz = buzzer_index_to_centihz(fidx);
                    if (centihz != 0) {
                        if (buzzer_hw_tone_start(s_buzzer.b->drive_pin, centihz,
                                                 s_buzzer.b->duty_percent)) {
                            s_buzzer.tone_on = true;
                        }
                    } else {
                        buzzer_hw_tone_stop(s_buzzer.b->drive_pin);   // rest: tone off
                        s_buzzer.tone_on = false;
                    }
                    s_buzzer.step_until_ms = now + ms;
                    return;
                }
                // Pattern finished.
                if (s_buzzer.pi + 1 < s_buzzer.pattern_count) {
                    s_buzzer.phase = BZ_PHASE_GAP;                    // gap before next pattern
                    s_buzzer.step_until_ms = now + kBuzzerInterPatternGapMs;
                    return;
                }
                // Last pattern of this repeat -> next repeat or finish (no gap
                // between repeats, matching the original playback).
                s_buzzer.rep++;
                if (s_buzzer.rep >= s_buzzer.outer) {
                    buzzer_stop_internal();
                    return;
                }
                s_buzzer.pi = 0;
                s_buzzer.poff = 3;
                s_buzzer.nsteps = s_buzzer.melody[s_buzzer.poff++];
                s_buzzer.si = 0;
                break;   // stay in BZ_PHASE_STEP

            case BZ_PHASE_GAP:
                s_buzzer.pi++;
                s_buzzer.nsteps = s_buzzer.melody[s_buzzer.poff++];
                s_buzzer.si = 0;
                s_buzzer.phase = BZ_PHASE_STEP;
                break;

            default:
                buzzer_stop_internal();
                return;
        }
    }
}

void buzzerService(void) {
    if (!s_buzzer.active) {
        return;
    }
    if ((int32_t)(millis() - s_buzzer.step_until_ms) < 0) {
        return;   // current step/gap still in progress
    }
    // Wait elapsed: silence the current tone, then advance.
    if (s_buzzer.tone_on) {
        buzzer_hw_tone_stop(s_buzzer.b->drive_pin);
        s_buzzer.tone_on = false;
    }
    buzzer_run();
}

void initPassiveBuzzers(void) {
    for (uint8_t i = 0; i < globalConfig.passive_buzzer_count; i++) {
        const PassiveBuzzerConfig* b = &globalConfig.passive_buzzers[i];
        if (b->drive_pin == 0xFF) {
            continue;
        }
        pinMode(b->drive_pin, OUTPUT);
        digitalWrite(b->drive_pin, LOW);
        if (b->enable_pin != 0xFF) {
            pinMode(b->enable_pin, OUTPUT);
            buzzer_set_enable(b, false);
        }
    }
}

void handleBuzzerActivate(uint8_t* data, uint16_t len) {
    if (len < 3) {
        uint8_t err[] = {0xFF, 0x77, 0x01, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }
    uint8_t inst = data[0];
    if (inst >= globalConfig.passive_buzzer_count) {
        uint8_t err[] = {0xFF, 0x77, 0x02, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }
    PassiveBuzzerConfig* b = &globalConfig.passive_buzzers[inst];
    if (b->drive_pin == 0xFF) {
        uint8_t err[] = {0xFF, 0x77, 0x03, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    uint8_t outer = data[1];
    if (outer == 0) {
        outer = 1;
    }
    uint8_t pattern_count = data[2];
    if (pattern_count == 0) {
        uint8_t err[] = {0xFF, 0x77, 0x04, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    uint16_t scan = 3;
    for (uint8_t pi = 0; pi < pattern_count; pi++) {
        if (scan >= len) {
            uint8_t err[] = {0xFF, 0x77, 0x05, 0x00};
            sendResponse(err, sizeof(err));
            return;
        }
        uint8_t nsteps = data[scan++];
        uint32_t need = (uint32_t)nsteps * 2u;
        if (scan + need > len) {
            uint8_t err[] = {0xFF, 0x77, 0x05, 0x00};
            sendResponse(err, sizeof(err));
            return;
        }
        scan = (uint16_t)(scan + need);
    }
    if (scan != len) {
        uint8_t err[] = {0xFF, 0x77, 0x06, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }
    if (len > sizeof(s_buzzer.melody)) {
        uint8_t err[] = {0xFF, 0x77, 0x05, 0x00};
        sendResponse(err, sizeof(err));
        return;
    }

    // Preempt any melody already playing, then take our own copy of the payload
    // (the incoming buffer is recycled the moment we return).
    buzzer_stop_internal();
    memcpy(s_buzzer.melody, data, len);
    s_buzzer.melody_len = len;
    s_buzzer.b = b;
    s_buzzer.outer = outer;
    s_buzzer.rep = 0;
    s_buzzer.pattern_count = pattern_count;
    s_buzzer.pi = 0;
    s_buzzer.poff = 3;
    s_buzzer.nsteps = s_buzzer.melody[s_buzzer.poff++];
    s_buzzer.si = 0;
    s_buzzer.phase = BZ_PHASE_STEP;
    s_buzzer.tone_on = false;
    s_buzzer.play_start_ms = millis();
    s_buzzer.active = true;

    // Start the first step, then ACK "accepted & started" immediately.
    buzzer_run();

    uint8_t ok[] = {0x00, 0x77, 0x00, 0x00};
    sendResponse(ok, sizeof(ok));
}

void passiveBuzzerPowerOffAlert(void) {
    // Preempt any active melody so the shutdown chirp always sounds.
    buzzer_stop_internal();

    const PassiveBuzzerConfig* b = nullptr;
    for (uint8_t i = 0; i < globalConfig.passive_buzzer_count; i++) {
        const uint8_t pin = globalConfig.passive_buzzers[i].drive_pin;
        if (pin != 0 && pin != 0xFF) {
            b = &globalConfig.passive_buzzers[i];
            break;
        }
    }
    if (!b) {
        return;
    }
    const uint32_t centihz = buzzer_index_to_centihz(nG8);   // ~6271.93 Hz
    buzzer_set_enable(b, true);
    buzzer_hw_tone_start(b->drive_pin, centihz, b->duty_percent);
    delay(80);
    buzzer_hw_tone_stop(b->drive_pin);
    delay(80);
    buzzer_hw_tone_start(b->drive_pin, centihz, b->duty_percent);
    delay(80);
    buzzer_hw_tone_stop(b->drive_pin);
    buzzer_set_enable(b, false);
    buzzer_drive_off(b);
}
