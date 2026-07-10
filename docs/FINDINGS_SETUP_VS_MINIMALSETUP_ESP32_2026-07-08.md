# Findings — `setup()` vs `minimalSetup()` on the ESP32 path

**Date:** 2026-07-08
**Scope:** ESP32 (`TARGET_ESP32`) boot only. nRF path excluded.
**Source:** `Firmware/src/main.cpp` @ `a4af6ef`, cross-checked against
`ble_init.cpp`, `display_service.cpp`, `wifi_service.cpp`.
**Line numbers** are from the tree at analysis time.

---

## 1. TL;DR

`setup()` has **two mutually exclusive branches**, chosen by the wake cause:

- **NORMAL BOOT** (power-on / soft reset) → runs the full init sequence inline
  (config → IO → **display + boot screen** → BLE → WiFi → mdata → **buttons + touch**).
- **DEEP-SLEEP WAKE** → calls `minimalSetup()` and `return`s. `minimalSetup()` brings up
  only **config → IO → BLE advertising**, then arms a timed advertising window and hands
  off to `loop()`. Display, WiFi, buttons, and touch are **not** initialized.

The wake path defers the expensive work: if (and only if) a client connects during the
advertising window, `loop()` calls `fullSetupAfterConnection()`, which brings up WiFi and
**seeds the panel type** (but not a full display init). If no client connects before the
window expires, the device goes straight back to deep sleep having spent the minimum energy.

The design intent is **battery**: a wake that nobody talks to should cost only a
config-load + sensor-init + a short BLE advertisement, never a panel power-cycle or a
full-screen refresh.

---

## 2. The common prologue (runs on *every* ESP32 boot)

Everything in `setup()` **before** the branch executes on both paths — `minimalSetup()` is
called *from inside* `setup()` after this prologue, so it inherits it:

| Step | Lines | Notes |
|---|---|---|
| Log UART / USB serial `begin()` + 100 ms settle | 44–50 | `OPENDISPLAY_LOG_UART` vs USB `Serial` |
| Firmware version + Git SHA banner | 51–62 | |
| `esp_reset_reason()` + log | 63–65 | Distinguishes hidden mid-cycle resets |
| `esp_sleep_get_wakeup_cause()` → branch decision | 66–83 | `!= ESP_SLEEP_WAKEUP_UNDEFINED` ⇒ wake |

On a **wake**: `woke_from_deep_sleep = true`, `deep_sleep_count++`, `minimalSetup()`, `return`
(main.cpp:68–75).
On a **normal boot**: `woke_from_deep_sleep = false` and the prologue falls through into the
full sequence (main.cpp:76–82).

`woke_from_deep_sleep` and `deep_sleep_count` are `RTC_DATA_ATTR` (main.h:301–302): they
survive deep sleep **and** soft resets, but not a real power loss. A non-zero
`deep_sleep_count` on a NORMAL BOOT therefore signals a hidden panic/WDT/brownout reset.

---

## 3. Call-sequence comparison

### `setup()` — NORMAL BOOT (main.cpp:84–105)

```
full_config_init()        // load config from flash
initio()                  // LEDs, buzzers, pwr_pin LOW, data buses, sensors
initDisplay()             // ← power panel, draw boot screen, FULL refresh, sleep panel, rail down
ble_init()                // == ble_init_esp32(true)
initWiFi(false)           // non-blocking STA start
updatemsdata()            // refresh BLE manufacturer data (2nd time — see §5.4)
initButtons()
initTouchInput()
```

### `minimalSetup()` — DEEP-SLEEP WAKE (main.cpp:271–284)

```
full_config_init()        // same
initio()                  // same
ble_init_esp32(true)      // same BLE init as normal boot (see §5.1)
advertising_timeout_active = true
advertising_start_time = millis()
// returns to loop(); no display, no WiFi, no buttons, no touch
```

### `fullSetupAfterConnection()` — deferred, called from `loop()` on connect (main.cpp:286–303)

```
initWiFi(false)                       // WiFi deferred to here
if (Seeed_GFX driver) { log; return } // Seeed manages its own panel state
memset(&bbep); mapEpd(); bbepSetPanelType(&bbep, …)  // seed panel type ONLY
```

---

## 4. Step-by-step difference table

| Init step | NORMAL BOOT | `minimalSetup()` (wake) | `fullSetupAfterConnection()` |
|---|:---:|:---:|:---:|
| Serial + FW banner + reset/wake log | ✅ (prologue) | ✅ (inherited) | — |
| `full_config_init()` | ✅ | ✅ | ❌ |
| `initio()` (LEDs, buzzers, `pwr_pin`, I²C buses, sensors) | ✅ | ✅ | ❌ |
| `initDisplay()` (rail up, boot screen, **full refresh**, panel session, rail down) | ✅ | ❌ | ❌ |
| BLE stack + advertising (`ble_init_esp32(true)`) | ✅ | ✅ (identical) | ❌ |
| `initWiFi(false)` (non-blocking STA) | ✅ | ❌ (deferred) | ✅ |
| `updatemsdata()` (mfr data) | ✅ (redundant 2nd call) | ⚠️ via BLE init only | ❌ |
| Panel type seed (`bbepSetPanelType`) | ✅ (inside `initDisplay`) | ❌ | ✅ (type only, no session) |
| Panel rotation (`bbepSetRotation`) | ✅ | ❌ | ❌ (see §6.2) |
| `initButtons()` | ✅ | ❌ | ❌ |
| `initTouchInput()` | ✅ | ❌ | ❌ |
| Arms timed advertising window | ❌ | ✅ | — |

---

## 5. Detailed findings

### 5.1 BLE init is byte-for-byte identical on both paths
`ble_init()` (ble_init.cpp:133) is a one-line wrapper: `ble_init_esp32(true)`. `minimalSetup()`
calls `ble_init_esp32(true)` directly. So the BLE stack — device name `OD<chipid>`, 512-byte
MTU request, service/char `0x2446`, CCCD, advertising — comes up **the same way** on a cold
boot and a wake. There is no "lightweight BLE" mode; the only thing minimal about the wake
path is what surrounds BLE, not BLE itself.

**Implication:** BLE bring-up cost (radio init + advertising start) is paid in full on every
wake. It is not a candidate for further trimming without changing `ble_init_esp32`.

### 5.2 Display is fully skipped on wake — by design
`initDisplay()` (display_service.cpp:1134+) is the single most expensive boot step: it powers
the panel rail (`pwrmgm(true)`, ~900 ms settle), renders the boot screen + QR, runs a **full
EPD refresh** (`waitforrefresh(60)`), sleeps the panel, and drops the rail. `minimalSetup()`
skips **all** of it. A wake that only advertises therefore never touches the panel rail —
the largest single energy saving of the wake path.

The panel is only re-established when an image actually arrives: the direct-write handler
(`imageDataWritten` → `directWrite`, display_service.cpp:1513–1529) does its own
`pwrmgm(true)` + `bbepInitIO` + `bbepWakeUp` + `bbepSendCMDSequence(pInitFull)` +
`bbepSetAddrWindow`. That path depends on `bbep` already carrying the correct **panel type**
(for `pInitFull` / geometry), which is exactly what `fullSetupAfterConnection()` seeds.

### 5.3 WiFi is deferred from `setup()` to `fullSetupAfterConnection()`
Normal boot starts STA immediately (non-blocking, main.cpp:100). The wake path does **not**
start WiFi in `minimalSetup()`; it starts it in `fullSetupAfterConnection()` (main.cpp:288),
i.e. only after a BLE client connects. So during the bare advertising window the WiFi radio
stays off — correct for power, and it means a no-connection wake never spins up WiFi.

Note `initWiFi(false)` is non-blocking either way (wifi_service.cpp:115–117): STA is kicked
off and LAN comes up later when associated. It also no-ops early if WiFi isn't enabled /
configured (wifi_service.cpp:88–104).

### 5.4 `updatemsdata()` is effectively called twice on normal boot
`ble_init_esp32(true)` already calls `updatemsdata()` internally when
`update_manufacturer_data == true` (ble_init.cpp:244–246). Normal boot then calls
`updatemsdata()` **again** at main.cpp:102 — a redundant refresh. The wake path relies on the
single call inside BLE init and is not missing anything. (Minor; the second call is cheap but
dead.)

### 5.5 Buttons and touch are never initialized on the wake cycle
Neither `minimalSetup()` **nor** `fullSetupAfterConnection()` calls `initButtons()` /
`initTouchInput()`. Across an entire wake→advertise→(connect)→deliver→sleep cycle, local
input hardware is inert. `loop()` still *calls* `processButtonEvents()` / `processTouchInput()`
(e.g. main.cpp:202–203), but with no init behind them there is nothing to service.

This is consistent with intent (a wake is a brief, headless delivery window, not an
interactive session), but it is a **behavioral difference worth stating explicitly**: a
button press or touch during a wake window is not registered. See §6.1.

### 5.6 Sensors and I²C buses *are* initialized on wake
`initio()` runs on both paths (main.cpp:86 / 276) and includes `initDataBuses()` +
`initSensors()` (display_service.cpp:567–570). So battery/temp sensors and the I²C bus are up
on a wake — necessary, since the BLE manufacturer data (`updatemsdata`) reports live sensor
values in the advertisement.

---

## 6. Risks / fragile couplings (measured — no live bug confirmed)

### 6.1 No local UI during a wake delivery *(intended, but silent)*
As in §5.5, buttons/touch don't work during the wake window. If future requirements want a
button to, say, force a full refresh or cancel a delivery while awake, that would require
adding the inits to `fullSetupAfterConnection()`. Flagged so it isn't discovered by surprise.

### 6.2 `fullSetupAfterConnection()` seeds panel **type** but not **rotation**
Normal boot sets rotation via `bbepSetRotation(&bbep, rotation*90)` (display_service.cpp:1171);
the wake path never does. Today this is **harmless** because the delivery path is a raw
framebuffer direct-write (`bbepSetAddrWindow(0,0,width,height)`), which does not consult the
GFX rotation state — the host sends bytes already in panel orientation, and no GFX primitives
are drawn on the wake path. It becomes a bug the moment anything GFX-rendered (a status
string, a partial-refresh overlay) is drawn on a wake-delivered frame. Treat the `bbep`
rotation field as **undefined on the wake path**.

### 6.3 Every wake pays `full_config_init()` + `initio()`
These two (config parse from flash + sensor/bus/LED init) are the bulk of non-BLE wake work.
They are unavoidable given the current architecture (config isn't cached in RTC RAM, sensors
must be read for the advertisement), but they are where wake latency/energy goes after BLE.

---

## 7. Post-`setup()` control flow (why the split matters to `loop()`)

The branch chosen in `setup()` steers `loop()` via `advertising_timeout_active`:

- **Wake path** — `minimalSetup()` set `advertising_timeout_active = true`, so `loop()` enters
  the dedicated wake-window branch (main.cpp:111–132) and **nothing else in `loop()` runs**:
  - connected → `fullSetupAfterConnection()`, clear `advertising_timeout_active` /
    `woke_from_deep_sleep`, resume normal loop (main.cpp:112–118);
  - else if `millis() - advertising_start_time ≥ sleep_timeout_ms` (default 10000 if 0) →
    `enterDeepSleep()` (main.cpp:119–129);
  - else `delay(50)` and keep waiting (main.cpp:130–131).
- **Normal boot** — `advertising_timeout_active` stays `false`; `loop()` runs its full body.
  Deep sleep is gated behind a one-time **2-minute first-boot delay**
  (`FIRST_BOOT_DEEP_SLEEP_DELAY_MS = 120000`, main.h:309) that only applies when
  `!woke_from_deep_sleep && deep_sleep_count == 0 && power_mode == 1` (main.cpp:206–222).

Two distinct config knobs govern the wake path, easy to conflate:
- `power_option.sleep_timeout_ms` — **how long to advertise / wait for a connection** after a
  wake (the advertising window; main.cpp:120).
- `power_option.deep_sleep_time_seconds` — **how long to stay asleep** once re-sleeping
  (main.cpp:330).

`advertising_timeout_active` / `advertising_start_time` are plain globals (main.h:305–306),
reset to `false`/`0` each boot — correct, since the advertising window is per-wake and must not
survive across a sleep.

---

## 8. Summary of the essential differences

1. **Display:** full init + boot screen + full refresh on normal boot; **completely skipped**
   on wake (panel only comes up on actual image delivery). — biggest energy delta.
2. **WiFi:** started in `setup()` on normal boot; **deferred to post-connection** on wake.
3. **Buttons/touch:** initialized on normal boot; **never** on the wake cycle.
4. **BLE:** **identical** on both paths.
5. **Config + IO + sensors:** **identical** on both paths (paid every wake).
6. **Panel state on wake:** only **type** is seeded (`fullSetupAfterConnection`), not rotation
   or a full session — sufficient for raw direct-write delivery, fragile for GFX drawing.
7. **Control:** the wake path arms a timed advertising window that monopolizes `loop()` until
   it either connects (→ full setup) or times out (→ deep sleep); normal boot runs the full
   `loop()` with a one-time 2-minute delay before its first sleep.
