# Plan — idle-session watchdog: disconnect a BLE client that holds the link doing nothing

**Date:** 2026-07-29
**Branch:** `fix/nrf-no-adv-while-connected` (or a fresh `feat/ble-idle-session-kick` off it)
**Scope:** one new transport method, one new loop()-serviced watchdog, one tunable
(`OD_BLE_IDLE_SESSION_TIMEOUT_MS`, default 120 s). Five files, ~46 inserted lines,
no existing line modified. No protocol change, no config-packet change, no
cross-repo work.

### Revision history

**Draft 1** proposed the watchdog with `pollActivity()`-style RX/TX queue-head
tracking. **Draft 2** removed that tracking and raised the timeout 60 s → 120 s,
arguing the extra margin made it unnecessary. **Draft 3** reinstated the tracking
after an adversarial review (Codex, 2026-07-29) showed Draft 2 would disconnect
working clients. **Draft 4** replaced queue-head diffing with a direct activity stamp at the BLE
callback and notify sites, and fixed the idle definition to exactly "no command
received, no response sent, queues empty" (§2) — dropping `transferActive()` and
`epdRefreshInProgress`, the latter unobservable from `loop()` in any case.
**Draft 5 — this one — stamps on RX only** (§3e): every response is generated inside
a command handler, so a TX stamp is redundant except for the delayed post-refresh
ACK, where firing early is defensible and now documented. Checking that surfaced a
hole present in every earlier draft — a connected client that never enables its CCCD
leaves a response queued forever and was permanently immune to the kick, fixed by
the `&& ble.notifyReady()` qualifier in §3b.

The review findings that shaped Drafts 3–4, both verified against the code:

- *Draft 2's watchdog would kick clients that are actively working.* Its stamp
  condition only reads whether the queues are **still** non-empty at the end of a
  pass, and by then they never are — `serviceBleRx()` flushes TX after every
  command ([main.cpp:534](../src/main.cpp#L534)) and `serviceBleTx()` runs again at
  [:671](../src/main.cpp#L671), both before the watchdog's position at
  [:697](../src/main.cpp#L697). `bleRxQueuePending()`/`bleTxQueuePending()` are
  plain head≠tail tests ([command_queue.cpp:151-153](../src/command_queue.cpp#L151-L153),
  [:186-188](../src/command_queue.cpp#L186-L188)), so a command that arrives *and*
  drains inside one pass leaves no trace. A client sending a command every 30 s
  would have been disconnected 120 s after connecting. Head-change tracking is not
  an optimisation; it is the only thing that observes BLE traffic at all.
- *The 60 s refresh bound the simplification rested on does not exist.* This repo's
  own audit shows `waitforrefresh()`'s real wall-clock bound is **~126 s**, because
  `bbepIsBusy()` adds `delay(10) + delay(1)` inside each of the 6000 iterations
  ([FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md §B1](FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md),
  loop at [display_service.cpp:811-825](../src/display_service.cpp#L811-L825)).
  Draft 2 cited the `timeout = 60` argument and treated it as seconds of wall
  clock. §B2 of the same audit adds up to ~54 s of notify-semaphore waiting in a
  single drain pass, independently of any refresh.

Three further review findings are adopted as amendments (§3a, §3d, §6.1); one is
recorded as an accepted breakage (§3c). Findings that did **not** survive checking
are listed in §7.

---

## 1. Purpose

**An idle client holding the connection locks every other caller out of the
device.** That is what this change exists to fix; everything else in this section
is either evidence for it or a side benefit.

The device serves one BLE client at a time and stops advertising for the duration,
so a peer that connects and then does nothing is not merely wasting its own
session — it is denying the device to Home Assistant, to the CLI, and to any other
host, for as long as it chooses to stay attached. Nothing currently bounds that.
The three timers that look like they might, do not:

| Existing timer | What it actually does | Why it does not cover this |
|---|---|---|
| `TRANSFER_WATCHDOG_MS` = 15 min ([display_service.cpp:582](../src/display_service.cpp#L582), run from [main.cpp:697](../src/main.cpp#L697)) | Tears down a stuck DIRECT/PIPE/PARTIAL transfer | Frees transfer state only. The link survives, and it never fires for a client that connected and sent nothing. |
| `securityConfig.session_timeout_seconds` ([encryption.cpp:221-232](../src/encryption.cpp#L221-L232)) | Clears the encryption session | Clears crypto state only; no disconnect. Also `0` = never. |
| `OD_LAN_READ_TIMEOUT_S` = 30 s ([opendisplay_protocol.h:984](../include/opendisplay_protocol.h#L984), enforced at [wifi_service.cpp:949-956](../src/wifi_service.cpp#L949-L956)) | Drops an idle **LAN** client | LAN transport only. BLE has no equivalent. |

So a peer that connects and then goes quiet — a crashed host, a BlueZ connection
the supervising process abandoned, a scanner app left open on a phone, an
`animate.py` run someone Ctrl-Z'd — holds the device hostage until *its* side gives
up, which may be never.

### 1a. The lockout, mechanically

Advertising stops for the whole connection, on both targets, by design and now by
explicit guard: nRF at [ble_transport_nrf.cpp:267](../src/ble_transport_nrf.cpp#L267)
(one peripheral role slot from `Bluefruit.begin(1, 0)`, so `sd_ble_gap_adv_start()`
returns `NRF_ERROR_CONN_COUNT`), ESP32 at
[ble_transport_esp32.cpp:283](../src/ble_transport_esp32.cpp#L283). Both make
`setManufacturerData()` return false while connected.

A second caller therefore cannot even find the device, let alone connect: it is not
advertising, so a scan does not list it and a connect-by-address has nothing to
answer it. From the other host's point of view the tag is simply gone. For HA that
surfaces as delivery failures against a device that is powered, in range and
healthy — and its per-operation `async with OpenDisplayDevice(...)` pattern
(`delivery.py:319-340`) gives it no way to queue behind the squatter; each attempt
just fails.

Two further consequences of the same suppression, for the whole time the idle peer
stays attached: `updatemsdata()` cannot publish, so battery/temperature/button/touch
state in the advertisement is frozen at whatever it was when the session began, and
any advertising-interval boost armed by a button press cannot be restored (the
mechanism documented at length in
[PLAN_NRF_NO_ADV_WHILE_CONNECTED_2026-07-29.md](PLAN_NRF_NO_ADV_WHILE_CONNECTED_2026-07-29.md)).

### 1b. Side benefit: on ESP32 battery targets it also pins the tag awake

`pollActivity()` stamps `lastActivityMs` on **every pass while `connCount > 0`**
([main.cpp:366-368](../src/main.cpp#L366-L368)) — a live link is treated as
activity in itself, deliberately. `platformIdle()`'s deep-sleep branch requires a
quiet window since that stamp ([main.cpp:599-607](../src/main.cpp#L599-L607)), and
`workInFlight` includes `ble.isConnected()` ([main.cpp:742](../src/main.cpp#L742)),
so the loop takes the `delay(1)` arm forever.

An idle connected client therefore prevents deep sleep for as long as it stays
connected. On a battery tag that is the difference between microamps and
milliamps — the most expensive thing an unauthenticated peer can currently do to
the device without sending a byte.

This is a **secondary** motive, and it is worth being explicit about the ordering:
if the lockout in 1a did not exist, the power cost alone would not justify
disconnecting a client that might have a reason to be there. It is the exclusivity
that makes an idle session everyone else's problem. Consequences for the design:

- The kick must **restore advertising**, not merely free the radio — which it does
  via the existing `serviceBleAdvertisingRestart()` path (§3a), on the target where
  the stack does not re-arm by itself.
- The timeout is best read as *"how long may one caller lock everyone else out?"*,
  not *"how long may a client be lazy?"* — see §3c.
- A future alternative that fixes 1a without disconnecting anyone (advertising
  while connected, so a second caller can queue) would supersede the whole
  approach; §7.4 records why that is not available today.

---

## 2. What "idle" must mean

**Definition (decided 2026-07-29): a session is idle when no command has been
received, no response has been sent, and both queues are empty.** Nothing else.

Activity is observed **directly at the BLE receive callback**, not inferred from
per-pass queue-head snapshots (§3e explains why that is both simpler and strictly
more accurate):

| Clause | Observed at | Term |
|---|---|---|
| no command received | `bleRxQueuePush()` ([command_queue.cpp:50](../src/command_queue.cpp#L50)) — the single RX ingress, called only from the two stack callbacks ([nrf:155](../src/ble_transport_nrf.cpp#L155), [esp32:147](../src/ble_transport_esp32.cpp#L147)) | `millis() - bleLastRxMs()` |
| no response sent | **not stamped** — every response is generated inside a command handler, so it is already implied by the RX stamp. See §3e "why TX is not stamped" for the one exception and why it is accepted | — |
| queues empty | `bleRxQueuePending() \|\| (bleTxQueuePending() && ble.notifyReady())` | see §3e "the unsubscribed-client hole" |
| (not a session) | `ble.isConnected()` | as-is |

**Transfer and refresh state are deliberately NOT terms.**

- `epdRefreshInProgress` would be dead code. Both assignment pairs bracket
  straight-line blocking work with no return to `loop()`
  ([display_service.cpp:2452-2473](../src/display_service.cpp#L2452-L2473),
  [:3346-3356](../src/display_service.cpp#L3346-L3356)), so a loop()-level check
  can never observe it true. Its intended coverage — the blocking dead zone, now
  known to reach ~126 s — is already handled: the command that triggered the
  refresh advanced `rxHead` in the pass that blocks, and its response advanced
  `txHead`, so the pass that resumes stamps regardless of duration.
- `transferActive()` is dropped as a *policy* choice, and it changes behaviour
  versus earlier drafts: **a mid-transfer session whose client has gone silent for
  the full window is now kicked**, rather than waiting out the 15-minute transfer
  watchdog. That is the intent of an idle timer — a stalled transfer holds the
  radio, the panel rail and (on ESP32) wakefulness, and it cannot progress without
  the client. Teardown is not new code: the disconnect raises
  `s_disconnectCleanupPending`, and `serviceBleDisconnectCleanup()`
  ([main.cpp:388-423](../src/main.cpp#L388-L423)) already handles exactly this case
  for a peer-initiated drop — `cleanupDirectWriteState(true)`,
  `cleanupPartialWriteOnDisconnect()`, `resetPipeWriteState()`, behind the
  `ownerStillUp` guard. The 15-minute watchdog remains as the backstop for a
  transfer abandoned by a client that *stays* connected and chatty.

What this buys: the kick still cannot interrupt a command mid-dispatch (it runs
between passes, never inside a handler), and it can no longer be indefinitely
suppressed by stuck state — which is what `transferActive()` as a busy term would
have done, since a wedged `pipeState.active` would have pinned the stamp forever.

---

## 3. Design

### 3a. `BleTransport::disconnect()` — new seam method

Declared beside the other lifecycle calls in
[ble_transport.h](../src/ble_transport.h), documented as **asynchronous**: the
stack's disconnect callback fires later, so `takeDisconnectedEvent()` and the
existing `serviceBleDisconnectCleanup()` / `serviceBleAdvertisingRestart()` path
own teardown and the advertising re-arm. A caller must never pair this with
inline cleanup.

**Signature: `bool disconnect()`**, not `void` (review amendment). Both stacks
report failure and the earlier `void` seam would have swallowed it: NimBLE returns
false when `ble_gap_terminate()` rejects the request (`NimBLEServer.h:66`,
`NimBLEServer.cpp:315-332`) and Bluefruit propagates the SoftDevice status
(`bluefruit.cpp:625-635` → `BLEConnection.cpp:206`). Contract: **true** = request
accepted *or* already disconnected (nothing to do), **false** = the stack refused
and the link is still up. On false the watchdog retries in ~1 s instead of logging a
disconnect that never happened and then sitting out another full window.

- **nRF:** `Bluefruit.disconnect(s_connHandle)` — `bluefruit.h:171` →
  `BLEConnection::disconnect()` → `sd_ble_gap_disconnect(hdl,
  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)` (`BLEConnection.cpp:206`).
  Guard on the file-static `s_connHandle` rather than `Bluefruit.connHandle()`:
  the disconnect callback invalidates it ([ble_transport_nrf.cpp:134](../src/ble_transport_nrf.cpp#L134)),
  so a stale handle already means "gone". No advertising calls here —
  `restartOnDisconnect(true)` ([:210](../src/ble_transport_nrf.cpp#L210)) re-arms
  the radio, which is what `restartsAdvertisingOnDisconnect()` reports.
- **ESP32:** `s_server->disconnect(s_connHandle)` —
  `NimBLEServer.h:66`, default reason `BLE_ERR_REM_USER_CONN_TERM` (0x13),
  the same "remote user terminated" code the nRF path sends. Null-check
  `s_server` and `BLE_HS_CONN_HANDLE_NONE`.

There is already one raw call of this shape — `enterDFUMode()` at
[device_control.cpp:855-859](../src/device_control.cpp#L855-L859) reaches past the
seam into Bluefruit. Deliberately **not** converted here: it first sets
`restartOnDisconnect(false)` and then tears the SoftDevice down, so it wants
different semantics. Noted as a follow-up, not scope.

### 3b. `checkIdleSessionTimeout()` — the watchdog

A file-static in [main.cpp](../src/main.cpp), placed with the other
loop()-serviced BLE helpers and called from `loop()` immediately after
`checkTransferTimeouts()` ([main.cpp:697](../src/main.cpp#L697)). Shape:

```c
static void checkIdleSessionTimeout() {
    if (OD_BLE_IDLE_SESSION_TIMEOUT_MS == 0) return;   // compile-time disable
    if (!ble.isConnected()) return;
    // "queues empty" -- the second clause of the §2 definition, and NOT implied by
    // the stamp: a frame queued and never drained leaves a stale stamp with work
    // still outstanding. The notifyReady() qualifier is load-bearing -- see §3e.
    if (bleRxQueuePending() || (bleTxQueuePending() && ble.notifyReady())) return;

    const uint32_t idleMs = millis() - bleLastRxMs();
    if (idleMs < OD_BLE_IDLE_SESSION_TIMEOUT_MS) return;

    static uint32_t nextAttemptMs = 0;
    if (nextAttemptMs != 0 && (int32_t)(millis() - nextAttemptMs) < 0) return;
    od_log_info("Idle BLE session %u ms (limit %u ms) - disconnecting client",
                (unsigned)idleMs, (unsigned)OD_BLE_IDLE_SESSION_TIMEOUT_MS);
    if (!ble.disconnect()) {
        od_log_warn("Idle-session disconnect rejected by the stack - retrying in 1 s");
        nextAttemptMs = millis() + 1000u;
        return;
    }
    nextAttemptMs = 0;
    bleMarkRxActivity();   // arm a fresh window; the disconnect event lands shortly
}
```

**Why the stamp, and not a per-pass queue-head comparison** (which is what Draft 3
used — see §3e for the full argument): by the time this runs at
[main.cpp:697](../src/main.cpp#L697), `serviceBleRx()` has drained every queued
command and flushed TX after each one ([:534](../src/main.cpp#L534)), and
`serviceBleTx()` has run again ([:671](../src/main.cpp#L671)). Both `*Pending()`
predicates are head≠tail tests ([command_queue.cpp:151-153](../src/command_queue.cpp#L151-L153),
[:186-188](../src/command_queue.cpp#L186-L188)), so on a pass that just serviced a
burst they read false. Anything that samples *state at a pass boundary* is therefore
blind to ordinary traffic; only observing the arrival and the notify themselves is
reliable. Draft 2 sampled residual queue depth and would have kicked working
clients; Draft 3's head-diffing fixed that but still reconstructed the event from
snapshots. This observes the event.

Restamping on a successful kick gives the one-kick-per-window property, since
`Bluefruit.disconnect()` / `NimBLEServer::disconnect()` only *request* termination
and `isConnected()` can stay true for several passes.

The stamp also makes the blocking dead zone a non-issue now that
its real bound is ~126 s rather than 60 s
([FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md §B1](FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md)):
the command that *triggered* the refresh was stamped on arrival, and its response
stamped again on notify, so the pass that resumes reads a fresh stamp regardless of
how long the dead zone lasted. The 120 s default therefore does not have to exceed
the blocking bound at all — see §3c — and §2 drops `epdRefreshInProgress` as
unobservable.

Why that position in the pass: RX/TX have already been serviced and the
disconnect/advertising flags already consumed, so the queue tests read post-service
state and the kick cannot pre-empt a frame that arrived this pass. It is also
**before** `platformIdle()`, so on ESP32 the pass that kicks is followed by a pass
whose `pollActivity()` sees the connection-count edge, re-arms the full idle hold,
and then sleeps normally — no special-casing needed.

**Not** added to `idleDelay()`'s early-return set: nothing here changes
asynchronously off the loop task, and the nRF park is 1000 ms
([main.h:335](../src/main.h#L335)), so a kick lands at most one park late.

### 3c. The tunable

`OD_BLE_IDLE_SESSION_TIMEOUT_MS`, `#ifndef`-guarded in
[main.h](../src/main.h) beside `OD_NRF_IDLE_WAIT_MS`, overridable per environment
from `platformio.ini`, `0` = disabled.

**Default: 120000 (120 s).** Read it as the **lockout budget** — the longest a
single caller may keep every other host from reaching the device (§1) — bounded
below by what a legitimate client can need. With §3e's activity stamp the value no
longer has to cover the blocking-refresh dead zone at all (the triggering command
stamped on arrival, before the loop blocked), so the floor rests purely on host
behaviour:

- **It must clear every host-side inter-command timeout, and now that is the whole
  safety argument** — with transfer and refresh state out of the busy set (§2),
  nothing else holds the stamp during device-side work. `TIMEOUT_ACK` 5 s,
  `TIMEOUT_FIRST_CHUNK` 10 s (`py-opendisplay device.py:447-449`),
  `TIMEOUT_PIPE_START` 30 s (`commands.py:92`), and the 90 s refresh/END-ack pair
  (`device.py:458-460`) all sit inside 120 s. The 90 s pair is the binding one: a
  host waiting out a slow refresh sends nothing, and only the queue-head advance
  from its own END command keeps it safe. 120 s over a 90 s host timeout is 33%
  margin — thin enough that this number should not be lowered without re-checking
  those constants, and it is why `TIMEOUT_REFRESH` is worth watching if
  `waitforrefresh()`'s real ~126 s bound is ever actually hit (§7.1).
- **The ceiling is the one worth arguing about, given the purpose.** 120 s is the
  worst case a blocked second caller waits, and for HA that is one or two failed
  `drawcustom` deliveries before the device reappears — recoverable, since the next
  scheduled update succeeds. If field reports show real callers being starved, the
  fix is to *lower* this (60 s is safe with §3e's stamp in place; the earlier
  objection to 60 s was only ever about the dead zone), not to redesign. That
  asymmetry — safe to lower, needs care to raise — is why the conservative value
  ships as the default.
- 4× `OD_LAN_READ_TIMEOUT_S` (30 s). More than the other transport, deliberately:
  BLE reconnect costs more — advertising
  re-acquisition at a 160–1000 ms interval
  ([ble_transport_nrf.cpp:48-49](../src/ble_transport_nrf.cpp#L48-L49)) plus a
  fresh connection, versus a TCP handshake.
- The production consumers connect per-operation: the HA integration wraps each
  delivery in `async with OpenDisplayDevice(...)`
  (`custom_components/opendisplay/delivery.py:319-340`, `services.py:467-479`), the
  `opendisplay` CLI has no long-lived subcommand, and `tools/od-device-cli.py` is
  one-shot argparse with no REPL.
- **Correction (review):** Draft 2 generalised that into "no consumer holds a BLE
  link idle", which is false. `py-opendisplay examples/animate.py` opens one
  `OpenDisplayDevice` context and loops indefinitely
  (`examples/animate.py:156-215`), sleeping `--interval` between frames with no
  upper bound on that value (`:257-263`, default 1000 ms), and it pre-processes
  every frame *inside* the open connection (`:156-168`). See §3c-note.

**§3c-note — accepted breakage.** `animate.py --interval` above 120000, or a
pre-processing pass over a large frame set that takes longer than 120 s, will now
be disconnected mid-run, and the surrounding `async with` does not reconnect. This
is accepted rather than designed around:

- At the default 1 s interval, and at any interval a person would use for an
  animation, the head tracking in §3b stamps on every frame — the script is
  unaffected.
- The pre-processing window is the sharper edge, since it happens after connect
  with zero BLE traffic. It is bounded by host CPU, not by the device.
- The alternative — an application keepalive or lease opcode — is a protocol
  change, which is out of proportion to one example script.

Consequence for scope: the earlier "no cross-repo work" claim is now **"no
cross-repo work required to ship; one follow-up recommended"** — move
`animate.py`'s `prepare_image()` loop above the `async with`, and/or have it
reconnect on `BleakError`. Filed against `py-opendisplay`, not a blocker here.

**Alternative considered and rejected for now:** driving it from a provisioned
config field (e.g. one of `PowerOption.reserved[4]`,
[opendisplay_structs.h:505](../include/opendisplay_structs.h#L505)). That is the
right long-term home if the value ever needs to differ per deployment, but it
costs a canonical `opendisplay-protocol` edit + `--push` to four firmware repos +
a `tools/od-device-cli.py` `BLOCKS` update, for a knob no field report has asked
to vary. Deferred, and cheap to add later precisely because the macro is the only
reader.

### 3d. Chunked-config cleanup on disconnect (review amendment)

A chunked config write keeps state across BLE commands in `chunkedWriteState`
([config_parser.h:33-47](../src/config_parser.h#L33-L47)), set active by
`CMD_CONFIG_WRITE` ([communication.cpp:405-438](../src/communication.cpp#L405-L438))
and cleared only by the final chunk or an error
([:462-501](../src/communication.cpp#L462-L501)). `serviceBleDisconnectCleanup()`
does **not** reset it ([main.cpp:388-423](../src/main.cpp#L388-L423)), and
`handleWriteConfigChunk()`'s entry gate is `chunkedWriteState.active` alone with no
binding to the session that opened it ([:462-467](../src/communication.cpp#L462-L467)).

So an abandoned config write leaves a live buffer that the *next* client can append
to. **This is pre-existing** — it is equally reachable from a peer-initiated
disconnect or a link-loss today, so the watchdog does not create it. But the
watchdog does create a new, automatic way to reach it, so this plan fixes it:

```c
// in serviceBleDisconnectCleanup(), beside resetPipeWriteState()
chunkedWriteState.active = false;
chunkedWriteState.receivedSize = 0;
chunkedWriteState.receivedChunks = 0;
```

Deliberately **not** added to §3b's busy set, on review's own reasoning: an
abandoned chunked write would then suppress the watchdog forever, which is the
opposite of what it is for. The state is reset *by* the disconnect, not protected
*from* it. It sits behind the same `ownerStillUp` guard as the transfer cleanups,
so a LAN-owned session is unaffected.

### 3e. The RX activity stamp — observing the receive callback directly

One choke point, in `command_queue.cpp`, already shared by both targets:

```c
// command_queue.cpp
static volatile uint32_t s_lastRxMs = 0;

void bleMarkRxActivity(void) { s_lastRxMs = millis(); }
uint32_t bleLastRxMs(void) { return s_lastRxMs; }
```

- **RX:** `bleMarkRxActivity()` at the top of `bleRxQueuePush()`
  ([:50](../src/command_queue.cpp#L50)), which both stack callbacks call and nothing
  else does ([nrf:155](../src/ble_transport_nrf.cpp#L155),
  [esp32:147](../src/ble_transport_esp32.cpp#L147)).
- **Connect:** in `serviceBleEvents()`'s existing `takeConnectedEvent()` branch
  ([main.cpp:462-468](../src/main.cpp#L462-L468)), so the first window is measured
  from the connection rather than from a previous session's last frame.

**Why TX is not stamped.** Every response this firmware emits is produced inside a
command handler — `sendResponse()` has no caller outside the dispatch path, in
`communication.cpp`, `device_control.cpp`, `buzzer_control.cpp` or
`display_service.cpp`; there are no device-initiated notifications (button and touch
state reach the host through the advertisement, not GATT). So a TX stamp would
almost always be re-stamping microseconds after the RX stamp that caused it, and
"no response sent" is implied by "no command received".

The exception is a **delayed** response: the post-refresh ACK/NACK at
[display_service.cpp:2489](../src/display_service.cpp#L2489) and
[:2494](../src/display_service.cpp#L2494) is sent when the refresh finishes, which
the ~126 s `waitforrefresh()` bound
([FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md §B1](FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md))
allows to be long after the END command that triggered it. With RX-only stamping, a
refresh that outlasts the 120 s window means the client is kicked on the pass right
after it receives its ACK.

**That is accepted, and on the stated purpose (§1) it is arguably correct.** The
host's own `TIMEOUT_REFRESH` is 90 s (`py-opendisplay device.py:460`), so a device
that took over 120 s has already blown the host's deadline — that operation is lost
either way, and the client is by then a stale holder of a device another caller is
locked out of. It reconnects if it is still interested. Normal refreshes (~16 s on
Spectra 6-colour) are nowhere near the bound; what changes is only that the window
is measured from the END command rather than from its ACK, i.e. the kick can come up
to one refresh-duration earlier than it otherwise would.

**The unsubscribed-client hole** (found while checking this, and present in every
draft before it). `serviceBleTx()` holds responses queued when a client is connected
but has not enabled its CCCD ([command_queue.cpp:206-207](../src/command_queue.cpp#L206-L207)),
and `notifyReady()` is `connected && notifyEnabled()`
([ble_transport_nrf.cpp:241-243](../src/ble_transport_nrf.cpp#L241-L243),
[esp32:261](../src/ble_transport_esp32.cpp#L261)). So a client that writes one
command and never subscribes leaves `bleTxQueuePending()` true **forever** — an
unqualified "queues empty" clause would make exactly the squatter this feature
targets permanently immune to it. Hence the `&& ble.notifyReady()` qualifier in
§3b: a queued response counts as work in flight only when it can actually drain.

**Why this beats the queue-head snapshot it replaces.** Three reasons, in order of
weight:

1. **It observes the event, not a residue of it.** No dependency on where in the
   loop pass the check sits, or on what has already drained. That coupling is
   precisely what made Draft 2 wrong, and Draft 3's head-diffing only worked around.
2. **It catches traffic the head never records.** `bleRxQueuePush()` rejects empty,
   oversized and ring-full frames — it owns every drop reason
   ([nrf:151-155](../src/ble_transport_nrf.cpp#L151-L155)) — and a rejected frame
   advances no head. Stamping *before* validation means a client hammering a full
   ring reads as busy, which it is; the head comparison would have read it as idle
   and kicked a client that was mid-burst.
3. **No aliasing and no shadow state.** The RX head is modulo `COMMAND_QUEUE_SIZE`
   (34), so a full wrap inside one pass reads as "unchanged" — `pollActivity()`
   documents that as an accepted risk
   ([main.cpp:337-341](../src/main.cpp#L337-L341)). A monotonic timestamp has no
   such failure mode, and drops the `prev*Head` statics.

**Threading.** The store runs on the stack callback task (nRF SoftDevice event task,
NimBLE host task) — the only writer, now that TX is not stamped — and the load on
the loop task. `volatile uint32_t`, naturally
aligned, is a single load/store on both Cortex-M4 and the Xtensa/RISC-V ESP32 cores,
so there is no tearing and no lock is needed — and a plain flag store is exactly
what the copy-and-flag callback contract permits
([ble_transport_nrf.cpp:118-124](../src/ble_transport_nrf.cpp#L118-L124)). `millis()`
is callable from both contexts: each is a FreeRTOS task, not an ISR. The RX stamp
lands *before* `imageWriteLogQuietFrame()` and the rest of the push body, so it
costs one store on the hot path of a pipe burst.

**Wrap:** `millis() - stamp` is unsigned subtraction, correct across the 49.7-day
rollover, matching every other `millis()` deadline in the firmware. The
`nextAttemptMs` retry uses a signed-difference compare for the same reason.

---

## 4. Commits and total diff

One code commit, plus this document.

1. **`feat(ble): disconnect idle BLE sessions after OD_BLE_IDLE_SESSION_TIMEOUT_MS`**
2. **`docs(ble): idle-session watchdog plan`** — this file.

Seven files:

| File | Change | ~lines |
|---|---|---|
| [ble_transport.h](../src/ble_transport.h) | `bool disconnect();` + its contract comment | 8 |
| [ble_transport_nrf.cpp](../src/ble_transport_nrf.cpp) | `Bluefruit.disconnect(s_connHandle)` + handle guard, returns status | 9 |
| [ble_transport_esp32.cpp](../src/ble_transport_esp32.cpp) | `s_server->disconnect(s_connHandle)` + null/handle guards, returns status | 9 |
| [command_queue.h](../src/command_queue.h) | `bleMarkRxActivity()` / `bleLastRxMs()` declarations | 4 |
| [command_queue.cpp](../src/command_queue.cpp) | the stamp + one call site in `bleRxQueuePush()` (§3e) | 7 |
| [main.h](../src/main.h) | `OD_BLE_IDLE_SESSION_TIMEOUT_MS` `#ifndef` block | 6 |
| [main.cpp](../src/main.cpp) | `checkIdleSessionTimeout()` (§3b) + call site + connect-branch stamp + §3d reset | 26 |

~69 inserted lines. Every hunk is an insertion except two one-line additions into
existing bodies (`bleRxQueuePush`, the connect branch) and the three-line §3d reset,
so nothing that avoids the new watchdog changes behaviour. `serviceBleTx()` is now
untouched — one fewer hot-path edit than Draft 4, since RX-only stamping removes the
per-notify store during a pipe burst.

Draft 2 claimed ~46 lines by dropping activity tracking altogether; that saving was
not value, it was the defect. Draft 3 restored it as ~6 lines of queue-head diffing
in `main.cpp`; Draft 4 moves it to ~12 lines split across `command_queue.*`, buying
the three correctness properties in §3e for two files and ~8 lines.

**Why the three transport files are not avoidable.** Calling
`Bluefruit.disconnect()` / `s_server->disconnect()` straight from `main.cpp` would
save two files, but `main.cpp` names no stack type today: keeping Bluefruit and
NimBLE types out of application code is the stated contract of
[ble_transport.h](../src/ble_transport.h) and of
[PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md](PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md).
It would also need a `#ifdef TARGET_NRF` / `#else` pair at the call site, so the
"saving" is ~4 lines of seam traded for ~6 lines of target guard plus a broken
invariant.

---

## 5. Test plan

Build: `pio run` (all eleven CI environments) — the change touches both transport
implementations and shared `main.cpp`, so a single-env build proves nothing.

Bench, nRF (`nrf52840custom-debug`, RTT log):

1. **Kick fires.** Connect with nRF Connect / `bleak`, subscribe, send nothing.
   Expect the `Idle BLE session … disconnecting client` line at ~120 s, then
   `=== BLE CLIENT DISCONNECTED (nRF) ===` with reason 0x16 (local host
   terminated) on the peer side, `Disconnect reason:` logged by
   `serviceBleEvents()`, and advertising visible again from a scanner within a
   second or two.
2. **MSD unfreezes.** Confirm the post-disconnect `s_msdUpdatePending` republish
   ([main.cpp:510](../src/main.cpp#L510)) lands — the advertisement's loop counter
   advances again after the kick.
3. **A stalled upload IS kicked** (behaviour change from earlier drafts — §2).
   Start a pipe upload, kill the host process mid-stream so the link stays up with
   no further frames. Expect the kick ~120 s after the last frame, then the normal
   deferred teardown: `resetPipeWriteState()` /
   `cleanupDirectWriteState(true)` / `cleanupPartialWriteOnDisconnect()` via
   `serviceBleDisconnectCleanup()`, and the panel rail released rather than held to
   the 15-minute watchdog. Check the log shows cleanup *after* the disconnect
   event, not inside the kick.
   Then the inverse: a **healthy** upload of a large image over a slow link must
   not be kicked — every accepted frame advances `rxHead` and every ACK advances
   `txHead`, so a transfer that is progressing at all is never idle.
4. **No kick around a refresh.** Push a full-frame image to a Spectra panel
   (~16 s blocking refresh) with the host idle before and after. Expect no kick
   during the refresh, and the next kick **120 s after the END command arrived** —
   i.e. ~104 s after the refresh returns, not 120 s after it. That is the documented
   consequence of RX-only stamping (§3e), not a defect; what would be a defect is a
   kick *during* the refresh or before the ACK reaches the host.
5. **The regression Draft 2 would have shipped** — the single most important test.
   Connect and send one cheap command (e.g. `CMD_FIRMWARE_VERSION`) every 30 s for
   10 minutes, doing nothing else. Expect **zero** kicks. Draft 2's watchdog kicks
   at ~120 s here, because each command drains inside its pass and leaves both
   `*Pending()` predicates false.
6. **One kick per session**, not one per pass: exactly one log line per idle
   session. Also check the `disconnect rejected by the stack` path is silent in
   normal operation.
11. **§3e, the unsubscribed squatter — the case that was immune in every earlier
   draft.** Connect *without* enabling notifications, write one command (so a
   response is queued and cannot drain), then idle. Expect the kick at ~120 s. With
   an unqualified `bleTxQueuePending()` term this client is never kicked at all,
   which is the precise opposite of the purpose in §1.
10. **§3e, the case snapshots missed.** Saturate the RX ring (a burst deeper than
   `COMMAND_QUEUE_SIZE`, or oversized frames) so `bleRxQueuePush()` starts
   rejecting, and hold that for over 120 s. Expect **no** kick: rejected frames
   still stamp. Draft 3's head comparison would have read this as idle — the heads
   do not advance on a drop — and disconnected a client mid-burst.

Bench, ESP32 battery target (`power_mode == 1`, `deep_sleep_time_seconds > 0`):

7. **Deep sleep resumes, on the full timeline.** Connect, idle. Expect the kick at
   ~120 s, then sleep after the *post-disconnect* quiet hold — the disconnect edge
   re-stamps `lastActivityMs` via the connection-count change
   ([main.cpp:342-368](../src/main.cpp#L342-L368)), so sleep is
   `sleep_timeout_ms` (or the 10 s `DEFAULT_IDLE_HOLD_MS`,
   [main.h:308](../src/main.h#L308)) later, and only once any `min_wake_time_seconds`
   hold has expired ([main.cpp:308-320](../src/main.cpp#L308-L320)). Verify with a
   non-default `sleep_timeout_ms` too, and record total awake time as
   *120 s + hold*, not 120 s — that is the number the power budget needs.

Negative tests:

8. Build one env with `-DOD_BLE_IDLE_SESSION_TIMEOUT_MS=0`; confirm the watchdog
   compiles out to a no-op and an idle session is never kicked.
9. **§3d:** start a chunked config write, send chunk 1 of 3, let the kick fire.
   Reconnect and send a chunk — expect a NACK (state was reset), not silent
   appending to the departed client's buffer.

---

## 6. Residuals and known gaps (not fixed here)

1. **The encryption session survives the kick — deliberately deferred, with the
   reason stated.** Nothing clears it on BLE disconnect:
   `clearEncryptionSession()` is called only on config reload
   ([communication.cpp:66](../src/communication.cpp#L66)), re-auth, integrity
   failure, and its own age timeout. That contradicts the wire documentation for
   `session_timeout_seconds` — *"0 = no timeout (persists until disconnect)"*
   ([opendisplay_structs.h:916](../include/opendisplay_structs.h#L916)) — and the
   host clears its side on any disconnect (`py-opendisplay device.py:688-740`), so
   after a kick the two disagree about session lifetime until the next
   authentication replaces the state.

   Review argued this belongs in this change because the watchdog manufactures
   disconnects. It is still deferred, for a reason the review itself identified:
   `encryptionSession` is a **single global shared with the LAN transport**
   (`wifi_service.cpp:798`, `:874` clear it), so clearing it on a BLE disconnect
   needs a transport-ownership decision — the same class of decision that
   `serviceBleDisconnectCleanup()`'s `ownerStillUp` guard exists to make for
   transfers. That is a security-relevant change with its own test matrix and it
   should not ride along inside a power/discoverability fix. It is a **named
   follow-up, not an unknown**: clear the session when the disconnect event is
   consumed *if* no LAN session is live.
2. **Reason code asymmetry.** nRF sends 0x13 via
   `BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION`, ESP32 sends NimBLE's default
   `BLE_ERR_REM_USER_CONN_TERM` (also 0x13). Same code, reached two different
   ways; if a future NimBLE release changes its default this diverges silently.
3. **The event-flag coalescing weakness** documented at
   [ble_transport.h:74-77](../src/ble_transport.h#L74-L77) applies to the
   disconnect this raises exactly as it does to a peer-initiated one. Neither
   worsened nor fixed.
4. **No host-visible warning before the kick.** The client learns only from the
   disconnect. A pre-kick notification would need a protocol opcode; out of scope.
5. **`enterDFUMode()` still bypasses the transport seam**
   ([device_control.cpp:857](../src/device_control.cpp#L857)) — see 3a.

---

## 7. Review findings not adopted, and why

From the adversarial review (Codex, 2026-07-29). Recorded so they are not
re-litigated, and so a reader can see what was checked rather than assumed.

1. **"Fix `waitforrefresh()` to use a `millis()` deadline, and reconcile
   py-opendisplay's 90 s refresh timeout with the real ~126 s bound."** Both are
   real defects and both are already tracked elsewhere —
   [FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md §B1](FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md)
   notes an unmerged `waitForPanelIdle()` on `debug/freeze-fix-phase2`. Neither is a
   prerequisite here: with §3b's head tracking the watchdog is insensitive to how
   long the dead zone is, so this change neither depends on nor worsens them.
2. **"Verdict: redesign."** Downgraded to *implement with amendments*. Of the seven
   findings, one was fatal to Draft 2's watchdog (adopted, §3b), one corrected a
   load-bearing number (adopted, §3b/§3c), three are small additive amendments
   (§3a `bool`, §3d chunked config, §5 test 7), one is an accepted breakage in an
   example script (§3c-note), and one — encryption ownership — is a named follow-up
   with a stated reason (§6.1). The transport seam, the watchdog's position in the
   pass, and the busy set as amended all survived. That is a revision, not a
   redesign.
3. **Claims the review checked and *confirmed*, so they stay as written:** calling
   either stack's disconnect from the loop task is safe (link tuning already runs
   there, [main.cpp:461-468](../src/main.cpp#L461-L468), and both disconnect
   callbacks are copy-and-flag only); MSD publication is suppressed while connected
   on both targets; ESP32 needs the application to restart advertising and nRF does
   not; an idle connected client does prevent ESP32 deep sleep indefinitely; every
   `waitforrefresh()` call site passes `60` (the error was reading that as seconds);
   nothing clears the encryption session on BLE disconnect.
4. **"Is there a cheaper mechanism — advertise while connected, or lean on the GAP
   supervision timeout, instead of disconnecting?"** Both were considered and
   neither is available:
   - *Advertising while connected* splits into two things. Keeping the
     advertisement **fresh** (non-connectable) would unfreeze the MSD but does
     nothing for the lockout in §1a — a second caller still cannot connect, which is
     the actual purpose. Letting a second caller **connect** needs more than a
     radio-config change: the transfer and session state is global and
     single-client throughout — `pipeState` / `pipeReorder`
     ([display_service.cpp:576-577](../src/display_service.cpp#L576-L577)),
     `partialCtx`, `directWriteActive`, `chunkedWriteState`
     ([config_parser.h:33-47](../src/config_parser.h#L33-L47)), one
     `encryptionSession`, and a single `s_connHandle` per transport. Multi-link
     support is a re-architecture, not an alternative to a 20-line watchdog.
   - *Supervision timeout* is the wrong instrument: it detects a link that has
     **failed**, not one that is alive and idle. A healthy peer answers every
     connection event, so the supervision timer never expires no matter how long it
     squats. Nothing in this firmware even sets connection parameters.
5. **Non-issues it ruled out for me:** NFC has no dispatcher entry in this firmware
   (`communication.cpp:632-636`, falls through as unknown), and buzzer playback is
   capped at 30 s (`buzzer_control.cpp:15-17`) — so neither can span the window with
   the busy set false. Draft 2 had listed both as open questions.
