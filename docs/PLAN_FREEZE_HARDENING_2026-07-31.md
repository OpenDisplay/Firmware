# Freeze-Hardening the OpenDisplay Firmware — 2026-07-31

A self-contained four-phase plan for the BLE e-paper firmware, written from the code
as it stands on `fix/nonce-replay-window` (last code commit `9ca1d8f`, rebased onto the
squashed `main` at `aae5bdf`; every commit after it on this branch is docs-only, so the
citations below still describe the tree).

Every claim below was verified by direct reading of the current tree and is cited to
`file:line` so a reviewer can re-check rather than trust. The loop/BLE unification
(PR `#132`) and the nonce rewrite (this branch) both landed recently and changed the
shape of several subsystems, so nothing here is taken on inherited assumption — the
ground truth is re-established from scratch below.

## Conformance with `CONNECTION_POLICY.md`

[`CONNECTION_POLICY.md`](CONNECTION_POLICY.md) is the **normative** ruleset for
connection behaviour: it defines what must be true. This plan **schedules** it — when
each rule is built, on which mechanism, and how it is verified. Where the two disagree
the policy wins, and this revision exists to remove the disagreements: the policy's
"supersedes" list is discharged below rather than left as a standing conflict.

| Policy rule | Lands in | What this revision changed |
|---|---|---|
| **R1** one admitted client, globally | Phase 3 | unchanged in substance |
| **R2** identity is `(transport, handle, epoch)` | Phase 2 | the owner token gains an **epoch**, allocated in the connect callback for *every* instance; it was a `(transport, handle)` pair |
| **R3** a contender is refused, and refusal is inert | Phase 2 (mechanism) + Phase 3 (policy) | adds the **per-handle instance table**, **identity-bearing disconnect** events, per-link **subscribe** filtering and **handle-targeted notify** — the plan previously had only handle-bearing *connect* events and write filtering |
| **R3a** a firmware-initiated drop waits for link-down | Phase 2 | the seam **waits synchronously** for the link to go down before the abort releases; the plan previously released at request time |
| **R4** idle timeout, ungated by transfer state | Phase 2 (clock) + Phase 3 (policy) | the clock stamps a **recognised command from the current owner**, not any queued frame, and is re-stamped by a single `endRefresh()` helper |
| **R5** refresh watchdog | **out of scope**, named | recorded under [residual risk](#residual-risk-honest-list); the FastEPD refresh path has no bound at all, which the plan did not previously say |
| **R6** abort on every non-refused disconnect | Phase 2 | the invocation set gains **deep sleep** (R7e row 3) and states R6's exceptions |
| **R7d** within-pass ordering | Phase 3 | new: the loop order is normative, not incidental |

Two rules cost nothing to schedule because they are already satisfied: R6's buzzer/LED
carve-out and its WARM-panel survival are the design `abortToKnownState` already had,
and R4's no-transfer-gate was reconciled in the previous revision.

> **Revision 2026-07-31b (external review).** An adversarial review of this plan and
> the policy found three defects that would have surfaced mid-implementation, corrected
> in both documents: (1) the owner token was loop-task-only while callback-side write
> filtering needed to read it on the host task — the token is now a single atomic word
> claimed by CAS at the earliest transport hook, and the epoch narrows to 16 bits so
> the word stays lock-free; (2) `abortToKnownState` step 10 dropped a BLE handle
> unconditionally, which is wrong for a LAN owner (the transfer watchdog is
> origin-agnostic) — the drop now dispatches on the owner's transport; (3)
> `bleDropAndWait()` polled the aggregate `connectedCount()`, which never reaches zero
> while a refused contender is attached — the predicate is now the owner's
> instance-table entry, ticked on a plain bounded delay rather than `idleDelay()`,
> whose event early-out degrades into a busy spin mid-teardown.

> **Revision 2026-07-31c (same review, second batch).** Three further findings shared
> one root cause — queued frames are anonymous — and are fixed together by
> CONNECTION_POLICY R3 requirement 6: every queued frame now carries its writer's
> packed instance-identity word, stamped in `onWrite` from the same owner-word load
> the write filter already does, and the dispatcher executes a frame only if its tag
> still equals the owner word. That one mechanism closes the teardown window (the
> departing owner writing during its own abort), dissolves the boundary-lost-to-
> handle-reuse hazard, and makes the activity clock's "from the owner" test true
> instance identity instead of transport-only. It **retires the RX-boundary
> mechanism** — `s_rxBoundaryAtDisconnect`, `takeDisconnectedEvent`'s boundary
> out-param, and `bleRxQueueDiscardTo` all go — and the abort's step 9 now resets
> both rings, not just TX.

> **Revision 2026-07-31d (closing the review).** The remaining findings, corrected in
> both documents: 7a gains rows 9–10 and the admission-decided-once rule (racing
> arrivals are serialized by the claim CAS; a lingering refused contender never
> inherits a freed slot); the deep-sleep abort's rationale is rewritten — neither RAM
> nor hardware state survives in a way that needs it, so the abort stands on teardown
> uniformity at a mid-session exit; the auth-abuse `FE` is best-effort (stack
> acceptance plus a bounded negotiated-interval dwell, not guaranteed receipt); and
> three miscited lines are fixed
> (`sessionOrigin` stamps, dispatcher rejection sites, the Bluefruit `disconnect()`
> signature).

## Phase map

| # | Phase | Depends on | State today |
|---|---|---|---|
| 1 | Nonce / replay correctness | — | **Shipped** on this branch (`e2e95cd`…`19335e6`) |
| 2 | BLE-HAL foundation: link-drop seam (with the R3a wait), instance identity + owner token, the instance table, callback-side filtering, frame identity tags, activity clock, abort-to-known-state — **plus contender refusal, moved here from Phase 3** | — | **Implemented** on `feat/phase2-ble-hal-foundation` (`dbec776`, `bb7ad1d`); landed, not closed |
| 3 | Idle drop + the remaining exclusivity policy | Phase 2 | **Implemented** on `feat/phase3-exclusivity-idle-drop`; landed, not closed |
| 4 | Auth-abuse disconnect | Phase 2, Phase 3 | Prototype exists off-branch, not here |

> **Refusal moved from Phase 3 to Phase 2 during implementation.** Phase 2 is not
> safely shippable without it, so the split as originally drawn was wrong rather
> than merely inconvenient. Admission is decided once per instance and never
> revisited (7a row 10), so a client that reconnects into a still-held slot — the
> ordinary case when `loop()` was blocked in a refresh — becomes a permanent
> contender; on nRF it occupies the only peripheral link and the device stops
> accepting anyone until that client happens to leave. The two alternatives were
> both worse and both were tried: releasing the token in the disconnect callback
> admits a new owner while the departed session's transfer, crypto and TX ring are
> still live, and skipping the refusal scan while the slot is unowned leaves a
> decided loser attached forever.
>
> What stayed in Phase 3: the idle timeout and every other path that reclaims a
> *held* slot. Refusal only makes the "decided once" rule true; it never evicts.
> LAN accept also became refuse-not-evict here for the same reason (its eviction
> path could strand the token until reboot).

**Phase order note.** Phase 2 is the foundational layer: every transport/HAL
*mechanism* the later phases stand on — the portable `disconnect()`, connection instance
identity and the owner token, the instance table with callback-side write/subscribe/notify
filtering, the activity clock, and the shared abort routine. Phase 3 is **policy** on top of those
mechanisms (when to refuse and when to drop); Phase 4 is the auth-abuse policy. Phase 2
lands first because 3 and 4 both call into it — building the foundation last (as an
earlier draft did, with exclusivity as Phase 2) created a dependency cycle, since the
idle drop calls the abort routine.

**Two cross-phase deliverables** thread through Phases 2–4 and are specified once
here rather than repeated:

- **Threshold discipline — at the point of use, not in a new header.** Every tunable
  this plan introduces (the R3a link-down wait bound, the idle-drop timeout, the
  auth-abuse count and its flush deadline) is a compile-time `#ifndef`-guarded
  `#define` **in the file that consumes it**, each carrying a comment naming the
  *client behaviour it assumes*. No threshold is a wire/config field, so none touches
  the hard constraint.

  The four differ in how load-bearing they are, and the comments should say so.
  `OD_BLE_IDLE_TIMEOUT_MS` carries the most weight (it is the sole reclaim path, and
  under R4 it can end a live upload); `OD_BLE_LINK_DOWN_WAIT_MS` carries the least —
  per CONNECTION_POLICY R3a its expiry is not a failure needing recovery, just an
  early exit into an abort that runs regardless.

  This follows the repo's existing convention rather than inventing one. The model is
  [wifi_service.cpp:470-472](../src/wifi_service.cpp):

  ```c
  #ifndef OD_LAN_ROAM_RSSI_THRESHOLD
  #define OD_LAN_ROAM_RSSI_THRESHOLD (-75)   /* dBm; valid range -100..10 */
  #endif
  ```

  and likewise `OD_TINFL_DICT_SIZE`, `OD_CHARGER_FLAG_*`, `OD_LOG_LEVEL`
  ([od_log.h:16-18](../src/od_log.h)); `TRANSFER_WATCHDOG_MS` is a plain `static const`
  in [display_service.cpp:582](../src/display_service.cpp). There is no central
  tunables header in this repo and this plan does not add one.

  *An earlier draft specified a `src/session_policy.h` collecting all four.* It was
  cut. It would have been the only file of its kind, and it groups by **type**
  ("these are all thresholds") rather than by dependency: the four are consumed by two
  unrelated subsystems — the idle drop by the loop-side policy helpers, auth-abuse by
  `communication.cpp` — so the header buys a new include edge shared by two callers
  that need nothing else from each other. The goal behind it was that the assumptions
  be legible rather than bare numbers; that is served by the mandatory
  client-behaviour comment, which reads *better* next to the code that acts on it, and
  by the client-side CI assertions below. If a shared home is ever genuinely needed,
  `structs.h` is the existing common hub.

  **They do not go in the BLE transport headers either.** These are policy, and
  Phase 2 is mechanisms-only by construction. This is settled precedent here, in the
  same direction: [ble_transport.h:89-93](../src/ble_transport.h) records that the
  loop-serviced deferred-work flags were *moved out* of the transport because they
  "encode application policy, not link state, so exporting them from the transport
  seam was backwards." The same reasoning puts the activity clock beside the owner
  token rather than in the transport; deciding how long is too long belongs to the
  loop-side policy code that Phase 3 adds.
- **A companion HIL test per phase**, under `tests/`, following the existing
  `tests/serial_stall_test.py` pattern (pytest driving a real board through
  `py-opendisplay`). These *are* the Verification sections — versioned with the
  code, not prose. See [Verification model](#verification-model) below.

## Hard constraint — NO wire protocol change

`include/opendisplay_protocol.h` must not change, and no new opcode or response
code may be added. Verified for every phase below: dropping a link, refusing a
connection, and idle teardown are all HCI-level (a disconnect *reason* byte, not
an app-protocol field); `RESP_AUTH_REQUIRED` already exists and is used in its
documented meaning. If any phase turns out to need a wire change it stops and the
change goes through `../opendisplay-protocol` first.

---

## What the current code actually does (ground truth)

Established by direct reading of the tree, 2026-07-31. These are the facts the
phases build on; each is cited so a reviewer can re-check rather than trust.

### Connection model is asymmetric and, on ESP32, unguarded

- **nRF** caps at one central in hardware: `Bluefruit.begin(1, 0)`
  ([ble_transport_nrf.cpp:164](../src/ble_transport_nrf.cpp)). The SoftDevice
  refuses a second central at the link layer. Advertising re-arms itself
  (`restartOnDisconnect(true)`, `:210`).
- **ESP32** allows **three** centrals: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS = 3` is
  baked into the precompiled NimBLE framework and a `-D` override is inert (the
  precompiled `sdkconfig.h` wins). `onConnect`
  ([ble_transport_esp32.cpp:81-93](../src/ble_transport_esp32.cpp)) does **no**
  count check and **no** rejection; a second central's handle simply **overwrites**
  the single scalar `s_connHandle` (`:87`), and its writes land in the same RX ring
  undistinguished. This is a live multi-central exposure, not a hypothetical.
- **Every piece of per-link state on ESP32 is a global scalar any central can move**,
  which is why CONNECTION_POLICY R3 needs six requirements at the callback and
  dispatch boundary rather than one. Besides `s_connHandle`: `s_notifySubscribed` is set by whichever central
  subscribed last (`onSubscribe` discards its `connInfo`, `:129`), and `onWrite`
  discards its `connInfo` too (`:135`), so a contender's frames enter the incumbent's
  RX ring.
- **Notifications go to every subscribed client — a live leak, present today.**
  `BleTransport::notify` calls `s_txCharacteristic->notify(data, len)`
  ([ble_transport_esp32.cpp:269-277](../src/ble_transport_esp32.cpp)), the two-argument
  overload. NimBLE's third parameter defaults to `BLE_HS_CONN_HANDLE_NONE`, documented
  as "send the notification to **all subscribed clients**." A second central that
  connects and subscribes therefore receives every response the incumbent is sent,
  including authentication traffic, before `loop()` runs at all and with no policy
  decision having been made. Fixing it is a one-argument change (Phase 2).
- **Connect and disconnect events coalesce**, and their side-band data is single-slot.
  Both are plain `volatile bool`
  ([ble_transport_esp32.cpp:33-34](../src/ble_transport_esp32.cpp)) and the header
  records the weakness itself: "a second same-type event arriving inside the
  check-then-clear window is lost" ([ble_transport.h:66-71](../src/ble_transport.h)).
  `s_disconnectReason`, `s_rxBoundaryAtDisconnect` and `s_connHandle` are each one
  slot, so each event overwrites the last. Harmless today — `serviceBleEvents()`
  decides nothing per-connection ([main.cpp:461-500](../src/main.cpp)) — and a
  correctness problem the moment each event drives an admission decision.
- **LAN** is single-client, last-in-wins: a second TCP accept evicts the first
  ([wifi_service.cpp:871-877](../src/wifi_service.cpp)).
- **BLE and LAN can both be live at once.** There is no connection-level
  arbitration. The only ownership is per-*transfer*: `sessionOrigin`, stamped at
  transfer START ([display_service.cpp:2159,2200,2712](../src/display_service.cpp)),
  enforced per-frame by `frameOwnsSession()` and per-disconnect by
  `serviceBleDisconnectCleanup()`.

### No application code can drop a BLE link through the transport

- `BleTransport` ([ble_transport.h](../src/ble_transport.h)) exposes **no**
  `disconnect()`. `end()` is a full-controller teardown, and a no-op on nRF.
- ESP32 captures the conn handle (`s_connHandle`, `ble_transport_esp32.cpp:87`)
  but **never calls** `NimBLEServer::disconnect()`. The capability is one line
  away and unused.
- nRF has exactly one host-initiated disconnect in the whole firmware —
  `Bluefruit.disconnect(Bluefruit.connHandle())`
  ([device_control.cpp:857](../src/device_control.cpp)), inside DFU entry, reaching
  past the abstraction into Bluefruit directly. **Bluefruit's public `disconnect()`
  takes only a handle and always sends reason 0x13 — there is no reason argument to
  honour** — a fact the seam design
  below has to respect.

### No stall detection reaches a hung `loop()`

- nRF has **no watchdog at all** ("every fault handler is `b .`",
  [od_log.h:40](../src/od_log.h)).
- ESP32's `loop()` is **not** subscribed to the task WDT: Arduino leaves
  `loopTaskWDTEnabled = false` and nothing here calls `esp_task_wdt_add()` for the
  loop task, so `loop()` is unsupervised. (Whatever `CONFIG_FREERTOS_WATCHDOG_TIMEOUT_S`
  is set to is immaterial — no framework code arms a loop watchdog from it.)
- The only wall-clock teardown is `checkTransferTimeouts()`
  ([display_service.cpp:584-638](../src/display_service.cpp)), and it measures total
  elapsed from transfer **START** — 15 minutes (`TRANSFER_WATCHDOG_MS = 900000`). It
  is a total-duration bound, **not** a stall/inactivity timeout: a transfer that
  stalls at minute 1 is still not torn down until minute 15, and a slow-but-
  progressing transfer is cut off at 15 minutes regardless of progress.
- **The refresh BUSY-wait is bounded on one path only.** On `bb_epaper`,
  `waitforrefresh(60)` loops `timeout * 100` times at 10 ms and then fails
  ([display_service.cpp:803-831](../src/display_service.cpp)). **On FastEPD there is no
  bound whatsoever**: `waitforrefresh()` delegates to `fastepd_wait_refresh()` (`:805`),
  which ignores its timeout argument outright — `(void)timeout_sec; return
  !s_init_failed;` ([display_fastepd.cpp:277-280](../src/display_fastepd.cpp)) — and the
  real blocking lives above that call, inside `fullUpdate()`/`fastUpdate()`. This is
  CONNECTION_POLICY R5's exposure, and it is out of scope here; see residual risk.

### An idle connected client is never dropped

- `pollActivity()` stamps `lastActivityMs` whenever `connCount > 0`
  ([main.cpp:366](../src/main.cpp)) — a live link is treated as activity in
  itself. So a client that connects, authenticates, and goes silent holds the
  device out of its idle path **forever**.
- `session_timeout_seconds` ([encryption.cpp:254-265](../src/encryption.cpp))
  measures from session START not last activity, clears the *session* but **not**
  the *link*, and is only evaluated when a command arrives — so it never fires on
  a silent client. It defaults to 0 (disabled).
- There is **no** BLE idle link-drop. LAN has one (`OD_LAN_READ_TIMEOUT_S = 30`,
  [wifi_service.cpp:952](../src/wifi_service.cpp)); BLE has no equivalent.

### State with no disconnect-time reset (Phase 2 surface)

Confirmed missing or open-coded, i.e. what an abort must newly cover:

- `encryptionSession` — **not** cleared on BLE disconnect. Crypto state survives a
  link drop. `clearEncryptionSession()` runs on session-timeout-at-command, a new
  auth, config reload ([communication.cpp:66](../src/communication.cpp)), and LAN
  teardown ([wifi_service.cpp:798,874](../src/wifi_service.cpp)) — but no BLE
  disconnect path is among them.
- `chunkedWriteState` (config chunked upload,
  [config_parser.h:47](../src/config_parser.h)) — **no reset function**; cleared
  only by open-coded inline assignments in `communication.cpp`, untouched by
  disconnect and by the watchdogs.
- The response TX ring — **no** flush/discard primitive (only `bleRxQueueDiscardTo`
  exists, RX side).
- The RX ring is **anonymous** — `CommandQueueItem` is `{data, len, pending}`
  ([command_queue.h:72-76](../src/command_queue.h)); nothing records which link a frame
  came from. The disconnect path compensates with a boundary captured at link-down
  ([ble_transport_esp32.cpp:105](../src/ble_transport_esp32.cpp)) and discarded to on
  the loop ([main.cpp:480-489](../src/main.cpp)) — a mechanism Phase 2 retires
  (requirement 6 below).
- `directWriteTouchSuspended` — reset only *inside* `cleanupDirectWriteState()`, so
  a teardown routed through the partial path can leave touch suspended.
- Buzzer and LED — serviced each loop pass, and **no session-teardown stop API exists.
  The abort deliberately does not add one**; see the carve-out in `abortToKnownState`
  below. Both are bounded and self-terminating — the buzzer's `outer` repeat count is a
  `uint8_t` coerced to at least 1 and playback calls `buzzer_stop_internal()` at
  `rep >= outer` ([buzzer_control.cpp:215-217,288-291](../src/buzzer_control.cpp)); the
  LED runs a stepped pattern to completion
  ([device_control.cpp:530-541](../src/device_control.cpp)). Neither can run forever, so
  neither is state a later connection can inherit.

  **The stop routines themselves exist but are file-static**, which matters for the one
  caller that does need them: `buzzer_stop_internal()`
  ([buzzer_control.cpp:147](../src/buzzer_control.cpp)) and `led_stop_internal(bool
  clear_mode)` ([device_control.cpp:347](../src/device_control.cpp)). Deep sleep must
  silence both (7e row 3), so Phase 2 adds two thin **public wrappers** — sleep APIs, not
  teardown APIs. Nothing in the abort may call them.

---

## Phase 1 — Nonce / replay correctness  ✅ SHIPPED

Shipped on this branch (`e2e95cd`…`19335e6`), recorded here for completeness. What
landed:

- The AES-CCM anti-replay state moved from a 512 B ring of raw counter values to a
  32 B sliding bitmap (`src/nonce_window.h`, a dependency-free pure state machine).
- Check split from commit: `nonceCheck()` decides and writes nothing; `nonceCommit()`
  runs only *after* the CCM tag verifies. So packet loss is no longer counted as
  tampering, and an unauthenticated peer cannot advance replay state.
- The forward-distance cap was **removed** and comparison made numeric, not modular:
  a counter ahead of `last_seen` is accepted at any distance (the tag is the gate),
  which fixed a cliff where a forward gap past the cap stranded the session
  unrecoverably. A consumed counter is still never re-accepted (`last_seen` only
  moves up; below it, bitmap-caught or rejected on width).

**Verified:** host suite 47,445 checks under `-Werror`+ASan/UBSan (and 1,635
failures against the pre-change code, proving the tests discriminate);
`nrf52840custom`, `esp32-c3-N16`, `esp32-N4` build.
**Not verified:** the entire hardware matrix.

Nothing in Phase 1 is reopened here. One carry-forward: an **auth-abuse disconnect**
was prototyped alongside the nonce work on a separate branch
(`feat/nonce-replay-and-auth-guard`) but is **not** on this branch, and is redesigned
fresh as Phase 4.

---

## Phase 2 — BLE-HAL foundation (mechanisms)

**Goal:** every transport/HAL *mechanism* the later phases build on — the portable
link-drop seam and its R3a wait, connection instance identity and the owner token, the
instance table with callback-side write/subscribe/notify filtering, the activity clock,
and the idempotent abort-to-known-state routine. No *policy* lives here (Phase 3 decides
when to refuse and when to drop); Phase 2 only makes each action possible and each fact
observable.

**Phase 2 already closes two live holes on its own**, before any Phase 3 policy exists:
the notify leak (a second central receiving the incumbent's responses) and command
injection into the incumbent's RX ring. Both are callback-side filtering, and neither
waits on an admission decision. If Phase 3 slips, these should still land.

### The link-drop seam — `BleTransport::disconnect(uint16_t handle)`

Add to the abstraction ([ble_transport.h](../src/ble_transport.h)) and implement per
target. It takes an explicit **handle**, not just "the current connection", because
Phase 3's admission needs to drop a *specific* link; pass the current handle for the
common case.

- **ESP32:** `s_server->disconnect(handle, BLE_ERR_REM_USER_CONN_TERM)`. Return the
  call's bool; log WARN on failure. Note the library already treats "the link is
  gone" as success — `NimBLEServer::disconnect` returns `true` for `BLE_HS_ENOTCONN`,
  `BLE_HS_EALREADY` and `UNK_CONN_ID` (`NimBLEServer.cpp:321-332`), so a WARN here
  means a genuine failure, not a benign race with a client that left first.
- **nRF:** `Bluefruit.disconnect(handle)`. Lift the pattern from
  [device_control.cpp:857](../src/device_control.cpp) but keep
  `restartOnDisconnect(true)` (unlike DFU, which disables it).

**Reason fixed at 0x13, and the seam hard-codes it.** A host-initiated disconnect
must use a Core-Spec-legal `HCI_Disconnect` reason. `BLE_ERR_REM_USER_CONN_TERM`
(**0x13**) is legal; `BLE_ERR_CONN_LIMIT` (0x09) is **not**, and the controller
silently rejects it (0x12) — the gatecrasher stays connected while the code looks
like it worked. The stacks are asymmetric, and both were read rather than assumed:

- NimBLE takes a reason and *defaults it to 0x13* —
  `disconnect(uint16_t connHandle, uint8_t reason = BLE_ERR_REM_USER_CONN_TERM)`
  (`NimBLEServer.h:66`), forwarded to `ble_gap_terminate`.
- Bluefruit takes **only a handle** — `AdafruitBluefruit::disconnect(uint16_t conn_hdl)`
  (`bluefruit.h:171`) delegates to `BLEConnection::disconnect(void)`, which calls
  `sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)`
  (`BLEConnection.cpp:206`). There is no reason parameter to pass, let alone one to
  honour.

So the seam exposes no `reason` parameter: 0x13 is the only value this plan wants,
the value NimBLE already defaults to, and the only value nRF can send. Both stacks
do take a **handle**, which is what the seam's signature carries.

**Also fix the inbound reason, which currently lies (ESP32).** Not a new feature —
a correctness fix to what is already logged. `s_disconnectReason` is a `uint8_t`
([ble_transport_esp32.cpp:35](../src/ble_transport_esp32.cpp)) assigned from
NimBLE's `int reason` with a truncating cast (`:99`). NimBLE uses two ranges: HCI
reasons wrapped as `BLE_HS_ERR_HCI_BASE + code` (`0x200 + code`), and host-layer
`BLE_HS_E*` codes in `1..31`. The cast keeps only the low byte, so an HCI reason
survives by luck (`0x213 & 0xFF == 0x13`) while `BLE_HS_ENOTCONN` (7) truncates to
`0x07` and reads back as the unrelated HCI "memory capacity exceeded". The log at
[main.cpp:472](../src/main.cpp) then prints it as decimal `%u`, so the two collide
on screen as well as in storage. nRF is unaffected — it stores a raw HCI `uint8_t`
from the SoftDevice with no wrapping ([ble_transport_nrf.cpp:38,135](../src/ble_transport_nrf.cpp)).

Fix: widen `s_disconnectReason` and `takeDisconnectedEvent`'s reason out-param to
`uint16_t` ([ble_transport.h:81](../src/ble_transport.h), one caller at
[main.cpp:471](../src/main.cpp)), drop the cast, and log `0x%03X` so a wrapped HCI
reason (`0x213`) and a host reason (`0x007`) are visibly distinct. No enum, no
classifier — just stop discarding half the value. (The *other* out-param, `rxBoundary`,
is retired outright by requirement 6 — the boundary mechanism it fed is superseded by
frame tags.)

*Deferred, deliberately:* normalizing the inbound reason into an `OdDiscReason`
enum (`SUCCESS / REMOTE / LOCAL / TIMEOUT / MIC_FAILURE / OTHER`). Nothing in
Phases 2–4 branches on *why* a link dropped — the abort runs the same teardown
regardless, and a self-initiated drop is identified by its `*DropPending` flag, not
by reading the reason back. The classifier would feed a log line and nothing else.
The likely first real consumer is MIC-failure handling (0x3D signals encryption
desync); when that lands it is a small header and a `switch`, and the `uint16_t`
raw value preserved here is exactly its input, so nothing above has to be redone.

All disconnect calls are made from the **loop task** (a `serviceBleLinkDrop` hook, or
inline in the loop-serviced helpers), never a stack callback — a callback that severs
its own link mid-dispatch is exactly the class of bug `#132` removed.

#### The drop waits for link-down (CONNECTION_POLICY R3a)

**`disconnect()` requests termination; it does not perform it.** `NimBLEServer::disconnect()`
returns true even for `BLE_HS_ENOTCONN`/`BLE_HS_EALREADY` (`NimBLEServer.cpp:321-332`),
so a true return means "requested," not "down." An earlier draft of this plan released
the owner token at request time, which would let a new connection be admitted while the
old link was still physically up. R3a supersedes that: the drop is **synchronous** — the
seam requests termination, then waits cooperatively and with a bound until the link is
actually down, and only then does the abort release the slot.

So the seam is not the bare call but a small helper beside it:

```
bool bleDropAndWait(uint16_t handle);   // request + bounded cooperative wait; true if down
```

Three properties of the current tree make this the simple form it looks like, and R3a
records the investigation that rejected a cross-pass `DROPPING` state as more machinery
than the problem needs:

- **Link-down is per-handle pollable without consuming the event.** The disconnect
  callback writes the departing instance's table entry (requirement 5 below) at the
  moment the link drops, so the wait's predicate is "the owner's `(handle, epoch)`
  entry is no longer live" — a scan of the instance table. **The aggregate
  `connectedCount()` must NOT be the predicate**: it is the stack's total peer count
  on both targets — `s_server->getConnectedCount()`
  ([ble_transport_esp32.cpp:257-259](../src/ble_transport_esp32.cpp)),
  `Bluefruit.connected()` ([ble_transport_nrf.cpp:235-239](../src/ble_transport_nrf.cpp)) —
  and R1 explicitly permits a refused contender to be transiently attached, so dropping
  the owner moves the count 2→1, never to 0, and the wait would sit out its full bound
  on a link that is already down. The disconnect *event* stays queued for
  `serviceBleEvents()` ([main.cpp:461-500](../src/main.cpp)) to consume on its normal
  path — the wait neither consumes nor reorders it. (What survives of that path is the
  event flow, flag and reason; its RX-boundary capture is retired by requirement 6.)
- **The wait ticks on a short plain `delay()`, not `idleDelay()`.** An earlier draft
  named `idleDelay()` the right primitive for its early-out on `ble.eventPending()`
  ([main.cpp:749](../src/main.cpp)). That early-out is exactly wrong here: the
  predicate is table state, not event arrival, and mid-teardown events are deliberately
  left unconsumed — so once any event is pending (the owner's own disconnect, or an
  unserviced contender's connect), every `idleDelay()` call returns immediately and the
  wait degrades into a busy spin for its remaining bound. A plain `delay(2)` tick
  services *neither* RX nor transport events — the safety property actually wanted —
  and costs a few milliseconds of latency against a bound sized in tens of them.
- **The epoch makes an expired wait harmless.** If the bound expires with the old link
  still up, that link is inert by construction: its writes are filtered as non-owner,
  and its late disconnect is inert on stale epoch (7b rows 4 and 9). This is why the
  bound is the least load-bearing threshold in the plan.

*The wait can never land inside a refresh.* Every caller is loop-task-only and already
deferred while `epdRefreshInProgress` ([main.cpp:389](../src/main.cpp)).

*Timing.* An alive peer terminates within a few connection intervals — tens of ms; the
firmware requests no interval, so the central's negotiated value applies. A peer already
gone is reaped by the link layer at ~4–6 s. So `OD_BLE_LINK_DOWN_WAIT_MS` wants to cover
a few connection intervals with margin — tens to low hundreds of ms — not a supervision
timeout. It is deliberately *not* sized against the 120 s idle timeout.

### Connection instance identity and the owner token

A tiny arbiter, one new translation unit (`src/link_owner.h/.cpp`) or folded into
`communication.cpp`. **Identity is the triple `(transport, handle, epoch)`**, per
CONNECTION_POLICY R2 — an earlier draft of this plan used a `(transport, handle)` pair,
which R2 supersedes:

```
enum LinkOwner { OWNER_NONE, OWNER_BLE, OWNER_LAN, OWNER_TERMINAL };
struct LinkId { LinkOwner who; uint16_t handle; uint16_t epoch; };

// The token itself is ONE 32-bit word: [31:30] transport | [29:16] handle | [15:0] epoch.
// All-zero == unowned; epoch 0 is never allocated; 0xC0000000 (transport 0b11,
// handle 0, epoch 0) == OWNER_TERMINAL, the deep-sleep admission gate.
uint16_t linkNextEpoch(void);            // __atomic_fetch_add; connect callback, EVERY instance
bool   linkClaim(LinkId id);             // one CAS on the word; safe from stack callbacks
void   linkRelease(LinkId id);           // CAS holder -> NONE; loop task only, after R3a's
                                         // wait; full-identity match, so it can never zero
                                         // the terminal word (and never accepts it as id)
LinkId linkMarkTerminal(void);           // atomic exchange -> OWNER_TERMINAL, returning the
                                         // DISPLACED owner identity (possibly NONE) — the
                                         // identity the terminal caller hands the abort;
                                         // deep-sleep path only, BEFORE the abort (R7e row 3)
LinkId linkOwnerId(void);                // one atomic load; callable from ANY task
bool   linkIsOwner(LinkId id);           // full-triple comparison; handle alone is never enough
```

The token is **connection-level**: at most one transport-and-link owns the session at
a time. `OWNER_LAN` uses handle 0 (single TCP client by construction); `OWNER_BLE`
carries the conn handle, which the arbiter records — authoritative "who owns the link",
separate from the transport's `s_connHandle` scalar that the newest connect overwrites.

**Why the epoch, and where it is allocated.** BLE conn handles are small integers the
stack reuses — NimBLE allocates from 0 upward, so a client that disconnects and
reconnects can be handed the *same* handle. This firmware defers work by design, and
`serviceBleDisconnectCleanup` can run tens of seconds late when `loop()` was blocked in
a refresh, a hazard the code already documents at
[main.cpp:398-403](../src/main.cpp) — so a deferred operation carrying a stale handle can
otherwise match a newer session and act on it. The epoch turns "same handle" into "same
connection instance," which is what every deferred consumer actually needs.

**The epoch is allocated in the connect callback, for every connection instance,
admitted or not** — never on successful claim. This is the trap R2 calls out explicitly:
a *refused* contender never claims, so on claim-time allocation it would carry no epoch,
and 7a row 4 (a contender reusing the incumbent's handle after a stale link) could not be
distinguished from the incumbent at all. Allocation must precede the admission decision,
because the identity is what the decision is *made on*. On admission the token copies the
instance's already-allocated epoch.

Scope is one boot — no deferred RAM state survives a reset, so cross-reset uniqueness is
neither required nor claimed. The epoch is **16 bits, a deliberate narrowing** (an
earlier draft had `uint32_t`): the one-word token below must stay lock-free, and neither
Cortex-M4 nor the ESP32 ISAs have a lock-free 64-bit CAS, so the triple packs as
`transport(2) | handle(14) | epoch(16)` — HCI conn handles are spec-bounded at 0x0EFF,
so 14 bits holds them with headroom. The invariant that justifies the width (per R2's
wrap rule, where the full conditional argument lives): no outstanding event may survive
a full counter cycle. Epochs churn at link-layer connection rate — tens of ms per
instance — so a full 2^16 cycle needs about half an hour of continuous connect churn
inside a single blocking window that later *completes*; a hung refresh that never
completes (R5's gap) never resumes the loop, so nothing is ever consumed there and a
collision has no consumer to mislead. `linkNextEpoch` re-draws when the fetch-add
yields 0, so wrap cannot mint the reserved unowned encoding.

**The token is one atomic word, claimed at the earliest transport hook — NOT a
loop-task-only global.** An earlier draft made the token plain loop-side state (the
`g_commandOrigin` argument, [communication.cpp:30-36](../src/communication.cpp)) while
separately requiring callback-side write filtering and an atomic claim at the callback
(R7d). Those are incompatible: `onWrite` fires on the NimBLE host task before any
loop-side admission has run — during a refresh, up to ~16 s before one — so a loop-only
token gives the filter nothing to compare against, and leaves no rule for the unowned
window before first admission. Per R2 the resolution is that the token *is* the
published word:

- **Claim is a compare-and-swap on the word**, executed at the earliest transport
  hook — the BLE connect callback (host task) and the LAN accept (loop task). CAS
  success *is* admission; failure marks the instance a contender, which Phase 3's
  loop-side scan refuses. This makes R7d's "the claim itself must be atomic at the
  callback" a mechanism rather than an aspiration, and it closes the unowned window:
  the host task processes a peer's connect before any of its writes, so by the time
  the first client's first write reaches `onWrite`, the word already names it owner.
- **The filters read the word with one `__ATOMIC_ACQUIRE` load**: `onWrite` and
  `onSubscribe` compare their instance's identity against it on the host task;
  `notify()` reads it on the loop task for the target handle.
- **Release stays loop-task-only** (CAS holder → NONE), strictly after the R3a wait.
- **Order in the connect callback:** allocate the epoch, publish the table entry, then
  CAS — so a successful claim never names an instance the loop cannot yet see.

`linkNextEpoch` is `__atomic_fetch_add` — BLE allocates on the host task and LAN on the
loop task, so a plain increment would race the two. The *instance identity* in the
table follows the same publication rule as before: written on the stack callback task,
read on the loop task, atomics discipline per the instance table below.

Phase 2 establishes only the mechanism and the baseline: the first BLE connect claims
`OWNER_BLE` with its handle and epoch; disconnect releases it (wired into `abort` below).
Deciding what to do with a *second* contender is Phase 3 policy (it refuses; see
[the governing decision](#the-governing-decision-admission-never-evicts)).

### The instance table and callback-side filtering

CONNECTION_POLICY R3 requires **six** things at the callback and dispatch boundary. An
earlier draft of this plan had two of them (a handle-bearing connect event, write
filtering) and treated the rest as absent problems; the policy's review of the ESP32
callbacks found that a contender perturbs shared state *before any loop-side decision
runs*, and the review after that found queued frames outlive their session — so all
six are Phase 2 mechanisms. In table form, against the ground truth above:

| # | Requirement | Site today | Why it cannot wait for Phase 3 |
|---|---|---|---|
| 1 | **Per-link write filtering** — drop a non-owner's write before the RX ring | `onWrite`, `(void)connInfo` ([:135](../src/ble_transport_esp32.cpp)) | loop-side refusal has not run yet during a ~16 s refresh block; a gatecrasher can inject a full transfer's worth of commands |
| 2 | **Per-link subscribe filtering** — subscription state per instance | `onSubscribe`, `(void)connInfo` ([:129](../src/ble_transport_esp32.cpp)) | a contender's subscribe clears/overwrites the incumbent's apparent notify-readiness, stalling its TX |
| 3 | **Handle-targeted notify** — pass the owner's conn handle | `notify(data, len)` ([:269-277](../src/ble_transport_esp32.cpp)) | closes a **live leak** of the incumbent's responses, auth traffic included |
| 4 | **Identity-bearing disconnect events** | `takeDisconnectedEvent` carries reason + RX boundary, no handle ([ble_transport.h:81](../src/ble_transport.h)) | every consumer must ignore an event whose identity is not the owner's (7b) |
| 5 | **State that survives lost edges** — the instance table | coalescing `volatile bool` pair ([:33-34](../src/ble_transport_esp32.cpp)) | under this policy a lost event is a lost *admission decision* |
| 6 | **Frame identity** — tags on queued frames, re-checked at dispatch | `CommandQueueItem` is `{data, len, pending}`, no identity ([command_queue.h:72-76](../src/command_queue.h)) | a delayed frame from a dead instance is indistinguishable from the new owner at dispatch; a boundary flush cannot save it once the table slot was reused |

Requirement 3 is worth calling out separately: it is a **one-argument change that closes
a live leak independent of the rest of this plan**, and it is the cheapest item in
Phases 2–4 by a wide margin. It should not wait behind the table.

Requirements 1–3 all read the same fact — who owns the slot: the write and subscribe
filters on the host task, `notify()` on the loop task. The one-word owner token above
is what makes that read legal from both — a single atomic load, compared against the
callback's own instance identity. None of the filters touches any other loop-side
state.

The connect event still becomes identity-bearing (handle **and** epoch, per R2), so
loop() can act on a specific newcomer — but under requirement 5 it is a hint, not the
mechanism.

#### Requirement 5: a fixed per-handle instance table, not an event queue

Today's coalescing is tolerable because `serviceBleEvents()` decides nothing
per-connection: a connect means "reset `rebootFlag`, update MSD, tune the link," a
disconnect means "flush the RX ring to the boundary, raise the cleanup flag"
([main.cpp:461-500](../src/main.cpp)). Under this policy each event drives an admission
decision about a specific instance, so a lost event is a lost decision — two concrete
failures, both reachable inside one refresh block:

- **Lost connect → an unrefused contender.** Two centrals connect while `loop()` is
  blocked; the flag is set twice and read once. One is refused; the other is connected,
  never evaluated, and invisible to the loop.
- **Lost disconnect → the slot held by a ghost.** Owner disconnects, then a contender
  connects and disconnects, all within one block. The flag coalesces and the side-band
  identity is the *last* writer's. The loop sees a disconnect that does not match the
  owner, treats it as inert, and never releases — every new client refused until the
  idle timeout reclaims the slot. A device-wide outage of one full timeout.

**The fix is a table the loop scans, not a queue it drains.** Sized by the connection
cap: 3 on every ESP32 target here, 1 on nRF. Each entry holds `(handle, epoch,
reason)` — **metadata only, ~8 bytes, never frames**, which is what keeps it inside
the one-command-queue constraint. There is no separate `state` field: liveness *is*
the packed `(handle, epoch)` identity word (all-zero = empty), per the publication
rule below. There is no `rxBoundary` field either — requirement 6 retires the boundary
mechanism, which is what lets entries be overwritten freely on churn. Callbacks write their own handle's entry; the
loop compares the table against its own notion of the owner. That inversion dissolves the
overflow question rather than answering it:

- **It cannot overflow.** State is bounded by the connection cap, not by event rate.
  Contender churn overwrites entries for handles already gone. No eviction policy to
  specify, because nothing is queued.
- **Lost edges stop mattering.** A contender that connects and disconnects wholly within
  a refresh block leaves no entry — correct, since there is nothing left to refuse.
- **Owner release is a comparison, not an event.** If the owner's `(handle, epoch)` is no
  longer live in the table, the owner is gone, however many edges were missed.
- **Ghosts stay visible.** Any live entry that is not the owner is a contender still
  needing refusal, so a missed refusal self-corrects on the next pass instead of leaking
  a slot.

*Search by handle; do not index by it.* NimBLE allocates from 0 upward in practice, so
direct indexing usually works, but a 3-entry linear search costs the same and cannot be
broken by a stack change that hands out sparse handles.

**Publication must be atomic — and liveness is part of the identity word.** Entries are
written on the NimBLE host task and read on the loop task. A multi-field `volatile`
struct is not an atomic snapshot — and `volatile` is not an inter-task tool in C++
regardless. Each entry's packed `(handle, epoch)` word doubles as its liveness: all-zero
means empty, a release-store publishes it at connect, and the disconnect callback
clears it with another release-store — never a separate `state` flag that could race
the identity. The R3a wait's acquire load therefore sees identity and liveness in one
shot, which is what makes "the owner's entry is no longer live" a sound predicate. The
one side field (`reason`) follows the `__atomic_*` discipline the RX ring in this repo
already uses ([command_queue.cpp:62,92](../src/command_queue.cpp)) and is consumed only
after the identity word says down.

**nRF needs only requirement 4 of the filtering set in practice.** `Bluefruit.begin(1, 0)`
([ble_transport_nrf.cpp:164](../src/ble_transport_nrf.cpp)) configures the SoftDevice for
a single peripheral link, so cross-central injection is unreachable at the link layer;
its one-entry table is degenerate. Its write callback also discards the handle it is
given ([ble_transport_nrf.cpp:148](../src/ble_transport_nrf.cpp)) — latent, not live, but
fix it with the ESP32 filter so the two targets read the same. Requirement 6 applies to
nRF in full, though: a single-link target still queues frames that can outlive their
session across a disconnect/reconnect pair inside one refresh block.

#### Requirement 6: tagged frames retire the RX boundary

CONNECTION_POLICY R3 requirement 6, scheduled here. The policy carries the full
rationale (three review findings, one root cause: anonymous frames); this is the
implementation shape:

- **`bleRxQueuePush` gains a `uint32_t tag` parameter**, and `CommandQueueItem` gains
  the field ([command_queue.h:72-76](../src/command_queue.h)) — four bytes × 18–34
  slots is 72–136 B, per-frame metadata in the one ring, not a second ring. `onWrite`
  passes the packed identity word it already loaded for the requirement-1 filter, so
  the stamp is free; the nRF write callback does the same. The tag is written into
  the slot **before** the release-store that publishes the head
  ([command_queue.cpp:92](../src/command_queue.cpp)), exactly like `data` and `len`,
  or the consumer's acquire load is not guaranteed to see it.
- **Dispatch checks the tag before parsing.** `serviceBleRx()` pops `(frame, tag)`,
  drops the frame (counted, logged at debug) if `tag != linkOwnerWord()`, and otherwise
  publishes it to the dispatcher as `g_commandInstance` beside `g_commandOrigin`
  ([communication.cpp:30-36](../src/communication.cpp)) — loop-task-only single-writer,
  the same argument as `g_commandOrigin` itself. LAN dispatch sets `g_commandInstance`
  to the LAN owner's word directly; LAN frames never traverse the BLE ring.
- **Retired outright:** `s_rxBoundaryAtDisconnect`
  ([ble_transport_esp32.cpp:40,105](../src/ble_transport_esp32.cpp)), the `rxBoundary`
  out-param of `takeDisconnectedEvent`
  ([ble_transport_esp32.cpp:347-351](../src/ble_transport_esp32.cpp), consumer at
  [main.cpp:470-489](../src/main.cpp)), and `bleRxQueueDiscardTo`
  ([command_queue.h:112](../src/command_queue.h)). The disconnect consumer keeps
  identity and reason; stale frames self-discard at dispatch instead of being flushed
  to a boundary that a table overwrite could lose.

### The activity clock — a recognised command from the owner

Phase 3's idle drop needs to know how long the owner has been *silent*. Today's
`lastActivityMs` cannot serve: `connCount > 0` re-stamps it every pass
([main.cpp:366](../src/main.cpp)), so a live-but-quiet link never ages. LAN's
`lastLanActivityMs` cannot serve either: it stamps on `got > 0`, i.e. any bytes read
([wifi_service.cpp:946](../src/wifi_service.cpp)), so a flooder holds the slot with
garbage.

**Activity is defined by CONNECTION_POLICY R4, and only by it:**

```
idle  :=  no inbound command from the owner on the owning transport
          AND no refresh in progress
```

A frame counts only if it reaches the dispatcher and is **recognised as a command from
the current owner**.

> **This supersedes an earlier draft of this section, which stamped at RX intake** —
> `bleRxQueuePush()`'s success path ([command_queue.cpp:50](../src/command_queue.cpp)) —
> and argued that stamping only queued frames kept a garbage flooder from holding the
> link. That argument does not hold: the queue accepts **any** non-empty payload within
> the size cap ([command_queue.cpp:50-93](../src/command_queue.cpp)), including a
> two-byte malformed frame or an unknown opcode, which the dispatcher only rejects later
> ([communication.cpp:544,754](../src/communication.cpp)). Intake stamping rejects empty,
> oversized and ring-full frames and nothing else, so it leaves a flooder able to hold
> the slot indefinitely — precisely the failure the idle drop exists to prevent.

**The stamp point is the shared dispatcher.** `imageDataWritten()`
([communication.cpp:541](../src/communication.cpp)) is the single place all three
transports (nRF BLE, ESP32 BLE, ESP32 LAN) dispatch through. Stamp there, after the
`len < 2` guard and gated on two tests:

- **Recognised:** `commandName(command) != nullptr`. Unknown opcodes return nullptr and
  fall to the switch default's "Unknown command" error, so they are not activity. This
  reuses the existing recognition predicate rather than adding a second, drift-prone one.
- **From the owner:** the frame's instance identity — its requirement-6 tag, published
  to the dispatcher as `g_commandInstance` — equals the owner word. *This supersedes an
  earlier draft of this bullet, which compared `g_commandOrigin`
  ([communication.cpp:30-36](../src/communication.cpp)) against the owning transport.*
  Transport is not identity: a delayed frame from a dead BLE instance is
  indistinguishable from the new BLE owner by transport alone, and would stamp the new
  owner's clock. In practice the dispatch tag check has already dropped such a frame
  before the stamp is reached; the stamp's own full-word test is one redundant compare,
  kept because the two sites can otherwise drift.

Recognition deliberately sits *before* the auth gate: `CMD_AUTHENTICATE` must count as
activity or a client cannot complete a handshake without racing the clock. An
unauthenticated peer that sends recognised-but-rejected commands is therefore held off by
Phase 4's auth-abuse counter, not by this clock — which is the correct division, since
the counter can distinguish "wrong credentials" from "silent."

**One consequence worth naming: the clock is now loop-task-only.** Intake stamping ran on
the NimBLE host / Bluefruit callback task and needed `__atomic_store_n`/`__atomic_load_n`
(`__ATOMIC_RELAXED`). `imageDataWritten()` runs on the loop task, from `serviceBleRx()`
([main.cpp:513](../src/main.cpp)) and from `handleWiFiServer`, so the clock is a single-
writer plain global — no atomics, same argument as `g_commandOrigin`. The atomics
discipline is still required for the instance table above; it is just not required here.

**Where it lives: with the token, not in the transport.** The clock is now keyed on
*ownership*, which makes it policy, not link state — and this repo has settled precedent
in that direction: [ble_transport.h:89-93](../src/ble_transport.h) records that the
loop-serviced deferred-work flags were moved out of the transport because they "encode
application policy, not link state, so exporting them from the transport seam was
backwards." So the clock sits beside the owner token in `link_owner.h/.cpp`:

```
uint32_t linkMsSinceOwnerCommand(void);   // 0 when unowned
void     linkStampOwnerCommand(void);     // dispatcher, on a recognised owner command
void     linkStampRefreshEnd(void);       // endRefresh(), see below
```

One clock suffices because R1 admits one owner; R4's "each transport enforces its own
timer and constant" is satisfied by the *constants* differing — BLE's 120 s local define
against LAN's `OD_LAN_READ_TIMEOUT_S` — not by duplicating the clock.

**The clock must not run during a refresh.** `epdRefreshInProgress` brackets a *blocking*
call on the loop task ([display_service.cpp:2446-2467](../src/display_service.cpp),
[:3358-3368](../src/display_service.cpp)): `loop()` does not execute for the refresh's
duration, but wall-clock time passes. A naive `millis() - lastStamp` accrues the whole
refresh and can drop an actively engaged client the instant `loop()` resumes.

This is also what answers the intake-stamping rationale that has now been dropped. That
draft stamped at intake because a loop-side stamp would record when `loop()` *drained* a
frame rather than when it arrived, inflating silence by whatever the loop was blocked on
— and the thing it is blocked on is a refresh. The refresh exclusion addresses that
directly and correctly; intake timing addressed it only as a side effect, while getting
the definition of activity wrong.

Implementation requirements for the exclusion, per R4:

- **A loop-side edge detector cannot see the edge** — both transitions happen inside the
  blocking handler. The re-stamp must be invoked *at* the transition, via a single
  `endRefresh()` helper that **both** bracket sites call, not by polling the flag. Both
  sites currently assign `epdRefreshInProgress = false` inline; the helper replaces both
  assignments, so a future third refresh path cannot forget it.
- **Re-stamp the current owner's clock only**, and only if the same instance identity
  still owns the slot.
- Re-stamping can only ever *delay* a drop, never cause a spurious one — which is why it
  is safe to apply unconditionally at the transition.

**The baseline is the later of admission, last recognised command, and last refresh
end** — the init fix. A naive "`UINT32_MAX` until first command" would put a freshly
admitted, still-silent client instantly past any timeout:

```
linkMsSinceOwnerCommand() := millis() - max(admittedMs, lastCommandMs, refreshEndMs)
                             // 0 when unowned
```

A new client thus gets the full idle window before its first command. On LAN the
baseline is **TLS handshake completion**, not TCP accept (R7a) — handshake traffic is not
a command; see Phase 3.

### `abortToKnownState(reason, bool dropLink, LinkId ownerId)`

New `src/session_guard.h/.cpp` (both targets; LAN parts under
`#ifdef OPENDISPLAY_HAS_WIFI`, **not** `TARGET_ESP32` — `esp32-N4` is ESP32 without
WiFi). `ownerId` is the identity the abort acts for, and it is a **parameter, not a
re-derivation**: ordinary callers pass a snapshot of `linkOwnerId()` taken before
calling (or use a two-argument convenience overload that snapshots it); the terminal
caller passes the identity `linkMarkTerminal()` displaced, because by then the word
reads terminal and a re-derivation would act for the wrong identity. Steps 10 and 11
below consume it. Ordered teardown:

1. Log first (one line, the reason).
2. Optional client NACK — **skip when `dropLink`** (the link is about to go).
3. `cleanupDirectWriteState(true)` — panel power + touch-resume.
4. `cleanupPartialWriteOnDisconnect()`.
5. `resetPipeWriteState()`.
6. **new** `resetChunkedWriteState()` — a real primitive replacing the open-coded
   inline clears in `communication.cpp`; call it here and from those sites.
7. **new** `touchForceResume()` — asserts the suspend counter reached 0 and clears
   `directWriteTouchSuspended` even when teardown bypassed `cleanupDirectWriteState`.
   A new public idempotent API, not an existing primitive.
8. `clearEncryptionSession()` — **new on the disconnect path**; today crypto state
   survives a link drop.
9. **new** ring reset primitives — `bleTxQueueReset` and `bleRxQueueReset`.
   Discarding RX outright is sound because callback filtering (requirement 1) means
   every frame in it passed the owner check when written. `bleRxQueueReset` follows
   the SPSC contract in CONNECTION_POLICY requirement 6: **consumer-side discard
   only** — acquire-load the producer's head, release-store that snapshot into the
   tail, write neither the head nor any slot — so it cannot race an in-flight push;
   and it must never run while a peek is outstanding (every returning abort caller is
   loop-side after RX consumption; deep sleep, the one in-dispatch caller, never
   returns). A frame the owner writes *after* this step, during step 10's wait, is
   deliberately not re-flushed: it carries the departing instance's tag
   (requirement 6) and fails the dispatch check once step 11 releases — the same
   construction that makes an expired R3a wait harmless. An earlier draft flushed TX
   only, which left R6's "both rings drained" unmet and the teardown window open.
10. If `dropLink`: drop **by the owner's transport** — the token records it, and this
    routine is not BLE-only (the transfer watchdog that calls it is origin-agnostic).
    `OWNER_BLE` → `bleDropAndWait(ownerHandle)`: request termination, then wait
    cooperatively until the link is actually down or `OD_BLE_LINK_DOWN_WAIT_MS`
    expires (R3a); not the bare seam call. `OWNER_LAN` → a new public
    `wifiLanDropOwnedSocket()` seam in `wifi_service.cpp` — needed because
    `tlsCloseSession()` is file-static ([wifi_service.cpp:281](../src/wifi_service.cpp)),
    so the abort cannot reach the pieces directly. It performs the LAN-local subset of
    today's `disconnectWiFiServer()` ([wifi_service.cpp:794-808](../src/wifi_service.cpp)):
    `tlsCloseSession()`, `wifiClient.stop()`, `wifiServerConnected = false`,
    `tcpReceiveBufferPos = 0` — everything *except* `clearEncryptionSession()` and
    `requestTransferSessionCleanup()`, which are this routine's own steps 8 and 3–5,
    so the two never nest. A TCP close is synchronous; no wait bound applies on LAN.
11. `linkRelease(ownerId)` — full-triple release, and **strictly after** step 10.

**Steps 10 and 11 are ordered, and that order is the whole of R3a.** An earlier draft
released the token in the same breath as *requesting* the disconnect, which would let a
new connection be admitted while the old link was still physically up. If the wait in
step 10 expires the release still happens — expiry is an early exit, not a failure — and
the stale link is inert by construction: its writes are filtered as non-owner, and its
late disconnect is inert on stale epoch (7b rows 4 and 9). That is the guarantee that
let R3a drop the cross-pass `DROPPING` state an intermediate draft had introduced.

Note the asymmetry with step 2: the NACK is skipped when `dropLink` because the link is
about to go, whereas Phase 4's auth-abuse drop must *deliver* its final `FE` first. Phase
4 therefore runs its own bounded TX barrier **before** calling the abort, rather than
asking the abort to hold the link open — see Phase 4.

**Buzzer and LED are NOT stopped — deliberately.** An earlier draft added
`buzzerStop()` / `ledFlashStop()` as step 8. That is wrong: buzzer and LED are
user-facing *effects*, not session state. A client that fires a buzz and immediately
drops the link is a normal pattern — command, then disconnect to save power — and
truncating the buzz mid-note defeats the command's entire purpose. Nothing about a
playing melody corrupts or confuses a later connection, unlike a half-open pipe
session, a suspended touch input, or a live crypto session. And because this policy
fires the abort far more often than a plain disconnect once did (idle timeout,
transfer watchdog, auth-abuse), the regression would be correspondingly more visible.
Both are bounded and self-terminating (see ground truth), so leaving them running
cannot wedge anything. No stop step is added here.

**The one exception is not an exception to this.** Deep sleep does silence both, because
sleep stops the clocks the effects run on — but that lives in the deep-sleep path and
calls the wrappers directly. `abortToKnownState` never silences anything, including on
the deep-sleep call in the invocation set below. Keeping the two apart is what stops a
future edit from "unifying" them and quietly truncating every buzz on an idle drop.

**Panel power is NOT force-killed here — deliberately.** An earlier draft added an
`epdSessionForceOff()` step "unless refreshing". That is wrong: `epdSessionForceOff()`
powers off every state except `PWR_OFF`, **including `PWR_WARM`** (the only early
return is `if (pwrmgmState == PWR_OFF) return`,
[display_service.cpp:420-421](../src/display_service.cpp)) — a disconnect during a
refresh is deferred, so by the time abort runs the panel can be WARM with
`epdRefreshInProgress` false, and the step would kill exactly the panel that must
survive. Panel power is handled correctly by steps 3–5: `cleanupDirectWriteState`
forces off only a `PWR_ACTIVE` (mid-transfer) session and no-ops on WARM, matching
the existing "ACTIVE-only teardown" invariant in `serviceBleDisconnectCleanup`. So a
WARM keep-alive panel survives an abort — including an auth-abuse or idle drop of a
client while the panel is warm from a prior push.

Idempotent and loop-task-only: every step is either already a no-op when its state
is inactive, or made one.

### The complete invocation set

Collected here rather than left implicit across three phases, because the value of a
single shared teardown routine depends entirely on every teardown actually reaching
it. Three callers, and one governing invariant: **`dropLink=false` iff no drop is
wanted from the abort — either the link is already gone (the disconnect-cleanup case)
or the whole stack is about to be torn down with admission terminally gated (the
deep-sleep case, via `linkMarkTerminal()` below)**.

| Condition | `dropLink` | Phase |
|---|---|---|
| Disconnect event serviced: `s_disconnectCleanupPending && !epdRefreshInProgress && !ownerStillUp`, **and the event's identity matches the owner** (7b) | `false` | 2 |
| Deep sleep, forced or idle — **after `linkMarkTerminal()`, before `ble.end()`** (R7e row 3) | `false` | 2 |
| `serviceIdleTimeout()`: owned **by BLE** `&& !epdRefreshInProgress && linkMsSinceOwnerCommand() > OD_BLE_IDLE_TIMEOUT_MS` — **no** `transferActive()` gate, per R4 (LAN's reclaim is its own `OD_LAN_READ_TIMEOUT_S` path, per R4's per-transport rule) | `true` | 3 |
| Auth-abuse counter reaches its threshold, **after** the bounded TX barrier drains the `FE` or `OD_AUTH_ABUSE_FLUSH_MS` expires | `true` | 4 |

**Deep sleep is a caller — for teardown uniformity, not for surviving state**
(CONNECTION_POLICY R7e row 3, which carries the twice-corrected rationale in full).
*Earlier drafts justified this with state surviving sleep — RAM in one draft, hardware
in the next; both false against the tree:* wake re-enters `setup()` with RAM reloaded,
only `RTC_DATA_ATTR` survives ([main.cpp:129-150](../src/main.cpp)); and the sleep
path already forces the panel off before sleeping ([main.cpp:812](../src/main.cpp))
with touch re-initialised on wake ([main.cpp:238](../src/main.cpp)). The real reason:
deep sleep is a **mid-session exit** — forced sleep bypasses the live-link guard
([main.cpp:789](../src/main.cpp)) and the path does not arbitrate a LAN owner — whose
path hand-rolls a private teardown subset (panel force-off, advertising stop, stack
end, effect silencing). Routing the session half through the abort first makes sleep's
teardown identical to every other session end by construction, instead of a parallel
copy that every future session resource must be added to — the same anti-drift
argument that made the transfer watchdog a caller. The sleep path keeps its own sleep
quiescing on top: `epdSessionForceOff()` (WARM included — no panel sleeps powered;
this call must never move into the abort) and the buzzer/LED silencing below.

**Order: `linkMarkTerminal()` first, then the abort, then `ble.end()`** (the R7e row 3
ordering trap). Without the gate, the abort's step 11 frees the word while the owner's
link may still be up and advertising is still on — a connect on the host task could
win the freed word in that window and the new owner would be destroyed by `ble.end()`
with no abort ever run for it. With the word exchanged to `OWNER_TERMINAL` first,
claims fail for the rest of the shutdown, and wake reloads RAM clean.
`linkMarkTerminal()` **returns the displaced owner identity**, and that is the
identity the sleep path hands the abort to act for — after the exchange,
`linkOwnerId()` reads terminal, not the departing owner, so the abort must not
re-derive it. Step 11's `linkRelease(displacedId)` then finds the word not matching
and is naturally inert; `linkRelease` matches the full identity and never accepts the
terminal word itself, so nothing can CAS the gate back to zero. `dropLink=false` because
`ble.end()` takes the stack down immediately after; there is no link left to drop
politely, and no loop pass will service the resulting event.

**Deep sleep also silences buzzer and LED — settled, and it is a *sleep* change, not an
abort one.** The abort leaves both running by design, and deep sleep cuts the clocks they
depend on, so at this one transition "let the effect finish" cannot hold: the effect
*cannot* finish. Of the two consistent resolutions, this plan takes **silence on the way
down** rather than making sleep wait via the `workInFlight` gate
([main.cpp:694-699](../src/main.cpp)) — sleep is never delayed by a playing effect.

The argument is hardware state rather than symmetry. `enterDeepSleep` runs
`ble.stopAdvertising()` / `delay(200)` / `ble.end()` / `delay(100)`, then
`armButtonWakeSources()` and `powerLatchHoldForSleep()`
([main.cpp:806-836](../src/main.cpp)) — all outside `loop()`, so `buzzerService()` never
ticks through any of it. A tone still on is therefore not a melody playing out; it is a
driven pin held through teardown and into sleep, sounding continuously and drawing current
until the next wake. Waiting would only postpone that.

Three scoping rules, each of which a careless implementation gets wrong:

1. **In the deep-sleep path, never in `abortToKnownState`.** R6's carve-out is untouched:
   an idle, auth-abuse or watchdog drop still leaves a melody playing.
2. **Deep sleep only — not every terminal transition.** Power-latch off (7e row 4)
   deliberately *plays* a chirp on the way down, `passiveBuzzerPowerOffAlert()` immediately
   before `powerLatchTriggerOff()` ([device_control.cpp:83](../src/device_control.cpp)). A
   blanket "silence at every terminal transition" deletes that alert.
3. **ESP32-only.** `enterDeepSleep` sits inside `#ifdef TARGET_ESP32`
   ([main.cpp:757](../src/main.cpp)), so nRF takes on nothing here.

Placement: silence **before** `armButtonWakeSources()` / `powerLatchHoldForSleep()`, so
pin state is settled before the wake pads and latch hold are configured. Relative order
against `ble.end()` does not matter. The two public wrappers this needs are noted in the
ground truth above; `led_stop_internal`'s `clear_mode` should match `handleLedStop`'s
`true` ([device_control.cpp:587](../src/device_control.cpp)), so the observable result is
the same as the client having sent LED_STOP.

**The other terminal transitions stay exempt** (R6 exception 2, R7e rows 1, 2, 4): nRF DFU
entry ([device_control.cpp:847-866](../src/device_control.cpp)), ESP32 DFU/reboot
([:880](../src/device_control.cpp)) and power-latch off ([:942](../src/device_control.cpp))
all disconnect and then leave — the MCU resets, jumps to a bootloader, or loses power — so
"ready for a new connection" is meaningless and no loop pass will ever service the event.
Each may take a synchronous abort instead if that is ever wanted; the exemption is the
default.

**Explicitly not a caller: refusing a contender.** Admission calls
`ble.disconnect(newHandle)` (or `incoming.stop()` on LAN) and nothing else — no
`abortToKnownState`, no `s_disconnectCleanupPending`, no `linkRelease`. The
incumbent's session must be untouched. This is the case most likely to be got wrong
in implementation, since refusal and teardown sit in the same handler and differ only
in which handle they act on.

**Also not callers, deliberately.** `clearEncryptionSession()` at
[communication.cpp:66](../src/communication.cpp) (config reload) and
[encryption.cpp:261](../src/encryption.cpp) (session timeout) are crypto lifecycle,
not session aborts; they stay as they are.

**Resolved: `checkTransferTimeouts()` is a caller.** The 15-minute watchdog
([display_service.cpp:584-638](../src/display_service.cpp)) routes its teardown through
`abortToKnownState(dropLink=true)` and stops carrying its own. There is exactly one
teardown routine, which is the whole point: this plan cites *that very function* as the
reason a shared routine is needed, so exempting it would have argued for the routine
while leaving the original drift source untouched.

| Condition | `dropLink` | Phase |
|---|---|---|
| `checkTransferTimeouts()` fires on a direct-write or partial transfer past `TRANSFER_WATCHDOG_MS` | `true` | 2 |

This is a **behaviour change**, deliberately taken, in three ways:

1. **Crypto is now cleared.** The watchdog previously left the encryption session
   intact. It no longer does.
2. **The link is now dropped.** `dropLink=true` rather than `false`, which follows
   from (1) rather than being an independent choice: once the session is cleared, a
   retained link is a confusing state — the client's next command draws
   `RESP_AUTH_REQUIRED` with no event to explain it, and under Phase 4 those refusals
   feed the auth-abuse counter until the client happens to re-authenticate. A dropped
   link is an unambiguous signal, it frees the exclusive slot (CONNECTION_POLICY R1)
   from a demonstrably broken client, and it makes the watchdog's semantics identical
   to the idle and auth-abuse drops. The client reconnects and restarts the transfer —
   which it had to do anyway, since the transfer state is gone either way. Note the
   watchdog is **origin-agnostic** — both branches test transfer state, not origin
   ([display_service.cpp:592,609](../src/display_service.cpp)), so a LAN transfer can
   time out too — which is why step 10 dispatches on the owner's transport: a
   timed-out LAN owner loses its socket, not an unrelated BLE handle.
3. **Teardown is no longer selective.** The two branches previously cleaned one
   transfer half each; the abort clears all transfer state. Under one-client
   exclusivity the halves are not independently owned, so this is a simplification
   rather than a loss.

The cost is that a legitimately slow-but-progressing transfer, cut off by the
from-START duration bound, now also loses its link and session. That is acceptable
because it must restart regardless, and because the real defect there is the
duration-vs-stall bound itself, recorded under residual risk.

**Not folded in: the orphaned-pipe healer.** The third branch of
`checkTransferTimeouts()` (`pipeState.active && !pipeState.error && !directWriteActive
&& !partialCtx.active` → `resetPipeWriteState()`) is an *invariant repair*, not a
transfer timeout — it heals an internal inconsistency that should never arise. Dropping
a healthy client's link and session over an internal bookkeeping error would be
disproportionate. It stays as it is, and stays a plain `resetPipeWriteState()`.

### Wire `serviceBleDisconnectCleanup` through it

`serviceBleDisconnectCleanup` ([main.cpp:388-423](../src/main.cpp)) already defers
correctly and already checks `ownerStillUp`. Phase 2 routes its teardown body through
`abortToKnownState(..., dropLink=false)` (the link is already gone) so the disconnect
path and the abort path can never drift. Keeping two separate teardown paths is
exactly how the direct-write watchdog once tore down a panel while leaving its pipe
session live — a bug this branch already fixed in `checkTransferTimeouts`, and one a
single shared routine prevents from recurring.

**No special nRF deferral is needed for the session clear.** An earlier draft called
for deferring `clearEncryptionSession()` on nRF to avoid a `memset(session_key)`
racing an inline `aes_ccm_decrypt`. That race does not exist in the current
architecture: nRF's write callback only *enqueues*
([ble_transport_nrf.cpp:148-156](../src/ble_transport_nrf.cpp)); all decrypt and
dispatch happen on the loop task in `serviceBleRx()`
([main.cpp:513](../src/main.cpp)), and `serviceBleDisconnectCleanup` is already
loop-task. The abort — session clear included — runs on the loop task, never
concurrently with a decrypt. No `nrfSessionClearPending` machinery.

### Verification

Disconnect mid-direct-write, mid-partial, mid-pipe, mid-chunked-config-write, and
mid-refresh (WARM survives); assert every flagged state is clean afterward, touch is
resumed, crypto cleared, **both rings reset**; assert a second transfer starts clean.
Deep sleep entered mid-transfer wakes with no residue (the R7e row 3 caller).

The frame tag (requirement 6): frames queued by a departing instance — including one
written *during* the teardown window, after the step-9 ring reset — never dispatch
once the token is released, and never stamp the new owner's activity clock; a
reconnecting client (fresh epoch, possibly the same handle) starts with a ring whose
stale frames are dropped at dispatch, with the drop counted and visible in the log.

Callback-boundary mechanisms, each checkable before any Phase 3 policy exists: a second
central's writes are dropped at the callback while the token is held; **its subscribe
does not disturb the incumbent's notify state**; and **it receives no notifications at
all** — the last is the live-leak fix and wants a sniffer or a second bleak client
reading, since a passing incumbent proves nothing about what leaked.

The clock: `linkMsSinceOwnerCommand()` is 0 when unowned, ages only on true silence, is
**not** refreshed by a malformed or unknown-opcode frame (the R4 correction — send junk
that `bleRxQueuePush` happily accepts and confirm the clock keeps running), and is
re-stamped across a refresh so a client engaged either side of a ~16 s refresh is never
dropped.

Host-buildable parts get unit tests: the `linkClaim`/`linkRelease` state machine on the
full triple; epoch discrimination — a claim carrying a reused handle with a new epoch
must not match the incumbent, and a release carrying a stale epoch must not release; and
the claim CAS under contention — two racing claims (host threads suffice) must end with
exactly one owner and one contender. Build all envs.

Three seam-specific bench checks a build cannot cover. **The drop actually drops:** call
the seam from the loop task on both nRF and ESP32 and confirm the link goes down on a
scanner or the client — the 0x09 trap above is precisely a case where the code looks like
it worked, so "it compiled" proves nothing. **The drop waits, on the right predicate:** instrument
`bleDropAndWait()` and confirm it observes the owner's instance-table entry go down
before returning on a live peer, that the token is still held throughout — the R3a
ordering is invisible from outside — and that it returns promptly with a refused
contender still attached, the case where the aggregate `connectedCount()` would have
sat out its full bound. **The reason log is honest:** a real client disconnect logs a
sensible HCI reason, and a NimBLE host-layer reason now logs as `0x0xx` rather than
masquerading as an HCI code.

---

## Phase 3 — Connection-exclusivity policy + idle drop

**Goal:** the *policy* on top of Phase 2's mechanisms — refuse any contender while the
slot is held, and reclaim the slot from an incumbent that has gone silent. Phase 2
already makes a second BLE central harmless (its writes, subscribes and notifications
are filtered, and it cannot own the token); Phase 3 makes it *clean* (actively
disconnected) and closes the idle-link hole. It consumes the instance table, the owner
token and `linkMsSinceOwnerCommand()` — all Phase 2 — and adds no new transport state.

**Phase 3 is CONNECTION_POLICY R7 made executable.** The permutation tables there (7a
admission, 7b disconnect, 7c idle, 7d ordering, 7e terminal) are normative and are not
restated here; this phase says where each is enforced and what changes in the tree.
Any combination the tables do not list is a specification gap to take back to the
policy, not implementer's discretion.

### The governing decision: admission never evicts

**A contender is always refused while the slot is held. Reclaiming a slot is the job
of the idle timeout alone, never of the accept path.** These are two independent
mechanisms and this plan deliberately keeps them that way.

An earlier draft made admission a three-way rule (refuse if the incumbent is
transferring or young-idle; *evict* it if idle past a threshold, then admit the
newcomer). That is rejected. What it bought — a faster reclaim when a stale link
lingers — is not worth what it cost:

- **It made an incumbent's fate depend on whether someone else happened to knock.**
  The same idle client is kept or killed for reasons it cannot observe, which is
  hard to reason about and harder to test.
- **It needed a whole extra threshold** (evict-idle age) that this plan's own
  residual-risk list already flagged as the one requiring the most conservative
  tuning, since too aggressive a value refuses a legitimate reconnect.
- **It put a multi-step teardown at a stack-event boundary** — disconnect incumbent,
  `abortToKnownState`, release token, then let the newcomer claim — with the newcomer
  already connected throughout. Pure refusal never touches incumbent state at all.

The cost accepted in exchange is that a returning client waits out the idle timeout
rather than ~10 s. That cost is smaller than it looks, and it differs by transport:

- **BLE: mostly absorbed below us.** The firmware never sets a supervision timeout —
  it takes whatever the central negotiates (commonly ~4–6 s). So an incumbent that is
  genuinely *gone* is reaped by the link layer without firmware involvement, and the
  idle timeout only has to handle a client that is alive and silent. Refusing a
  contender in *that* case is arguably the correct answer anyway.
- **LAN: genuinely dependent on the timeout.** TCP has no supervision timeout; a
  half-open socket persists indefinitely without keepalives. `OD_LAN_READ_TIMEOUT_S`
  (30 s) is the only reclaim path, which is precisely why LAN already has one.

### Within-pass ordering (R7d) — normative, not incidental

**Fix the order first, because everything below depends on it.** The current loop order
is `serviceBleEvents()` → BLE RX → deferred disconnect cleanup → LAN accept/read
([main.cpp:624](../src/main.cpp)), and connect and disconnect flags are consumed
connect-first regardless of actual arrival order
([main.cpp:461-471](../src/main.cpp)) — exactly the ambiguity R7d removes. Without a
stated order, two conforming implementations pick different winners. Within one pass:

1. **Owner disconnects** (7b) — the abort first, whose *final* step releases, so a
   slot freed this pass is available to an admission decision in the *same* pass.
   Never release before the abort: a claim CAS can succeed the instant the word is
   zeroed, and an abort still running after that would tear down the new session.
2. **Contender refusal, and the LAN accept** (7a). Admission itself is the hook-side
   CAS: for BLE it already happened — or failed — in the connect callback, so this
   step only *refuses* live instances whose CAS failed; the LAN accept runs here
   because the loop is its earliest hook, and its claim is the same CAS. No loop-side
   rule picks a winner between transports; the word does.
3. **Inbound traffic**, which stamps the activity clock.
4. **Idle timeout** (7c) — last, so traffic parsed in step 3 counts. This is what
   satisfies R4's ordering constraint for LAN, where inbound bytes may be sitting in the
   socket when the deadline is evaluated.

**But the authoritative arbitration point is the earliest transport hook — the BLE
connect callback and the LAN accept — not the loop.** Fixed loop ordering cannot
reconstruct true cross-transport arrival order: a BLE connect during a refresh and a LAN
socket queued in the listen backlog are not comparable by the time `loop()` resumes. The
loop order resolves *ties within a pass* only; the claim itself must be atomic at the
callback — mechanically, the one-word owner CAS from Phase 2 — and where the two
disagree the callback wins. Do not build correctness on step order alone.

### Enforcement

- **ESP32 admission — refuse, unconditionally.** Scanning the instance table, any live
  entry whose `(handle, epoch)` is not the owner's is a contender: `ble.disconnect(its
  handle)` and stop. Do **not** raise `s_disconnectCleanupPending`, do **not**
  `linkRelease()`, do **not** inspect the incumbent's state at all — no `transferActive()`
  test, no idle-age test. The incumbent's session is untouched by construction rather than
  by a guard that could be got wrong. Note this is a **table scan, not an event handler**
  (Phase 2 requirement 5): a refusal missed because two connects coalesced self-corrects on
  the next pass, where an event-driven version would leak the contender permanently.
  Because refusal is idempotent and inert, re-refusing an entry that is already tearing
  down costs nothing. nRF gets the same refusal free from `begin(1,0)`; this bullet is the
  ESP32 analogue. **The scan never admits**: admission is one CAS at each instance's own
  connect hook, decided once and never revisited (7a rows 9–10) — a contender whose
  refusal is still pending when the slot frees stays refused, and the freed slot goes to
  the next *new* instance. Racing arrivals, including a BLE connect against a LAN accept,
  are serialized by the word, not by scan or loop order.

  **7a row 4 is the case to test.** A contender reusing the incumbent's handle after a
  stale link must be refused, and the *only* thing distinguishing it from the incumbent is
  the epoch — which is why R2 allocates one for every instance, admitted or not.
- **Proactive idle drop — the sole reclaim mechanism.** Since admission never evicts,
  this is the *only* way a held slot is ever released short of the client leaving. A
  loop-serviced `serviceIdleTimeout()`: if the slot is owned **by BLE**, no refresh is
  in progress, and `linkMsSinceOwnerCommand() > OD_BLE_IDLE_TIMEOUT_MS`, call
  `abortToKnownState(dropLink=true)` — its step 10 drops the owner's link and waits for
  link-down, its step 11 releases; the R3a order, all within the one pass (7c row 1).
  This is the BLE side of R4's each-transport-its-own-timer rule: LAN's reclaim stays
  its existing `OD_LAN_READ_TIMEOUT_S` path (whose teardown routes through the abort
  per R6). An `#ifndef`-guarded define in the file that services it, not a wire/config
  field.

  **There is no `!transferActive()` gate**, per
  [CONNECTION_POLICY](CONNECTION_POLICY.md) R4, which supersedes an earlier draft of
  this bullet. An in-flight transfer confers no protection: a client that goes silent
  *during an upload* is precisely the case that wedges the device, and a transfer gate
  would exempt exactly it. Idleness excludes only refresh-in-progress — via the
  `endRefresh()` re-stamp Phase 2 builds, since `loop()` is blocked throughout a refresh
  while wall-clock time passes (7c row 3). The from-START watchdog remains the backstop
  for the remaining case: a transfer that keeps sending recognised commands but never
  ends.
  - *Default: `OD_BLE_IDLE_TIMEOUT_MS = 120000` (120 s).* Set deliberately generous,
    and note this is **double** an earlier draft's 60 s — the reasoning inverted when
    R4 landed, so the direction of the change is not an oversight:

    - While the idle drop was gated on `!transferActive()`, the timeout could only
      ever kill an *idle* client, so erring short was cheap and a shorter value
      shortened the lockout.
    - R4 removed that gate. The timeout can now terminate an **in-progress upload**
      whose client has gone quiet, so erring short no longer costs a stale session —
      it costs a legitimate transfer. Conservative is now the safer direction.

    The cost is bounded and falls only on one case: a returning client waits up to
    120 s if a stale-but-*alive* incumbent holds the slot. An incumbent that is
    genuinely gone is reaped by the link layer in ~4–6 s (the firmware sets no
    supervision timeout, so the central's negotiated value applies), so the 120 s
    lockout never applies to a crashed or out-of-range client.

    **120 s is settled.** It is a chosen value rather than a measured one, and it is not
    gated on a measurement: implementation proceeds on it. What remains is *drift
    detection*, not verification — the `py-opendisplay` assertion below fails if a client
    change ever pushes legitimate inter-command silence toward 120 s, which is the same
    treatment every other threshold here gets. Record the reasoning, not a pending
    confirmation, in the comment on the define.
  - *Why it cannot live where its LAN cousin does, and what that costs.*
    `OD_LAN_READ_TIMEOUT_S` is **not** a local tunable: it is defined at
    [opendisplay_protocol.h:984](../include/opendisplay_protocol.h) and documented at
    `:84` and `:945` as a client-visible contract ("the server drops a client only
    after `OD_LAN_READ_TIMEOUT_S` with no traffic"). Its home is the wire header
    because the client is entitled to know the number. The hard constraint forbids
    touching that header, so the BLE timeout is forced local — deliberately
    asymmetric with the LAN one, and invisible to clients except through the
    client-side CI assertions below. That is the accepted trade, not an oversight: a
    wrongly-dropped BLE client reconnects, so the cost of the client not knowing the
    exact number is bounded. If the BLE timeout ever needs to be genuinely
    client-visible, that is a wire change and goes through `../opendisplay-protocol`
    first — at which point it belongs in the protocol header beside its LAN cousin,
    not in firmware.
  - *Deep sleep:* the idle drop leaves `lastActivityMs` and the deep-sleep quiet window
    alone — this is a *link* drop, not a sleep decision. After it `connCount` falls to 0,
    `pollActivity` stops re-stamping, and the existing idle/deep-sleep path takes over.
    (Separately, deep sleep itself becomes an abort caller — R7e row 3, Phase 2. That is
    a change to the *sleep* path, not to this one.)
- **LAN, one consistent model — including LAN-vs-LAN.** The token is connection-level,
  so a LAN accept while *any* transport owns the slot is refused (`incoming.stop()`),
  and symmetrically a BLE connect while LAN owns is refused. `handleWiFiServer` accept
  ([wifi_service.cpp:869-877](../src/wifi_service.cpp)) gains the token check and
  `linkClaim({OWNER_LAN, 0, epoch})` — the same CAS the BLE connect callback uses, so
  cross-transport arbitration is the word itself, not loop ordering.

  **The claim happens at TCP accept, before the TLS handshake** (R7a row 2). The
  handshake is driven incrementally across later loop passes
  ([wifi_service.cpp:905-920](../src/wifi_service.cpp)), so deferring the claim until it
  completes would leave the slot free for a BLE connect or a second socket in the
  meantime — a race the accept-time claim closes. Three consequences follow, and each is
  a real code change on that path:

  - A second accept *during* the handshake is refused (rows 5/7 apply) — it does not get
    to displace a half-established session.
  - **TLS handshake failure is an owner disconnect**: the existing
    `disconnectWiFiServer()` at [wifi_service.cpp:918](../src/wifi_service.cpp) must now
    run R6's abort and release the token, or a failed handshake strands the slot until
    the idle timeout.
  - **Handshake traffic is not activity.** The idle baseline starts at handshake
    completion, which the code already stamps ([wifi_service.cpp:910](../src/wifi_service.cpp)).

  **No separate handshake deadline is added, deliberately.** An earlier reading required
  one. It is unnecessary: because the baseline does not start until the handshake
  completes, a handshake that never finishes leaves the clock at its accept-time stamp and
  the existing 30 s `OD_LAN_READ_TIMEOUT_S` drop fires. A dedicated deadline would only
  tighten that window — not worth a second tunable until something shows 30 s is too slow.

  **This is a behaviour change for LAN, not just a new cross-transport check.** Today
  that path is unconditional last-in-wins: a second TCP accept tears down TLS, clears
  crypto and stops the previous client, with no test of what it was doing. Under the
  rule above it becomes a refusal, which matters more on LAN than on BLE because TLS
  bypasses app-layer auth by design — so today *any* host on the network can kill an
  in-flight display push simply by opening a socket, with no credentials. Refusing
  closes that.

  **A pre-existing bug on the same path, fixed by the same change.** The accept-side
  eviction clears TLS/crypto but never calls `requestTransferSessionCleanup()` — unlike
  `disconnectWiFiServer()`, which does ([wifi_service.cpp:807](../src/wifi_service.cpp)).
  So an evicted client's in-flight direct-write/pipe/partial state stays live, and
  because both clients are `ORIGIN_LAN`, `frameOwnsSession()` does not stop the *new*
  client's frames from landing in the *evicted* one's transfer — the same class of hole
  as the ESP32 multi-central case. Making the path refuse rather than evict removes the
  bug by removing the eviction; nothing is left needing the cleanup call.

  LAN's reclaim path is unchanged in *mechanism* and remains `OD_LAN_READ_TIMEOUT_S`
  ([wifi_service.cpp:952](../src/wifi_service.cpp)), which already drops an idle client
  after 30 s and is already ungated by transfer state — so LAN needs no new timer, only
  R4's two semantic corrections. `lastLanActivityMs` is close to a true activity clock
  already: stamped at connect, TLS-handshake completion, bytes read and frame dispatch
  ([wifi_service.cpp:886,910,946,972](../src/wifi_service.cpp)) and — unlike BLE's
  `lastActivityMs` — never re-stamped merely for being connected.
  - *The `got > 0` stamp must go.* `:946` stamps on **any bytes read**, not on a
    recognised frame, so a plain-mode flooder defeats both the 30 s read timeout and any
    policy built on that clock. This is the same defect the BLE clock had in intake form,
    and Phase 2 fixes both at once: stamping moved to `imageDataWritten()`, which LAN also
    dispatches through, so LAN inherits "recognised command from the owner" without a
    second implementation. Delete the `:946` stamp rather than adding a parallel one —
    two clocks for one rule is how they drift.
  - *The refresh exclusion applies to LAN too.* `endRefresh()` re-stamps the owner's
    clock whoever the owner is; a LAN client mid-push across a ~16 s refresh is exposed to
    exactly the same spurious drop as a BLE one.
  - *The 30 s constant is settled and unchanged.* `OD_LAN_READ_TIMEOUT_S` satisfies R4 as
    it stands: it is already ungated by transfer state, which is the substance of the
    rule, and R4 governs only its **semantics** — which stamp counts as activity, and the
    refresh exclusion — both firmware-local and both fixed above. Its **value** is a
    wire-header contract and out of bounds here, so "does 30 s satisfy R4" is not an open
    question but a closed one: yes, with the two stamping corrections applied. The
    asymmetry with BLE's 120 s is deliberate and follows from where each constant is
    allowed to live.

### One thing to get right (easy to assume wrong)

The ESP32 central cap **cannot** be forced to 1 with a `-D` build flag — the
`CONFIG_BT_NIMBLE_MAX_CONNECTIONS = 3` in the precompiled `sdkconfig.h` wins, and a
local override is silently inert. Exclusivity must be enforced in firmware, as above,
not by config. R1 is therefore phrased in terms of **admission, not physical links**,
and that is not a weakening: NimBLE establishes a second central's link *before* it
calls `onConnect` ([ble_transport_esp32.cpp:81-93](../src/ble_transport_esp32.cpp)) and
the server API has no pre-connection filter, so a transient second *physical* link
necessarily exists while it is being refused. What R1 constrains is what is
*serviceable*; what R3's callback-side filtering constrains is what that transient link
can touch, which is nothing. An implementation that reports "two links were briefly up"
is conforming; one where the second link moved any shared state is not.

(`serviceBleDisconnectCleanup`'s `ownerStillUp` guard is **already** unconditional as of
PR `#132`, [main.cpp:404-409](../src/main.cpp) — Phase 3 adds policy, not that
restructuring.)

### Verification

Admission (7a): two centrals against one ESP32, second always refused whatever the
incumbent is doing; **row 4** — a contender reusing the incumbent's handle after a stale
link is refused, which is the epoch's whole justification and the one case a
handle-only implementation passes by accident; **row 10** — a contender still connected
when the incumbent departs is *not* admitted: it stays refused and the slot goes to the
next fresh connect; BLE⇄LAN arbitration both directions; a
second LAN client refused rather than evicted, with the first's transfer surviving.

Refusal is inert (R3), the property most likely to be got wrong since refusal and
teardown sit in the same handler: a refused stranger's connect **and** disconnect leave
the incumbent's transfer, crypto session, notify state and panel power untouched — check
the `esp32-N4` no-WiFi path specifically. Two coalesced connects (both arriving inside one
refresh block) still end with both contenders refused, which is the table-scan property
rather than an event-handler one.

Idle drop (7c): a client that connects, authenticates and idles past the timeout is
dropped; a fresh client gets the full window before its first command (the init fix); a
streaming client is not dropped; a keepalive-sending client is not; **a client that goes
silent mid-upload IS dropped** — the R4 case, and the one an earlier `!transferActive()`
gate would have exempted; a client engaged either side of a ~16 s refresh is **not**
dropped (the `endRefresh()` re-stamp). After a drop the device returns to
advertising/idle, and the slot is claimable by a new client in the same pass a disconnect
freed it (R7d step 1 before step 2).

---

## Phase 4 — Auth-abuse disconnect

**Goal:** drop the link after a bounded run of BLE commands that never authenticate,
so an unauthenticated peer cannot hold the exclusive slot (on ESP32, the *only* slot
the owner token would otherwise hand it) indefinitely.

### Design (fresh — a prototype exists off-branch but is not adopted wholesale)

`feat/nonce-replay-and-auth-guard` carries `fbc7ab2`/`b4fafb5`, which implement this
but (a) drop the link **inline** on nRF — flagged as loop-starving — and (b) place
two `serviceBleAuthAbuseDisconnect()` call sites in the per-target loop arms that
`#132` then merged, so they no longer have a home. Reuse the *counter* logic; drop
the placement.

- **Count only BLE.** A per-session counter of consecutive commands answered with
  `RESP_AUTH_REQUIRED`, incremented **only when `g_commandOrigin == ORIGIN_BLE`**.
  The generic auth gate at [communication.cpp:584,591](../src/communication.cpp) and
  the config-write sites at `:410,472` are also reachable via the LAN-TLS bypass,
  where app-layer auth is intentionally unnecessary; counting those without the
  origin gate would let LAN-TLS traffic increment a counter that disconnects **BLE**.
  Reset to 0 on any authenticated command.
- **Threshold 10** (justify against the client's legitimate handshake, which
  authenticates within one exchange — 10 is generous). Overflow raises
  `s_authAbuseDropPending`; a loop-serviced `serviceBleAuthAbuseDisconnect()` handles
  it. One placement, both targets — the whole reason Phase 2's seam and the unified
  loop exist. Per the threshold discipline above, the count lives `#ifndef`-guarded in
  `communication.cpp` beside the auth gate that increments it, and
  `OD_AUTH_ABUSE_FLUSH_MS` beside the servicer that enforces it — not in a shared
  header.
- **Best-effort delivery of the final `FE` before dropping — a real barrier, not one
  flush, and honestly not a guarantee.** The last `00 xx FE` *should* reach the client
  so it is not dropped without a stated reason. A single `serviceBleTx()` then
  disconnect does not even get the frame to the stack reliably: TX deliberately
  retains an entry on mbuf backpressure or a missing CCCD
  ([command_queue.cpp:190](../src/command_queue.cpp)), and the final response may not
  even enqueue if the 10-slot ring is full. So the drop is gated on a bounded barrier:
  `serviceBleAuthAbuseDisconnect()` drains TX each loop pass and proceeds only once the
  TX ring has drained the `FE` **or** a bounded deadline (`OD_AUTH_ABUSE_FLUSH_MS`,
  ~500 ms) elapses — then it drops regardless, so a wedged/un-draining client cannot keep
  the abuser attached. **An empty ring proves stack acceptance, not receipt**: the ring
  advances when `notify()` returns true ([command_queue.cpp:190-199](../src/command_queue.cpp)),
  which means NimBLE queued an *unacknowledged* notification — nothing confirms it went
  on air. So after the drain, the servicer dwells
  `min(remaining deadline, one negotiated connection interval + margin)` before
  dropping. The interval is the central's choice, not ours; today both targets read the
  negotiated value only inside link-tune *logging*
  ([ble_transport_esp32.cpp:74-77](../src/ble_transport_esp32.cpp),
  [ble_transport_nrf.cpp:81](../src/ble_transport_nrf.cpp)) and `BleTransport` exposes
  no accessor — so Phase 2's transport work adds one (`connIntervalMs(handle)`, or a
  value published at the link-tune callback), with a conservative fallback
  (`OD_AUTH_ABUSE_DWELL_FALLBACK_MS`, ~50 ms) for when no negotiated value has been
  seen. **Any dwell truncated by the deadline — including to zero — is the best-effort
  case and may forfeit the `FE`**; only a drain early enough for the full
  interval-plus-margin dwell makes on-air delivery *expected* rather than hoped for.
  That is as far as best-effort can go without an indication — a wire change this plan
  is forbidden.
  Then `abortToKnownState(dropLink=true)`, whose step 10 is itself the R3a bounded
  wait for link-down before the token is released.

  **Two bounded waits in sequence, and they compose rather than conflict** — this is the
  shape CONNECTION_POLICY R3a predicts. The flush barrier runs *before* the abort because
  the abort's step 2 deliberately skips the client NACK when `dropLink` (the link is about
  to go); asking the abort to also hold the link open for a response would put two
  contradictory jobs in one routine. So the ordering is: drain the `FE` (bounded) →
  `abortToKnownState(dropLink=true)` → request termination and wait for link-down
  (bounded) → release. Both waits are bounded, both proceed on expiry, and neither
  treats expiry as a failure — but their mechanics differ: the flush barrier spans
  loop passes (`serviceBleAuthAbuseDisconnect()` drains TX each pass), while the R3a
  wait inside the abort ticks on its plain bounded `delay()`.

### Depends on

Phase 2 (the seam and its R3a wait, `abortToKnownState`, the owner token) and Phase 3
(it slots into the same admission/idle policy layer).

**And Phase 3 depends on *it* for one case**, which is worth stating because the division
is easy to get backwards. Under R4 the activity clock stamps a *recognised* command
before the auth gate — it has to, or a client could not complete a handshake without
racing the clock. So a peer that floods **recognised but never-authenticating** commands
keeps its clock fresh and the idle drop never fires on it. That peer is Phase 4's job,
not Phase 3's. The two together are exhaustive: garbage that is not a recognised command
never stamps the clock and is dropped by the idle timeout; recognised commands that never
authenticate are dropped by the counter.

### Verification

A BLE peer sending N unauthenticated commands is dropped at the threshold with the
`FE` observed **on air** first *when the drain and the full interval-plus-margin dwell
both complete inside the deadline* (a sniffer, necessarily — ring state proves only
stack acceptance, and the barrier is the subtle part); a deadline-truncated dwell may
forfeit the `FE` by design;
the drop still happens within the deadline if the client stops reading; a legitimate
client authenticating on its first exchange is never dropped; the counter resets
across a good command; **LAN-TLS traffic never increments it**; on nRF the drop is not
loop-starved.

---

## Verification model

Every phase distinguishes two states, because "the code merged" and "the gap
closed" are not the same claim. Phase 1 is the live example: it is shipped and
host-tested, yet its entire hardware matrix is unrun — landed, not closed.

- **Landed** = builds on all envs + host tests pass. A phase may merge here.
- **Closed** = its companion HIL script has passed on **both** an nRF and an ESP32
  board. The plan tracks a phase as open until then.

The HIL scripts are the executable form of each Verification section, under
`tests/`, pytest driving a real device through `py-opendisplay`/bleak
(`tests/serial_stall_test.py` is the existing template):

| Phase | Script | Asserts |
|---|---|---|
| 1 (retroactive) | `test_nonce_gap.py` | a transfer survives a forced >256 forward counter gap; a nonce-dropped `0x0081` frame is repaired by the client's SACK path and the upload completes |
| 2 | `test_abort_state.py` | disconnect mid-{direct, partial, pipe, chunked-config, refresh}; every flagged state clean afterward, touch resumed, crypto cleared, both rings reset, WARM panel survives; a frame written by the departing owner during the teardown window never dispatches after release (the requirement-6 tag); a buzzer melody and LED pattern in flight at the abort **keep playing to completion**; deep sleep entered mid-transfer wakes with no residue (R7e row 3) and **silences a playing buzzer/LED without waiting for it** — with the pin confirmed quiet through sleep, not merely the state flag cleared; power-latch off still sounds its shutdown chirp; the drop holds the token until link-down (R3a) |
| 2 | `test_link_isolation.py` | a gatecrasher's writes are dropped at the callback while the token is held; its subscribe does not move the incumbent's notify state; **it receives no notifications** — the live-leak fix, needs a second reader or a sniffer; `linkMsSinceOwnerCommand()` is 0 when unowned, ages on true silence, is **not** refreshed by malformed or unknown-opcode frames, and is re-stamped across a refresh; a stale-epoch frame left queued across a reconnect neither dispatches nor stamps the clock |
| 3 | `test_exclusivity.py` | two centrals against one ESP32 → second always refused, incumbent idle or transferring; **7a row 4** — a contender reusing the incumbent's handle after a stale link is refused (the epoch case a handle-only build passes by accident); **7a row 10** — a contender still connected when the incumbent departs stays refused, and the slot goes to the next fresh connect; two connects coalesced inside one refresh block still end with both refused (the table-scan property); a second LAN client is refused, not evicted, and the first's transfer survives; BLE⇄LAN arbitration both directions; refused-stranger connect **and** disconnect do not tear down the incumbent (the `esp32-N4` no-WiFi path) |
| 3 | `test_idle_drop.py` | a fresh silent client survives its first window then is dropped; a streaming client is not; a keepalive-sending client is not; **a client silent mid-upload IS dropped** (the R4 case a transfer gate would exempt); a client engaged either side of a ~16 s refresh is not; a LAN flooder sending unrecognised bytes is dropped at 30 s despite the traffic; the device returns to advertising after the drop |
| 4 | `test_auth_abuse.py` | N unauthenticated BLE commands → drop at the threshold with the `FE` on air first when the drain and full dwell complete inside the deadline (sniffer — ring state proves only stack acceptance); drop still occurs within the deadline if the client stops reading, forfeiting the `FE` by design; a first-exchange auth is never dropped; the counter resets across a good command; LAN-TLS never increments it; on nRF the drop is not loop-starved |

**Threshold drift is caught in the client's CI, not ours.** These thresholds
assume specific `py-opendisplay` behaviours (handshake authenticates
within one exchange; retransmits carry fresh, higher counters; keepalive cadence).
Add an assertion of each to `py-opendisplay`'s test suite, so a client change that
would invalidate a firmware constant breaks *there* — the same move already used
for the `0x04`-NACK reasoning recorded in `sendPipeNack()`. Every
threshold-triggered drop also logs at WARN with the measured value, so field tuning
has data rather than guesses.

## Cross-cutting: what still has no watchdog

Two distinct gaps, both out of scope, both named here rather than assumed away.

**A stuck refresh (CONNECTION_POLICY R5).** R4 excludes refresh from idleness — the
`endRefresh()` re-stamp is precisely that exclusion — so **a refresh that never completes
is not caught by the idle timeout, by construction**. That is a deliberate trade, not an
oversight: without the exclusion, an actively engaged client is dropped the instant a
~16 s refresh ends. But it means the exposure moves rather than closing, and on FastEPD
targets it is total: `fastepd_wait_refresh()` ignores its timeout argument outright
([display_fastepd.cpp:277-280](../src/display_fastepd.cpp)), so the naive "panel never
signals done" case is fully unbounded. The `bb_epaper` path is bounded at 60 s
([display_service.cpp:803-831](../src/display_service.cpp)), which is a bound but not a
useful one for a session policy.

R5 names the shape of the fix and this plan does not build it: no loop-serviced watchdog
can observe a stuck refresh, because `loop()` is blocked for its entire duration, so it
needs an independent timebase (hardware WDT fed from `loop()`, a timer ISR, or a separate
task); recovery must run from a safe context, which realistically means an MCU reset
rather than panel/SPI teardown from an ISR; and there is no refresh start timestamp in the
tree, so the watchdog must add one. The one thing Phase 2 contributes toward it is
`endRefresh()`: a single helper both bracket sites call is the natural place a future
start/stop timestamp pair lands.

**Loop liveness.** None of Phases 2–4 add a loop-liveness monitor either. A `loop()`
genuinely wedged inside a non-yielding operation is still uncaught on nRF (no watchdog)
and on ESP32 (`loop()` unsubscribed from the TWDT). The realistic mitigation — subscribe
`loop()` to the ESP32 TWDT and add an nRF hardware WDT fed from `loop()` — is a separate
effort whenever it is
taken up; it is the true "supervisor," and it is none of the four phases here.

## Deliberately not changed

- No wire/protocol/config-schema change (hard constraint).
- No `include/opendisplay_protocol.h` or `include/opendisplay_structs.h` edit — which is
  what forces `OD_BLE_IDLE_TIMEOUT_MS` to be firmware-local while `OD_LAN_READ_TIMEOUT_S`
  stays a client-visible contract in the wire header.
- **No per-connection command queue.** CONNECTION_POLICY's hard constraint: one RX ring
  and one TX ring, shared by all transports. The instance table this plan adds is
  metadata only (~8 bytes per slot); nothing that holds frames is ever replicated per
  connection. Callback-side write filtering (Phase 2 requirement 1) is what makes that
  possible — with only the owner's frames entering the ring there is never a second
  client's traffic to separate. The requirement-6 identity tag adds four bytes per
  slot — per-frame metadata in the one ring, never a second ring — and supersedes
  `bleRxQueueDiscardTo`'s boundary flush, which Phase 2 retires.
- The from-START transfer watchdog stays as the backstop; Phase 3's idle drop is
  additive, not a replacement.
- The orphaned-pipe healer in `checkTransferTimeouts()` stays a plain
  `resetPipeWriteState()` — it repairs an internal invariant, and dropping a healthy
  client's link over a bookkeeping error would be disproportionate.
- The nonce subsystem (Phase 1) is not reopened.

## Residual risk (honest list)

These are the gaps this plan **cannot** design away, distinct from the ones it now
tracks as work (HIL verification, and the client-side drift assertions — those have owners
and exit criteria above, so they are no longer "risk"). Threshold *selection* is no longer
on either list: every value except the R3a wait bound is settled above.

- **No loop-liveness watchdog** (see the watchdog section above). A `loop()`
  wedged inside a non-yielding operation is still uncaught on nRF, and a true hard
  fault is unrecoverable there. Deliberately left as a separate future effort.
- **No refresh watchdog, and the exposure is now *explicit* rather than latent**
  (CONNECTION_POLICY R5). R4's refresh exclusion is a deliberate hole in the idle
  timeout: a refresh that never completes is not caught, and cannot be, since the clock
  is re-stamped at the transition. On FastEPD targets there is no bound anywhere in the
  path. This plan makes the situation no worse — the exclusion only ever *delays* a drop
  — but it does make the idle timeout unable to serve as an accidental backstop, which
  before R4 it arguably was. R5 is the named owner of the gap; it is not scheduled here.
- **The R3a wait can expire with the link still up.** The bound is sized for a few
  connection intervals, so an unresponsive peer can outlast it. This is the least
  consequential item on the list because expiry is an early exit rather than a failure:
  the abort runs regardless, and the stale link is inert by construction — its writes are
  filtered as non-owner and its late disconnect is inert on stale epoch. The residual
  exposure is a physical link lingering until the link layer reaps it at ~4–6 s, holding
  no slot and touching nothing.
- **Thresholds remain heuristics even though they are settled.** *Settled* means decided
  and not gated on a measurement — it does not mean proven. The mandatory
  client-behaviour comment on each define, plus the client-side assertions, make the
  assumptions legible and drift-detectable, but the numbers are still judgement calls
  against a client that can change. The auth-abuse drop is self-limiting (a
  wrongly-dropped client reconnects). The one that carries real weight is
  `OD_BLE_IDLE_TIMEOUT_MS` (120 s): with admission refusing rather than evicting, it is
  the sole path by which a held slot is ever reclaimed, and with R4 removing the transfer
  gate it can also terminate a live upload. It is set generously precisely because the
  second error is the worse one — but that trade is a judgement, and it is the number to
  revisit first if field behaviour disappoints. The residual exposure it accepts is a
  returning client waiting up to 120 s behind a stale-but-alive incumbent.
- **A wedged transfer is now mostly caught, but not entirely.** CONNECTION_POLICY R4
  removed the `!transferActive()` gate, so the common wedge — a client that starts a
  transfer and *goes silent* — is dropped by the idle timeout like any other silent
  client. What remains uncaught is narrower: a client that keeps sending recognised
  commands while its transfer never completes. That one is still bounded only by
  `TRANSFER_WATCHDOG_MS`, because the from-START watchdog is a total-duration bound
  rather than a stall timeout. The full fix is a genuine stall timeout gating on
  *transfer active **and** progressing*, using the same activity clocks Phase 2 and LAN
  already provide; it is a candidate for the next phase after this plan, alongside the
  loop-liveness watchdog.
- **"Closed" depends on hardware nobody has run yet.** The verification model makes
  this explicit rather than papering over it: until the HIL scripts pass on both an
  nRF and an ESP32 board, every phase — including Phase 1 — is landed, not closed.
