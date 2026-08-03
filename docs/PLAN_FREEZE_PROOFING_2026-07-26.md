# Freeze-Proofing the OpenDisplay Firmware (branch: debug/ble-hardening)

> **Revised 2026-07-26** after adversarial review. The review found 4 Critical defects — two of
> which meant the headline deliverable did not work. All corrections are folded in below and
> tagged `[C1]`…`[X7]`. Full review: `docs/FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md`
> (**first action on implementation: move it there from
> `~/.claude/plans/create-a-comprehensive-plan-majestic-hamster-agent-a0d75f8717fc20eb0.md`** —
> plan mode blocked writing it into the repo).

## Context

Field failures: during PIPE_WRITE uploads from Home Assistant, lost ACKs / blind retransmits leave the device **frozen or unresponsive**. Investigation traced four wedge mechanisms plus several unbounded waits:

1. **Nonce replay-window overrun** — the client burns a nonce per transmission (incl. retransmits); losing a full 32-frame window puts the next frame >32 ahead of `last_seen_counter`; rejections accumulate into `integrity_failures >= 3` → `clearEncryptionSession()` mid-transfer. The device then answers everything `0xFE` while `directWriteActive` keeps the panel powered 15 min. Bonus bug: `verifyNonceReplay` commits `last_seen_counter` **before** CCM tag verification ([encryption.cpp:149-155](../src/encryption.cpp)).

   > **Corrected 2026-07-26 after Phase 1 shipped.** Two claims in the sentence above were wrong and are struck:
   > - **"3 rejections (= client's `MAX_PTO`)" is a false coincidence.** `MAX_PTO = 3` yields only *two* probe sends (the client increments and raises at the threshold before sending, `device.py:2721-2726`), and the client aborts the transfer on the **first** NACK, not the third (`device.py:834-838` raises `IntegrityCheckError`, uncaught by the pipe loop at `device.py:2714-2717`). Within one transfer `integrity_failures` plausibly reaches 1, not 3. Reaching 3 needs repeated attempts on the same session.
   > - **The freeze actually reproduced on the bench was not a forward nonce gap at all.** It was a *session-identity divergence*: the client lost its session while the device still believed one was live, and py-opendisplay silently degraded to **unencrypted 230-byte `0x0071` chunks** (`device.py:1916-1929`, `:772-776`; `commands.py:70`). At 232 bytes those frames clear the firmware's short-frame gate and enter `decryptCommand`, where 8 bytes of image data are read as a session id → `NONCE_BAD_SESSION` → fatal NACK, forever. See `PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md` § "What actually happened on the bench".
   >
   > The nonce-gap defect class is real and Phase 1 fixed it. It is simply **not** the mechanism behind the observed field failure, and no baseline capture (Phase 1 Step 5 Test 0) has yet been taken to establish what is.
2. **Pipe fatal-NACK latch** — `sendPipeNack` leaves `pipeState.active=true` forever ([display_service.cpp:2562-2578](../src/display_service.cpp)); `transferActive()` latches → touch dead ([touch_input.cpp:584](../src/touch_input.cpp)), WiFi roam dead; the 15-min watchdog keys on `directWriteActive`, just cleared → nothing bounds it.
3. **Queue overflow** — response ring (10) drops newest on full ([communication.cpp:117-121](../src/communication.cpp)); command ring drops on full ([esp32_ble_callbacks.h:128](../src/esp32_ble_callbacks.h)). The **command** ring is never flushed on disconnect, so stale commands survive. *(The response ring IS drained whenever no central is connected — [main.cpp:307-312](../src/main.cpp) — so stale responses were never the wedge* `[L1]`*.)*
4. **Cross-transport session clobber** — LAN accept/close unconditionally `clearEncryptionSession()` ([wifi_service.cpp:879, :804](../src/wifi_service.cpp)), killing a live BLE session. BLE+LAN can be connected simultaneously today.
5. **Unbounded waits** — `pwrmgmLockTake` infinite spin ([display_service.cpp:401-408](../src/display_service.cpp)); `powerOff` stuck-button loop ([power_latch.cpp:87-90](../src/power_latch.cpp)); FastEPD refresh has no firmware-side bound.
6. **Session lifetime is unbounded and unscoped** — nothing clears `encryptionSession` on a BLE disconnect (only LAN does, [wifi_service.cpp:804](../src/wifi_service.cpp)), so the key and `last_seen_counter` survive into the next connection. The survival is unusable for resumption (a reconnecting client resets its counter to 0 → rejected as out-of-window or as a ring replay) and it enables a real attack: `verifyNonceReplay` exempts `counter_diff == 0` from the replay-ring check ([encryption.cpp:136](../src/encryption.cpp)), so a captured last-frame replayed after the owner disconnects is **accepted and re-executed**. Separately, `session_timeout_seconds` expiry is evaluated inside `isAuthenticated()` on every command ([encryption.cpp:195-199](../src/encryption.cpp)) and so can fire mid-transfer — a deterministic wedge on a long upload, since the client's proactive re-auth is deliberately skipped for the whole pipe stream ([device.py:778-786](../../py-opendisplay/src/opendisplay/device.py)).

**User decisions (confirmed):** software-only supervisor (no hardware WDT — reset state, never reboot; a reboot wipes RTC incl. `displayed_etag` and forces a boot-screen redraw); clear encryption session on BLE disconnect; **encryption is always scoped to the life of the connection — `session_timeout_seconds` expiry is disabled**; BLE idle timeout = 5 minutes, kept firmware-local (not promoted to the canonical protocol header).

**Platform facts (verified):**
- ESP32: commands queue via SPSC ring (NimBLE host task → loop task, [main.cpp:406-423](../src/main.cpp)); responses via a 10-slot ring. nRF: NO queues — `imageDataWritten` runs inline on the Bluefruit *Callback* task; `Bluefruit.begin(1,0)` already caps BLE at 1 link.
- `-DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` does NOT work: precompiled `sdkconfig.h:613` redefines it to 3 and wins (empirically verified — do not re-add). Enforce in `onConnect` (NimBLE fills `m_connectedPeers` BEFORE `onConnect`, so `getConnectedCount() > 1` is a valid gatecrasher test).
- OTA exception satisfied by design: ESP32 has no OTA (0x0051 = `esp_restart`); nRF DFU jumps to the bootloader with the app gone. Comment it for future OTA work.
- Drain-loop trap: [main.cpp:409-417](../src/main.cpp) caches `tail` before dispatch, stores `tail+1` after — a flush from handler context gets clobbered.
- `setConnectableMode(NON)` internally calls `setFlags(0)` (one-way), so re-push `setAdvertisementData(*advertisementData)` before `start()` — same trap already documented at [ble_init.cpp:307-312](../src/ble_init.cpp).

---

## Hard constraint — NO wire protocol changes

**Nothing in this plan may change the BLE/LAN wire protocol.** This is a firmware-internal robustness effort; every fix must be observably compatible with today's clients (`py-opendisplay`, the HA integration, the web configurator) and with the other three firmware repos.

Concretely, the following are **out of bounds** for every phase:
- Editing `include/opendisplay_protocol.h` — it is a byte-for-byte vendored copy of `../opendisplay-protocol/src/opendisplay_protocol.h`. No local edit, and no change pushed through the canonical repo either.
- Adding, removing, or renumbering any `CMD_*` opcode or `RESP_*` code; changing the meaning of an existing one.
- Changing frame layout, framing, header/field sizes, nonce or CCM parameters, or the auth handshake sequence.
- Changing the config-packet layout in `include/opendisplay_structs.h` (field add/remove/resize/reorder), which is the same contract by another name.
- Changing any value the protocol header specifies — notably the **30 s auth-challenge window** (already recorded under *Deliberately NOT changed*) and the **LAN 30 s idle timeout** (`opendisplay_protocol.h:984`).
- Changing client-observable behaviour documented in `docs/pipe-write-protocol.md` (SACK semantics, discard rules, NACK meaning).

What **is** in bounds, and why each stays inside the constraint:
- Firmware-local constants that no client reads: `OD_NONCE_WINDOW`, `OD_BLE_IDLE_DISCONNECT_MS`, the pipe error-release deadline, supervisor/backstop timeouts, `COMMAND_QUEUE_SIZE`. None appear on the wire; a client cannot observe their value, only the (already-legal) behaviour they produce.
- Widening the replay window and fixing the `counter_diff == 0` hole — accepting *more* legitimate frames and rejecting a replay are both already-permitted outcomes of the existing nonce rules.
- Disabling `session_timeout_seconds` expiry — the field's own spec already defines `0 = no timeout (persists until disconnect)` ([opendisplay_structs.h:916](../include/opendisplay_structs.h)); the firmware simply behaves as if the field is always 0. The struct field stays, unchanged in size and position, and becomes advisory-only.
- Dropping a link (idle timeout, supervisor abort, connection-exclusivity refusal) — disconnect is always a legal outcome; clients already handle it and reconnect.
- Sending an existing NACK/`RESP_*` code in a new situation, as long as the code's documented meaning is unchanged.

If any phase appears to require a protocol change to work, **stop and escalate** — do not push a header change through `../opendisplay-protocol` as part of this work. Documentation-only additions (a note in `docs/pipe-write-protocol.md` §5.1, field notes in `tools/od-device-cli.py`) are permitted and expected, provided they describe behaviour the current spec already allows.

Verification of the constraint itself, run before any phase is called done:
```bash
cd ../opendisplay-protocol && tools/sync_protocol_header.py --check --only Firmware   # must pass, unchanged
cd ../Firmware && git diff main --stat -- include/opendisplay_protocol.h include/opendisplay_structs.h   # must be empty
```

---

## Phase order

Reordered per review: **root cause first, owner token before anything depends on it, supervisor before the escalations that rely on its accounting.**

### Phase 1 — Nonce/replay correctness `(was Phase 3 — highest value-per-risk, ship first)`

> ## ✅ SHIPPED 2026-07-26 — `0a60712`…`23ecaed` on `debug/freeze-fix-phase2`
>
> Ground truth, with `file:line` anchors, is the **"As-built"** section of
> [`PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md`](PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md). The bullets
> below are kept for provenance but **three of them describe a design that was superseded before
> implementation** — they are struck through and corrected inline. Read the Phase 1 plan, not this
> entry, before touching the code.
>
> **Verified:** 12/12 `pio run` environments build; host test `tools/test_nonce_window.cpp` passes
> 38,199 checks under UBSan/ASan; a separate `host-tests` CI job gates every push.
> **Not verified:** the *entire* hardware matrix, including the baseline capture (Test 0) and the
> test that decides whether an interrupted upload actually completes (Test 2b).

[encryption.cpp](../src/encryption.cpp) / [encryption_state.h](../src/encryption_state.h) / **new** [nonce_window.h](../src/nonce_window.h) / [communication.cpp](../src/communication.cpp)
- Split `verifyNonceReplay` → pure `nonceCheck()` (OK / BAD_SESSION / OUT_OF_WINDOW / REPLAY, **no state writes**) + `nonceCommit(counter)` (advance `last_seen_counter` + seen-set). **Shipped as specified** — `verifyNonceReplay` deleted outright, `nonceCheck`/`nonceCommit` file-static, pure logic in a dependency-free `src/nonce_window.h`.
- `decryptCommand`: `nonceCheck` → nonce failures return false **without** touching `integrity_failures` (loss ≠ tampering; only a CCM tag failure is tamper evidence) → CCM decrypt → on success `nonceCommit` + reset counter; on tag failure increment (≥3 → clear session, unchanged). **Shipped as specified.**
- ~~`[M1]` Make the window **symmetric**: `OD_NONCE_WINDOW = ±128` with `replay_window[]` grown 64 → 256 (+1.5 KB `.bss`). If `esp32-N4` won't link, fall back to ±64.~~ **Superseded.** The value ring was replaced with an **RFC 4303 sliding bitmap**: `OD_NONCE_BACKWARD_BITS = 256` (`uint64_t[4]`, 32 B) and a separate `OD_NONCE_FORWARD_CAP = 128`. Net struct change is **−480 B**, not +1.5 KB, so **the `esp32-N4` link-headroom gate is moot** — it links at 81,468 B, *below* the pre-Phase-1 figure. `[M1]`'s jam-forward concern was withdrawn: it cannot occur once commit happens after CCM verification.
- ~~Move `replay_window_index` out of the function static into `encryptionSession` so `clearEncryptionSession()` resets it.~~ **Superseded — the field no longer exists.** A bitmap has no insertion index, so the bug is structurally impossible rather than fixed.
- ~~**Close the `counter_diff == 0` replay hole** … replace the special case with an explicit `has_seen_counter` bool (or a sentinel initial value).~~ **Superseded — no `has_seen_counter` was needed.** Under the bitmap, "not seen" is a clear bit rather than a reserved sentinel, so a fresh session (`last_seen = 0`, all-zero bitmap) accepts counter 0 exactly once with no exemption. The `!= 0` term is simply gone. *(Note: this hole is also wider than this bullet says — the old accept path wrote the ring unconditionally, so replaying the highest-seen frame 64× flushed every genuine entry and re-opened the whole backward window. See `[H3]` in the Phase 1 plan.)*
- Log: distinguish `nonce out-of-window` from `CCM tag failure %u/3`. **Shipped, plus more than specified:** both nonce logs demoted to WARN and given **independent** 5 s rate-limit budgets, and the session-id-mismatch line no longer dumps two full session IDs.
- **Added, not in this entry:** *Step 4b* — a nonce-rejected `CMD_PIPE_WRITE_DATA` (`0x0081`) frame is now answered with **silence** instead of a fatal `RESP_NACK`, so the client's SACK path can repair it. This is conformance to `docs/pipe-write-protocol.md` §5.2 ("NACKs are reserved for unrecoverable conditions … not ordinary packet loss"), not a wire change. **It is also the highest-value change in Phase 1 and the least verified** — see Test 2b.
- **Added after implementation, in response to a live hardware failure** (`55a2478`, `77ebdcd`, `23ecaed`): a session-id mismatch now answers `RESP_AUTH_REQUIRED` rather than a fatal NACK, and **the BLE link is dropped after 10 consecutive `0xFE` answers**. The link drop is Phase 5 work pulled forward — see the Phase 5 entry below.
- **Hard-constraint check: passes.** `git diff 02bdd5c..HEAD -- include/` is empty; no opcode or response code was added; both `RESP_AUTH_REQUIRED` and `RESP_NACK` are used in their documented meanings.

### Phase 2 — Bound the refresh waits `(was Phase 0; scope cut 2026-07-26)`

> **Scope cut — Phase 2 is now three items, not seven.** Current plan:
> [`PLAN_PHASE2_REFRESH_BOUNDS_2026-07-26.md`](PLAN_PHASE2_REFRESH_BOUNDS_2026-07-26.md).
> The earlier [`PLAN_PHASE2_BOUND_WAITS_2026-07-26.md`](PLAN_PHASE2_BOUND_WAITS_2026-07-26.md) is
> **obsolete** — kept only for the analysis behind the cut items.
>
> **In scope:** `[X2]` `epdRefreshInProgress` on both boot paths · `[X3]` a real
> `fastepd_wait_refresh` · a real wall-clock `waitforrefresh` deadline (P2-8, added by the Phase 2
> plan and *not* in the original list below). Two files — `src/display_service.cpp`,
> `src/display_fastepd.cpp`. No new file, no `src/main.cpp` change, no `platformio.ini` change.
>
> **Dropped, with the residual each leaves open:**
>
> | Dropped | Residual now carried |
> |---|---|
> | `[C2]` `pwrmgmLockTake` deadline | The spin stays **unbounded on both targets**. A holder that never releases blocks its waiter forever. No `panelStateUnknown` flag is produced, so **Phase 3's `abortToKnownState` has nothing to report** — drop that from its remit. |
> | `powerOff` stuck-button bound | ESP32-only; needs a hardware fault; removes a recovery path rather than creating a freeze. |
> | Loop-drain 2 s cap | Withdrawn as unsound, not merely descoped — see below. |
> | `[L3]` inert TWDT flag | The dead `=120` knob stays in 9 ESP envs, still implying a watchdog that does not exist. |
> | Loop-liveness monitor (P2-9) | **A stalled `loop()` is now undetected on both targets.** ESP32's TWDT will not fire (every long wait yields, so IDLE0 is never starved) and nRF has no watchdog at all. |
>
> **Consequence for Phase 6.** Phase 2 was to be the "defensive floor"; it now delivers *bounded
> refreshes* only, not *detected stalls*. Everything in the table above lands on the supervisor —
> and on nRF, where none of the ESP32 wall-clock watchdogs run, there is nothing between a stall and
> Phase 6. Weigh that when sequencing Phase 6, which already had to be extended to nRF.
>
> **The loop-drain cap is withdrawn on the merits, not descoped.** The parent premise here — "a full
> window of commands can hold `loop()` for minutes" — does not survive checking: 32 pipe DATA frames
> cost 0.1–1 s total, the genuinely long case is a single END triggering a 30–60 s refresh (which a
> between-commands check cannot interrupt), and stacked refreshes are unreachable because a second
> `0x0072` short-circuits at [display_service.cpp:2366](../src/display_service.cpp). Do not
> re-propose it; if a saturation signal is wanted it belongs to Phase 7's `[H1]`.

**Original item list, retained for the record:**

- `[C2]` **`pwrmgmLockTake` — do NOT steal.** Legitimate holds already exceed 10 s: `bbepWaitBusy` caps at **30 000 ms** for 3/4/7-colour panels (`bb_ep.inl:3959-3975`) and `epdSessionForceOffLocked` holds the lock across `bbepSleep` → `bbepWaitBusy` (`bb_ep.inl:4122`). A steal on a bare 0/1 flag with no owner means two tasks drive the same SPI/CS, the true holder's later `Give` unlocks it under the stealer (mutual exclusion permanently dead), and `pwrmgmState` ends up `PWR_ACTIVE` on a dead rail. **Instead:** `pwrmgmLockTake` returns `bool` with a **60 s** deadline (≥2× worst-case busy wait); on expiry log ERROR, return false, caller skips its panel work and sets a "panel state unknown" flag that `abortToKnownState` reports. If a forced take is ever genuinely needed, add `volatile TaskHandle_t pwrmgmOwner` so the original holder's `Give` becomes a detectable no-op.
- `powerOff` button wait ([power_latch.cpp:87-90](../src/power_latch.cpp)): bound at 10 s, then drop the latch anyway.
- `[X2]` Set/clear `epdRefreshInProgress` around **both boot-refresh paths** (`refreshBootScreenFull` [display_service.cpp:533-542](../src/display_service.cpp) and the FastEPD boot path at `:1588-1594`). Today a 30–60 s Spectra boot refresh is invisible to every `epdRefreshInProgress` gate — including the supervisor's "never interrupt a refresh" rule.
- `[X3]` **FastEPD really is unbounded.** `fastepd_wait_refresh()` is a stub that ignores its timeout ([display_fastepd.cpp:228-231](../src/display_fastepd.cpp)) and `waitforrefresh(60)` short-circuits to it, so the "60 s cap" does not exist on IT8951/E1004. Implement `fastepd_wait_refresh` as a real busy poll against the IT8951 LUT-busy register honouring `timeout_sec`, and wrap **`fastepd_direct_refresh`** (the path a real transfer takes, [display_service.cpp:2422-2423](../src/display_service.cpp)) — not just `fastepd_full_update`.
- `[X1]` **I2C — DOWNGRADED after verification; the original finding was wrong.** The review claimed a wedged GT911 spins unbounded and that touch polls too rarely to notice. Neither holds:
  - The driver **already gives up**: 5 consecutive read failures (`TOUCH_I2C_FAIL_DISABLE_THRESHOLD`, [touch_input.cpp:39](../src/touch_input.cpp)) trigger `touch_disable_controller(..., "too many I2C read failures")` ([:642](../src/touch_input.cpp), [:677](../src/touch_input.cpp)), and `TOUCH_I2C_FAIL_BACKOFF_MS` suppresses INT-driven re-reads while failing ([:609-611](../src/touch_input.cpp)). A wedged controller is dropped, not retried forever.
  - The poll rate is fine — the floor is 100 ms (`TOUCH_PROCESS_MIN_INTERVAL_MS`, [:38](../src/touch_input.cpp)), enforced globally at [:589](../src/touch_input.cpp) **regardless of the configured `poll_interval_ms`**, whose per-controller value at [:600](../src/touch_input.cpp) can only ever slow polling further. (Note: [opendisplay_structs.h](../include/opendisplay_structs.h) documents `0 = 25 ms default`; the firmware uses 100 and cannot go below it. Header/behaviour divergence, documentation-only, not fixed here.)
  - **So: no nine-clock SDA recovery, no new state machine.** The only residual is that each failing transaction blocks for the Arduino default (~50 ms on ESP32) since `Wire.setTimeOut()` is never called — worst case ~250 ms of blocked `loop()` before the controller is disabled. Bounded and acceptable. **Optional**: add `Wire.setTimeOut(25)` after each `Wire.begin()` (incl. `wireBeginForOpenDisplay`, [display_service.cpp:785-800](../src/display_service.cpp)) to halve that window. A wedged GT911 costs touch until reboot; it cannot freeze the device.
- Loop command drain: 2 s wall-clock cap alongside the count cap.
- `[L3]` Delete or rename the inert `-DCONFIG_FREERTOS_WATCHDOG_TIMEOUT_S=120` in every ESP env — the IDF 5.x symbol is `CONFIG_ESP_TASK_WDT_TIMEOUT_S` and the precompiled `sdkconfig.h` wins regardless. Leaving a dead knob that reads like a 120 s guarantee misleads the next reader. Add a comment that the real TWDT is 5 s/panic on IDLE0 and that today's long waits survive only because they all yield.

### Phase 3 — `abortToKnownState()` + queue flushes + drain-trap fix `(was Phase 1)`
New `src/session_guard.h/.cpp` (both targets; ESP32 parts `#ifdef TARGET_ESP32`, LAN parts `#ifdef OPENDISPLAY_HAS_WIFI` — **not** `TARGET_ESP32`, since `esp32-N4` is ESP32 without WiFi).
- Flags: `commandDrainAbortPending`, `commandQueueOverflowAbort`, `responseQueueOverflowAbort`; `g_lastProgressMs` + `markSessionProgress()`.
- `flushCommandQueue()` / `flushResponseQueue()` in main.cpp; loop-task only; `tail := head` snapshot (SPSC-safe — tail has a single writer).
- `abortToKnownState(reason, dropLink)`: log first → optional client NACK (skip when dropping link) → set drain-abort flag → flush command ring → `cleanupDirectWriteState(true)` → `cleanupPartialWriteOnDisconnect()` → `resetPipeWriteState()` → **new** `resetChunkedWriteState()` → **new** `touchForceResume()` → buzzer/LED stop → `epdSessionForceOff()` **only if** `!epdRefreshInProgress` → `clearEncryptionSession()` → flush response ring → if dropLink: disconnect → release owner token → `markSessionProgress()`.
- `[M3]` `touchForceResume()` must also clear `directWriteTouchSuspended` ([display_service.cpp:2035-2038](../src/display_service.cpp)) and assert the counter reached 0. Keep the stated ordering (`cleanupDirectWriteState` first).
- `[M5]` **Drain-trap fix — exact placement**: the check goes **between** [main.cpp:415](../src/main.cpp) and `:416`, i.e. immediately after `imageDataWritten` returns and *before* `commandQueue[tail].pending = false`, breaking without the tail store. Placed after `:416` it writes into a slot the producer may have re-filled. While there, delete the vestigial `pending` field — it has no readers in either ring.
- `[H4]` `g_commandInFlight` is a `volatile uint8_t` **depth counter**, not a bool. Bluefruit's `ada_callback_invoke` falls back to invoking the write callback **inline on the BLE task** when `rtos_malloc` fails (`BLECharacteristic.cpp:538-542`), so "single task on nRF" is not an invariant — heap pressure during a large transfer is exactly when it breaks.

### Phase 4 — Connection exclusivity `(was Phase 4, corrected)`
- Owner token: `OWNER_NONE/BLE/LAN`, `linkClaim()`/`linkRelease()`.
- `[C3]` **Refuse with `BLE_ERR_REM_USER_CONN_TERM` (0x13), NOT `BLE_ERR_CONN_LIMIT` (0x09).** `NimBLEServer::disconnect` forwards to `ble_gap_terminate` (`NimBLEServer.cpp:321-332`) and 0x09 is not in the Core Spec's legal `HCI_Disconnect` reason allowlist — the controller rejects it with 0x12 and the gatecrasher stays connected while the code looks like it worked. Check the `bool` return; log WARN on failure.
- `[C4]` **Do not let the refusal re-enter the shared cleanup.** `onDisconnect` ([esp32_ble_callbacks.h:57-70](../src/esp32_ble_callbacks.h)) is a blind flag-setter, and the `ownerStillUp` guard is inside `#ifdef OPENDISPLAY_HAS_WIFI` ([main.cpp:328-338](../src/main.cpp)) — so on **`esp32-N4`** a refused stranger's disconnect tears down the incumbent's live transfer (a new remote DoS). Two required changes: (a) capture the refused conn handle in `onConnect` and skip raising `bleDisconnectCleanupPending` for it in `onDisconnect`; (b) move the `ownerStillUp` early-return **out** of the `#ifdef` — its `getConnectedCount() > 0` half is unconditionally correct.
- `[M2]` Prefer **evicting an idle incumbent** over refusing a reconnect: after an abrupt client loss the link lingers until supervision timeout (4–32 s) and a returning client would be refused. If the incumbent has `!transferActive()` and last RX older than ~10 s, terminate the old link and accept the new one. Refuse only when the incumbent is actively transferring.
- LAN accept ([wifi_service.cpp:874-900](../src/wifi_service.cpp)): `linkClaim(OWNER_LAN)` or `incoming.stop()`; scope both `clearEncryptionSession()` sites to `OWNER_LAN`.
- Advertising while LAN owns: `esp32_set_ble_connectable(bool)` — stop → `setConnectableMode(NON/UND)` → re-push `setAdvertisementData` → start. Skip during the post-deep-sleep-wake window. `[M4]` **Check the return of both `setAdvertisementData()` and `start()`**; on failure force connectable mode, re-push, retry, and set `bleRestartAdvertisingPending` — otherwise a failed `start()` after a `stop()` leaves the radio permanently dark with nothing retrying.
- `[X7]` Add `advertisingHealthTick()`: no peer + not advertising (`getAdvertising()->isAdvertising()`) + no pending flag for >30 s → force restart, log WARN. Closes the "unresponsive but not frozen" hole at [ble_init.cpp:232-235](../src/ble_init.cpp), where a stale nonzero connection count *clears* the pending flag.

### Phase 5 — Disconnect hardening `(was Phase 2, corrected — now lands after the owner token)`
- `[H2]` ESP32 `serviceBleDisconnectCleanup`: place `flushCommandQueue(); flushResponseQueue(); clearEncryptionSession();` **after** the (now unconditional) `ownerStillUp` early-return, scoped to `linkOwner() == OWNER_BLE`. Putting the clear before the guard would let every WiFi-lost tick ([main.cpp:452-457](../src/main.cpp)) destroy a live BLE session — the very bug Phase 4 exists to fix, from a second code path.
- `[H4]` nRF `disconnect_callback`: **defer** the session clear — set `nrfSessionClearPending`, service from `loop()` when the in-flight depth counter is 0. A `memset(session_key)` landing mid-`aes_ccm_decrypt` on the inline-fallback path is a real (if rare) race.
- Pipe NACK latch: add `error_since_ms` to `PipeWriteState` (genuine +4 B addition — the struct has no timestamp today) and a `pipeErrorTick()` in loop. `[L2]` Use **10 s**, not 60 s, and describe it honestly as a **hardware-release deadline** — py-opendisplay treats every `0x81` NACK as immediately fatal and never re-reads the ACK position, so the "client-retry window" rationale was fiction. Confirmed this does **not** break `docs/pipe-write-protocol.md` §5.1 (client-observable discard behaviour is unchanged) — add a line to §5.1 noting the reset.

#### Session lifetime := connection lifetime

**Disable `session_timeout_seconds` expiry.** `checkEncryptionSessionTimeout()` ([encryption.cpp:221-232](../src/encryption.cpp)) always returns true for an authenticated session; age-based expiry is removed. Encryption is scoped to the life of the connection and nothing else.

- **No protocol change and no header edit.** The canonical field already documents `0 = no timeout (persists until disconnect)` ([opendisplay_structs.h:916](../include/opendisplay_structs.h)) — the firmware now behaves as if the field is always 0, which is an already-specified, already-supported value. The struct field stays (removing it is a cross-repo change); it simply becomes advisory-only on this firmware. Document it as ignored in `tools/od-device-cli.py`'s field notes.
- **No client breakage.** py-opendisplay's `_reauthenticate_if_needed` returns immediately when the value is 0 ([device.py:795-796](../../py-opendisplay/src/opendisplay/device.py)); with a legacy nonzero config it performs one unnecessary but harmless re-auth at 90% — that path is only reached from `_write`, never from `_write_pipe_frame`, so it cannot land mid-stream. Provision new units with 0 to skip the pointless handshake.
- **This removes a whole freeze class.** Expiry was evaluated inside `isAuthenticated()` on *every* command dispatch, so it could fire mid-transfer — deterministically wedging any upload longer than the configured timeout, since the client's proactive re-auth is skipped for the entire pipe stream. It also removes a query-with-side-effects: `isAuthenticated()` currently mutates session state as a side effect of being asked a question.
- With expiry gone, call site 1 of `clearEncryptionSession()` disappears, and site 2 (`handleAuthenticate`'s re-auth path, [encryption.cpp:583-585](../src/encryption.cpp)) simplifies to "authenticated → clear and re-challenge", which is the correct behaviour for a client-initiated re-auth.

**Make a dead session with a live link impossible.** Add the guard inside `clearEncryptionSession()` itself ([encryption.cpp:238](../src/encryption.cpp)) rather than at each call site, so future callers cannot regress it: if a client is connected and the clear was not client-initiated, raise a flag that `loop()` services by dropping the link. A cleared session under a live link is invisible to the client — it keeps sending encrypted frames that all bounce `0xFE` and never re-authenticates mid-stream — so this is the difference between a recoverable error and a wedge. Surviving call sites and their disposition:

> ### ⚠ Re-scoped: **Phase 1 already shipped a partial version of this guard** (`77ebdcd`, `23ecaed`)
>
> Phase 1's Scope boundaries said *"Do not add link-drop behaviour to `clearEncryptionSession()`.
> That guard is Phase 5."* A live bench failure forced the behaviour in early anyway, from the
> other end: `rejectUnauthenticated()` ([communication.cpp:114-166](../src/communication.cpp))
> counts **consecutive `RESP_AUTH_REQUIRED` answers** and drops the BLE link at 10, serviced by
> `serviceBleAuthAbuseDisconnect()` ([:168-201](../src/communication.cpp)) from `loop()` on ESP32
> and **inline** on nRF (where `loop()` runs at `TASK_PRIO_LOW` and is starved by the
> `TASK_PRIO_NORMAL` callback task during a flood). The counter is cleared by a successful decrypt
> ([:908](../src/communication.cpp)) and by a successful authentication
> ([encryption.cpp:691](../src/encryption.cpp)).
>
> **This does not complete Phase 5's guard — do not delete this section, and do not duplicate it
> either.** The shipped version keys on the *symptom* and only after ten wasted round trips; Phase 5
> keys on the *event* and drops immediately. Phase 5 must therefore:
>
> 1. **Subsume, not duplicate.** Land the guard inside `clearEncryptionSession()` as specified, then
>    reduce the Phase 1 counter to a backstop for the cases the clear-site guard cannot see
>    (a client that never had a session at all, and the `NONCE_BAD_SESSION` desync path). Do **not**
>    leave two independent disconnect requests racing each other.
> 2. **Fix the three defects the Phase 1 guard shipped with**, all recorded in
>    [`PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md`](PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md) § `77ebdcd`:
>    (a) the count is **not cleared on disconnect**, so a new client can inherit its predecessor's
>    rejections — a one-line `resetAuthGateRejects()` in `disconnect_callback`
>    ([device_control.cpp:227](../src/device_control.cpp)) and in `onDisconnect`
>    ([esp32_ble_callbacks.h:57](../src/esp32_ble_callbacks.h)); (b) on ESP32 the guard identifies
>    the offender as `getPeerInfo(0)` rather than the actual sender, because the command ring
>    discards the conn handle — **fold this into Phase 4**, which is already widening connection
>    identity; (c) the threshold of 10 is **below** py-opendisplay's default 16-frame pipe window
>    (`device.py:2689-2694`), so a *legitimate* client whose session dies mid-upload trips it. That
>    is probably the right outcome, but Phase 5 must decide it deliberately rather than inherit it.
> 3. **`[H4]` is discharged for the drop path only.** Verified against the Adafruit core: the
>    Bluefruit disconnect callback is queued via `ada_callback` onto the *same* FreeRTOS task as the
>    write callback, and `sd_ble_gap_disconnect()` is asynchronous — so an inline
>    `Bluefruit.disconnect()` cannot unwind into the callback it is called from. `[H4]`'s actual
>    hazard (a `memset(session_key)` landing mid-`aes_ccm_decrypt`) is untouched and still applies
>    to the **session clear** this section specifies, which nRF's `disconnect_callback` does not do
>    today.

| Site | Disposition |
|---|---|
| [encryption.cpp:585](../src/encryption.cpp) re-auth challenge | **Exempt** — client-initiated, expects a new session |
| [encryption.cpp:670](../src/encryption.cpp) `aes_cmac` failure | Exempt — aborts a session being born |
| ~~[encryption.cpp:695](../src/encryption.cpp) /~~ [:794-798](../src/encryption.cpp) `integrity_failures >= 3` | Must drop the link. **Phase 1 removed the nonce trigger as planned** — there is now exactly **one** call site, the CCM-tag arm; the pre-decrypt site at old `:695` no longer exists. |
| [communication.cpp:204](../src/communication.cpp) `reloadConfigAfterSave` | Must drop the link — a config write always arrives over a live link, and security settings may have changed. *(Shifted down ~136 lines by Phase 1's auth-guard block.)* |
| [wifi_service.cpp:804](../src/wifi_service.cpp) / [:879](../src/wifi_service.cpp) LAN | Scope to `OWNER_LAN` (Phase 4) |
| [config_parser.cpp:883](../src/config_parser.cpp) boot load | Exempt — not connected |

**Note in code that deep sleep already wipes the session.** `encryptionSession` is plain `.bss` ([main.h:289](../src/main.h)), not `RTC_DATA_ATTR`, so a deep-sleep cycle destroys it regardless. Comment this so nobody later "optimises" it into RTC memory — persisting the key and `last_seen_counter` across sleeps would reintroduce the replay vector Phase 1 closes.

### Phase 6 — The 10-minute supervisor `(was Phase 6, corrected)`

> ⚠️ **Phase 6 inherited work from the Phase 2 scope cut (2026-07-26).** Phase 2 no longer bounds
> `pwrmgmLockTake`, no longer bounds `powerOff`'s stuck-button wait, and — most significantly — no
> longer detects a stalled `loop()` on either target (the loop-liveness monitor was dropped). ESP32's
> TWDT will not cover the gap: every long wait yields, so IDLE0 is never starved and it does not
> fire. nRF has no watchdog at all and none of the ESP32 wall-clock watchdogs run there.
>
> So the supervisor is now the **first and only** thing that notices a stall, not the last line of a
> layered defence. Two consequences for this phase: its nRF arm moves from "must not be forgotten"
> to load-bearing, and a panel-lock holder that never releases is a fault class it must survive
> without any upstream bound or signal.

- `[C1]` **Progress means the state machine advanced — never "a command arrived" or "a notify succeeded."** After `clearEncryptionSession()`, [communication.cpp:664-670](../src/communication.cpp) answers every retry with `RESP_AUTH_REQUIRED` — a dispatch *and* a notify per retry — so dispatch/notify stamps keep `g_lastProgressMs` fresh forever and the supervisor never fires in the exact wedge it was built for. Stamp **only** at: `pipeState.expected_seq` advancing (inside the in-order accept), `directWriteBytesWritten` increasing, `chunkedWriteState.receivedChunks` incrementing, `partialCtx` byte counter advancing, refresh completion, and `handleAuthenticate` success. **Not** on command dispatch, notify, or LAN frame dispatch.
- Wedge: `(transferActive() || chunkedWriteState.active || directWriteActive) && now - g_lastProgressMs > 600000` → if `epdRefreshInProgress`, log and retry next pass; else `abortToKnownState("supervisor", true)`.
- `[H3]` **Keep the existing wall-clock watchdogs** ([main.cpp:436-442](../src/main.cpp), `checkPartialWriteTimeout`) as a backstop, raised to 20 min. They key on *start* stamps nothing refreshes, so they bound cases a progress predicate can't. Delete them only after hardware soak proves the progress arm fires.
- **nRF has NO transfer watchdog today — H3 is "keep" on ESP32 but "ADD" on nRF.** Both 900 s bounds live inside the `#ifdef TARGET_ESP32` arm of `loop()`: the direct-write check at [main.cpp:438](../src/main.cpp) and the `checkPartialWriteTimeout()` call beside it. The nRF `loop()` body is the `#else` arm and evaluates neither, so on nRF a stalled transfer is bounded by **nothing** — not today, and not by H3's "backstop" unless it is explicitly added there. Neither the original plan nor the adversarial review caught this. The supervisor and the wall-clock backstop must both be wired into the nRF `loop()` path, and the nRF hardware soak must cover a stalled transfer explicitly rather than assuming ESP32 parity.
- `[X5]` Add `transferActive()` to the `workInFlight` disjunction ([main.cpp:474-479](../src/main.cpp)) so a latched transfer with a dropped link can't reach `enterDeepSleep()` with the panel rail up.
- `[H4]` Abort only when the in-flight depth counter is 0. If stale >10 min while in-flight, log an ERROR heartbeat — Phase 2's bounds are the recovery story.
- No hardware WDT (user decision). Comment the OTA-exception rationale.
- `[X4]` Config chunked-write has no timer of its own ([communication.cpp:496](../src/communication.cpp) set, cleared only on completion/malformed/auth-fail) — it is covered by the supervisor predicate *once C1 is fixed*. Dependency noted deliberately.

### Phase 7 — Queue-full handling + BLE idle timeout `(was Phase 5, corrected — must land after Phase 6)`
- `[H1]` **Command-ring overflow logs and drops; it does NOT drop the link.** The pipe protocol is designed for a dropped frame (zero bit in the next SACK → client retransmits that chunk, `docs/pipe-write-protocol.md` §5.2) — one round trip vs. a link drop + re-auth + restarted transfer. Escalate to `abortToKnownState` only if overflow recurs while `transferActive()` **and** `g_lastProgressMs` is already stale, i.e. let the supervisor own the decision. Also fix the off-by-one in the [main.h:365-370](../src/main.h) comment: usable capacity is `COMMAND_QUEUE_SIZE - 1 = 32`, not 33 (producer refuses at `nextHead == tail`), so the documented "W=32 window + END" claim is false — bump `COMMAND_QUEUE_SIZE` to 34 on envs with DRAM to spare (**not** `esp32-N4`).
- Response-ring overflow: flag, serviced in loop, gated on `transferActive()`.
- **BLE idle disconnect.** `OD_BLE_IDLE_DISCONNECT_MS = 300000` (5 min; 0 disables). Gate on peer connected && `!epdRefreshInProgress`, then drop the link.

  **Definition of activity (authoritative):** *a chunk arriving in the command queue carrying a **valid command**, or a **continuation of a data upload**.* Nothing else stamps `g_lastLinkActivityMs`.

  `[C1]` "Valid" is the load-bearing word and it is what makes this timer resistant to the defect that killed the original RX-keyed design. Validity is not knowable at queue-insert time — `onWrite` runs on the NimBLE host task before decryption — so the stamp goes in `imageDataWritten` **after** the decrypt/auth gate passes ([communication.cpp:663-711](../src/communication.cpp)) **and** the opcode resolves to a known command (`commandName(command) != nullptr`, i.e. not the `default:` unknown-opcode branch). A post-`clearEncryptionSession()` retry flood therefore never stamps: every frame short-circuits to `RESP_AUTH_REQUIRED` at [communication.cpp:664-670](../src/communication.cpp) before reaching the stamp, and the link is dropped at 5 min.

  "Continuation of a data upload" means a `0x0071`/`0x0081` frame that was **accepted** (consumed in order, or queued in the reorder window) — **not** one silently discarded because `pipeState.error` is latched ([display_service.cpp:2811](../src/display_service.cpp)). Counting discarded frames would let a client retransmitting into a dead pipe hold the link forever, which is the same defect one layer down.

  **Relationship to the supervisor's `g_lastProgressMs` (Phase 6):** two distinct signals, deliberately. Idle activity answers *"is the client still talking sense?"*; supervisor progress answers *"is the state machine advancing?"* They compose: a client politely polling battery status every minute is active (valid commands) and correctly not dropped, while making no transfer progress — and the supervisor doesn't fire either, because its wedge predicate requires `transferActive()`. A client flooding undecryptable frames trips the idle timer at 5 min; a client sending valid frames into a stalled transfer trips the supervisor at 10 min.

  **LAN keeps its 30 s ([opendisplay_protocol.h:984](../include/opendisplay_protocol.h)); the asymmetry is a deliberate design choice, not an oversight.** LAN is a machine-to-machine push transport: a client connects, pushes, and closes, so 30 s of silence means it is gone or broken — drop it fast and free the socket. **Only BLE carries interactive sessions**, where a human using the web configurator or the HA UI legitimately pauses between commands (reading config, composing a change, waiting on a slow refresh). A 30 s BLE timeout would break interactive use; 5 min tolerates human latency while still bounding the hostage window. BLE reconnect is also far more expensive than a TCP reconnect — advertising, connection setup, re-auth, and possibly a deep-sleep wake.

  **The constant stays firmware-local.** Mirroring LAN by promoting it to the canonical header would be a cross-repo change through `../opendisplay-protocol` plus a `--push` to all four firmware repos; not warranted until the behaviour is proven on hardware. Revisit only if a client ever needs to read the value.

  **Interaction with deep sleep is cooperative, not adversarial.** `pollActivity()` refreshes `lastActivityMs` every pass while a client is connected ([main.cpp:243-258](../src/main.cpp): *"A live link … is activity in itself"*), so today a silent client pins the device out of deep sleep indefinitely — the hostage case. Dropping the link makes `connCount` fall to 0, `lastActivityMs` stops being refreshed, and the existing `sleep_timeout_ms` hold elapses normally. No change to the sleep gates is required.

---

## Files touched
`src/session_guard.h/.cpp` (new), `src/main.cpp`/`main.h`, `src/encryption.cpp`/`encryption_state.h`, `src/display_service.cpp`, `src/wifi_service.cpp`, `src/esp32_ble_callbacks.h`, `src/ble_init.cpp`, `src/device_control.cpp`, `src/power_latch.cpp`, `src/display_fastepd.cpp`, `src/touch_input.cpp`, `src/communication.cpp`, `src/structs.h`, `platformio.ini`, `docs/pipe-write-protocol.md`.

## Verification
- Session lifetime: verify a transfer longer than any legacy `session_timeout_seconds` completes untouched; verify a client-initiated re-auth mid-connection still works; verify a captured last-frame replayed after reconnect is now REJECTED (the `counter_diff == 0` hole).
- Per phase: `pio run -e nrf52840custom -e esp32-s3-N16R8 -e esp32-c3-N16 -e esp32-c6-N4 -e esp32-N4`. CI builds all **12** on push (the matrix grew; several places in these plans still say 11). ~~**`esp32-N4` is the gate** for Phase 1's `replay_window[256]` (+1.5 KB `.bss`) — if it won't link, drop to ±64.~~ **Moot as of Phase 1:** the sliding bitmap made the struct **480 B smaller**, and `esp32-N4` links at 81,468 B / 24.9% RAM — below where it started. A `host-tests` CI job now also compiles and runs `tools/test_nonce_window.cpp` under UBSan/ASan on every push.
- Hardware (py-opendisplay CLI): forward-gap nonce test (skip 100 counters) → session survives; true replay → rejected, session survives; kill client mid-pipe-window → reconnect, re-auth, clean push; forced pipe NACK → touch recovers ≤10 s; second BLE central → **verify on-air with a sniffer or `nRF Connect` that the refusal actually terminates** (this is the C3 trap); LAN connect during BLE session → accept-then-close; BLE connect while LAN owns → refused, telemetry still advertising; silent client 5+ min → dropped, advertising resumes, deep sleep reachable; stalled pipe with live connection **that keeps sending doomed frames** → supervisor still cleans at 10 min (this is the C1 regression test); regression: full Spectra transfer (60 s+ refresh) and an E1004 ~960 KB upload complete untouched.
- Every recovery action logs one ERROR/WARN line with reason + counters.
- Final soak: 24 h cron'd pushes alternating BLE/LAN with ~10% induced client kills.

## Deliberately NOT changed

Recorded so implementation does not "fix" these and review does not re-litigate them. Both were surfaced by `TIMER_AND_WATCHDOG_INVENTORY_2026-07-26.md`; both are decided.

- **The 30 s auth-challenge validity window stays exactly as-is.** Step 2 of `CMD_AUTHENTICATE` must arrive within 30 s of the challenge — stamped at [encryption.cpp:587-588](../src/encryption.cpp), enforced at [:604-608](../src/encryption.cpp). This is a **cross-repo wire contract**, not a firmware-local knob: [opendisplay_protocol.h:384](../include/opendisplay_protocol.h) specifies *"STEP 2 must arrive within 30 s of the challenge or it is rejected"* with `@targets: Firmware | NRF54 | Silabs | NRF52811`. Changing it would require an edit in `../opendisplay-protocol` plus a `--push` to four firmware repos. It is also **not** a liveness timer and cannot wedge anything — a late step 2 gets `AUTH_STATUS_ERROR` and the client restarts the handshake. Phase 1 restructures `encryption.cpp` but **must not touch this check**, including the known boot-window quirk (`server_nonce_time = 0` from `clearEncryptionSession` means a step 2 with no preceding step 1 passes the freshness test during the first 30 s of uptime; not exploitable, since the MAC still requires the master key). Correct Phase 5's "scoped to the connection **and nothing else**" phrasing to acknowledge this second, independent encryption time bound — wording only, no code.

- **No new bounding timer for the buzzer or the LED.** `abortToKnownState` stops both as part of teardown (Phase 3) — that is a teardown action, not a timer, and it stays. Nothing further is added. For the record, and correcting review finding `[X6]` which lumped them together:
  - **Buzzer is already bounded** — `kBuzzerMaxTotalMs = 30000` ([buzzer_control.cpp:17](../src/buzzer_control.cpp), enforced at `:168`). No gap. *(The source comment there claims a "5 s cap" against a 30 s constant; comment-only defect, left alone.)*
  - **LED flash is genuinely unbounded** — `processLedFlash` has no global cap equivalent. **Accepted as-is.** A stuck LED sequence wastes power but cannot freeze the device: it holds no lock, blocks no task, and is cleared by `abortToKnownState`, by disconnect teardown, and by an explicit `0x0075` LED_STOP.

## Explicit non-goals / residual risk
- A true CPU/peripheral hard hang remains detect-and-log only — accepted with the software-only decision. **But "recoverable by power cycle" is weaker than it sounds on latching devices.** The button *press* is captured by an ISR ([device_control.cpp:679-693](../src/device_control.cpp)), but the hold-duration evaluation and the power-off action run in `processButtonEvents()`, which is called **only** from `loop()`/`idleDelay()` ([main.cpp:481](../src/main.cpp), [:514](../src/main.cpp), [:527](../src/main.cpp), [:542](../src/main.cpp)) — and the power-off hold test itself is at [device_control.cpp:78](../src/device_control.cpp). So while `loop()` is blocked (e.g. inside a 60 s `waitforrefresh`, whose `delay(10)` yields to FreeRTOS but never services buttons), a long-press does not trigger power-off. On a `DEVICE_FLAG_BATTERY_LATCH` unit with no way to interrupt the rail, the user's fallback is unavailable for the duration of the block. This does not change the software-only decision, but it means the residual risk is "wait out the block or remove the battery", not "hold the button".
- No config-schema changes (timeouts are compile-time constants in v1).
- The client-side nonce burn (py-opendisplay) is untouched; firmware-side widening makes it safe.
- ~~`[M1]` A local attacker with a captured frame can still jam `last_seen_counter` forward within the window and stall a session; the supervisor is the recovery path. Symmetric windowing keeps the stall bounded by the ring.~~ **Withdrawn — this risk does not survive Phase 1.** With commit-after-verify, only a CCM-authenticated frame advances `last_seen_counter`, so an attacker can only re-commit counters the client genuinely transmitted and can never push past the client's own high-water mark. Repairs then carry fresh, higher counters, so no future client frame falls below the window. The jam was an artifact of the value-ring design.
- **Residual risk that replaces it:** a forward gap beyond `OD_NONCE_FORWARD_CAP = 128` is still reachable on a pathological link with `blocks_per_ack = 1` and a multi-thousand-chunk upload. Step 4b makes it non-fatal (silent drop → SACK repair) rather than impossible — **and that repair path has never been observed on hardware.**
