# Connection Policy — OpenDisplay Firmware

**Status:** normative ruleset. This document defines *what must be true*; it does not
schedule the work. Implementation is staged in
[`PLAN_FREEZE_HARDENING_2026-07-31.md`](PLAN_FREEZE_HARDENING_2026-07-31.md), whose
Phase 3 must conform to this document where the two disagree — this one wins.

Written against the tree at `fix/nonce-replay-window`. Every statement about current
behaviour is cited to `file:line` so a reviewer can re-check rather than trust.

No wire-protocol change: every rule here is enforced at the transport/HCI layer or in
firmware-local state. No new opcode, no new response code, no config-schema field.

**Hard constraint — ONE command queue.** There is exactly one RX command ring and one
TX ring, shared by all transports, and this policy must never introduce a per-connection
one. The RX ring is `PIPE_MAX_W + 2` slots ([command_queue.h:63](../src/command_queue.h))
— 18 or 34 depending on `PIPE_SMALL_DRAM_WINDOW` ([structs.h:46-54](../src/structs.h)) —
at `OD_BLE_MAX_FRAME` = 256 B each, so ~4.7 KB or ~8.8 KB. Replicating it across three
NimBLE connection slots would cost 14–26 KB on a device whose zlib window is 512 bytes.
Not a trade worth discussing.

This is not a constraint the policy merely tolerates; it is one the policy *enforces*.
R3 requirement 1 drops a non-owner's write at the callback, before it reaches the ring,
so only the owner's frames ever enter it — there is never a second client's traffic to
separate, and therefore never a reason to partition. Callback-side filtering and the
single queue are the same decision seen from two sides: without the filter you would be
pushed toward per-connection buffering to keep streams apart. Requirement 6 adds four
bytes of *per-frame* metadata to each slot — the writer's identity word — which is
still one ring holding each frame once: identity travels with the frame instead of
being reconstructed from a captured boundary. (An earlier revision said this
constraint "keeps `bleRxQueueDiscardTo(rxBoundary)` working unchanged"; requirement 6
**supersedes** that flush outright — dispatch-side identity filtering replaces it.)

Where this document calls for per-connection *state* (R2's instance identity, R3's
event delivery), that state is **metadata only** — an epoch, a liveness word, a
disconnect reason; on the order of 8 bytes per slot. Nothing that holds frames is ever
replicated per connection. (Requirement 6 additionally tags each *frame* with a
4-byte identity word — per-frame metadata inside the one ring, not per-connection
state.)

> **Revision note.** This document was reviewed against the tree after its first
> draft; that review found four defects that are corrected below and are called out
> where they land, because each is a trap an implementer would otherwise re-enter:
> the generation counter was allocated at the wrong moment (R2), refusal isolation
> needed far more than handle-bearing events (R3), the owner was released before the
> link was actually down (R3a), and the refresh BUSY-wait is *not* bounded on the
> FastEPD path (R5).

> **Second revision note (2026-07-31, external review).** A further adversarial review
> found three defects that made the ruleset unimplementable as written, corrected in
> place: the owner token was specified as loop-task-only state while R3's filtering
> needs to read it on the stack-callback task (fixed: the one-word CAS token, R2); a
> firmware-initiated drop was implicitly BLE-only while R6's teardown can fire on a
> LAN owner (fixed: transport dispatch, R3a); and the R3a wait polled the aggregate
> connection count, which never reaches zero while a refused contender is attached
> (fixed: the per-handle instance table is the predicate).

> **Third revision note (2026-07-31, same review, second batch).** Three further
> defects shared one root cause — **queued frames are anonymous** — and are corrected
> together by one mechanism, R3 requirement 6 (frame identity): the abort drained TX
> but not RX, and the departing owner could keep writing during its own teardown; the
> instance table could lose a departed owner's RX boundary to handle reuse before the
> loop scanned it; and the dispatcher's "from the owner" test compared transport only,
> so a delayed frame from a dead instance could stamp the new owner's activity clock
> and execute in its session. The frame tag supersedes the RX-boundary mechanism
> entirely.

> **Fourth revision note (2026-07-31, closing the review).** The remaining findings,
> corrected in place: table 7a modelled only one arrival at a time — rows 9–10 and
> the admission-decided-once rule close it; the deep-sleep abort's rationale claimed
> state survives sleep — RAM in one draft, hardware in the next, both false — and is
> corrected at 7e row 3, where the abort now stands on teardown uniformity at a
> mid-session exit; and the auth-abuse `FE` "delivery" is restated as
> best-effort in the plan, since an empty TX ring proves stack acceptance of an
> unacknowledged notification, not receipt.

---

## Definitions

**Connection instance** — one physical link, from the stack's connect callback to its
matching disconnect. Identified by `(transport, handle, epoch)`; see R2.

**Admitted** — the connection instance currently holding the slot, i.e. the owner.

**Refused** — a connection instance the firmware has decided not to admit. It may be
physically established for a short time while being torn down. It is never the owner.

**Owner state** — `NONE` or `ACTIVE` (admitted and serviceable). There is no
intermediate "dropping" state: a firmware-initiated drop waits synchronously for the
link to go down before releasing. See R3a. One further state exists on exactly one
path: **`TERMINAL`**, entered by `linkMarkTerminal()` in the deep-sleep sequence
(7e row 3) — admission permanently gated until wake reloads RAM. It is a one-way
gate, not a lifecycle state: nothing transitions out of it.

**Inbound command** — a frame from the owner that reaches the dispatcher and is
recognised as a command. Not merely bytes; not merely a queued buffer. See R4.

---

## The rules

### R1 — One admitted client, globally

**At most one *admitted* connection may exist across all transports at any time.** BLE
and LAN are not independent slots. A device serving a BLE client has no LAN capacity,
and the reverse.

**Phrased in terms of admission, not physical links, because the physical form is
unachievable on ESP32.** NimBLE establishes a second central's link *before* it calls
`onConnect` ([ble_transport_esp32.cpp:81-93](../src/ble_transport_esp32.cpp)); there is
no pre-connection filter in the server API. So a transient second *physical* link
necessarily exists while it is being refused. R1 constrains what is *serviceable*; R3
constrains what that transient link can touch, which is nothing.

*Today R1 is false in every direction.* BLE and LAN can both be live simultaneously;
there is no connection-level arbitration, only per-*transfer* ownership (`sessionOrigin`,
stamped at each transfer START, [display_service.cpp:2159,2200,2712](../src/display_service.cpp)).
On ESP32 the single-transport case also fails: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS = 3`
is baked into the precompiled NimBLE framework and cannot be lowered by a `-D`
override, and `onConnect` performs no count check — a second central's handle simply
overwrites the scalar `s_connHandle` (`:87`).

### R2 — Every connection instance carries a unique identity

**Identity is a triple, allocated per *physical connection*:**

```
(transport, handle, epoch)
```

- `transport` ∈ {`OWNER_BLE`, `OWNER_LAN`}.
- `handle` distinguishes simultaneous links on one transport (BLE conn handle; LAN
  uses 0, being single-socket by construction).
- `epoch` is a monotonically increasing counter making the identity unique *over time*.

> **The epoch is allocated in the connect callback, for every connection instance —
> admitted or not. It is emphatically NOT allocated on successful claim.** The first
> draft of this document said "incremented on every successful claim," which is
> self-defeating: a *refused* contender never claims, so it would carry no epoch, and
> table 7a row 4 — a refused contender that reused the incumbent's handle — could not
> be distinguished from the incumbent at all. Allocation must happen before the
> admission decision, because the identity is what the admission decision is *made
> on*. On admission the owner token copies the instance's already-allocated epoch.

**Why an epoch is needed at all.** BLE connection handles are small integers the stack
reuses: NimBLE allocates from 0 upward, so a client that disconnects and reconnects can
be handed the *same* handle. Any deferred operation carrying a stale handle — a queued
disconnect event, a pending abort, a write filter test — can otherwise match a
different, newer session and act on it. This firmware defers work by design:
`serviceBleDisconnectCleanup` can run tens of seconds late when `loop()` was blocked in
a refresh, a hazard the code already documents at
[main.cpp:398-403](../src/main.cpp). The epoch turns "same handle" into "same
connection instance," which is what every deferred consumer actually needs.

**Required properties:**

- **Comparison is on the full triple.** `handle` alone is never sufficient.
- **ESP32 needs per-live-handle instance state, not a scalar.** While NimBLE permits
  three links, the single `s_connHandle` ([ble_transport_esp32.cpp:87](../src/ble_transport_esp32.cpp))
  cannot represent them. A small fixed array indexed by handle is sufficient.
- **Publication must be atomic — and liveness lives in the identity word.** The triple
  is written on a stack-callback task and read on the loop task. A multi-field
  `volatile` struct is not an atomic snapshot; publish a single word (packed
  handle+epoch, all-zero = empty) with release/acquire ordering, or guard with the
  `__atomic_*` discipline the RX ring already uses
  ([command_queue.cpp:62,92](../src/command_queue.cpp)). An entry's liveness must be
  that same word — release-stored at connect, cleared at disconnect — never a separate
  `state` field that could race the identity; R3a's wait predicate depends on reading
  identity and liveness in one atomic load. The one side field (`reason`) is consumed
  only after the identity word reads as down.
- **Ownership must be readable — and claimable — from the callback task.** R3's
  write/subscribe filtering runs in stack callbacks, before any loop pass has had a
  chance to decide anything; R7d makes the earliest transport hook the authoritative
  arbitration point. A loop-task-only owner variable therefore cannot work — a prior
  draft specified one alongside callback-side filtering, which is a contradiction: at
  the moment a contender's `onWrite` fires, a loop-side token gives the filter nothing
  to compare against, and there is no rule for the unowned window before first
  admission. **The owner token is a single 32-bit word** — packed
  `transport(2) | handle(14) | epoch(16)`, all-zero meaning unowned — claimed with one
  `__atomic_compare_exchange` at the earliest transport hook (the BLE connect callback,
  on the host task; the LAN accept, on the loop task), read with one atomic load from
  any task, and released (CAS back to zero) only on the loop task, after R3a's wait.
  CAS success *is* admission; CAS failure is what marks the instance a contender, which
  the loop-side scan then refuses (R3). This closes the unowned window: the host task
  processes a peer's connect before any of its writes, so by the time a first client's
  first write reaches `onWrite`, the word already names its owner. In the connect
  callback the order is: allocate the epoch, publish the instance-table entry, then
  CAS — a successful claim never names an instance the loop cannot yet see. The epoch
  counter itself is `__atomic_fetch_add`, since BLE allocates on the host task and LAN
  on the loop task. One further reserved encoding, **`OWNER_TERMINAL`** — transport
  code 0b11, reserved word `0xC0000000` in the `[31:30] transport | [29:16] handle |
  [15:0] epoch` layout: `linkMarkTerminal()` exchanges the word to it unconditionally
  and **returns the displaced owner identity** (possibly none), which is what the
  terminal caller hands the abort to act for — after the exchange a fresh read of the
  word yields terminal, not the departing owner. Claims succeed only against the
  all-zero word, so admission is impossible from that point until a reset or wake
  reloads RAM. `linkRelease` matches the *full* identity and never accepts the
  terminal word as an argument, so the abort's release — called with the displaced
  identity — is naturally inert and nothing can CAS the gate back to zero. This is
  what lets a terminal transition run the ordinary abort unmodified (7e row 3).
- **Scope is one boot.** Uniqueness across reset is not required and is not claimed:
  no deferred RAM state survives a reset.
- **Wrap.** The epoch is **16 bits — a deliberate narrowing**, because the one-word
  token above must stay lock-free and neither Cortex-M4 nor the ESP32 ISAs have a
  lock-free 64-bit CAS; HCI connection handles are spec-bounded at 0x0EFF (12 bits),
  so `2 | 14 | 16` fits with headroom. The normative invariant is that no outstanding
  event may survive a full counter cycle. The argument for it is **conditional, and
  the condition is stated rather than hidden**: for a stale identity to be
  *misconsumed*, the loop must resume and consume it, so the churn window that matters
  is a blocking window that later **completes**. Epochs churn only at link-layer
  connection rate — tens of milliseconds per instance, on the host task — so a full
  2^16 cycle needs on the order of half an hour of *continuous* connect churn inside
  one completing block; no bounded refresh approaches that. The one unbounded block in
  the tree, R5's FastEPD refresh, does not break the invariant the way it first
  appears to: a refresh that never completes means the loop never runs again, so the
  outstanding event is never consumed at all and a collision has no consumer to
  mislead. If this margin is ever doubted, the mitigation is an allocator that skips
  epochs still present in the instance table — not a wider word, which the lock-free
  constraint forbids. Epoch 0 is never allocated (`linkNextEpoch` re-draws when the
  fetch-add yields 0, so wrap cannot mint it), reserving the all-zero word for
  "unowned."

### R3 — A contender is always refused, and refusal is inert

**While the slot is held, an incoming connection is refused** — before establishment
where the stack allows it, otherwise by immediate disconnection. Admission never evicts
an incumbent; reclaiming a held slot is exclusively the job of R4.

**Refusal must not perturb device state.** Refusing must not run `abortToKnownState`,
raise `s_disconnectCleanupPending`, call `linkRelease`, touch the encryption session,
touch transfer state, or alter panel power. The incumbent must be unable to observe
that a contender arrived.

> **Handle-bearing events are necessary but nowhere near sufficient.** The first draft
> stopped at "ignore the refused contender's disconnect event." Review of the ESP32
> callbacks showed that a contender perturbs shared state *before any loop-side
> decision runs*. All of the following are live defects in the current tree.

On ESP32, every one of these is global scalar state that any central can move:

| Shared state | Site | What a contender does to the incumbent |
|---|---|---|
| `s_notifySubscribed` | [esp32:81-93](../src/ble_transport_esp32.cpp) (connect), [:129](../src/ble_transport_esp32.cpp) (subscribe, `(void)connInfo`) | Clears/overwrites the incumbent's apparent notify-readiness, stalling its TX |
| RX ring | [:135](../src/ble_transport_esp32.cpp) `onWrite`, `(void)connInfo` | Injects commands into the incumbent's stream; can fill the ring and drop incumbent frames |
| TX / notify | [:269-277](../src/ble_transport_esp32.cpp) | **Leaks incumbent responses to the contender** — see below |
| `s_connHandle` | [:87](../src/ble_transport_esp32.cpp) | Overwritten, so link tuning and any future disconnect target the wrong link |

**The notify leak is the sharpest of these and is present today.** `BleTransport::notify`
calls `s_txCharacteristic->notify(data, len)` — the two-argument overload. NimBLE's
signature is `notify(value, length, connHandle = BLE_HS_CONN_HANDLE_NONE)`, documented
as "or `BLE_HS_CONN_HANDLE_NONE` to send the notification to **all subscribed
clients**." So a second central that connects and subscribes receives every response
the incumbent is sent, including authentication traffic — with no policy decision
having been made, and before `loop()` runs at all.

**Therefore R3 requires, at the callback boundary:**

1. **Per-link write filtering.** Drop a non-owner's write in `onWrite` before it
   reaches the RX ring.
2. **Per-link subscribe filtering.** A non-owner's `onSubscribe` must not move the
   owner's notify state; subscription state must be per-instance.
3. **Handle-targeted notification.** `notify()` must pass the owner's conn handle, so
   a subscribed non-owner receives nothing. This is a one-argument change and it
   closes a live leak independent of the rest of this policy.
4. **Identity-bearing disconnect events**, with every consumer ignoring an event whose
   identity does not match the owner. Today `takeDisconnectedEvent`
   ([ble_transport.h:81](../src/ble_transport.h)) carries a reason and an RX boundary
   but no handle, so this is a new transport requirement — *additional* to the
   handle-bearing connect event already planned.
5. **Connection state must survive lost edges — via a table, not a queue.** See below.
6. **Frame identity — every queued frame carries its writer's instance identity, and
   the dispatcher re-checks it against the owner word before executing.** See below;
   this requirement retires the RX-boundary mechanism.

**How the callbacks know the owner.** Requirements 1 and 2 run on the stack's host
task, before any loop pass — during a refresh, up to ~16 s before one. They read the
one-word owner token (R2) with a single `__ATOMIC_ACQUIRE` load and compare it against
their own instance's `(transport, handle, epoch)`; requirement 3's `notify()` reads the
same word on the loop task for the target handle. There is no unowned ambiguity to
special-case, because the claim itself happens in the connect callback (R2's CAS) and
the host task processes a peer's connect before any of its writes — a contender is
exactly an instance whose connect-time CAS failed, and its writes fail the comparison
from its first frame. None of the filters touches any other loop-side state.

#### Requirement 5 in detail: the instance table

Connect and disconnect events are coalescing booleans today
([ble_transport_esp32.cpp:33-34](../src/ble_transport_esp32.cpp),
[:341-354](../src/ble_transport_esp32.cpp)), and the header records the weakness: "a
second same-type event arriving inside the check-then-clear window is lost"
([ble_transport.h:66-71](../src/ble_transport.h)). The side-band data —
`s_disconnectReason`, `s_rxBoundaryAtDisconnect`, `s_connHandle` — is single-slot too,
so each event overwrites the last.

Today that is tolerable because `serviceBleEvents()` decides nothing per-connection: a
connect means "reset `rebootFlag`, update MSD, tune the link," a disconnect means
"flush the RX ring to the boundary, raise the cleanup flag"
([main.cpp:461-500](../src/main.cpp)). Under this policy each event drives an
*admission decision about a specific instance*, so a lost event is a lost decision:

- **Lost connect → an unrefused contender.** Two centrals connect while `loop()` is
  blocked in a refresh; the flag is set twice and read once. One is refused; the other
  is connected, never evaluated, and invisible to the loop.
- **Lost disconnect → the slot held by a ghost.** Owner disconnects, then a contender
  connects and disconnects, all within one refresh block. The flag coalesces and the
  side-band identity is the *last* writer's. The loop sees a disconnect that does not
  match the owner, treats it as inert (7b row 3), and never releases the owner. Every
  new client is refused until the idle timeout reclaims the slot — a device-wide
  outage of one full timeout.

**The mechanism is a fixed per-handle instance table, not an event queue.** Sized by
the connection cap: 3 on every ESP32 target here (`CONFIG_BT_NIMBLE_MAX_CONNECTIONS`
is 3 in the precompiled `sdkconfig.h` for S3/C3/C6, and absent for classic ESP32 so
NimBLE's own `#ifndef` default of 3 applies), 1 on nRF. Each entry holds
`(handle, epoch, reason)` — metadata only, ~8 bytes, never frames (see the
one-command-queue constraint above). There is no separate `state` field: liveness *is*
the packed `(handle, epoch)` identity word, per R2's publication rule — all-zero means
empty, so an entry cannot present a live identity with a stale state or vice versa.
There is no `rxBoundary` field either — requirement 6 retires the boundary mechanism,
which is what lets entries be overwritten freely on churn.

Callbacks write their handle's entry. **The loop does not consume a stream of edges; it
scans the table and compares it against its own notion of the owner.** That inverts the
problem and dissolves the overflow question entirely:

- **It cannot overflow.** State is bounded by the connection cap, not by event rate.
  Contender churn overwrites entries for handles that are already gone. There is no
  eviction policy to specify, because nothing is ever queued.
- **Lost edges stop mattering.** A contender that connects and disconnects wholly
  within a refresh block leaves no entry — correct, since there is nothing left to
  refuse.
- **Owner release is a comparison, not an event.** If the owner's `(handle, epoch)` is
  no longer live in the table, the owner is gone, however many edges were missed. This
  makes 7b rows 4 and 7 (stale epoch, duplicate disconnect) inert for free rather than
  by explicit rule.
- **Ghosts stay visible.** Any live entry that is not the owner is a contender still
  needing refusal, and it remains visible until refused — so a missed refusal
  self-corrects on the next pass instead of leaking a connection slot.

*Search the table by handle rather than indexing by it.* NimBLE allocates handles from
0 upward in practice, so direct indexing usually works, but a 3-entry linear search
costs the same at this size and cannot be broken by a stack change that hands out
sparse handles.

nRF needs only requirement 4 of the *callback-filtering* set in practice —
`Bluefruit.begin(1, 0)` configures the SoftDevice for a single peripheral link
([ble_transport_nrf.cpp:164](../src/ble_transport_nrf.cpp)), so cross-central injection
is unreachable at the link layer. Its write callback also discards the handle it is
given ([ble_transport_nrf.cpp:148](../src/ble_transport_nrf.cpp)), which is latent
rather than live. Requirement 6, however, applies to nRF in full: it is
transport-agnostic, because even a single-link target queues frames that can outlive
their session across a disconnect/reconnect pair inside one blocked-loop window.

#### Requirement 6 in detail: tagged frames, and the end of the RX boundary

Today a queued frame is anonymous — `CommandQueueItem` is `{data, len, pending}`
([command_queue.h:72-76](../src/command_queue.h)) — and the disconnect path compensates
with a *boundary*: the callback captures the ring head at link-down
([ble_transport_esp32.cpp:105](../src/ble_transport_esp32.cpp)), and the loop later
discards up to it ([main.cpp:480-489](../src/main.cpp),
`bleRxQueueDiscardTo`, [command_queue.h:112](../src/command_queue.h)). Review found
three defects that are all this one anonymity seen from different angles:

- **The teardown window.** R6's abort runs while the owner token is still held (release
  is its last step, after R3a's wait), so the departing owner's writes keep passing the
  requirement-1 filter and entering the ring *during* its own teardown — after any
  flush the abort performs. If a new owner is then admitted, those frames dispatch into
  the new session.
- **The boundary is losable.** The boundary lives in the departing instance's table
  entry (or today's single slot). If the stack reissues the handle to a newcomer before
  the loop scans — reachable inside one refresh block — the live entry overwrites the
  dead one and the boundary is gone, with the stale frames still queued.
- **Dispatch identity was transport-only.** `g_commandOrigin` says BLE-or-LAN, nothing
  more, so a delayed frame from a dead BLE instance is indistinguishable at dispatch
  from the new BLE owner: it stamps the new owner's R4 activity clock and executes in
  its session.

**The mechanism:** `onWrite` stamps each frame with the packed identity word of the
writing instance — the same word it already loaded for the requirement-1 filter, so
the stamp costs nothing new — and the dispatcher executes a frame **only if its tag
still equals the current owner word** (one atomic load and compare). A mismatched
frame is dropped and counted, never parsed. The invariant: *a frame dispatches iff
its instance was the owner both when it arrived and when it dispatches.*

Consequences, each replacing a patch with a property:

- **The RX-boundary mechanism is retired**: the capture at link-down, the `rxBoundary`
  side-band slot, `takeDisconnectedEvent`'s boundary out-param and
  `bleRxQueueDiscardTo` all go. Stale frames self-discard at dispatch, one compare
  each, however many edges or table overwrites were missed in between.
- **The table-overwrite hazard dissolves.** With no boundary to preserve, handle reuse
  before the loop scans loses only the departed instance's disconnect `reason` — a log
  line, not correctness. No tombstones, no versioned slots.
- **The teardown window closes.** The abort resets both rings outright — sound
  because requirement 1 guarantees every frame in them *passed the owner check when it
  was written* — and a frame the departing owner writes *after* that reset, during the
  R3a wait, carries the departing tag and fails the dispatch check once the token is
  released. This is the same construction that makes an expired R3a wait harmless, and
  it is what finally discharges the rejected `DROPPING` state's residual job.
- **R4's "from the current owner" becomes exact**: tag equals owner word — full
  instance identity, not transport.

**The ring-reset contract (SPSC-safe).** The RX ring is single-producer /
single-consumer: the callback task owns the head, the loop task owns the tail
([command_queue.cpp:23-24](../src/command_queue.cpp),
[command_queue.h:78](../src/command_queue.h)). `bleRxQueueReset()` is therefore
**consumer-side discard only**: acquire-load the head, release-store that snapshot
into the tail, and write neither the head nor any slot. A conventional reset that
wrote both indices or cleared slot contents would race a producer mid-copy — the push
copies payload before publishing the head with a release-store
([command_queue.cpp:92](../src/command_queue.cpp)). A push in flight either published
before the snapshot (discarded with the rest) or after it (survives, carrying the
departing owner's tag, and is dropped at dispatch) — which is exactly why the reset
needs no stronger guarantee than the tag already provides. Two corollaries:

- **Tag publication order:** the tag is written into the slot *before* the
  release-store that publishes the head, exactly like `data` and `len`, or the
  consumer's acquire load cannot be guaranteed to see it.
- **No reset while a peek is outstanding.** The consumer holds a pointer into the
  current slot across dispatch — the dispatcher decrypts in place and only then
  advances the tail ([command_queue.h:78-94](../src/command_queue.h)). Every
  *returning* abort caller is loop-side, after the pass's RX consumption; the one
  in-dispatch caller, deep sleep, never returns to the peeked slot, which is why it is
  safe. Any future abort caller that runs inside dispatch and returns must consume the
  current frame first.

LAN frames do not traverse the BLE ring: the single socket is parsed and dispatched on
the loop task within a pass, its buffer dies with the session (`tcpReceiveBufferPos =
0` in the close seam), and dispatch receives the LAN owner's identity word directly.
The tag is subject to R2's 16-bit epoch wrap argument, trivially: frames live in the
ring for seconds, not the half-hour a collision requires.

### R3a — A firmware-initiated drop waits for the link to go down

**The drop is synchronous: the seam requests termination, then waits — cooperatively
and with a bound — until the link is actually down, before the abort releases the
slot.** Expiry of the bound is an early exit, not a failure: the release then proceeds
anyway and the stale link is inert (see below). "Synchronous" means the release *waits
for* link-down; it does not mean link-down unconditionally precedes it.

> This corrects a first-draft error in two stages. The first draft released the owner
> in the same step as *requesting* the disconnect. The correction introduced a
> `DROPPING` owner state and a cross-pass state machine. That was then investigated
> against the tree and found to be more machinery than the problem needs — see below.

A BLE disconnect is asynchronous. `NimBLEServer::disconnect()` returning true means
termination was *requested* — it returns true even for `BLE_HS_ENOTCONN`/`EALREADY`
(`NimBLEServer.cpp:321-332`). Releasing the token at request time would let a new
connection be admitted while the old link is still physically up.

**Why synchronous, and why it needs no `DROPPING` state.** Three properties of the
current tree make the simple form correct:

- **Link-down is per-handle pollable, without consuming the event.** The disconnect
  callback writes the departing instance's table entry (R3 requirement 5) at the
  moment the link drops, so the wait's predicate is "the owner's `(handle, epoch)`
  entry is no longer live" — a scan of the instance table. **The aggregate
  `connectedCount()` must NOT be the predicate**: it is the stack's total peer count
  on both targets ([ble_transport_esp32.cpp:257-259](../src/ble_transport_esp32.cpp),
  [ble_transport_nrf.cpp:235-239](../src/ble_transport_nrf.cpp)), and R1 explicitly
  permits a refused contender to be transiently attached — dropping the owner then
  moves the count 2→1, never to 0, and the wait sits out its full bound on a link
  that is already down. The disconnect *event* stays queued for `serviceBleEvents()`
  to consume on its normal path — the wait neither consumes nor reorders it. (What
  survives of that path is the event flow, flag and reason; its RX-boundary capture is
  retired by R3 requirement 6.)
- **The wait ticks on a short plain `delay()`, not `idleDelay()`.** A first draft
  named `idleDelay()` the right primitive for its early-out on `ble.eventPending()`
  ([main.cpp:749](../src/main.cpp)). That early-out is exactly wrong here: the
  predicate is table state, not event arrival, and mid-teardown events are
  deliberately left unconsumed — so once any event is pending (the owner's own
  disconnect, or an unserviced contender's connect), every `idleDelay()` call returns
  immediately and the wait degrades into a busy spin for its remaining bound. A plain
  `delay(2)` tick services *neither* RX nor transport events — the safety property
  actually wanted — and costs a few milliseconds of latency against a bound sized in
  tens of them. The abort is loop-task-only and already deferred while
  `epdRefreshInProgress` ([main.cpp:389](../src/main.cpp)), so the wait can never land
  inside a refresh.
- **R2's epoch already provides what `DROPPING` was providing.** If the bounded wait
  expires and the slot is released with the old link still up, that link is inert *by
  construction*: its writes are filtered as non-owner (R3 requirement 1) and its late
  disconnect is inert on stale epoch (table 7b rows 4 and 9). A stale link cannot reach
  the new session. `DROPPING` was belt-and-braces over a guarantee R2 already makes.

**Transport dispatch.** A firmware-initiated drop acts on the *owner's* transport,
which the token records — R6's teardown can fire on a LAN owner (the transfer watchdog
is origin-agnostic), and dropping a BLE handle there would leave the owning socket
alive while its token is released, violating R1. `OWNER_BLE` drops through the seam
with the bounded wait above; `OWNER_LAN` stops the TLS context and closes the client
socket, which is synchronous — the wait, and the asynchrony problem this rule exists
for, are BLE-only.

**Timing.** An alive peer terminates within a few connection intervals — tens of
milliseconds; the firmware requests no interval, so the central's negotiated value
applies. A peer that is already gone would cost the remainder of the supervision
timeout, but such a peer is reaped by the link layer at ~4–6 s, far short of the 120 s
idle timeout, so that case is effectively unreachable at the drop site.

**The bound does not disappear — it relocates**, from "how long before we force-release
a wedged token" to "how long we wait before proceeding anyway." It is far less
load-bearing in the second form: expiry is not a failure needing recovery, just an
early exit into an abort that was going to run regardless. Sizing it is an open
question; it wants to cover a few connection intervals with margin, not a supervision
timeout.

**Phase 4 composes with this rather than fighting it.** The auth-abuse drop already
requires a bounded cooperative wait before disconnecting, to deliver its final `FE`.
Both are the same shape: cooperative wait, bounded, proceed on expiry.

### R4 — Each transport enforces an idle timeout, ungated by transfer state

**Idle** is defined as, and only as:

```
idle  :=  no inbound command from the owner on the owning transport
          AND no refresh in progress
```

**The idle timeout is NOT gated on a transfer being in progress.** This is deliberate
and is the rule's whole point: a client that goes silent *during an image upload* is
exactly the case that wedges the device today, and a `!transferActive()` gate would
exempt it. An in-flight transfer confers no protection; only inbound traffic does.

Consequences, stated because they are the cost of the rule:

- A silent client mid-upload **is dropped** and its partial transfer discarded by R6's
  abort. Partial upload state is never preserved across a drop.
- Any client whose legitimate inter-command gap can exceed the timeout will be dropped
  mid-transfer.

**Default: `OD_BLE_IDLE_TIMEOUT_MS = 120000` (120 s).** Set deliberately generous
because this rule made the direction of that error worse: while the drop was gated on
`!transferActive()` a short timeout only killed idle sessions, but with the gate gone
a short timeout kills legitimate *uploads*. The cost of being generous is bounded and
lands on one case only — a returning client waits up to 120 s if a stale-but-*alive*
incumbent holds the slot. A client that is genuinely gone is reaped by the link layer
in ~4–6 s (the firmware sets no supervision timeout, so the central's negotiated value
applies), so the lockout never applies to a crashed or out-of-range peer.

**What counts as activity.** A frame must reach the dispatcher and be **recognised as
a command from the current owner** — where "from the current owner" is full instance
identity, not transport: the frame's R3-requirement-6 tag must equal the owner word. A
transport-level test is insufficient, because a delayed frame from a dead BLE instance
is indistinguishable from the new BLE owner by transport alone and would stamp the new
owner's clock. (In practice the dispatch tag check has already dropped such a frame
before the stamp is reached; the stamp's own identity test is one redundant compare.)

> The first draft said "successfully queued or parsed," and pointed at
> `bleRxQueuePush()`'s success path. That is wrong and self-contradictory: the queue
> accepts any non-empty payload within the size cap
> ([command_queue.cpp:50-93](../src/command_queue.cpp)) — including a one-byte
> malformed frame or an unknown opcode, which the dispatcher only rejects later
> ([communication.cpp:544,754](../src/communication.cpp)). Stamping on queue success
> leaves a garbage flooder able to hold the slot indefinitely, which is precisely the
> failure the rule exists to prevent.

Two clocks in the tree are unusable as-is and must be fixed rather than reused:

- `pollActivity` stamps `lastActivityMs` whenever `connCount > 0`
  ([main.cpp:366](../src/main.cpp)) — a live-but-silent link never ages.
- LAN stamps `lastLanActivityMs` on `got > 0`, i.e. any bytes read
  ([wifi_service.cpp:946](../src/wifi_service.cpp)) — a flooder holds the slot with
  garbage.

**The clock must not run during a refresh.** `epdRefreshInProgress` brackets a
*blocking* call on the loop task ([display_service.cpp:2446-2467](../src/display_service.cpp),
[:3358-3368](../src/display_service.cpp)): `loop()` does not execute for the refresh's
duration, but wall-clock time passes. A naive `millis() - lastRx` accrues the whole
refresh and can drop an actively engaged client the moment `loop()` resumes.

Implementation requirements for the exclusion:

- **A loop-side edge detector cannot see the edge** — both transitions happen inside
  the blocking handler. The re-stamp must be invoked *at* the transition, via a single
  `endRefresh()` helper that both refresh sites call, not by polling the flag.
- **Re-stamp the current owner's clock only**, and only if the same instance identity
  still owns the slot. ("Both transports" is harmless under a perfect R1 but hides the
  identity requirement, and R1 is exactly what is being built.)
- Re-stamping can only ever *delay* a drop, never cause a spurious one.

**Ordering constraint.** On LAN, inbound bytes may be sitting in the socket when the
deadline is evaluated. The timeout check must run **after** the transport has had its
chance to parse this pass, or an active LAN client is dropped with its command already
in the buffer. BLE avoids this by stamping from callback context on arrival; LAN must
parse first. See R7d.

**Baseline.** The idle window is measured from the later of admission and last inbound
command, so a freshly admitted client gets a full window before its first command. On
LAN, admission is TCP accept, but the baseline starts at **TLS handshake completion**
(see R7a note), since handshake traffic is not a command.

**Per transport.** Each transport enforces its own timer and constant. LAN already has
one (`OD_LAN_READ_TIMEOUT_S` = 30 s, [wifi_service.cpp:952](../src/wifi_service.cpp)),
already ungated by transfer state, so LAN needs only the recognised-command stamping
and the refresh exclusion. BLE has no idle drop at all and needs the whole mechanism.

### R5 — A stuck refresh is a separate problem with a separate watchdog

R4 excludes refresh from idleness, so a refresh that never completes is **not** caught
by the idle timeout. That exposure is handled by a **refresh watchdog**, deliberately
*not* part of this policy but named here so it is tracked rather than assumed away.

Scoping it honestly, from the code:

- On the **`bb_epaper` polling path only**, the BUSY wait is bounded:
  `waitforrefresh(60)` loops `timeout * 100` times at 10 ms and then returns failure
  ([display_service.cpp:803-831](../src/display_service.cpp)).
- **On the FastEPD path there is no bound at all.** `waitforrefresh()` delegates
  immediately to `fastepd_wait_refresh()` (`:805`), which ignores its timeout argument
  outright — `(void)timeout_sec; return !s_init_failed;`
  ([display_fastepd.cpp:277-280](../src/display_fastepd.cpp)). The real blocking lives
  inside `fullUpdate()`/`fastUpdate()`, above that call and unbounded.

  > The first draft claimed the BUSY wait was bounded at 60 s generally. It is not.
  > On FastEPD targets the naive "panel never signals done" case is fully exposed.

- The residual exposure elsewhere is the driver call itself — `bbepRefresh()`,
  `fastepd_direct_refresh()`, `fastepd_partial_refresh()` — plus any SPI-level stall
  inside it, none bounded by the poll loop above it.
- **No loop-serviced watchdog can observe any of this**, because `loop()` is blocked
  for the refresh's entire duration. The watchdog needs an independent timebase: a
  hardware watchdog fed from `loop()`, a timer ISR, or a separate task.
- **The supervisor must recover from a safe context.** A timer ISR can *observe* a
  stuck refresh but must not run panel/SPI teardown from interrupt context; the
  realistic recovery is an MCU reset.
- There is no refresh start timestamp in the tree; the watchdog must add one.

### R6 — Every non-refused disconnect calls `abortToKnownState()`

**Any disconnect of the current owner — on any transport, for any reason, whether
client-initiated, link-layer, or firmware-initiated — runs `abortToKnownState()`**,
leaving the device ready for a new connection.

`abortToKnownState()` must leave, at minimum: no active direct-write, partial, pipe or
chunked-config transfer; touch resumed; encryption session cleared; RX and TX rings
drained of the departed session's traffic; the owner token released — except for the
terminal caller (7e row 3), where the word was exchanged to `TERMINAL` before the
abort and the release, called with the displaced identity, is deliberately inert:
there the postcondition is "the slot is not claimable", which the gate satisfies. Both rings are
drained by outright reset — sound because R3 requirement 1 means every frame in them
passed the owner check when it was written — and a frame the departing owner writes
*after* the reset, during R3a's wait, is covered by requirement 6's dispatch tag
check rather than by re-flushing. The RX reset must honour requirement 6's SPSC
contract: consumer-side discard only.

**Buzzer and LED are explicitly NOT stopped.** They are user-facing *effects*, not
session state. Firing a buzz and immediately dropping the link is a normal pattern —
command, then disconnect to save power — and truncating it defeats the command. A
playing melody cannot corrupt or confuse a later connection the way a half-open pipe
session, a suspended touch input or a live crypto session can, and both are bounded
and self-terminating: the buzzer's `outer` repeat count is a `uint8_t` coerced to at
least 1, with playback stopping at `rep >= outer`
([buzzer_control.cpp:215-217,288-291](../src/buzzer_control.cpp)); the LED runs a
stepped pattern to completion ([device_control.cpp:530-541](../src/device_control.cpp)).
Since this policy fires the abort far more often than a plain disconnect once did,
stopping them would be a correspondingly more visible regression. A WARM (post-refresh keep-alive) panel **survives** — the abort tears down
only a mid-transfer `PWR_ACTIVE` session, preserving the existing ACTIVE-only-teardown
invariant ([main.cpp:411-415](../src/main.cpp)). It runs on the loop task, is
idempotent, and is deferred while `epdRefreshInProgress`
([main.cpp:389](../src/main.cpp)).

**Exceptions — R6 does not apply to:**

1. **A refused contender** (R3). It was never the owner; its disconnect is inert.
2. **Terminal transitions**, where "ready for a new connection" is meaningless because
   the MCU is about to reset, sleep, or lose power. These paths disconnect the link
   and then leave, so no loop pass will ever service the event:
   - nRF DFU: `Bluefruit.disconnect()`, `delay(100)`, `sd_softdevice_disable()`, jump
     to bootloader ([device_control.cpp:847-866](../src/device_control.cpp)).
   - ESP32 DFU/reboot: BLE teardown then immediate restart
     ([device_control.cpp:880](../src/device_control.cpp)).
   - Power-latch off ([device_control.cpp:942](../src/device_control.cpp)) — power can
     be physically removed before any teardown.

   For these, either accept the exception as stated, or call
   `abortToKnownState(dropLink=false)` **synchronously before** the teardown/jump/sleep.
   Choose per path; the exception is the default. What is *not* acceptable is the first
   draft's unqualified "every disconnect," which these paths simply falsify.

### R7 — Permutation tables

Normative. Any combination not listed is a specification gap, not implementer's
discretion.

#### 7a — Admission

"Refuse" means: disconnect/close the contender and change nothing else (R3).

| # | Owner state | Incoming | Action | Owner after | Abort? |
|---|---|---|---|---|---|
| 1 | `NONE` | BLE connect `(h, e)` | Admit; `claim(BLE, h, e)` | `ACTIVE BLE(h,e)` | no |
| 2 | `NONE` | LAN accept | Admit; `claim(LAN, 0, e)` at **TCP accept** | `ACTIVE LAN(0,e)` | no |
| 3 | `ACTIVE BLE(h1,e1)` | BLE connect `(h2, e2)` | **Refuse** `h2` | unchanged | no |
| 4 | `ACTIVE BLE(h1,e1)` | BLE connect `(h1, e2)` — handle reused after a stale link | **Refuse** — epoch differs, so this is a new instance despite the matching handle (R2) | unchanged | no |
| 5 | `ACTIVE BLE(h1,e1)` | LAN accept | **Refuse**: `incoming.stop()` | unchanged | no |
| 6 | `ACTIVE LAN(0,e1)` | BLE connect `(h,e)` | **Refuse** `h` | unchanged | no |
| 7 | `ACTIVE LAN(0,e1)` | LAN accept | **Refuse**: `incoming.stop()` | unchanged | no |
| 8 | `ACTIVE`, drop in flight | any | **Refuse** — the slot is still held until the synchronous wait completes (R3a) | unchanged | no |
| 9 | `NONE` | two or more connects/accepts race the free slot | The claim CAS serializes them: exactly one wins whatever tasks they arrive on; every loser is a contender, refused | `ACTIVE` (the CAS winner) | no |
| 10 | `NONE` (just released) | a previously refused contender, link still up | **Stays refused** — admission is decided once, at the instance's own connect hook, and never revisited | `NONE` until a new instance connects | no |
| 11 | `TERMINAL` (7e row 3) | any connect/accept | **Refuse** — the claim CAS fails against the terminal word; the stack is about to go down | `TERMINAL` | no |

**Admission is decided exactly once per instance, at its connect hook — and never
re-evaluated.** Rows 9 and 10 close a gap review found in this table, which modelled
only one arrival at a time. The rule is the mechanical consequence of the CAS design
made normative: an instance attempts the claim exactly once, in its own connect
callback or accept, and there is no code path that retries it later. Three cases
follow without further rules:

- **Racing arrivals need no tiebreak.** Two connects during a blocked refresh, or a
  BLE connect racing a LAN accept, are serialized by the word itself — the CAS winner
  is the owner regardless of which task got there first or what order the loop later
  scans the table (row 9). "BLE before LAN" was never a winner rule; see R7d.
- **A freed slot is claimable only by instances that connect after the free.** A
  contender that arrived while the slot was held lost its one CAS and is being
  disconnected; it cannot inherit the slot however long its physical link lingers
  (row 10). The client behind it simply reconnects, and its *new* instance claims.
  This keeps R3 absolute — "a contender is always refused" has no asterisk for slots
  that free up later — and it is what makes the loop's refusal scan order-free and
  idempotent: re-refusing a doomed entry is always correct, admitting one never
  happens there.
- **The table scan never admits.** The loop's only admission-adjacent job is refusal
  of CAS losers (R7d step 2); every claim happens at a hook. An implementation that
  admits from the scan — e.g. "slot is free and this entry looks live, claim it for
  them" — violates this table even when it happens to pick the right instance.

Row 7 is a **behaviour change**: LAN accept is unconditional last-in-wins today
([wifi_service.cpp:869-877](../src/wifi_service.cpp)). It matters more than the BLE
rows because LAN-TLS bypasses app-layer auth by design, so today any host on the
network can displace an in-flight push by opening a socket, with no credentials.

**LAN claims at TCP accept, before the TLS handshake.** The handshake is driven
incrementally across later loop passes ([wifi_service.cpp:905](../src/wifi_service.cpp)),
so deferring the claim until it completes would leave the slot free for a BLE connect
or a second socket in the meantime. Consequently:

- A second accept *during* the handshake is refused (rows 5/7 apply).
- **TLS handshake failure is an owner disconnect** — it runs R6's abort and releases.
- Handshake traffic is **not** activity for R4; the idle baseline starts at handshake
  completion.
- **A handshake deadline is deferred, not required.** An earlier draft required the
  handshake carry its own bounded deadline. Deferred: a socket stuck mid-handshake is
  already reclaimed by `OD_LAN_READ_TIMEOUT_S`, because the idle baseline does not
  start until the handshake completes, so a handshake that never finishes leaves the
  clock at its accept-time stamp and the existing 30 s drop fires. A dedicated
  deadline would only tighten that window, which is not worth a second tunable until
  something shows 30 s is too slow.

Rows 3–8 are what make R1 true on ESP32, where the link layer will not. nRF enforces
rows 3–4 at the link layer via `Bluefruit.begin(1, 0)`
([ble_transport_nrf.cpp:164](../src/ble_transport_nrf.cpp)); firmware must still
implement them so behaviour is identical across targets and so rows 5–6 work at all.

#### 7b — Disconnect

**Generic rule, which the rows below instantiate:** *for any owner state, a disconnect
whose full instance identity does not match the owner is inert.*

| # | Owner | Disconnect identity | Action | Owner after | Abort? |
|---|---|---|---|---|---|
| 1 | `ACTIVE BLE(h1,e1)` | matches | `abortToKnownState(dropLink=false)`, whose **final** step releases | `NONE` | **yes** |
| 2 | `ACTIVE LAN(0,e1)` | matches | `abortToKnownState(dropLink=false)`, whose **final** step releases | `NONE` | **yes** |
| 3 | any `ACTIVE` | refused contender `(BLE, h2, ·)` | Inert (R3) | unchanged | no |
| 4 | `ACTIVE BLE(h1,e1)` | `(BLE, h1, e0)` — stale epoch | Inert — late event from a prior instance (R2) | unchanged | no |
| 5 | `ACTIVE LAN` | any BLE identity | Inert (cross-transport) | unchanged | no |
| 6 | `ACTIVE BLE` | any LAN identity | Inert (cross-transport) | unchanged | no |
| 7 | any | duplicate of an already-consumed identity | Inert (idempotent) | unchanged | no |
| 8 | `NONE` | any | No-op | `NONE` | no |
| 9 | `NONE` | the link a timed-out synchronous drop left up (R3a) | Inert — already released and aborted; the identity is stale by then | `NONE` | no |

`dropLink=false` throughout: the link is already gone. Row 4 is the ABA case the epoch
exists to catch, reachable in practice because a disconnect event can be serviced tens
of seconds late when `loop()` was blocked in a refresh, by which time the handle may
have been reissued.

Rows 3–7 all collapse to the generic rule above; they are enumerated because each was a
distinct hazard before the epoch made them uniform. Note there is no row for a
firmware-initiated drop completing: R3a's synchronous wait means the abort and release
have already happened inside the drop, so the event that follows is just row 9.

#### 7c — Idle timeout

| # | Owner | Refresh in progress | Inbound silence | Action | Abort? |
|---|---|---|---|---|---|
| 1 | `ACTIVE` | no | `>` timeout | Drop via the owner's transport (BLE: the seam + R3a wait; LAN: synchronous socket close), then abort and release — all within the one pass | **yes** |
| 2 | `ACTIVE` | no | `≤` timeout | Nothing | no |
| 3 | `ACTIVE` | **yes** | any | Nothing — not idle by definition (R4); `endRefresh()` re-stamps the owner's clock | no |
| 4 | `NONE` | any | n/a | Nothing — no timer runs without an owner | no |

Row 1 applies **whether or not a transfer is in flight** (R4).

#### 7d — Within-pass ordering

**Normative, because without it two conforming implementations pick different
winners.** The current loop order is `serviceBleEvents()` → BLE RX → deferred
disconnect cleanup → LAN accept/read ([main.cpp:624](../src/main.cpp)), and connect and
disconnect flags are consumed connect-first regardless of actual arrival order
([main.cpp:461-471](../src/main.cpp)) — which is exactly the ambiguity this section
removes.

Within one loop pass, evaluate in this order:

1. **Owner disconnects** (7b) — the abort first, whose *final* step releases, so a
   slot freed this pass is available to an admission decision in the same pass.
   Release is never before the abort: a claim CAS can succeed the instant the word is
   zeroed, and an abort still running after that (its ring resets included) would then
   tear down the *new* session's state.
2. **Contender refusal, and the LAN accept** (7a). Admission itself is the hook-side
   CAS (R2): for BLE it already happened — or failed — in the connect callback, so
   this step only *refuses* live instances whose CAS failed. The LAN accept runs here
   because the loop is its earliest hook, and its claim is the same CAS. No loop-side
   rule picks a winner between transports; the word does.
3. **Inbound traffic**, which stamps the activity clock.
4. **Idle timeout** (7c) — last, so traffic parsed in step 3 counts. This is what
   satisfies R4's ordering constraint for LAN.

**The authoritative arbitration point is the earliest transport hook — the BLE connect
callback and the LAN accept — not the loop.** Fixed loop ordering cannot reconstruct
true cross-transport arrival order (a BLE connect during a refresh and a LAN socket
queued in the listen backlog are not comparable by the time `loop()` resumes), and it
must not be relied on for correctness. It resolves *ties within a pass* only; the claim
itself must be atomic at the callback — mechanically, the one-word owner CAS of R2.
Where the two disagree, the callback wins.

#### 7e — Terminal transitions

Per R6 exception 2. "Sync abort" = call `abortToKnownState(dropLink=false)`
synchronously before the transition.

| # | Transition | Link handling | R6 abort |
|---|---|---|---|
| 1 | nRF DFU entry | `Bluefruit.disconnect()` + 100 ms, then SoftDevice disable | Exempt (or sync abort) — MCU jumps to bootloader |
| 2 | ESP32 DFU / reboot | BLE teardown, immediate restart | Exempt — MCU resets |
| 3 | Deep sleep (forced or idle) | `linkMarkTerminal()`, then sync abort, then `ble.end()` | **Sync abort — required, not exempt.** Not because state survives sleep (it does not; see below) but because sleep is a *mid-session exit* whose path otherwise hand-rolls a private teardown subset that drifts from the real one. The terminal gate must precede the abort — see the ordering trap below |
| 4 | Power-latch off | Power removed | Exempt — nothing survives |

**Row 3 is resolved: deep sleep calls `abortToKnownState()`, and the reason is
teardown uniformity at a mid-session exit — not surviving state.**

> **Corrected rationale (review, twice).** An earlier revision justified this row
> with "deep sleep is not a reset; touch-suspend, panel power and the owner token all
> survive it." That is false: ESP32 deep-sleep wake re-enters `setup()` and reloads
> RAM from the image — only `RTC_DATA_ATTR` state survives, as the boot code itself
> records ([main.cpp:129-150](../src/main.cpp)) — so the owner token, the
> touch-suspend counter and every transfer flag are rebuilt clean on wake. A second
> attempt justified it with lingering *hardware* state instead; also wrong against
> the tree: the sleep path already forces the panel rail off unconditionally before
> sleeping ([main.cpp:812](../src/main.cpp), `epdSessionForceOff()` at
> [display_service.cpp:420](../src/display_service.cpp)), touch is re-initialised on
> wake ([main.cpp:238](../src/main.cpp)), and deep-sleep pad hold is enabled only for
> the power-latch pin ([power_latch.cpp:59](../src/power_latch.cpp)).
>
> The real reason the abort is required: **deep sleep is a mid-session exit** —
> forced sleep bypasses the live-link guard ([main.cpp:789](../src/main.cpp)) and the
> path does not arbitrate a LAN owner, so it can begin with a transfer in flight —
> and its path already hand-rolls a private teardown (panel force-off, advertising
> stop, stack end, and now effect silencing). Without the abort, that private subset
> must be kept in sync with the real teardown forever, and every session resource
> added later must be added in both places — the same drift hazard that made the
> transfer watchdog an abort caller. Routing the session half through
> `abortToKnownState()` first makes sleep's teardown identical to every other
> session end by construction.

The division of labour is exact, and mirrors the buzzer/LED rule: the abort runs
first and does *session* teardown only (ACTIVE-only panel handling, effects
untouched); the sleep path then does its own *sleep* quiescing — panel force-off
**including WARM** (no panel stays powered through sleep, which is why
`epdSessionForceOff()` stays in the sleep path and must never move into the abort)
and buzzer/LED silencing. The two compose; neither substitutes for the other.

**One ordering trap, found by review: gate admission *before* the abort.** The
abort's final step releases the token, deep sleep passes `dropLink=false`, and the
owner's link can still be physically up — so between the release and `ble.end()`, a
connect on the host task could win the freed word, and the new owner would then be
destroyed by the stack teardown with no abort ever run for it. The deep-sleep path
therefore calls **`linkMarkTerminal()`** (R2) — an unconditional atomic exchange of
the owner word to the reserved `OWNER_TERMINAL` encoding, returning the displaced
owner identity — *before* `abortToKnownState()`, and hands that displaced identity to
the abort (after the exchange, reading the word yields terminal, not the departing
owner, so the abort must not re-derive it). Claims fail against any nonzero word, so
admission is impossible from that point; the abort's release, called with the
displaced identity, finds the word not matching and is naturally inert; wake reloads
RAM and the word starts clean. This also amends the plan's `dropLink=false` invariant: false means "no drop
wanted from the abort", satisfied either because the link is already gone or because
the stack is about to be torn down behind a terminal gate.

*Interaction with the buzzer/LED carve-out (R6).* The abort deliberately leaves buzzer
and LED running, and deep sleep cuts the clocks they depend on — so at this one
transition the "let the effect finish" rationale cannot hold, because the effect
*cannot* finish.

**Resolved: deep sleep silences both.** Of the two consistent options — make sleep
*wait* for a playing effect via the `workInFlight` gate ([main.cpp:694-699](../src/main.cpp)),
or *cut* the effect on the way down — this policy takes the second. Sleep is not delayed
by a playing effect; the effect is stopped immediately before the MCU sleeps.

The deciding argument is hardware state, not policy symmetry. `enterDeepSleep` runs
`ble.stopAdvertising()` / `delay(200)` / `ble.end()` / `delay(100)` and then
`armButtonWakeSources()` and `powerLatchHoldForSleep()`
([main.cpp:806-836](../src/main.cpp)) — all outside `loop()`, so `buzzerService()` never
ticks during it. A tone left sounding is therefore *not* a melody finishing gracefully:
it is a driven pin held through the teardown and then into sleep, sounding continuously
and drawing current until the next wake. Letting sleep wait would merely delay that;
stopping is the only outcome that leaves the pins in the state sleep expects.

**Three scoping rules this must not be over-generalised into:**

1. **It lives in the deep-sleep path, never in `abortToKnownState`.** R6's carve-out is
   unchanged: an idle, auth-abuse or watchdog drop still leaves a melody playing.
2. **It applies to deep sleep only, not to every terminal transition.** In particular
   **power-latch off (row 4) deliberately *plays* a chirp on the way down** —
   `passiveBuzzerPowerOffAlert()` is called immediately before `powerLatchTriggerOff()`
   ([device_control.cpp:83](../src/device_control.cpp)). A blanket "silence at every
   terminal transition" would delete that alert.
3. **Deep sleep is ESP32-only** (`enterDeepSleep` sits inside `#ifdef TARGET_ESP32`,
   [main.cpp:757](../src/main.cpp)), so this adds no nRF obligation.

*Implementation note.* Both stop routines exist but are file-static —
`buzzer_stop_internal()` ([buzzer_control.cpp:147](../src/buzzer_control.cpp)) and
`led_stop_internal(bool clear_mode)` ([device_control.cpp:347](../src/device_control.cpp)) —
so this needs two thin public wrappers. They are *sleep* APIs, not session-teardown APIs,
and nothing in the abort may call them.

---

## What this changes

Relative to the tree:

1. No connection-level arbitration exists; it must be built (R1, R2).
2. ESP32 admits up to three centrals with no check, and a contender can corrupt the
   incumbent's subscribe state, inject into its RX ring, and **receive its
   notifications** (R3).
3. LAN accept evicts rather than refuses (R3, 7a row 7).
4. No BLE idle drop exists; LAN's exists but stamps on raw bytes and runs through
   refreshes (R4).
5. Disconnect events carry no identity, and connect/disconnect events coalesce (R3).
6. **No *BLE* disconnect path clears the encryption session**, and LAN's clearing is
   conditional and divergent — cleared at [wifi_service.cpp:798](../src/wifi_service.cpp)
   only when `wifiClient.connected()`, and again on replacement at `:874`. Several
   teardown paths are open-coded and drift-prone (R6).
7. No refresh start timestamp and no independent timebase exist for R5; the FastEPD
   refresh path has no timeout bound whatsoever.
8. Queued frames carry no instance identity
   ([command_queue.h:72-76](../src/command_queue.h)); the disconnect path compensates
   with a captured RX boundary ([main.cpp:480-489](../src/main.cpp)), which R3
   requirement 6 replaces with per-frame tags.

**Relative to `PLAN_FREEZE_HARDENING_2026-07-31.md`: reconciled.** That plan was revised
against this document and now schedules it rather than diverging from it; its
"Conformance" table maps each rule to the phase that builds it. The four points this
document previously superseded have been discharged in the plan — the `!transferActive()`
gate removed (R4), the owner token given a per-instance epoch (R2), the callback boundary
extended to identity-bearing *disconnect* events plus subscribe/notify filtering and the
instance table (R3), and the drop made synchronous (R3a).

Two rules remain unscheduled and are named as such in the plan rather than absorbed:
**R5** (refresh watchdog) is out of scope and recorded under its residual risk, with the
unbounded FastEPD path called out; **R7e** (terminal transitions) is covered only for
deep sleep, which the plan adds to its abort invocation set — the other three rows keep
the exemption R6 grants them.

This ordering does not change: where the two disagree, this document still wins.

## Open questions

- **The R3a wait bound** — the one unset number in this policy, and the least
  load-bearing. It is a wait bound rather than a recovery deadline: expiry is an early
  exit into an abort that runs regardless, and R2's epoch makes the stale link inert.
  Wants to cover a few connection intervals with margin — tens to low hundreds of
  milliseconds — not a supervision timeout. Everything else here is settled; this can be
  picked at implementation time without reopening the policy.

**Settled — the timeout values.** Both second-denominated timeouts are decided, and
neither is gated on a measurement before implementation:

- **BLE, `OD_BLE_IDLE_TIMEOUT_MS` = 120 s.** A chosen value, not a measured one, set
  deliberately generous because R4 inverted the direction of the error: with the transfer
  gate gone, erring short costs a legitimate upload rather than a stale session. The
  accepted cost is a returning client waiting up to 120 s behind a stale-but-*alive*
  incumbent; a client that is genuinely gone is reaped by the link layer at ~4–6 s, so
  the lockout never applies to it.
- **LAN, `OD_LAN_READ_TIMEOUT_S` = 30 s, unchanged.** It already satisfies R4's substance
  — it is ungated by transfer state — so R4 changes only its *semantics*: stamp on a
  recognised command rather than on raw bytes read, and exclude refresh. Its *value* is a
  client-visible wire-header contract and is out of bounds here regardless.

The asymmetry between them is deliberate and follows from where each is allowed to live.
What remains is drift detection in `py-opendisplay`'s CI, not verification of the numbers
— a client change that pushed legitimate inter-command silence toward either value would
fail there.

**Resolved — deep sleep vs the buzzer/LED carve-out** (7e row 3). Deep sleep **silences
both**, in the deep-sleep path and never in the abort; sleep is not delayed by a playing
effect. The rationale is hardware state — `buzzerService()` does not tick during
`enterDeepSleep`, so a tone left on sounds continuously into sleep rather than finishing
— and the scoping (abort unchanged, power-latch off keeps its chirp, ESP32-only) is
recorded at 7e row 3.

**Resolved since the first draft — event delivery.** The first draft required
"non-coalescing event delivery" and left the queue's overflow behaviour as an open
question. There is no queue: R3 requirement 5 now specifies a fixed per-handle instance
table that the loop scans, so there is nothing to overflow and no eviction policy to
decide. See also the one-command-queue constraint at the top of this document.

**Resolved since the first draft:** `checkTransferTimeouts()` **does** route through
`abortToKnownState(dropLink=true)`. R6 governs disconnects and the watchdog is not one,
so this is an extension of R6's *teardown* to a non-disconnect trigger rather than a
consequence of it: there is one teardown routine and the watchdog uses it. The
rationale, the three behaviour changes it brings, and the one branch deliberately left
out (the orphaned-pipe invariant repair) are recorded in the freeze-hardening plan's
invocation set.
