# Adversarial Review — PLAN_PHASE1_NONCE_REPLAY_2026-07-26

**Date:** 2026-07-26 · **Reviewing:** [`PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md`](PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md) (branch `debug/ble-hardening`)
**Scope:** `src/encryption.cpp`, `src/encryption_state.h`, their callers, and the two clients that
drive them (`py-opendisplay`, the web client `ble-common.js`)

> Companion to [`FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md`](FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md).
> Where the two disagree, this document is the later and more specific analysis — in particular
> it **retracts** part of that review's `[M1]` (see `M2` and `L1`).

**Independently re-verified after the review was produced** (not taken on trust): `C1`'s
selective-repair transmit site and `max_retx` formula; `H1`'s `IntegrityCheckError` path and the
pipe loop's single `except`; `H2`'s shared nonce space in `encryptResponse`; `H3`'s
unconditional ring write; `L3`'s arithmetic; `L5`'s 11-leg CI matrix. All confirmed as
described.

---

## Verdict

**Phase 1 is safe to ship alone — no new freeze mode, no new remote DoS, no regression on either
target — but not as written.** Two edits are mandatory (`C1`, `M1`), one is strongly recommended
(`H1`), two need a paragraph each (`H2`, `H3`).

D1–D4 are real (three exactly as described, one overstated), the check/commit split is the
correct shape, and the sliding-bitmap state machine is correct as specified. `−496 B`,
`+1,536 B`, `PIPE_MAX_W` 32/16, `MAX_PTO = 3`, the `protocol.h:384` citation and the "one counter
burned per transmission" premise all check out. Three things do not: `C1`, `H1`, `H2`.

**Findings: 1 Critical, 3 High, 3 Medium, 7 Low.**

---

## CRITICAL

### C1 — `OD_NONCE_FORWARD_CAP = 64` is derived from a bound that does not hold

**Plan element:** Decision A — *"worst-case unseen run = `PIPE_MAX_W + MAX_PTO` = 32 + 3 = 35 …
This is a **hard bound, not an estimate**"* — plus Step 6, which instructs writing that sentence
into `communication.cpp` as the invariant future changes are checked against.

**What the derivation assumes:** *"After the window is exhausted it blocks for an ACK; on timeout
it resends exactly one chunk (`_send(window_base)`), never another window."* Accurate for the
**PTO** path ([device.py:2715-2726](../../py-opendisplay/src/opendisplay/device.py)) and for
**new** sends (`:2688-2694`, gated on `(next_to_send - window_base) < window`). It ignores the
third transmit site.

**The site it ignores** — selective repair, which spends no window credit at all:

```python
device.py:2789-2796
                for m in missing:  # oldest first
                    ...
                    if do_retx:
                        await _send(m)
                        pending_retx[m] = 0
                        retx_count += 1
```

`missing` is every hole below `highest_recv` (`device.py:2771-2772`), up to `W-1` entries. Each
hole is re-sent again every `PIPE_RETX_ACK_SPACING = 2` ACKs (`commands.py:99`,
`device.py:2782-2788`). The only cap on the total is
`max_retx = max(3*window, ceil(n * 0.5))` — **96** for `W = 32`, and far larger for a
multi-thousand-chunk upload (`device.py:2672`, `commands.py:96`).

**And the client deliberately hoards ACKs to spend on those rounds:**

```
device.py:781   Passes ``drain_stale=False`` so queued sliding-window ACKs are preserved.
device.py:786   await self._conn.write_command(self._encrypt_frame(data), response=response, drain_stale=False)
```

So `B` backlogged ACKs, processed while the device is not consuming anything (mid-`bbepWaitBusy`,
or with the 33-slot command ring full — [esp32_ble_callbacks.h:119-129](../src/esp32_ble_callbacks.h),
which drops on the NimBLE host task *before* decrypt), buy `⌈B/2⌉` repair rounds of up to `H`
holes each. Firmware supplies the backlog: it ACKs every `ack_every` accepted frames
([display_service.cpp:2854](../src/display_service.cpp)) **and** once per `ack_every`
out-of-order arrivals plus one immediately when a gap opens (`:2875-2892`).

**The arithmetic.** With `W` in flight, `H` holes and `N = ack_every`, the device emits about
`(W−H)/N` ACKs before it stalls, and the client can spend them on `⌈(W−H)/(2N)⌉` repair rounds:

| `W` | `N` | worst `H` | repair transmissions | + PTO probes | total gap |
|---|---|---|---|---|---|
| 32 | 8 (py default, `device.py:497`) | 16 | 16 | 2 | ~18 |
| 32 | 4 (HA default, `const.py:51`) | 20 | 40 | 2 | ~42 |
| 32 | **2** | 16 | 16 × 4 = **64** | 2 | **66** ✗ |
| 32 | **1** | 16 | 16 × 8 = 128, capped at `max_retx` = 96 | — | **~96** ✗ |
| 16 | 4 (web client, `ble-common.js:38-39`) | 8 | 8 | 2 | ~10 |

`N = 1` and `N = 2` are not hypothetical: `blocks_per_ack` is a Home Assistant options-flow field
with `min=1, max=32` (`config_flow.py:104-113`), `py-opendisplay` documents it as "1..32"
(`device.py:525`), firmware clamps only at the *top*
([display_service.cpp:2723-2725](../src/display_service.cpp)), and
[main.cpp:269](../src/main.cpp) explicitly sizes the response-flush path for *"small negotiated
ack_every (N_eff 1-2)"*.

**Why it matters.** Not a freeze — after the D1 fix an over-cap gap drops frames and keeps the
session. It matters because (a) the plan instructs writing "35 < 64" into the source as a
permanent invariant, and (b) it converts a *recoverable* transfer into a failed one exactly in
the lossy conditions Phase 1 exists to survive: at cap 128 the device re-syncs when it drains; at
cap 64 it rejects and the client burns its remaining `max_retx` budget rediscovering that by
timeout.

**Correction:** (1) set `OD_NONCE_FORWARD_CAP = 128` — under the bitmap the cap costs nothing,
and the DoS argument for keeping it tight does not survive `M2`; (2) re-derive from the client's
**retransmit budget** `max_retx = max(3·W, n/2)`, stating plainly that firmware cannot bound it
from its own constants; (3) write the *mechanism* into the Step 6 comment, not a number that a
`blocks_per_ack` change in another repo silently invalidates.

---

## HIGH

### H1 — Decision E's "the client recovers on its own" is false

`decryptCommand` returning false produces an unencrypted 3-byte NACK for **every** rejection
reason:

```c
communication.cpp:698-703
        if (!decryptCommand(...)) {
            od_log_error("ERROR: Decryption failed");
            uint8_t response[] = {RESP_ACK, (uint8_t)(command & 0xFF), RESP_NACK};
            sendResponseUnencrypted(response, sizeof(response));
```

`RESP_NACK = 0xFF` ([opendisplay_protocol.h:698](../include/opendisplay_protocol.h)). The
client's `_read` intercepts that shape before any pipe-frame classification:

```
device.py:833-838
        if len(raw) == 3 and raw[2] == 0xFF:
            raise IntegrityCheckError(...)
```

and the pipe send loop's only `except` is `BLETimeoutError` (`device.py:2716`).
`IntegrityCheckError` propagates straight out of `_stream_pipe_chunks`; the upload fails on the
**first** out-of-window frame. There is no retransmit machinery to recover with — the frame the
client would have repaired is the one whose NACK aborted it.

Phase 1's field benefit is real but narrower than claimed: the device stays clean and ready for
the *next* connection instead of answering `0xFE` to everything. The in-progress transfer still
dies.

**Correction:** on `NONCE_OUT_OF_WINDOW` / `NONCE_REPLAY` for `0x0081`, send **nothing** (let
PTO/SACK treat it as the plain frame loss it is) or send the normal pipe ACK. Keep `RESP_NACK`
for tag failures. ~5 lines in `communication.cpp:698-703` behind a reason code out of
`nonceCheck`; it converts Phase 1 from "the device survives" into "the transfer survives".
Decision E's recorded wire change remains right for non-pipe opcodes.

### H2 — Device and client share one nonce space: CCM keystream reuse (shipping defect, unaddressed)

```c
encryption.cpp:158-174
void getCurrentNonce(uint8_t* nonce) {
    memcpy(nonce, encryptionSession.session_id, 8);
    uint64_t counter = encryptionSession.nonce_counter;      // starts at 0 (:210, :655)
    for (int i = 0; i < 8; i++) nonce[8 + i] = (counter >> (56 - i * 8)) & 0xFF;
}
```

```python
crypto.py:92-113
def get_nonce(session_id: bytes, counter: int) -> bytes:
    return session_id + counter.to_bytes(8, "big")
...
    ccm_nonce = nonce_full[3:]  # 13 bytes
```

`encryptResponse` ([encryption.cpp:740-743](../src/encryption.cpp)) and `decryptCommand`
(`:704-705`) both take `nonce_full[3:16]`. Same `session_key`, same `session_id`, **no direction
separator**, both counters reset to 0 at session start (`encryption.cpp:210`/`:655`;
`device.py:735`; `ble-common.js:1259`). Response #*k* and command #*k* therefore encrypt under an
identical (key, nonce). CCM is CTR underneath: the keystream depends only on key and nonce, not
the AD (which does differ), so `C_resp ⊕ C_cmd = P_resp ⊕ P_cmd`. Responses are short and highly
predictable (`{RESP_ACK, cmd, status}`, pipe ACKs), so a passive eavesdropper recovers the
leading plaintext bytes of the matching command. No authenticity break, but a textbook
nonce-reuse confidentiality failure.

**Correction:** the fix is a direction bit — a wire change of the same cost class as Decision E,
and correctly out of Phase 1 scope. What *is* in scope: **record it** alongside Decision E so one
future wire revision fixes both. Doing nothing and not writing it down is the only unacceptable
outcome, because Phase 1 is the change that makes a future reader believe this layer has been
audited.

### H3 — D3 understated: the exemption lets an attacker flush the entire replay ring

The core claim is verified ([encryption.cpp:136](../src/encryption.cpp)). What the plan misses is
that the accept path writes the ring unconditionally, *including* on the exempted `diff == 0`
re-accept:

```c
encryption.cpp:152-154
    static uint8_t replay_window_index = 0;
    encryptionSession.replay_window[replay_window_index] = nonce_counter;
    replay_window_index = (replay_window_index + 1) % 64;
```

Replaying the highest-seen frame **64 times** overwrites all 64 slots with that one value,
evicting every genuine entry, while `last_seen_counter` never moves (`:149-151` is
`>`-conditional) so the ±32 check at `:131` still admits `[L−32, L]`. **Every one of the last 32
genuine commands then becomes replayable** — config writes, power-off, buzzer, LED.

Reachable on ESP32: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` is **3** (`sdkconfig.h:613`; the `-D…=1`
override is inert), the write callback does not discriminate conn handles
([esp32_ble_callbacks.h:119-129](../src/esp32_ble_callbacks.h)), and `isAuthenticated()`
(`:195-199`) is a global flag with no peer binding — a second central can write captured frames
into a live session. On nRF `Bluefruit.begin(1, 0)` ([ble_init.cpp:152](../src/ble_init.cpp))
caps at one link.

**Correction:** no design change (the bitmap closes it). Restate D3's consequence and add a
Step 5 hardware test: replay the final frame 64× then replay an *older* in-window counter — must
be `NONCE_REPLAY`.

---

## MEDIUM

### M1 — Step 2 and Decision D prescribe different deltas, and Step 2's is UB on attacker-controlled input

Step 2: `diff = (int64_t)counter - (int64_t)last_seen_counter`. Decision D: *"compute the delta as
`(int64_t)(counter - last_seen)` rather than subtracting two casted `int64_t`s."* Not the same
expression; Step 2's is wrong. The 8 counter bytes are plaintext and parsed **before** tag
verification ([encryption.cpp:119-121](../src/encryption.cpp), called from `:691` ahead of
`aes_ccm_decrypt` at `:714`), so an unauthenticated attacker controls the delta.
`(int64_t)nonce_counter` for `≥ 2^63` is an out-of-range conversion and the subtraction overflows
— UB that the plan's own `-fsanitize=undefined` gate will flag against the expression Step 2
mandates. Today's `:130` has exactly this construct.

Decision D's form is not sufficient either: the Step 2 table uses `-diff` twice, and `-diff` is
UB when `diff == INT64_MIN` (reachable with `counter = last_seen + 2^63`).

**Correction:** drop signed arithmetic entirely:

```c
const uint64_t fwd  = counter - last_seen;   /* wraps; 0 when equal              */
const uint64_t back = last_seen - counter;   /* wraps; fwd + back == 0 mod 2^64  */
if (fwd == 0)                       return bit_test(bm, 0) ? NONCE_REPLAY : NONCE_OK;
if (fwd <= OD_NONCE_FORWARD_CAP)    return NONCE_OK;
if (back < OD_NONCE_BACKWARD_BITS)  return bit_test(bm, back) ? NONCE_REPLAY : NONCE_OK;
return NONCE_OUT_OF_WINDOW;
```

Make Step 2's table match so `nonce_window.h` and the plan cannot diverge.

### M2 — "Tight is correct here" rests on a jam-forward DoS that cannot happen

Decision A inherits `[M1]` from the parent review. Under Phase 1's own design it does not hold:

1. **A replay cannot push `last_seen_counter` past the client's high-water mark.** After the D2
   fix only a CCM-verified frame commits (Step 3), so the only counters an attacker can commit are
   ones the client actually transmitted. No *future* client frame is ever below the window.
2. **Repairs carry fresh, higher counters.** `_encrypt_frame` increments on every transmission
   including repairs (`device.py:747-760`, `:759`) — the plan cites this itself. The "stranded"
   frames do not exist.

The only frames a jam can strand are ones already in flight at lower counters, and those land in
the 127-wide **backward** window, where they are accepted as unseen. Net damage: zero.

**Correction:** delete the "Why not more" paragraph; it is the only argument that produced 64
instead of 128. Note that `[M1]`'s concern was an artifact of the value-ring design and does not
survive commit-after-verify + bitmap.

### M3 — The `resetNonceState()` fold merges two blocks that are not equivalent

| Field | `clearEncryptionSession` (`:205-217`) | `handleAuthenticate` (`:654-662`) |
|---|---|---|
| `nonce_counter` | `= 0` (`:210`) | `= 0` (`:655`) |
| `last_seen_counter` | `= 0` (`:211`) | `= 0` (`:656`) |
| `replay_window` | `memset` (`:217`) | `memset` (`:660`) |
| `integrity_failures` | `= 0` (`:212`) | `= 0` (`:657`) |
| `authenticated` | `= false` (`:209`) | `= true` (`:654`) |
| `session_start_time`/`last_activity` | `= 0` (`:213-214`) | `= currentTime` (`:658-659`) |
| keys/nonces/`ccm_ctx` | wiped (`:203-208`) | populated |
| `auth_attempts`, `server_nonce_time` | `= 0` (`:215-216`) | `server_nonce_time = 0` only (`:662`) |

Only four fields are common; the rest are opposite. A fold worded as "bitmap +
`last_seen_counter`" silently drops `nonce_counter = 0` the first time someone tidies the
surrounding lines — and a device that keeps its outbound counter across a re-auth while the client
restarts at 0 walks into `H2`'s keystream reuse with *itself*.

**Correction:** define `resetNonceState()` =
`{ nonce_counter = 0; last_seen_counter = 0; memset(replay_bitmap); integrity_failures = 0; }`,
naming all four fields. Leave every other field where it is.

---

## LOW

**L1 — D4 is not a live bug.** The facts are right (`replay_window_index` is a function static at
`:152`, reset by neither `:217` nor `:660`); the consequence is not. Both reset sites `memset` the
ring to **all zeros**, so writing from an arbitrary offset into a uniformly-empty 64-slot ring
with a +1 index produces exactly the same strict-FIFO eviction order as starting from 0. No
counter's accept/reject differs. The one real artifact is the `0`-as-empty sentinel: counter 0 can
never be accepted through the backward branch — masked today by D3's exemption, i.e. **D3 is
load-bearing for D4's representation**. Demote D4 to "latent"; keep the fix.

**L2 — `nonceCommit()` placement is under-specified.** The success arm has an early return between
`:717` and `integrity_failures = 0`:

```c
encryption.cpp:717-725
    if (success) {
        uint8_t payload_length = decrypted_with_length[0];
        if (payload_length > encrypted_len - 1) { ...; return false; }   // authentic frame
        ...
        encryptionSession.integrity_failures = 0;
```

If the commit lands after `:722`, an authentic-but-malformed frame is left replayable — and
today's code *does* commit it (`:149-153` is unconditional), so this is a behaviour change. Say
"first statement of the success arm, `:718`".

**L3 — Decision B's storage table contradicts its own formula.** `2W × 8 B` = 16 B per unit ✓ for
the 32-row (512 B). Rows 2–3 are exactly 2× too large: 127 × 16 = **2,032 B** not 4,064;
255 × 16 = **4,080 B** not 8,160. Conclusion unaffected.

**L4 — `PIPE_MAX_W + MAX_PTO` over-counts by one.** `MAX_PTO = 3` yields **two** probe sends: the
client increments then raises at the threshold before sending (`device.py:2721-2726`). 35 → 34
(18 on `esp32-N4`). Conservative direction; moot under `C1`.

**L5 — CI step runs 11×.** `.github/workflows/main.yaml` is one `build` job with an 11-entry
`matrix.environment` (`:10-23`). Use a separate top-level `host-tests` job. Confirmed safe:
`tools/test_nonce_window.cpp` is invisible to every firmware build —
`build_src_filter = +<*> -<main_mbed.cpp>` (`platformio.ini:34`) is relative to the default
`src_dir` and no env adds `tools/`.

**L6 — Line-reference drift.** The ring drop is `esp32_ble_callbacks.h:127-128` (`:126` is the
RELEASE store on the *success* path), not `:126-127`; the NACK is `communication.cpp:698-703`, not
`:700-701`. Everything else spot-checked is exact.

**L7 — `NONCE_BAD_SESSION` silently stops being tamper evidence.** Today a `session_id` mismatch
counts (`:122-128` → `:691-696`); the plan routes it to "untouched". Probably right, but it is a
decision, not a consequence of D1. Also `:123-127` logs both full session IDs at `od_log_error` —
with counting removed an attacker can drive that line indefinitely. State the change;
demote/rate-limit the log as Step 4 already does for the out-of-window log.

---

## Verified CORRECT — do not churn on these

1. **D1 is real and is the wedge.** `:691-696` increments `integrity_failures` on *any*
   `verifyNonceReplay` failure including plain packet loss, clearing at 3, after which
   `communication.cpp:665-670` answers everything `RESP_AUTH_REQUIRED`.
2. **D2 is real.** `last_seen_counter` (`:149-151`) and the ring write (`:153`) both land before
   `aes_ccm_decrypt` at `:714`. Step 3's "called from exactly one place" is achievable —
   `decryptCommand` is the sole caller.
3. **D3 is real and exploitable** (`:136`). See `H3` — worse than the plan says.
4. **The bitmap state machine is correct as specified.** Worked by hand: fresh session
   (`last_seen=0`, bitmap 0) accepts counter 0 exactly once via `fwd==0` + clear bit 0, needing no
   `has_seen_counter`; forward commit by `d` moves old bit *i* (counter `L−i`) to bit `i+d`, which
   under `L'=L+d` denotes `L−i` ✓ for d = 1, 63, 64, 65, 127; `d ≥ 128` clears wholesale and every
   discarded counter is then ≥128 behind so it is rejected on width, never mis-reported unseen;
   backward bit indices 1…127 valid with `back ≥ 128` rejected, so the backward window is exactly
   `OD_NONCE_BACKWARD_BITS − 1 = 127` as stated; a forward cap < 128 means a legal slide never
   needs a wholesale clear, so the two sides genuinely decouple, and the plan's claim that a cap
   *wider* than the backward window would still be harmless is also true. Only the arithmetic form
   (`M1`) and the cap value (`C1`) are wrong.
5. **"The ring is not buggy" — correct conclusion, off-by-one proof.** Attempts to construct a
   sequence evicting an accepted counter `d` from the 64-slot ring while `d` is still inside ±32
   all fail. The tight count is `2W − 1 = 63`, not `2W`: all counters acceptable while `d` stays
   in-window lie in `[L₀−32, d+32]`, and at least one member is provably already seen (the
   `last_seen_counter` value preceding `d`, or — in the fresh-session corner — counter 0, blocked
   by the zeroed ring's sentinel). 63 writes leave the index one short of `d`'s slot. So `D=64` is
   sound **with exactly one slot of margin**, and the parent plan's 256/128 likewise. `D ≥ 2W` is
   conservative and its conclusion holds; the real argument for the bitmap — non-obvious
   combinatorics maintained by hand across two files — is strengthened by how tight the margin
   actually is.
6. **Decision C is clean.** `verifyNonceReplay` has exactly three occurrences outside its
   definition: `encryption.h:17`, `main.h:276`, and the sole call at `encryption.cpp:691`. Nothing
   in `src/`, `tools/`, or the nRF build path references it. (`Firmware_NRF54` has its own private
   copy at `opendisplay_pipe.c:427-455` — separate repo, unaffected, but it carries all four
   defects and should be scheduled.)
7. **Decision D's factoring is sound and build-safe** — see `L5`.
8. **Sizing arithmetic.** 512 B → 16 B = **−496 B** ✓. 256-entry ring = 2,048 B → **+1,536 B** ✓.
   No `platformio.ini` change; the `esp32-N4` link-headroom question is genuinely moot.
9. **Client premises.** One counter burned per transmission including repairs
   (`device.py:747-760`) ✓; window blocks for an ACK and PTO resends exactly one chunk
   (`:2688-2694`, `:2715-2726`) ✓; `MAX_PTO = 3` at `commands.py:93` ✓; `PIPE_MAX_W` 32 /
   16-on-`esp32-N4` (`structs.h:45-53`) ✓. Missing from the premise set: the SACK repair site
   (`C1`).
10. **The `communication.cpp:772-775` invariant survives.** Commit still happens for every frame
    that decrypts, including duplicates `handlePipeWriteData` discards. Only the name and ordering
    clause need rewriting, as Step 6 says.
11. **Scope boundaries are right.** Leaving the 30 s auth-challenge window alone is correct (wire
    contract at `include/opendisplay_protocol.h:384`, cannot wedge anything). Deferring
    `session_timeout_seconds` and the link-drop guard to Phase 5 is correct, and the plan is
    admirably explicit that Phase 1 closes only the nonce arm.
12. **No new failure mode on either target.** nRF has no command ring — `imageDataWritten` runs
    inline on the Bluefruit callback task ([ble_init.cpp:157](../src/ble_init.cpp)),
    single-threaded and in order — so its only gap source is air loss, which the link layer
    repairs; a wider forward cap is inert there. On ESP32 nothing in Phase 1 touches the ring, the
    loop watchdogs, or advertising. The LAN TLS path bypasses `decryptCommand` entirely
    ([communication.cpp:663](../src/communication.cpp), origin-gated on `ORIGIN_LAN_TLS`) and is
    untouched by every Phase 1 change; the plain-LAN path shares the one `encryptionSession`
    exactly as today. `handleAuthenticate`'s re-auth path (`:583-585` → `clearEncryptionSession()`
    → `:654-662`) resets both counters and the ring on both sides, unchanged in shape.

---

## Summary of findings

| # | Sev | Class | Finding | Correction |
|---|-----|-------|---------|-----------|
| C1 | Critical | plan | Forward cap 64 derived from a non-bound; SACK repair (`device.py:2792`) + preserved ACK backlog (`:781`) reach ~66 at `ack_every=2`, ~96 at `ack_every=1`, both user-selectable (`config_flow.py:104-113`) | Cap **128**; re-derive from `max_retx`; rewrite the Step 6 comment as a mechanism |
| H1 | High | plan | Decision E's "client recovers on its own" is false — 3-byte `0xFF` NACK raises `IntegrityCheckError` (`device.py:834`) uncaught by the pipe loop | Send no NACK (or a pipe ACK) for nonce rejections on `0x0081`; firmware-only |
| H2 | High | shipping | Device/client counters share one nonce space → CCM keystream reuse (`encryption.cpp:158-174` vs `crypto.py:92-113`) | Out of scope, but **record it** next to Decision E |
| H3 | High | shipping | D3 understated: exempted re-accept writes the ring (`:152-154`), so 64 replays flush it and unlock the last 32 counters | Restate D3; add the ring-flush hardware test |
| M1 | Medium | plan | Step 2 vs Decision D disagree; both, plus `-diff`, are UB on attacker-controlled counters | Two wrapping `uint64_t` deltas; no signed arithmetic |
| M2 | Medium | plan | "Tight is correct" rests on a jam-forward DoS that cannot occur post-D2 | Delete "Why not more"; it is the only argument against C1 |
| M3 | Medium | plan | `resetNonceState()` folds blocks sharing only four fields; drops `nonce_counter = 0` | Define the helper by naming all four fields |
| L1 | Low | plan | D4 is latent, not live | Demote the claim; keep the fix |
| L2 | Low | plan | `nonceCommit()` vs the `payload_length` early return at `:719-722` | "First statement of the success arm, `:718`" |
| L3 | Low | plan | Storage table rows 2–3 are 2× its own formula | 2,032 B and 4,080 B |
| L4 | Low | plan | `MAX_PTO=3` yields 2 probe sends | 34 / 18 (moot under C1) |
| L5 | Low | plan | CI step inside an 11-leg matrix | Separate `host-tests` job |
| L6 | Low | plan | Two line refs drift | Fix in place |
| L7 | Low | plan | `NONCE_BAD_SESSION` policy change unremarked; log attacker-drivable | State it; demote/rate-limit |

## Is Phase 1 safe to implement as written?

**Safe: yes.** No new freeze mode, no new remote DoS, no regression on either target. Every change
is confined to a path whose current behaviour is strictly worse. It is genuinely independent of
Phases 2–6 and genuinely deliverable alone.

**As written: no.** Mandatory before implementation: `OD_NONCE_FORWARD_CAP = 128` with the
derivation replaced (`C1` + `M2`), and the unsigned delta form (`M1`) so the code and the UBSan
gate agree. One more (`H1`) decides whether Phase 1 saves the *transfer* or only the *device*; it
is five lines and should be in scope. `H2` and `H3` need a paragraph each, not code. With those,
Phase 1 is the right first change and should ship ahead of the rest of the program.
