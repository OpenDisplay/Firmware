# Phase 1 Implementation Plan — Nonce / Replay Correctness

**Branch:** `debug/ble-hardening` · **Date:** 2026-07-26
**Parent plan:** [`PLAN_FREEZE_PROOFING_2026-07-26.md`](PLAN_FREEZE_PROOFING_2026-07-26.md) § "Phase 1"
**Review that shaped it:** [`FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md`](FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md) `[M1]`

> **Revised 2026-07-26 after adversarial review** —
> [`FINDINGS_PHASE1_PLAN_REVIEW_2026-07-26.md`](FINDINGS_PHASE1_PLAN_REVIEW_2026-07-26.md)
> (1 Critical, 3 High, 3 Medium, 7 Low; verdict: safe to ship alone, but not as written).
>
> **Applied here:** `C1` — forward cap 64 → **128**, and the "hard bound" derivation replaced
> (Decision A); `M2` — the jam-forward DoS argument withdrawn, since it cannot occur once D2 is
> fixed; `H1` — **new Step 4b**, stop answering a nonce-rejected pipe frame with a fatal NACK.
> Bitmap widened to `uint64_t[4]` so the cap stays inside the window. `M1` — Step 2 now
> specifies **unsigned wrapping deltas**, resolving both the plan's internal contradiction and
> the UB on attacker-controlled counters.
>
> `H2` recorded under Decision E (shipping defect, cross-repo wire fix — no Phase 1 code);
> `H3` folded into the D3 row and a new hardware test 4b; `M3` `resetNonceState()` now names all
> four fields; `L1`-`L7` corrected in place.
>
> **All 14 findings are now addressed.** The only ones carrying no code change are `H2`
> (recorded, deferred to a protocol revision) and `L1` (D4 demoted to latent — the fix stays).

> ## ⚠ THIS PLAN HAS SHIPPED — READ ["As-built"](#as-built-what-actually-shipped) FIRST
>
> Everything above and below this banner is the plan **as written before implementation**. The
> code landed on `debug/freeze-fix-phase2` (commits `0a60712`…`23ecaed`) and diverges from the
> plan in three places, one of which is a deliberate scope expansion into Phase 5. The
> [As-built section](#as-built-what-actually-shipped) at the end of this file is the ground
> truth, with `file:line` anchors into the real code.
>
> **Nothing in Step 5's hardware list has been run.** Test 0 (the baseline that settles the D1
> mechanism caveat) and Test 2b (whether Step 4b's silent drop actually lets py-opendisplay's
> SACK path repair and complete an upload) are both still open. See
> ["Unverified on hardware"](#unverified-on-hardware--the-honest-list).

Phase 1 is the root-cause fix and ships first. It is self-contained: it has no dependency on any
later phase and delivers field benefit on its own — unlike Phase 3, which is dead code until
Phase 5/6 call it. Scope is the encryption layer (`encryption.cpp`, `encryption_state.h`, a new
`nonce_window.h`) plus a small, deliberate change in `communication.cpp` (Step 4b); the full list
is in "Files touched".

---

## What is actually wrong today

Four distinct defects live in `verifyNonceReplay()` ([encryption.cpp:114-156](../src/encryption.cpp))
and its one caller `decryptCommand()` ([:688-735](../src/encryption.cpp)):

| # | Defect | Evidence | Consequence |
|---|---|---|---|
| **D1** | Nonce rejection is counted as **tamper evidence** | [:691-696](../src/encryption.cpp) — `verifyNonceReplay` false → `integrity_failures++` → 3 ⇒ `clearEncryptionSession()` | Packet loss ≠ attack. A lost window puts the next frame out of range, and each such frame counts toward session destruction; at 3 the session is destroyed mid-transfer and everything then answers `0xFE`. **See the mechanism caveat below — how the count reaches 3 is not what this plan originally claimed.** |
| **D2** | State is committed **before** the CCM tag is verified | `last_seen_counter` at [:149-151](../src/encryption.cpp), ring write at [:153](../src/encryption.cpp), all *before* `aes_ccm_decrypt` at [:714](../src/encryption.cpp) | An unauthenticated attacker (or corrupt frame) advances the replay state of a live session. Forged counter `last_seen + 32` sticks even though the frame is discarded. |
| **D3** | `counter_diff == 0` is exempted from the replay-set check | [:136](../src/encryption.cpp) `nonce_counter <= last_seen && counter_diff != 0` | **Replay of the highest-seen frame is accepted and re-executed** — the tag is valid because the frame is genuine. Harmless for a pipe DATA frame (duplicate seq is discarded) but not for `CMD_CONFIG_WRITE`, `CMD_POWER_OFF`, or a buzzer/LED command, which is typically what the last frame of a session is. The `!= 0` term exists only so a fresh session's first frame (client counter 0 vs `last_seen_counter` initialised to 0 at [:211](../src/encryption.cpp)) isn't flagged. **`[H3]` — worse than one replayed command:** the accept path writes the ring unconditionally ([:152-154](../src/encryption.cpp)), *including* on this exempted re-accept, while `last_seen_counter` never moves (`:149` is `>`-conditional). So replaying the highest-seen frame **64 times flushes every genuine entry out of the ring**, after which the whole `[L−32, L]` backward window is replayable — the last 32 genuine commands, not just the last one. Reachable on ESP32: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` is really 3 (`sdkconfig.h:613`), the write callback does not discriminate connection handles, and `isAuthenticated()` ([:195-199](../src/encryption.cpp)) is a global flag with no peer binding, so a *second* central can feed captured frames into a live session. nRF is capped at one link by `Bluefruit.begin(1, 0)`. |
| **D4** | `replay_window_index` is a **function static** | [:152](../src/encryption.cpp) | `clearEncryptionSession()` memsets the ring ([:217](../src/encryption.cpp)) but cannot reset the index. **`[L1]` — latent, not live:** both reset sites zero the *whole* ring, and writing from an arbitrary offset into a uniformly-empty 64-slot ring with a +1 index gives the same strict-FIFO eviction order as starting from 0, so no counter's accept/reject decision differs today. It becomes a real bug the moment the ring stops being uniformly reset. Fix it anyway — the bitmap removes the field entirely. |

### ⚠ Mechanism caveat on D1 — verify on hardware before trusting the narrative

Earlier versions of this plan (and the parent plan's context section) asserted that a lost window
produces *"exactly 3 rejections, because the client's `MAX_PTO` is 3"*. **That coincidence is not
real, and the chain it describes may not be how the field failure actually happens.** Two
corrections compound:

1. **`MAX_PTO = 3` yields only two probe sends** (`[L4]`; `device.py:2721-2726` increments and
   raises at the threshold *before* sending). Two rejections do not reach a threshold of 3.
2. **The client aborts on the *first* rejection, not the third** (`[H1]`). The `RESP_NACK` for
   rejection #1 raises `IntegrityCheckError` (`device.py:833-838`), uncaught by the pipe loop —
   so the transfer is already dead and the client sends nothing further on that path.

So within a single transfer, `integrity_failures` plausibly reaches **1**, not 3. Reaching 3
requires *repeated* attempts on the same session — an HA retry of the whole upload, or unrelated
commands, each rejected because the device's `last_seen_counter` is stranded far below the
client's. That is plausible but **unverified**.

**Why this matters, and why it does not weaken Phase 1:**

- **It re-weights the fix.** If the client aborts at rejection #1, the observed field symptom —
  latched `pipeState.active`, dead touch, powered panel — needs no session destruction at all;
  the abort alone leaves the device latched. That makes **Step 4b (`[H1]`) potentially the
  highest-value change in Phase 1**, not a refinement of it, and it reinforces that the latch
  itself is only cleared by Phase 3's `abortToKnownState()`.
- **D1 is still a genuine defect and still worth fixing first.** Counting packet loss as tamper
  evidence is wrong on its own terms, and it is what turns a recoverable transfer failure into a
  device that answers `0xFE` to everything until reconnect. The fix does not depend on how the
  count reaches 3.

**Action:** Step 5 test 1-2 must **capture the baseline on unmodified firmware first** — log the
actual `integrity_failures` trajectory and whether the session is cleared during a real
window-loss event — before asserting the before/after story anywhere. If the session is never
actually cleared in the field, say so and re-rank the phases accordingly rather than defending
this plan's original framing.

Plus the structural issue `[M1]`: the window is **±32 symmetric today**
([:131](../src/encryption.cpp)) and one replayed frame can jam `last_seen_counter` forward,
stranding every legitimate frame between the old and new positions. How wide the forward side
should be is Decision A; how the seen-set is represented is Decision B. **Both are now
resolved below** — D3 and D4 disappear entirely under the chosen representation rather than
being patched.

---

## Design: split check from commit

```
decryptCommand(...)
  ├─ isAuthenticated()                      unchanged (Phase 5 removes the timeout side-effect)
  ├─ NonceResult r = nonceCheck(nonce)      PURE — no writes to encryptionSession
  │    ├─ OK              → continue
  │    ├─ BAD_SESSION     → return false, integrity_failures UNTOUCHED
  │    ├─ OUT_OF_WINDOW   → return false, integrity_failures UNTOUCHED   ← D1 fix
  │    └─ REPLAY          → return false, integrity_failures UNTOUCHED
  ├─ aes_ccm_decrypt(...)                   the ONLY tamper oracle
  │    ├─ tag fail        → integrity_failures++ ; >=3 ⇒ clearEncryptionSession()  (unchanged)
  │    └─ tag OK          → nonceCommit(counter)   ← D2 fix: commit AFTER authentication
  └─ integrity_failures = 0 ; updateEncryptionSessionActivity() ; return true
```

The rule in one line: **only a CCM tag failure is evidence of tampering; a nonce failure is
evidence of a lossy link.** Nonce failures drop the frame and keep the session.

---

## Steps

### Step 1 — `encryption_state.h`: replace the value ring with a sliding bitmap

```c
// Anti-replay: bit i == "counter (last_seen_counter - i) has been consumed".
// Bit 0 is last_seen_counter itself. Backward window is implicitly
// OD_NONCE_BACKWARD_BITS - 1; there is no separate window constant to keep in
// step, and no index to reset.
#define OD_NONCE_BACKWARD_BITS 256            // uint64_t[4], 32 B
#define OD_NONCE_FORWARD_CAP   128            // see Decision A
    uint64_t replay_bitmap[OD_NONCE_BACKWARD_BITS / 64];
```

- **Delete** `uint64_t replay_window[64]` (512 B) and the function-static
  `replay_window_index` ([:152](../src/encryption.cpp)). Net struct change: **−480 B**.
- **Why the backward width is 256, not 128:** keeping `OD_NONCE_FORWARD_CAP <
  OD_NONCE_BACKWARD_BITS` means a legal forward slide can never exceed the bitmap width, so the
  wholesale-clear branch in Step 3 is unreachable on any legitimate input. At width 128 with a
  128 cap the two are equal and a maximal slide clears the whole bitmap — still *correct* (every
  discarded counter is then out-of-window and rejected on width), but it puts the hardest branch
  on the normal path for the sake of 16 bytes. Keep the margin.
- **No `replay_window_index` field** — a bitmap has no insertion point, so **D4 cannot recur**.
- **No `has_seen_counter` field** — "not seen" is a clear bit, not a reserved value, so **D3
  cannot recur**. A fresh session is `last_seen_counter = 0` with an all-zero bitmap; the
  first frame at counter 0 has `fwd == 0`, finds bit 0 clear, and is accepted exactly once.
- `clearEncryptionSession()` ([:201-219](../src/encryption.cpp)) and the fresh-session block in
  `handleAuthenticate` ([:654-660](../src/encryption.cpp)) must **both** reset the nonce state.
  Fold that into one `resetNonceState()` helper so a third caller cannot drift — but **`[M3]`
  define it by naming all four fields**, because the two blocks share only these four and are
  *opposite* on everything else (`authenticated` false vs true, timestamps zeroed vs stamped,
  keys wiped vs populated):

  ```c
  static void resetNonceState(void) {
      encryptionSession.nonce_counter      = 0;   /* device's OWN outbound counter */
      encryptionSession.last_seen_counter  = 0;
      encryptionSession.integrity_failures = 0;
      memset(encryptionSession.replay_bitmap, 0, sizeof(encryptionSession.replay_bitmap));
  }
  ```

  A helper described loosely as "reset the bitmap and `last_seen_counter`" invites someone
  tidying the surrounding lines to drop `nonce_counter = 0`. That would leave the device's
  **outbound** counter running across a re-auth while the client restarts at 0 — walking
  straight into the keystream reuse described under Decision E `[H2]`, against itself.

### Step 2 — `encryption.cpp`: `nonceCheck()`, pure
```c
/* enum lives in src/nonce_window.h — the TYPE is shared (Step 4b needs it in
   encryption.h to carry a reason out of decryptCommand); the FUNCTIONS stay
   file-static (Decision C). Sharing a type grants no ability to commit state. */
enum NonceResult { NONCE_OK, NONCE_BAD_SESSION, NONCE_OUT_OF_WINDOW, NONCE_REPLAY };

static NonceResult nonceCheck(const uint8_t* nonce, uint64_t* counter_out);   /* encryption.cpp */
```
- Keep the existing `constantTimeCompare` session-id check ([:122](../src/encryption.cpp)) →
  `NONCE_BAD_SESSION`.
- Then, **unsigned arithmetic only** (`[M1]` — see below for why this is not a style choice):

```c
const uint64_t fwd  = counter - last_seen;   /* wraps; 0 when equal              */
const uint64_t back = last_seen - counter;   /* wraps; fwd + back == 0 mod 2^64  */

if (fwd == 0)                       return bit_test(bm, 0) ? NONCE_REPLAY : NONCE_OK;
if (fwd <= OD_NONCE_FORWARD_CAP)    return NONCE_OK;   /* ahead: cannot have been seen */
if (back < OD_NONCE_BACKWARD_BITS)  return bit_test(bm, back) ? NONCE_REPLAY : NONCE_OK;
return NONCE_OUT_OF_WINDOW;
```

  The `fwd == 0` case is where **D3 closes**: no `!= 0` exemption, the bit is simply tested like
  any other.

- **The four tests are ordered, and the order is load-bearing.** `fwd` and `back` sum to zero
  mod 2^64, so they cannot both be small: with the current constants `fwd <= 128 && back < 256`
  would need `fwd + back <= 384 ≡ 0 (mod 2^64)`, true only when both are zero — the case already
  consumed by the first test. A counter far from the window in either direction leaves both huge
  and falls through to `NONCE_OUT_OF_WINDOW`. Do not reorder.

- **Why unsigned, not `int64_t` deltas.** The 8 counter bytes are parsed off the wire at
  [:119-121](../src/encryption.cpp) and reach this function *before* `aes_ccm_decrypt`
  ([:714](../src/encryption.cpp)), so an **unauthenticated attacker controls both operands**.
  Converting a `uint64_t >= 2^63` to `int64_t` is implementation-defined before C++20, the
  subtraction can overflow outright (`counter = 2^63`, `last_seen = 1` → `INT64_MIN - 1`), and
  negating `INT64_MIN` is UB as well — so a table keyed on `diff`/`-diff` has three separate
  ways to be undefined on attacker-chosen input. Unsigned overflow is defined as modular
  arithmetic, making the expression total over all 2^64 inputs with no range precondition. This
  is the standard formulation in IPsec/DTLS implementations. Today's [:130](../src/encryption.cpp)
  uses the signed form; it is one of the things being replaced, not preserved.
  *(Practical note: on Xtensa and ARM the signed form almost certainly compiles to the wrapping
  behaviour anyway — no known live miscompilation. It is worth fixing because the correct form
  is simpler than the buggy one, the input is attacker-controlled in a security check, and
  Decision D's `-fsanitize=undefined` gate would otherwise fail against the plan's own code.)*

- **No writes to `encryptionSession` on any path.** This is the property Step 5 tests.

### Step 3 — `encryption.cpp`: `nonceCommit(uint64_t counter)`

Same `fwd`/`back` unsigned deltas as Step 2 — do not reintroduce a signed `diff` here.

- **Forward** (`fwd != 0`, i.e. `counter > last_seen_counter` in window terms): shift the bitmap
  left by `fwd`, clearing the vacated low bits; `last_seen_counter = counter`; set bit 0.
- **Backward/equal**: set bit `back`. `last_seen_counter` does not move.
- **Keep a `fwd >= OD_NONCE_BACKWARD_BITS` guard that zeroes the bitmap wholesale, but know that
  it is unreachable in practice.** `nonceCheck` rejects anything with `fwd > OD_NONCE_FORWARD_CAP`
  (128) before commit is ever called, and the bitmap is 256 wide — that margin is deliberate
  (Step 1). The guard exists so the function is total if called directly (the host test does
  exactly that) and so a future cap increase cannot silently produce an over-wide shift. It is
  still *correct* when it does fire: every counter it discards is then ≥256 behind and gets
  rejected on width.
- Shifting across a `uint64_t[4]` must handle `shift == 0` and `shift >= 64` explicitly —
  `x << 64` is undefined behaviour in C, and it is the classic bug in this pattern.
- Step 5's host test drives `fwd` = 0, 1, 63, 64, 65, 127, 128 (**the reachable range**, capped by
  `OD_NONCE_FORWARD_CAP`) and additionally 129, 191, 192, 255, 256, 257 **directly against
  `nonce_window.h`** to exercise the guard and the word-boundary shifts that the reachable range
  alone would leave untested.
- Called from exactly one place: after a successful `aes_ccm_decrypt`.

### Step 4 — `decryptCommand()` rewiring
- Replace the `verifyNonceReplay` block ([:691-698](../src/encryption.cpp)) with the `nonceCheck`
  call and its result handling from Step 2; **delete** the `integrity_failures++` on that path.
- Insert `nonceCommit()` as the **first statement of the `if (success)` arm**, i.e. at
  [:718](../src/encryption.cpp) — `[L2]` **not** merely "before `integrity_failures = 0`".
  There is an early `return false` in between, for a decrypted-but-malformed `payload_length`
  ([:719-722](../src/encryption.cpp)). That frame is *authentic* — it passed the CCM tag — and
  today's unconditional commit at `:149-153` does record it. Placing the commit after the early
  return would leave an authentic frame replayable, i.e. a silent behaviour change dressed as a
  refactor.
- Leave the tag-failure arm ([:729-733](../src/encryption.cpp)) exactly as-is.
- Logging: `nonce out-of-window (counter=%llu last_seen=%llu fwd=%llu) — frame dropped, session
  kept` at WARN vs. `CCM tag failure %u/3` at ERROR. The current out-of-window log is
  `od_log_error` ([:132](../src/encryption.cpp)) and will now fire routinely on a lossy link —
  demote it or it becomes noise that masks real errors. **`[L7]`** applies the same treatment to
  the session-id mismatch log at [:123-127](../src/encryption.cpp), which prints two full session
  IDs at ERROR: once nonce failures stop counting toward `integrity_failures`, nothing rate-limits
  an attacker driving that line. Demote and/or rate-limit it.
- **`[L7]` — state the `NONCE_BAD_SESSION` policy change explicitly.** Today a session-id
  mismatch *does* count as tamper evidence ([:122-128](../src/encryption.cpp) → `:691-696`);
  routing it to "`integrity_failures` untouched" alongside the loss cases is a deliberate
  decision, not a consequence of D1. It is the right call — a mismatched session id is what a
  stale client sends after the device re-authenticated, i.e. usually confusion rather than
  attack, and the CCM tag remains the tamper oracle — but it must be written down rather than
  arrived at silently.
- **Delete `verifyNonceReplay()`** and both of its declarations (Decision C: the body at
  [:114-156](../src/encryption.cpp), [encryption.h:17](../src/encryption.h),
  [main.h:276](../src/main.h)). The build is the check here — the compiler will name any caller
  we missed.

### Step 4b — Stop answering a dropped pipe frame with a fatal NACK `[H1]`

**Without this, Phase 1 saves the device but still loses the transfer.** Today every
`decryptCommand` failure — nonce *and* tag alike — produces the same unencrypted 3-byte
`RESP_NACK` ([communication.cpp:698-703](../src/communication.cpp)), and the client turns that
shape into a fatal exception before it ever reaches pipe-frame classification:

```
device.py:833-838   if len(raw) == 3 and raw[2] == 0xFF: raise IntegrityCheckError(...)
device.py:2716      the pipe send loop's ONLY except is BLETimeoutError
```

So one out-of-window frame aborts the whole upload, no matter how wide the cap is. The client
cannot repair the hole, because the frame it would have repaired is the one whose NACK killed
the transfer.

**Change:**

1. Give `decryptCommand` a reason out-param (or an enum return) so the caller can distinguish
   nonce rejection from tag failure. It has exactly one caller, so this is mechanical — but it
   touches the declarations in [encryption.h:21](../src/encryption.h) and
   [main.h:274](../src/main.h) as well.
2. At [communication.cpp:698-703](../src/communication.cpp): when the reason is
   `NONCE_OUT_OF_WINDOW` or `NONCE_REPLAY` **and** the opcode is `CMD_PIPE_WRITE_DATA`
   (`0x0081`), **send nothing at all**. Every other combination keeps today's `RESP_NACK`.

**Why silence is the correct answer and not a hack.** A pipe DATA frame is not
request/response — the client never blocks on a per-frame reply; it blocks on sliding-window
ACK reads. Dropping the frame silently is therefore *exactly* the signal "this frame was lost",
which is the one condition the pipe protocol is built to repair: the seq is absent from the next
SACK mask, the client retransmits it, and the transfer continues. Answering instead with a fatal
NACK converts recoverable loss into an aborted upload.

**This is a conformance fix, not a protocol change.** `docs/pipe-write-protocol.md` §5.2 already
specifies the rule:

> *"NACKs are reserved for unrecoverable conditions (bad payload, protocol violation), **not
> ordinary packet loss**."*

A frame rejected because its nonce fell outside the window **is** ordinary packet loss — it is
the direct consequence of frames having been dropped. Today's firmware answers it with a fatal
`0x81` NACK, which §5.1 defines as unconditionally fatal. **Today's behaviour therefore violates
the pipe spec as written; Step 4b restores conformance.** That also settles it against the parent
plan's *"NO wire protocol changes"* constraint: no documented client-observable behaviour
changes, because silence-on-loss is what the document already prescribes. No `.md` edit is
required — at most a clarifying sentence in §5.2 that a nonce-rejected data frame is classed as
loss, not as an unrecoverable condition.

**Deliberately narrow:**
- **Tag failures keep the NACK.** They are tamper evidence, not loss, and §5.2's "unrecoverable
  condition" is exactly right for them.
- **`0x0071` (legacy DIRECT_WRITE_DATA) is left alone.** It has a different ACK discipline that
  has not been analysed here, and the field failure lives on the pipe path. Note it as a
  deliberate exclusion so the next reader does not assume it was an oversight.
- **No canonical-header change** (Decision E still stands): no opcode, response code, or envelope
  changes.

### Step 5 — Verification
- **Host test** (Decision D): `tools/test_nonce_window.cpp` against `src/nonce_window.h`, run
  under UBSan/ASan. Full case list in Decision D — it covers the window state machine, *not*
  the `integrity_failures` behaviour, which is what hardware tests 1-2 below are for.
- **Build gate:** `pio run -e nrf52840custom -e esp32-s3-N16R8 -e esp32-c3-N16 -e esp32-c6-N4
  -e esp32-N4`. CI builds all 11.
- **Hardware** (py-opendisplay CLI + `tools/od-device-cli.py`). **Test 0 comes first:**
  0. **Baseline on unmodified firmware.** Induce a real window-loss event and record the
     `integrity_failures` trajectory, whether `clearEncryptionSession()` actually fires, and
     whether the client aborts at the first NACK. This settles the D1 mechanism caveat above.
     Without it, the before/after claims for tests 1-2 rest on an unverified model.
  1. Forward-gap **within** the cap: skip 100 counters mid-session → next frame accepted,
     transfer continues, session survives (today: session destroyed after 3).
  2. Forward-gap **beyond** the cap: skip 200 (> `OD_NONCE_FORWARD_CAP`) → frames rejected as
     out-of-window, but **`integrity_failures` stays 0 and the session survives**. This is the
     D1 regression test.
  2b. **`[H1]` — the gap must not kill the transfer.** Run test 2 *during a live pipe upload*
     and assert the upload **completes**: the rejected frames are silently dropped (Step 4b),
     absent from the next SACK mask, retransmitted, and repaired. Today, and in the pre-review
     version of this plan, the client raises `IntegrityCheckError` and aborts. This is the test
     that distinguishes "the device survived" from "the transfer survived".
  2c. **Worst-case client settings.** Repeat the pipe regression with
     `blocks_per_ack = 1` and `W = 32` — the configuration that makes the gap widest
     (Decision A). Confirms the 128 cap in the conditions that motivated it.
  3. True replay of an old counter → rejected, session survives.
  4. **Replay of the last frame of a session** (D3) → now REJECTED. Use a
     non-idempotent command (buzzer) so acceptance is observable.
  4b. **`[H3]` — ring-flush replay.** Replay the final frame **64 times**, then replay an
     *older* counter that is still inside the backward window. Must be `NONCE_REPLAY`. On
     today's firmware the 64 re-accepts flush the value ring and the older frame is **accepted
     and re-executed**; this test fails before the change and passes after, which is the only
     way to demonstrate the bitmap closed the widened hole rather than just the narrow one.
  5. Forged/corrupt tag ×3 → session still cleared (unchanged behaviour, deliberately).
  6. Regression: full Spectra transfer and an E1004 ~960 KB upload complete untouched.

### Step 6 — Comment hygiene
- [communication.cpp:772-775](../src/communication.cpp) documents that the replay counter
  "already advanced at decrypt time … so drops/dupes never desync it". The invariant still
  holds (commit happens for every frame that *decrypts*, including ones the pipe handler
  discards) but the function name and the ordering claim are now wrong. Rewrite it in the
  same change. **Write the mechanism, not a number** (Decision A, `[C1]`): the gap is driven by
  the client's *retransmit budget* `max_retx = max(3·W, n/2)` and by `blocks_per_ack`, both of
  which live in another repo and one of which is a user-facing Home Assistant setting — so
  `OD_NONCE_FORWARD_CAP` is a heuristic with headroom, **not** an invariant firmware can prove.
  A comment asserting a specific bound would be falsified silently by a client-side config
  change. Say that, and point at Decision A.

---

## Decisions

**All five are settled.** Nothing blocks implementation.

### Decision A — ~~RESOLVED: forward cap **128**~~ — **REVERSED 2026-07-31**

> ⛔ **This decision no longer holds. There is no forward cap.** The constant was removed in
> `aef3a6b` because the cap made a session permanently unrecoverable once a gap crossed it. See
> [Reversal of Decision A](#reversal-of-decision-a--the-forward-cap-was-removed-2026-07-31) at the
> end of this file. The analysis below is retained because its framing of *what the gap is* is
> still correct and is what ultimately showed the cap could not be sized safely — but do not
> implement from it.

> **Revised after adversarial review** (`C1`, `M2` in
> [`FINDINGS_PHASE1_PLAN_REVIEW_2026-07-26.md`](FINDINGS_PHASE1_PLAN_REVIEW_2026-07-26.md)).
> This decision previously said 64, derived from `PIPE_MAX_W + MAX_PTO = 35` and asserted as a
> **hard bound**. That derivation was incomplete and the assertion was wrong. Both are corrected
> below; the earlier reasoning is retained only where it is still valid. (`[L4]`: even that
> figure was one too many — `MAX_PTO = 3` yields **two** probe sends, since the client increments
> and raises at the threshold *before* sending, `device.py:2721-2726`. Moot now, but noted so the
> arithmetic is not re-derived wrongly later.)

**What the gap actually is** (unchanged, and still the right framing): not the in-flight chunk
depth, but the run of consecutive transmissions that never reach `decryptCommand` — the client
burns a counter per transmission whether or not it lands
([device.py:747-760](../../py-opendisplay/src/opendisplay/device.py)). Sources: frames lost on
air; frames dropped at the command ring
([esp32_ble_callbacks.h:127-128](../src/esp32_ble_callbacks.h), on the NimBLE host task *before*
decrypt); and retransmissions.

**Why there is no firmware-side hard bound.** The earlier derivation counted two of the client's
three transmit sites. New sends are window-credit-limited
([device.py:2688-2694](../../py-opendisplay/src/opendisplay/device.py)) and PTO probes resend
exactly one chunk (`:2715-2726`) — but **selective repair spends no window credit at all**:

```python
device.py:2789-2796
                for m in missing:          # every hole below highest_recv, up to W-1 of them
                    if do_retx:
                        await _send(m)     # fresh counter each; no credit consumed
                        retx_count += 1
```

and the client deliberately preserves queued ACKs to spend on repeat repair rounds
(`drain_stale=False`, `device.py:781-786`). The only ceiling is the client's retransmit budget:

```
max_retx = max(3 * W, ceil(n * 0.5))        device.py:2672, commands.py:96
```

which is 96 for `W = 32` and scales with *chunk count* for large uploads. Worked through, the
reachable gap is ~66 at `blocks_per_ack = 2` and ~96 at `blocks_per_ack = 1` — and
`blocks_per_ack` is a **user-settable Home Assistant option** (`min=1, max=32`), which firmware
clamps only at the top ([display_service.cpp:2723-2725](../src/display_service.cpp)).

**So the honest statement is: firmware cannot bound this from its own constants.** The bound
lives in a client in another repo, behind a user-facing setting. Any cap is a heuristic; the
question is only how much headroom it buys and what it costs.

**`OD_NONCE_FORWARD_CAP = 128`.** It covers the realistic worst case (~96) with margin, and
under the bitmap it costs **nothing** — the cap is a comparison, not storage. The old "tight is
correct" argument is withdrawn: it rested on the `[M1]` jam-forward DoS, which **cannot occur
once D2 is fixed.** After commit-after-verify, only a CCM-authenticated frame advances
`last_seen_counter`, so an attacker can only commit counters the client genuinely transmitted —
never past the client's own high-water mark. Repairs then carry *fresh, higher* counters
(`device.py:759`), so no future client frame ever falls below the window. The frames a jam could
strand do not exist. `[M1]` was an artifact of the value-ring design and does not survive
commit-after-verify plus a bitmap.

**Step 6's source comment must record the mechanism, not the number.** A `blocks_per_ack` change
in Home Assistant, or a larger `max_retx`, silently invalidates any figure written into
`communication.cpp`. The comment should say *why* the cap exists and *where* the real bound
lives, so the next reader knows it is a heuristic to be re-checked rather than an invariant that
has been proven.

**Residual risk, stated plainly.** A gap beyond 128 is still possible on a pathological link with
`blocks_per_ack = 1` and a multi-thousand-chunk upload. With Step 4b in place that is no longer
fatal — the frames are silently dropped and repaired by the normal SACK path — which is
precisely why `[H1]` is treated as in-scope rather than deferred.

**Backward width is a separate constant** — see Decision B. Under a bitmap the two sides are
independent: forward acceptance stores nothing.

### Decision B — RESOLVED: sliding bitmap, **IPsec/DTLS shifting style**, `uint64_t[4]` = 32 B

**Bitmap over value ring.** To be precise about why, because the ring is *not* buggy: a ring of
depth `D` policing a backward window `W` is sound iff `D >= 2W`, since at most `2W` distinct
counters can be accepted while any given one remains in-window. Today's 64/32 and the parent
plan's 256/128 both satisfy it. The objection is not correctness but that `D >= 2W` is a
hand-maintained coupling between constants in two files, provable only by non-obvious
combinatorics, in a function that already shipped one undetected state bug (D4). The bitmap
makes eviction and falling-out-of-window *the same event*, so the coupling ceases to exist —
and takes D3 and D4 with it (Step 1).

Storage scales at **16 B per unit of backward window** for a ring (`2W` entries × 8 B) versus
**1 bit** for a bitmap — a fixed 128:1 ratio at any width:

| Backward window | Ring (`2W × 8 B`) | Bitmap |
|---|---|---|
| 32 (today) | 512 B | 8 B |
| 127 | 2,032 B | 16 B |
| 255 (**chosen**) | 4,080 B | **32 B** |

`OD_NONCE_BACKWARD_BITS = 256` (`uint64_t[4]`, backward window 255) is generous for a tolerance
that is **never exercised in normal operation** — the client's counters are strictly increasing
and both transports preserve ordering — but at 32 B there is no reason to economize, and the
margin over `OD_NONCE_FORWARD_CAP = 128` keeps the wholesale-clear branch off the normal path
(Step 1). Net struct change is **−480 B** against today, versus **+1,536 B** for the ring plan.
The `esp32-N4` link headroom measured for the ring (81,940 → 83,476 B of 327,680, SUCCESS both
ways) is therefore moot; recorded only so the fallback question never gets reopened.

**Shifting (RFC 4303 / RFC 6347) over non-shifting (RFC 6479 / WireGuard).** Both are the same
algorithm; they differ only in how the window advances:

- **Shifting** — the bitmap is a plain integer shifted left by `diff` on advance. This is what
  IPsec ESP §3.4.3 and DTLS §4.1.2.6 describe.
- **RFC 6479 / WireGuard** — a *circular* bit array indexed by `counter mod size`, where
  advancing clears the blocks between the old and new positions instead of shifting. It also
  requires the array to be strictly larger than the window ("redundant bits") for the clearing
  to be safe.

**Pick shifting.** RFC 6479 exists to avoid the cost of shifting a *large* window — WireGuard
carries ~8,192 bits because it is a high-throughput VPN over UDP with genuine reordering.
Ours is 128 bits over two words at roughly 40 frames/s, where the whole operation is a handful
of instructions and is dwarfed by the AES-CCM decrypt of the same frame. Choosing RFC 6479
would buy nothing measurable and would add modular indexing plus the redundant-bits invariant —
more subtlety, in exactly the function where subtlety has already cost us. The one real hazard
in the shifting form is UB on `x << 64`, which Step 3 calls out and Step 5 tests directly.

Both styles are equally standard; this is a sizing call, not a security one.

### Decision C — RESOLVED: delete `verifyNonceReplay()` outright

No compatibility wrapper. Nothing outside `decryptCommand` should ever be able to commit nonce
state, and a surviving wrapper is an invitation to re-introduce exactly the commit-before-verify
bug (D2) that Phase 1 exists to remove. Three deletions, all in Step 4:

| Location | Action |
|---|---|
| [encryption.cpp:114-156](../src/encryption.cpp) | Delete the function body; `nonceCheck` + `nonceCommit` replace it |
| [encryption.h:17](../src/encryption.h) | Delete the declaration |
| [main.h:276](../src/main.h) | Delete the duplicate declaration |

`nonceCheck` and `nonceCommit` are **file-static** in `encryption.cpp` — they get no header
declaration at all, so the "only `decryptCommand` may commit session state" rule is enforced by
linkage rather than by convention. The pure logic they wrap lives in `src/nonce_window.h` and
is what the host test targets (Decision D), so staying static costs no testability.

**The `NonceResult` *type* is shared, and that is not a loophole.** Step 4b needs
`decryptCommand` to report *why* it failed, so the enum is declared in `nonce_window.h` and
reaches `encryption.h`. A visible type conveys no ability to read or mutate `encryptionSession`;
the functions that can are still unreachable outside `encryption.cpp`.

**Sequencing note:** this is not a standalone edit — `decryptCommand` is the sole caller, so the
deletion only compiles as part of Steps 1-4 landing together.

### Decision D — RESOLVED: standalone `tools/test_nonce_window.cpp`, no PlatformIO env

No `[env:native]` and no `test/` directory, so the 11-env matrix and a bare `pio run` are
untouched. (Note `.cpp`, not `.c` as first written — it includes a header shared with C++
firmware code.)

**This forces one structural refinement, and it is a good one.** The pure window logic must
compile with no Arduino, no mbedtls, and no `millis()` in its translation path — so it moves
into a dependency-free header operating on plain values:

```
src/nonce_window.h        static inline, zero dependencies: (bitmap*, last_seen, counter)
                          -> NonceResult / updated state. No session, no logging, no crypto.
src/encryption.cpp        static nonceCheck()/nonceCommit() wrap it with encryptionSession
                          + od_log_*. Still file-static (Decision C) — linkage still enforces
                          "only decryptCommand may commit session state".
tools/test_nonce_window.cpp  includes ONLY src/nonce_window.h
```

This keeps Decision C's guarantee intact while making the part worth testing reachable: the
bit-shifting state machine is exactly the code with edge cases, and it has no business knowing
about sessions or logging anyway.

```bash
g++ -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=undefined,address \
    tools/test_nonce_window.cpp -o /tmp/test_nonce_window && /tmp/test_nonce_window
```

`-fsanitize=undefined` is not decoration — it is what catches the `x << 64` UB in Step 3
automatically rather than relying on the test author to predict it.

**Coverage (all against `nonce_window.h` directly):**
- **Shift edges:** `fwd` = 0, 1, 63, 64, 65, 127, 128 (reachable), plus 129, 191, 192, 255, 256,
  257 driven directly against `nonce_window.h` to cover the word boundaries and the
  wholesale-clear guard (Step 3).
- **Purity of `nonceCheck` (D2):** snapshot the state, call `nonceCheck` on every result class,
  `memcmp` the state afterwards. This is the single most valuable assertion in the file — it is
  the property that "the tag is the only thing that may advance replay state" rests on.
- **D3:** commit a counter, re-present the same counter → `NONCE_REPLAY`. Cover `fwd == 0`
  specifically, which is the exempted case today.
- **Fresh session:** `last_seen = 0`, empty bitmap → counter 0 accepted exactly once, rejected
  on re-presentation. No `has_seen_counter` involved.
- **Wholesale slide:** forward jump ≥ `OD_NONCE_BACKWARD_BITS` → bitmap cleared; previously
  seen counters now return `OUT_OF_WINDOW`, **not** `REPLAY` (both reject, but conflating them
  would hide a genuine slide bug).
- **Differential/property test:** run a few thousand pseudo-random accept/replay/gap sequences
  against a naive `std::set` oracle that models "seen, within window". Cheap, and it covers
  the interleavings hand-written cases miss.
- **Counter arithmetic (`[M1]`):** assert the unsigned form from Step 2 behaves correctly at
  `counter = 2^63`, `last_seen = 1` and near `UINT64_MAX` — the inputs that make the signed form
  undefined. Unreachable in normal operation (2^63 frames) but attacker-reachable, and free to
  get right. UBSan makes this test self-checking.

**Not covered here, deliberately:** that a nonce failure leaves `integrity_failures` untouched
(D1) lives in `decryptCommand`, not in the window logic. That assertion belongs to Step 5's
hardware tests 1-2.

**CI `[L5]`:** add a **separate top-level `host-tests` job** in `.github/workflows/main.yaml` —
**not** a step inside the existing `build` job, which is an 11-entry `matrix.environment`
(`:10-23`) and would run the host test eleven times. It needs no toolchain beyond the runner's
stock `g++` and gates every push alongside the firmware builds.

Confirmed build-safe: `tools/test_nonce_window.cpp` is invisible to every firmware build —
`build_src_filter = +<*> -<main_mbed.cpp>` (`platformio.ini:34`) is relative to the default
`src_dir`, and no env adds `tools/`.

### Decision E — RESOLVED: no wire change. Recorded for the future, **not actioned**

`decryptCommand` returning false yields an unencrypted `RESP_NACK`
([communication.cpp:698-703](../src/communication.cpp)) for both "lost your window" and
"tag failed". Phase 1 leaves that exactly as it is.

**Why it is acceptable to leave — corrected.** This decision originally claimed the client's
"existing retransmit/PTO machinery recovers on its own". **That was false** (`[H1]`): the 3-byte
`0xFF` NACK is intercepted at `device.py:833-838` and raised as `IntegrityCheckError`, which the
pipe send loop does not catch, so the transfer dies on the first rejected frame.

What makes leaving the *wire* alone acceptable is **Step 4b**, which fixes this firmware-side by
sending nothing at all for a nonce-rejected pipe DATA frame. Silence is already a first-class
signal in the pipe protocol — it means "lost", and the SACK path repairs it. No new response
code is required to get correct recovery; the client needs no change.

**Recorded for a future protocol revision** (do not implement in Phase 1, and do not let a
reviewer re-open it here): a distinct response code for "nonce out of window" would let a
client re-sync deliberately — abandon the in-flight window and re-authenticate — instead of
burning its `MAX_PTO` budget discovering the same thing by timeout. That is a strictly better
recovery, and worth doing *if* the wire is being revised for other reasons. It is not worth
doing on its own: it is a cross-repo change through `../opendisplay-protocol`, a `--push` to
all four firmware repos, and a coordinated py-opendisplay release, to save a few seconds on a
path Phase 1 already makes non-fatal.

#### `[H2]` Also recorded here: device and client share one nonce space (shipping defect)

**Not a Phase 1 change — recorded so one future wire revision fixes it together with the
response code above.** Phase 1 is the change that will make a future reader believe this layer
has been audited, so an unrecorded defect here is worse than one nobody has looked for.

The outbound nonce is built as `session_id || nonce_counter`
([encryption.cpp:158-174](../src/encryption.cpp)) from the device's **own** counter, and the
inbound nonce is `session_id || client_counter` off the wire. `encryptResponse`
([:740-743](../src/encryption.cpp)) and `decryptCommand` ([:704-705](../src/encryption.cpp))
both then take `nonce_full[3..15]` as the CCM nonce, under the **same `session_key`**, with
**no direction separator**, and both counters reset to 0 at session start
([:210](../src/encryption.cpp)/[:655](../src/encryption.cpp); `device.py:735`).

So device response #*k* and client command #*k* encrypt under an identical (key, nonce). CCM is
CTR underneath and the keystream depends only on key and nonce — the AAD differs but feeds only
the tag — so `C_resp ⊕ C_cmd = P_resp ⊕ P_cmd`. Responses are short and highly predictable
(`{RESP_ACK, cmd, status}`, pipe ACKs), so a passive eavesdropper recovers the leading plaintext
of the matching command. Authenticity is unaffected; this is a confidentiality failure.

**Age and blast radius (verified from history):** introduced in `b04a22b` *"Add encryption"*
(2026-03-10) — the construction is byte-identical today, and `fd0d73a` merely moved it from
`main.cpp` into `encryption.cpp`. It is **not** a regression from this branch. The same
construction is in all four firmware repos (`Firmware_NRF54/src/opendisplay_pipe.c:518-522`,
`Firmware_Silabs/opendisplay_pipe.c:476`, `Firmware_NRF/encryption.c:197`) and both clients,
because [opendisplay_protocol.h:203-209](../include/opendisplay_protocol.h) specifies the
envelope and `CCM nonce = nonce[3..15]` but says **nothing about the nonce's internal
structure** — there is no single place where a reviewer would have seen the missing direction
separator.

**Therefore the fix is a spec change first, not four patches:** define the nonce layout in the
canonical header *including* a direction bit, then `--push`. Whoever schedules it must also plan
the compatibility story — an old client and a new device would disagree on the nonce. See
`[M3]` in Step 1 for the in-scope consequence: `resetNonceState()` must keep zeroing
`nonce_counter`, or the device reproduces this reuse against itself across a re-auth.

**Action for Phase 1: none in code.** This paragraph, plus a note in
`../opendisplay-protocol/agents/` where cross-repo design issues live — this repo is not where
the next person will look for a protocol-level defect.

---

## Scope boundaries (do not drift)

- **Do not touch the 30 s auth-challenge freshness window** ([:587-588](../src/encryption.cpp)
  stamp, [:604-608](../src/encryption.cpp) enforcement). It is a cross-repo wire contract
  specified at [opendisplay_protocol.h:384](../include/opendisplay_protocol.h), it is not a
  liveness timer, and it cannot wedge anything — see the parent plan's "Deliberately NOT
  changed". This includes leaving the known boot-window quirk (`server_nonce_time == 0`) alone.
- **Do not disable `session_timeout_seconds` here.** That is Phase 5. Consequence to state
  plainly: Phase 1 alone does **not** close the mid-transfer freeze caused by expiry firing
  inside `isAuthenticated()` ([:195-199](../src/encryption.cpp)) — it closes the *nonce* arm
  only. Both arms must land before the field failure is fully addressed.
- **Do not add link-drop behaviour to `clearEncryptionSession()`.** That guard is Phase 5.
  After Phase 1 the CCM-tag path can still clear a session under a live link, leaving the
  client talking to a device that answers `0xFE` — a known, accepted gap until Phase 5.
- **Do not shrink the backward window to zero.** TLS over TCP accepts no out-of-order records
  at all and lets the AEAD tag enforce ordering for free (RFC 8446 §5.3), and OpenDisplay's
  situation is arguably TLS's rather than DTLS's — ordered transports, a strictly increasing
  client counter. But narrowing the accepted range is a larger behavioural change than
  widening it, other firmware repos share this protocol, and the field failure is a *forward*
  gap problem. Out of scope for Phase 1; noted so the option is not lost.
- No protocol-header edits, no config-schema changes, no client-side changes.

**Compliance with the parent plan's "NO wire protocol changes" constraint** — every Phase 1
change is inside its in-bounds list:

| Change | Why it is in bounds |
|---|---|
| Widened forward cap, bitmap replay set | Firmware-local constants no client reads; accepting more legitimate frames and rejecting a replay are both already-legal outcomes of the existing nonce rules |
| D3 fix (replay of the highest-seen counter now rejected) | Rejecting a replay is what the nonce rules already prescribe; the exemption was the deviation |
| Step 4b (no fatal NACK for a nonce-rejected `0x0081`) | **Conformance**, not change — `pipe-write-protocol.md` §5.2 already reserves NACKs for unrecoverable conditions, "not ordinary packet loss" |
| `decryptCommand` reason out-param | Internal signature; nothing on the wire |

Nothing here requires an edit to `include/opendisplay_protocol.h` or
`include/opendisplay_structs.h`. Run the constraint check from the parent plan before calling
Phase 1 done.

## Files touched

| File | Change |
|---|---|
| `src/nonce_window.h` | **new** — dependency-free window state machine (Decision D) |
| `src/encryption_state.h` | ring → bitmap, **−480 B** |
| `src/encryption.cpp` | the substance: `nonceCheck`/`nonceCommit`, `decryptCommand` rewiring, `verifyNonceReplay` deleted |
| `src/encryption.h` | declaration removal (Decision C); `decryptCommand` reason out-param (Step 4b) |
| `src/main.h` | duplicate declaration removal (Decision C); same signature change (Step 4b) |
| `src/communication.cpp` | **Step 4b**: suppress the fatal NACK for nonce-rejected `0x0081` — plus the comment at [:772-775](../src/communication.cpp) |
| `docs/pipe-write-protocol.md` | *optional* — one clarifying sentence in §5.2 that a nonce-rejected data frame is classed as loss. No behaviour change to document: Step 4b makes the firmware conform to what §5.2 already says. |
| `tools/test_nonce_window.cpp` | **new** — host test (Decision D) |
| `.github/workflows/main.yaml` | separate `host-tests` job: compile + run the host test |

**No `platformio.ini` change**: the bitmap is smaller than what it replaces, so the `esp32-N4`
link headroom that gated the original proposal is not a consideration on any target.
**No protocol-header change** (Decision E). **No client-side change.**

---

# As-built: what actually shipped

**Written 2026-07-26 after reviewing the landed code.** Everything above this line is the plan as
written. This section is the ground truth. Line numbers are against the tree at `23ecaed`.

Commits, in order:

| SHA | Subject | Corresponds to |
|---|---|---|
| `0a60712` | replace 512 B replay value ring with 32 B sliding bitmap | Steps 1-4, Decisions A/B/C |
| `9b827f3` | split check from commit; stop counting packet loss as tampering | Steps 2-4 (D1/D2) |
| `eeadbe0` | do not answer a nonce-dropped 0x0081 frame with a fatal NACK | Step 4b |
| `44df35a` | ci: separate host-tests job | Decision D `[L5]` |
| `23a586a` | test: host test for the nonce sliding-window state machine | Step 5 / Decision D |
| `c87ff60` | review follow-ups (split log budgets, readable replay log, honest comments) | Step 4 logging, Decision C caveat |
| `55a2478` | answer session-id mismatch with AUTH_REQUIRED, not a fatal NACK | **not in the plan** — field-failure response |
| `77ebdcd` | drop the BLE link after 10 consecutive unauthenticated commands | **not in the plan** — Phase 5 work, pulled forward |
| `23ecaed` | drop the BLE link inline on nRF; loop() is starved mid-transfer | **not in the plan** — follow-up to `77ebdcd` |

## Per-step / per-decision status

| Item | Status | Evidence |
|---|---|---|
| **Step 1** — value ring → sliding bitmap | **As specified** | `src/encryption_state.h:22-28` — `uint64_t replay_bitmap[OD_NONCE_BITMAP_WORDS]` replaces `uint64_t replay_window[64]`. `OD_NONCE_BACKWARD_BITS 256` / `OD_NONCE_BITMAP_WORDS` at `src/nonce_window.h:33,42`. No `replay_window_index`, no `has_seen_counter` anywhere (`grep` for both returns nothing), so **D3 and D4 are structurally gone**, not patched. |
| **Step 1** — `resetNonceState()` naming all four fields `[M3]` | **As specified** | `src/encryption.cpp:123-128`, sets `nonce_counter`, `last_seen_counter`, `integrity_failures`, `memset(replay_bitmap)`. Called from both required sites: `clearEncryptionSession()` at `src/encryption.cpp:247` and `handleAuthenticate()`'s fresh-session block at `src/encryption.cpp:692`. The `[H2]`-motivated warning about `nonce_counter` is carried in the comment at `:114-122`. |
| **Step 2** — pure `nonceCheck()` | **As specified** | `od_nonce_check()` at `src/nonce_window.h:74-82` is byte-for-byte the four ordered tests from the plan, with unsigned wrapping `fwd`/`back` at `:75-76`. The session-aware wrapper `nonceCheck()` is file-static at `src/encryption.cpp:162-188` and writes nothing to `encryptionSession` on any path. |
| **Step 3** — `nonceCommit()` | **As specified (one structural difference)** | `od_nonce_commit()` at `src/nonce_window.h:126-145`; `od_nonce_bitmap_shift_left()` at `:87-105` handles `shift == 0` (`:88`) and `shift >= 256` (`:89-92`) explicitly. **Difference:** the plan put the wholesale-clear guard in `nonceCommit`; as built it lives inside the shift helper. Behaviourally identical and arguably better — the guard now protects *every* caller of the shift, not just the commit path. |
| **Step 4** — `decryptCommand` rewiring | **As specified** | `src/encryption.cpp:722-800`. `nonceCheck` at `:738`; the nonce-rejection arm at `:739-755` returns false with **no** `integrity_failures` touch. `nonceCommit(nonce_counter)` at `:782` is the **first statement of the `if (success)` arm**, ahead of the malformed-`payload_length` early return — `[L2]` honoured, with the reason spelled out in the comment at `:776-781`. The tag-failure arm at `:794-798` is unchanged. |
| **Step 4** — logging demotion + rate limit `[L7]` | **As specified, and better** | Out-of-window/replay log at `src/encryption.cpp:741-753` is `od_log_warn`, rate-limited; session-id mismatch at `:174-181` is `od_log_warn` with the two full session-ID dumps reduced to two bytes each. The plan asked for "a" rate limit; as built there are **two independent 5 s budgets** (`nonce_log_badsession_ms` / `nonce_log_window_ms`, `:137-145`) so a peer spamming session-id mismatches cannot silence the out-of-window line. Improvement over the plan. |
| **Step 4** — delete `verifyNonceReplay()` (Decision C) | **As specified** | Gone from `src/encryption.cpp`, `src/encryption.h`, and `src/main.h`. `grep -rn "verifyNonceReplay" src/ tools/ include/` returns nothing. |
| **Step 4b** — silent drop for nonce-rejected `0x0081` | **As specified** | `src/communication.cpp:866-868`: `if (nonce_loss && command == CMD_PIPE_WRITE_DATA) return;` where `nonce_loss` covers `NONCE_OUT_OF_WINDOW` and `NONCE_REPLAY` only. Tag failures still NACK (`:900-903`). `0x0071` deliberately untouched, documented at `:862-865`. Reason out-param plumbed through `src/encryption.h:24-28` and `src/main.h:274`. |
| **Step 5** — host test (Decision D) | **Done** | `tools/test_nonce_window.cpp`, 702 lines. `g++ -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=undefined,address` → **PASSED 38199 checks**. Every case in Decision D's coverage list is present: `test_fresh_session` (`:161`), `test_d3_same_counter_replay` (`:190`), `test_check_is_pure` (`:241`, the memcmp purity assertion), `test_shift_edges` (`:387`), `test_wholesale_slide` (`:408`, asserting `OUT_OF_WINDOW` not `REPLAY`), `test_bit_indices_after_shift` (`:449`), `test_counter_arithmetic_extremes` (`:493`), `test_differential_against_oracle` (`:623`). |
| **Step 5** — build gate | **Done** | `pio run` — **all 12 environments SUCCESS** (the plan and CI both said "11"; the matrix is now 12). `esp32-N4` links at 24.9% RAM / 81,468 B, i.e. **below** the 81,940 B the ring plan measured — the bitmap gave the byte budget back as predicted. |
| **Step 5** — hardware tests 0, 1, 2, 2b, 2c, 3, 4, 4b, 5, 6 | **NOT DONE** | See ["Unverified on hardware"](#unverified-on-hardware--the-honest-list). This is the single largest gap in Phase 1. |
| **Step 6** — comment hygiene | **As specified** | `src/communication.cpp:977-995`. Records the *mechanism* (client retransmit budget `max_retx = max(3*W, n/2)`, `blocks_per_ack` as a user-facing HA option) and explicitly calls `OD_NONCE_FORWARD_CAP` a heuristic, not an invariant — exactly what Decision A `[C1]` demanded. No number is asserted. |
| **Decision A** — forward cap 128 | **As specified** | `src/nonce_window.h:40` — `#define OD_NONCE_FORWARD_CAP 128`, with the "this is a heuristic, the bound lives in another repo" rationale in the comment at `:35-39`. |
| **Decision B** — shifting bitmap, `uint64_t[4]` | **As specified** | `src/nonce_window.h:33` (256 bits), `:87-105` (RFC 4303 shifting form, not RFC 6479 circular). |
| **Decision C** — file-static, no wrapper | **As specified, with an honest correction the plan did not anticipate** | `nonceCheck`/`nonceCommit` are file-static (`src/encryption.cpp:162`, `:190`) and have no header declaration. **But** the implementation noticed and documented at `src/encryption.cpp:152-160` that Decision C's "enforced by linkage" claim is weaker than written: `encryption_state.h` must include `nonce_window.h` for `OD_NONCE_BITMAP_WORDS`, and `main.h` includes `encryption_state.h`, so the `static inline` primitive `od_nonce_commit()` is visible in **every** translation unit alongside `extern encryptionSession`. Nothing stops a determined caller from committing state directly. **Decision C as written above is therefore inaccurate and this paragraph supersedes it:** linkage enforces the rule for the session-aware wrappers; convention enforces it for the raw primitive. |
| **Decision D** — standalone host test + separate CI job | **As specified** | `.github/workflows/main.yaml:8-27` — top-level `host-tests` job, not a step in the 11-entry matrix, with the `-fsanitize` rationale in-line. No `[env:native]`, no `test/` dir, `pio run` unaffected. |
| **Decision E** — no wire change, `RESP_NACK` left alone | **Superseded in part by `55a2478`** | See below. Phase 1 as shipped no longer answers a session-id mismatch with `RESP_NACK`; it answers `RESP_AUTH_REQUIRED`. Still no header change and no new response code. |

## Post-implementation changes — what the field failure forced

The last three commits were **not** in the plan. They were added after the implementation agent
finished, in response to a live hardware failure.

### What actually happened on the bench

A client lost its session mid-connection while the device still believed the session was live.
py-opendisplay's `_direct_write_chunk_size()` keys purely on `self._session_key is not None`
(`../py-opendisplay/src/opendisplay/device.py:1916-1929`), and `_write` picks the plaintext branch
under the same condition (`:772-776`), so the client silently fell back to **unencrypted
`0x0071` chunks of `CHUNK_SIZE = 230`** (`../py-opendisplay/src/opendisplay/protocol/commands.py:70`)
— 232 bytes on the wire.

232 bytes is **not** below the firmware's "unencrypted command received" length gate
(`BLE_CMD_HEADER_SIZE + 16 + 16 = 34`, `src/communication.cpp:818`), so those frames sailed past
the gate and into `decryptCommand`, where `nonceCheck` read 8 bytes of image data as a session id
and returned `NONCE_BAD_SESSION`. Before Phase 1 that counted toward `integrity_failures`, and
three of them cleared the session — ugly, but it is what made every subsequent command answer
`0xFE`, which is what eventually made the client re-authenticate. **Phase 1's `[L7]` change
removed that accidental recovery path** and left the device answering a fatal 3-byte `0xFF` NACK
forever to a client that could never resolve the mismatch by retrying.

**The parent plan's Context §1 narrative is wrong about this, and so is the "mechanism caveat"
above:** the observed field wedge did not come from a forward nonce gap at all. It came from a
session-identity divergence plus a client that degrades to plaintext instead of erroring. The
nonce-gap story remains a genuine defect class, but it is **not** what was reproduced on hardware.

### `55a2478` — session-id mismatch answers `AUTH_REQUIRED`, not `NACK`

`src/communication.cpp:893-897`. On `NONCE_BAD_SESSION`, call `rejectUnauthenticated(command)`
(3-byte `{RESP_ACK, cmd_lo, RESP_AUTH_REQUIRED}`) instead of falling through to the NACK.

**Verdict: correct, and in bounds.** The client classifies a 3-byte `0xFE` as
`AuthenticationRequiredError` (`device.py:824-827`) — a different exception hierarchy from the
`IntegrityCheckError` raised by `0xFF` (`device.py:834-838`) — and the HA integration escalates
`AuthenticationRequiredError` into a user-visible reauth flow rather than a silent abort. This is
`RESP_AUTH_REQUIRED` used in exactly its documented meaning ("this command requires a live
authenticated session"), which satisfies the parent plan's "sending an existing `RESP_*` code in a
new situation, as long as the code's documented meaning is unchanged" allowance. No header edit.

**Caveat that is not written down in the code:** py-opendisplay does **not** re-authenticate
reactively. `_reauthenticate_if_needed` is proactive and time-based only (`device.py:788-804`),
and the pipe send loop catches nothing but `BLETimeoutError` (`device.py:2714-2717`). So the
in-flight upload still dies; what `55a2478` buys is the *right kind* of death — one that HA
converts into a reauth — instead of an `IntegrityCheckError` loop with no exit. The comment at
`src/communication.cpp:884-886` ("the client raises `AuthenticationRequiredError` and
re-authenticates, and the mismatch clears in one round trip") **overstates this**: it clears on the
next *connection*, not the next round trip.

### `77ebdcd` — drop the BLE link after 10 consecutive unauthenticated commands

`src/communication.cpp:56-201`. New `rejectUnauthenticated()` / `resetAuthGateRejects()` /
`serviceBleAuthAbuseDisconnect()`; every `RESP_AUTH_REQUIRED` the encryption gate emits now routes
through the counter (`src/communication.cpp:815`, `:821`, `:895`).

**This crosses the plan's own scope boundary.** The Scope boundaries section above says verbatim:
*"Do not add link-drop behaviour to `clearEncryptionSession()`. That guard is Phase 5."* The
letter of that boundary is not violated — the drop is not in `clearEncryptionSession()`; it is
keyed on the *symptom* (repeated `0xFE`) rather than on the *event* (session cleared). But it is
unambiguously **the Phase 5 deliverable "make a dead session with a live link impossible", arrived
at from the other end**, and it should be recorded as such rather than as a Phase 1 refinement.

**Verdict: justified as a field-failure response, but it is scope creep and Phase 5 must now be
re-scoped around it** (see the parent plan's Phase 5 entry, updated accordingly). It is *not*
redundant with Phase 5: Phase 5's guard fires on the clear itself and drops the link immediately;
this one waits for ten wasted round trips first. Phase 5 should subsume it, not duplicate it.

**Three problems found in review, none of them blockers, none fixed here (documentation-only pass):**

1. **The threshold is below the client's pipe window.** `AUTH_GATE_MAX_CONSECUTIVE_REJECTS = 10`
   (`src/communication.cpp:87`), but `_send_pipe_chunks` blasts a full window of `0x0081` frames
   before its first read (`device.py:2689-2694`) with `w_eff = max(1, min(max_queue_size,
   dev_max_window, 32))`, **default 16**, and `_write_pipe_frame` deliberately skips re-auth for
   the entire stream (`device.py:778-786`). So when a session dies mid-upload — precisely the
   Phase 5 scenario — a **legitimate** client emits 16-32 gated frames back-to-back and trips the
   guard. That is arguably the right outcome (the upload is dead either way, and a clean reconnect
   is better than a spin), but the code's justification comment at `src/communication.cpp:70-80`
   reasons only about a client "probing several gated commands before it authenticates" and never
   considers the window burst. Worse, that stated justification is **not corroborated by
   py-opendisplay**: on the normal path the client sends *zero* gated commands before
   `CMD_AUTHENTICATE` (`device.py:652-675` — auth is the first write after connect), and `0x0044`
   named in the comment is not a py-opendisplay opcode at all (`READ_FW_VERSION` is `0x0043`,
   `commands.py:22`, and it bypasses the crypto wrappers entirely). The threshold of 10 is fine;
   the reasoning recorded for it is wrong.
2. **The count is not cleared on disconnect.** `resetAuthGateRejects()` has exactly two callers:
   a successful decrypt (`src/communication.cpp:908`) and a successful authentication
   (`src/encryption.cpp:691`). Neither `disconnect_callback` (`src/device_control.cpp:227-240`)
   nor `MyBLEServerCallbacks::onDisconnect` (`src/esp32_ble_callbacks.h:57-70`) clears it. The
   guard tries to compensate with `authGateLastHandle` (`src/communication.cpp:122-126`), but on
   nRF `Bluefruit.connHandle()` returns the single `_conn_hdl` (`bluefruit.cpp:643-646`), which
   is typically the *same* value for successive peripheral connections. So client A can accrue 9
   rejections, disconnect, and client B inherit them — the exact outcome the comment at
   `src/communication.cpp:121-123` promises cannot happen. Impact is small (B normally
   authenticates first, which resets), and **the fix is a one-line `resetAuthGateRejects()` call
   in each disconnect callback.** Not applied here.
3. **On ESP32 the guard cannot identify which central offended.** `authGuardLiveConnHandle()`
   returns `pServer->getPeerInfo(0).getConnHandle()` (`src/communication.cpp:104-112`) — peer
   *zero*, not the sender. The NimBLE write callback discards `connInfo`
   (`src/esp32_ble_callbacks.h:81-82`) and the command ring carries no handle, so the sender's
   identity is genuinely unavailable by the time `imageDataWritten` runs on the loop task
   (`src/main.cpp:415`). With `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` really being 3, a second central
   can therefore drive peer 0 — the legitimate client — off the link. This is the same missing
   peer-binding that finding `[D3]` above already documents for `isAuthenticated()`, so it adds no
   new capability an attacker did not have; but the guard's "drop only the link that actually
   earned it" comment (`src/communication.cpp:186-188`) is only true on nRF. Fixing it properly
   means widening the ESP32 command ring to carry the conn handle — a Phase 4 (connection
   exclusivity) change, not a Phase 1 one.

### `23ecaed` — drop the link inline on nRF

`src/communication.cpp:145-166`: on `TARGET_NRF` only, `rejectUnauthenticated()` calls
`serviceBleAuthAbuseDisconnect()` **inline** instead of leaving it to `loop()`.

**Verdict: the inline disconnect is SAFE on nRF, and both of the commit's factual claims check
out.** This was the highest-risk change in the branch and it survives scrutiny. The chain, verified
against the Adafruit core in `~/.platformio/packages/framework-arduinoadafruitnrf52-seeed`:

- **`loop()` really is starved.** The Arduino loop task is created at `TASK_PRIO_LOW = 1`
  (`cores/nRF5/main.cpp:88`, `cores/nRF5/rtos.h:58`); the "Callback" task that runs the write
  callback is `TASK_PRIO_NORMAL = 2` (`cores/nRF5/utility/AdaCallback.c:145`, `rtos.h:59`); the
  "BLE" event task is `TASK_PRIO_HIGH = 3` (`bluefruit.cpp:473`). A sustained flood of write
  callbacks therefore preempts `loop()` indefinitely. On top of that, the nRF `loop()` calls
  `serviceBleAuthAbuseDisconnect()` only *after* `idleDelay(sleep_timeout_ms)`
  (`src/main.cpp:522-531`), which can be seconds. The deferral genuinely does not work here.
- **`Bluefruit.disconnect()` cannot unwind into the callback we are inside.** It resolves to
  `BLEConnection::disconnect()` → `sd_ble_gap_disconnect(...)`
  (`libraries/Bluefruit52Lib/src/BLEConnection.cpp:204-207`), which is an **asynchronous**
  SoftDevice call: it queues the terminate and returns.
- **The disconnect callback is queued to the same task as the write callback, so it cannot
  preempt an in-flight command.** `BLE_GAP_EVT_DISCONNECTED` dispatches via
  `ada_callback(NULL, 0, Periph._disconnect_cb, ...)` (`bluefruit.cpp:849`), and `ada_callback`
  always enqueues onto the single "Callback" task queue (`AdaCallback.c:102-138`). The write
  callback reaches the same task because `setWriteCallback(fp, useAdaCallback = true)` defaults to
  the ada path (`BLECharacteristic.h:108`, dispatched at `BLECharacteristic.cpp:536-542`), and
  `src/ble_init.cpp:157` uses the default. **Strict serialization through one FreeRTOS queue** is a
  stronger safety argument than the one written in the source comment.
- **`[H4]` does not apply.** `[H4]`'s hazard is a `memset(session_key)` landing mid-`aes_ccm_decrypt`.
  Two independent reasons it cannot happen here: (a) nRF's `disconnect_callback`
  (`src/device_control.cpp:227-240`) does **not** call `clearEncryptionSession()` at all — it only
  runs `cleanupDirectWriteState`/`cleanupPartialWriteOnDisconnect`/`resetPipeWriteState`; and
  (b) by the time the drop is requested, `decryptCommand` has already returned — the caller does
  `rejectUnauthenticated(command); return;` — so there is no in-flight decrypt on this task
  either. Even on the rare inline-fallback path where `ada_callback` fails on `rtos_malloc` and
  `_wr_cb` runs on the BLE task (`BLECharacteristic.cpp:541`), the disconnect stays async and the
  teardown still lands on the Callback task afterwards.
- **There is prior art in this repo.** `enterDFUMode()` already calls
  `Bluefruit.disconnect(Bluefruit.connHandle())` from command-dispatch context
  (`src/device_control.cpp:844-848`).
- **The 0xFE really is on the air first.** nRF's `sendResponseUnencrypted` notifies inline via
  `imageCharacteristic.notify()` with no response ring (`src/communication.cpp:375-387`), so the
  comment at `:154-157` is accurate.

**One factual error in the code comments, which should be fixed when someone next touches the
file.** `src/communication.cpp:171-173` claims *"the nRF disconnect callback runs synchronously
from `Bluefruit.disconnect()`"*. It does not — `sd_ble_gap_disconnect` is async
(`BLEConnection.cpp:206`) and the callback is queued (`bluefruit.cpp:849`). This directly
contradicts the correct statement 13 lines earlier at `:158-160`. The *code* is right either way
(clearing `authAbuseDisconnectPending` before the disconnect is the conservative order regardless),
but a future reader relying on that comment would reason wrongly about re-entrancy.

## Things the plan asserts that the code contradicts

1. **Decision C's "enforced by linkage rather than by convention"** — half true. See the Decision C
   row above and `src/encryption.cpp:152-160`.
2. **Decision E's "Phase 1 leaves that exactly as it is"** — no longer true for
   `NONCE_BAD_SESSION`, which now answers `RESP_AUTH_REQUIRED` (`src/communication.cpp:893-897`).
3. **Step 4's `[L7]` policy statement** — "It is the right call" was written without foreseeing
   that routing `NONCE_BAD_SESSION` to "does not count" also removes the only mechanism that ever
   made a desynced client re-authenticate. `55a2478` restores that path deliberately. The `[L7]`
   reasoning is still sound; it was just incomplete.
4. **"CI builds all 11"** (Step 5) and `.github/workflows/main.yaml`'s "11-entry matrix" comment —
   the matrix is **12** environments. Cosmetic, but wrong in three places.
5. **The D1 mechanism caveat's framing** — the field failure that was actually reproduced was a
   session-identity divergence, not a forward nonce gap. See "What actually happened on the bench".

## Hard-constraint check — passes

| Constraint | Result |
|---|---|
| No edit to `include/opendisplay_protocol.h` | ✅ `git diff 02bdd5c..HEAD -- include/` is empty |
| No edit to `include/opendisplay_structs.h` | ✅ same |
| No config-schema change | ✅ no `tools/od-device-cli.py` `BLOCKS` change needed; no struct field added/resized/reordered |
| No new opcode or response code | ✅ `RESP_AUTH_REQUIRED` and `RESP_NACK` are both pre-existing, used in their documented meanings |
| Nothing beyond what `docs/pipe-write-protocol.md` permits | ✅ Step 4b is conformance to §5.2; `55a2478` sends an existing code in a new situation, expressly allowed by the parent plan |
| No Phase 5 work pulled in without acknowledgement | ❌ **`77ebdcd` pulls Phase 5's link-drop forward.** Acknowledged here and in the parent plan. |

## Unverified on hardware — the honest list

**Not one item of Step 5's hardware matrix has been run.** Phase 1 has been verified by
compilation (12/12 envs) and by a host-side state-machine test (38199 checks) and by nothing else.
Everything below is still open:

- **Test 0 — baseline on unmodified firmware.** Never run. The D1 mechanism caveat above therefore
  remains **unsettled**, and the "before/after" story for tests 1-2 still rests on an unverified
  model. The bench failure that *was* observed (session-id divergence + plaintext fallback) is a
  different mechanism entirely, which makes running Test 0 more important, not less.
- **Test 2b — does Step 4b actually let the transfer complete?** ⚠ **This is the one that matters
  most and it is completely unverified.** The entire justification for Step 4b is that silently
  dropping a nonce-rejected `0x0081` frame lets py-opendisplay's SACK path notice the hole,
  retransmit, and finish the upload. Nobody has watched that happen. The code path is
  `src/communication.cpp:866-868` — three lines whose correctness is a claim about a client in
  another repo. Reading the client supports the claim (the pipe loop blocks on ACK reads, not
  per-frame replies) but reading is not running. **Until Test 2b passes on hardware, treat "Phase 1
  saves the transfer" as a hypothesis and "Phase 1 saves the device" as the only supported claim.**
- **Tests 1, 2, 2c** — forward gap within/beyond the cap, and the `blocks_per_ack = 1`, `W = 32`
  worst case that motivated the 128 figure. Not run. `OD_NONCE_FORWARD_CAP = 128` is unvalidated
  against a real link.
- **Tests 3, 4, 4b** — true replay rejected; replay of the last frame of a session (D3) rejected
  with an observable non-idempotent command; the 64× ring-flush replay (`[H3]`) that is the only
  test distinguishing "the bitmap closed the widened hole" from "it closed the narrow one". The
  host test covers the equivalent state-machine transitions, but not end-to-end over BLE.
- **Test 5** — three forged tags still clear the session.
- **Test 6** — full Spectra transfer and an E1004 ~960 KB upload complete untouched.
- **Unlisted, added by the last two commits and therefore untested by construction:**
  - Does the nRF inline disconnect actually drop the link mid-flood? The starvation analysis says
    the deferred version could not, but neither version has been observed on a board.
  - Does a legitimate client whose session dies mid-pipe-upload get dropped at 10 rejections, and
    does HA recover cleanly from that disconnect (as opposed to from the `0xFE` it would otherwise
    have seen)? Per the pipe-window arithmetic above this **will** happen with default settings.
  - Does `55a2478`'s `AUTH_REQUIRED` actually drive HA's reauth flow end to end?

---

# Reversal of Decision A — the forward cap was removed (2026-07-31)

**Commit `aef3a6b`, on `fix/nonce-replay-window` (rebased onto the squashed `#132`/`#133`/`#134`
`main`).** Decision A is reversed. `OD_NONCE_FORWARD_CAP` no longer exists, and comparison moved
from modular to numeric ordering. Decisions B, C and D stand.

## What the cap did

The cap did not merely fail to help — it converted a transient link fault into a permanent session
fault. Once a gap exceeded 128:

1. `od_nonce_check()` returns `NONCE_OUT_OF_WINDOW`, so nothing commits.
2. `last_seen_counter` therefore never advances — that is the D2 fix working as designed.
3. The client re-encrypts every retransmission with a **fresh, higher** counter and never resends
   the original ciphertext (`_write_pipe_frame`, `device.py:2683`), so the next frame is rejected at
   a *greater* distance than the last.
4. Every subsequent frame is rejected, forever. Only re-authentication recovers, and the client
   deliberately does not re-authenticate mid-transfer (`device.py:778`). The transfer stalls until
   the 15-minute stuck-transfer watchdog releases the panel.

Step 4b's silent drop does not rescue this. Dropping silently avoids the immediate fatal teardown a
`0x81` NACK would cause, but the transfer is dead either way.

## Why the cap could not be sized

Decision A's own framing was right and is what condemns it: the ceiling is the client's retransmit
budget `max_retx = max(3*W, n/2)`, scaled by `blocks_per_ack`, a user-facing Home Assistant option
in another repo. That is order thousands for a full-panel upload — and it **accumulates across
aborted attempts**, because the client's counter keeps climbing while `last_seen` is frozen. With
`W = 32` and `blocks_per_ack = 1`, 16 queued gap-ACKs at `PIPE_RETX_ACK_SPACING = 2` burn 128
counters on repairs alone. No firmware-side number is defensible.

## Why removing it costs nothing

The cap was vestigial once D2 landed. All 8 counter bytes sit inside the CCM nonce, so a tampered
counter changes the keystream and fails the tag; `nonceCommit()` runs only on the success arm; and
passing the check mutates nothing. An attacker who cannot forge a tag could not advance `last_seen`
at any distance, cap or no cap. Nor is it DoS protection: the session id is cleartext in every
frame, so anyone able to flood CCM with a capped window could flood it without one.

## What replaced it

Numeric ordering, matching RFC 4303 Appendix A2, which likewise has no forward bound:

```
counter == last_seen            -> bit 0 set ? REPLAY : OK
counter >  last_seen            -> OK          (any distance; the tag is the gate)
counter <  last_seen, back<256  -> bit[back] set ? REPLAY : OK
counter <  last_seen, back>=256 -> OUT_OF_WINDOW
```

Consequences worth recording:

- **Modular arithmetic is gone.** It made a counter far behind indistinguishable from one far
  ahead, which is what allowed an ancient counter to present as an enormous forward jump — the
  "sharp edge" the old header admitted, where committing one would rewind `last_seen` and clear the
  bitmap. That is now impossible by construction rather than by the caller's contract. **Do not**
  reintroduce a bound as `cap = UINT64_MAX` or as "not-backward implies forward": either restores
  the overlap. `test_far_behind_is_never_forward()` pins both.
- **Counters no longer wrap**, per RFC 4303 §3.3.3. Reaching `UINT64_MAX` requires
  re-authentication; wrapping would reuse a `(key, nonce)` pair. Unreachable in practice.
- **`NONCE_OUT_OF_WINDOW` now means only "too far behind."** The rejection log computes direction
  from the counters instead of inferring it from the reason, which would print an underflowed
  20-digit distance.
- **`OD_NONCE_BACKWARD_BITS` keeps its value but loses its old justification**, which was stated in
  terms of the cap ("kept strictly greater than `OD_NONCE_FORWARD_CAP`"). It is now purely
  out-of-order tolerance, and its exact value is not load-bearing: a backward rejection is
  self-healing, because the retransmit carries a higher counter that is accepted unconditionally.

## Effect on the rest of this document

| Item | Status |
|---|---|
| **Decision A** | **Reversed.** No forward cap. |
| **Decision B** — shifting bitmap, `uint64_t[4]`, IPsec/DTLS style | **Stands.** The representation is unchanged; only the arithmetic over it moved from modular to numeric. |
| **Decision C** — file-static wrappers | **Stands**, including its as-built correction. |
| **Decision D** — standalone host test + separate CI job | **Stands.** |
| **Decision E** — no wire change | **Stands.** This reversal changes no byte on the wire: the accept set only grows, so no peer needs updating in lockstep. |
| **Step 6** — "write the mechanism, not a number" | **Stands, and is now literal**: there is no number to write. The `communication.cpp` comment records why there cannot be one. |
| **Step 5 hardware tests 1, 2, 2c** | **Obsolete as written.** They existed to validate the 128 figure. What replaces them is confirming a transfer survives a gap that *would* have crossed it. |
| **"Unverified on hardware"** | Still accurate, and this reversal did not change it: the cap was condemned by analysis, not by the bench. |

## Verification

Host suite: **47445 checks** pass under `-Werror` with ASan+UBSan. The same suite run against the
pre-change implementation fails **1635** checks, which is the evidence that the new tests
discriminate rather than merely pass. Added `test_forward_gap_is_not_a_cliff()` (the defect as a
*sequence*, since a point test at `last_seen+129` would also pass under `cap = UINT64_MAX`),
`test_far_behind_is_never_forward()`, and `od_nonce_never_accepts_consumed()` — a sweep over every
counter ever committed, which is the one assertion in the file that does not restate the code. The
oracle no longer prunes its seen set; that pruning encoded the implementation's forgetting and so
could only ever agree with it.

Builds: `nrf52840custom`, `esp32-c3-N16`, `esp32-N4`.

Still unverified on hardware, unchanged from the list above.
