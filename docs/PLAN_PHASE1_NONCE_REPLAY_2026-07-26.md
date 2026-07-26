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

### Decision A — RESOLVED: forward cap **128**

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
