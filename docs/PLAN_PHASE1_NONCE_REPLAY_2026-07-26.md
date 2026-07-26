# Phase 1 Implementation Plan — Nonce / Replay Correctness

**Branch:** `debug/ble-hardening` · **Date:** 2026-07-26
**Parent plan:** [`PLAN_FREEZE_PROOFING_2026-07-26.md`](PLAN_FREEZE_PROOFING_2026-07-26.md) § "Phase 1"
**Review that shaped it:** [`FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md`](FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md) `[M1]`

Phase 1 is the root-cause fix and ships first. It is self-contained: it touches only
`encryption.cpp` / `encryption_state.h` (plus one stale comment in `communication.cpp`), has no
dependency on any later phase, and delivers field benefit on its own — unlike Phase 3, which is
dead code until Phase 5/6 call it.

---

## What is actually wrong today

Four distinct defects live in `verifyNonceReplay()` ([encryption.cpp:114-156](../src/encryption.cpp))
and its one caller `decryptCommand()` ([:688-735](../src/encryption.cpp)):

| # | Defect | Evidence | Consequence |
|---|---|---|---|
| **D1** | Nonce rejection is counted as **tamper evidence** | [:691-696](../src/encryption.cpp) — `verifyNonceReplay` false → `integrity_failures++` → 3 ⇒ `clearEncryptionSession()` | **The field freeze.** Packet loss ≠ attack. A lost 32-frame window puts the next frame out of range; the client's `MAX_PTO = 3` retries (`py-opendisplay/src/opendisplay/protocol/commands.py:93`) produce exactly 3 rejections ⇒ session destroyed mid-transfer, everything answers `0xFE`, panel stays powered. |
| **D2** | State is committed **before** the CCM tag is verified | `last_seen_counter` at [:149-151](../src/encryption.cpp), ring write at [:153](../src/encryption.cpp), all *before* `aes_ccm_decrypt` at [:714](../src/encryption.cpp) | An unauthenticated attacker (or corrupt frame) advances the replay state of a live session. Forged counter `last_seen + 32` sticks even though the frame is discarded. |
| **D3** | `counter_diff == 0` is exempted from the replay-set check | [:136](../src/encryption.cpp) `nonce_counter <= last_seen && counter_diff != 0` | **Replay of the highest-seen frame is accepted and re-executed** — the tag is valid because the frame is genuine. Harmless for a pipe DATA frame (duplicate seq is discarded) but not for `CMD_CONFIG_WRITE`, `CMD_POWER_OFF`, or a buzzer/LED command, which is typically what the last frame of a session is. The `!= 0` term exists only so a fresh session's first frame (client counter 0 vs `last_seen_counter` initialised to 0 at [:211](../src/encryption.cpp)) isn't flagged. |
| **D4** | `replay_window_index` is a **function static** | [:152](../src/encryption.cpp) | `clearEncryptionSession()` memsets the ring ([:217](../src/encryption.cpp)) but cannot reset the index, so a new session's first N accepts overwrite an arbitrary rotation of the ring. Live bug today, independent of everything else. |

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
#define OD_NONCE_BACKWARD_BITS 128            // uint64_t[2]
#define OD_NONCE_FORWARD_CAP    64            // see Decision A — derived, not chosen
    uint64_t replay_bitmap[OD_NONCE_BACKWARD_BITS / 64];
```

- **Delete** `uint64_t replay_window[64]` (512 B) and the function-static
  `replay_window_index` ([:152](../src/encryption.cpp)). Net struct change: **−496 B**.
- **No `replay_window_index` field** — a bitmap has no insertion point, so **D4 cannot recur**.
- **No `has_seen_counter` field** — "not seen" is a clear bit, not a reserved value, so **D3
  cannot recur**. A fresh session is `last_seen_counter = 0` with an all-zero bitmap; the
  first frame at counter 0 has `diff == 0`, finds bit 0 clear, and is accepted exactly once.
- `clearEncryptionSession()` ([:201-219](../src/encryption.cpp)) and the fresh-session block in
  `handleAuthenticate` ([:654-660](../src/encryption.cpp)) must **both** zero the bitmap and
  `last_seen_counter`. They are duplicated today; fold them into one `resetNonceState()`
  helper so a third caller cannot drift.

### Step 2 — `encryption.cpp`: `nonceCheck()`, pure
```c
enum NonceResult { NONCE_OK, NONCE_BAD_SESSION, NONCE_OUT_OF_WINDOW, NONCE_REPLAY };
static NonceResult nonceCheck(const uint8_t* nonce, uint64_t* counter_out);
```
- Keep the existing `constantTimeCompare` session-id check ([:122](../src/encryption.cpp)) →
  `NONCE_BAD_SESSION`.
- Then, with `diff = (int64_t)counter - (int64_t)last_seen_counter`:

  | Case | Result |
  |---|---|
  | `diff > OD_NONCE_FORWARD_CAP` | `NONCE_OUT_OF_WINDOW` |
  | `diff > 0` | `NONCE_OK` — ahead of the mark, cannot have been seen |
  | `diff == 0` | bit 0 set ? `NONCE_REPLAY` : `NONCE_OK` ← **no `!= 0` exemption; D3 closed** |
  | `-diff >= OD_NONCE_BACKWARD_BITS` | `NONCE_OUT_OF_WINDOW` |
  | `diff < 0` | bit `-diff` set ? `NONCE_REPLAY` : `NONCE_OK` |

- **No writes to `encryptionSession` on any path.** This is the property Step 5 tests.

### Step 3 — `encryption.cpp`: `nonceCommit(uint64_t counter)`
- **Forward** (`counter > last_seen_counter`): shift the bitmap left by `diff`, clearing the
  vacated low bits; `last_seen_counter = counter`; set bit 0. A `diff >=
  OD_NONCE_BACKWARD_BITS` zeroes the bitmap wholesale — correct, because every counter it
  discarded is now below the backward window and will be rejected on width alone.
- **Backward/equal** (`counter <= last_seen_counter`): set bit `last_seen_counter - counter`.
  `last_seen_counter` does not move.
- Shifting across a `uint64_t[2]` must handle `shift == 0` and `shift >= 64` explicitly —
  `x << 64` is undefined behaviour in C, and it is the classic bug in this pattern. Step 5's
  host test covers `diff` = 0, 1, 63, 64, 65, 127, 128, and `OD_NONCE_FORWARD_CAP`.
- Called from exactly one place: after a successful `aes_ccm_decrypt`.

### Step 4 — `decryptCommand()` rewiring
- Replace the `verifyNonceReplay` block ([:691-698](../src/encryption.cpp)) with the
  `nonceCheck` switch above; **delete** the `integrity_failures++` on that path.
- Insert `nonceCommit()` inside the `if (success)` arm ([:717](../src/encryption.cpp)), before
  `integrity_failures = 0`.
- Leave the tag-failure arm ([:729-733](../src/encryption.cpp)) exactly as-is.
- Logging, per parent plan: `nonce out-of-window (counter=%llu last_seen=%llu diff=%lld) —
  frame dropped, session kept` at WARN vs. `CCM tag failure %u/3` at ERROR. The current
  out-of-window log is `od_log_error` ([:132](../src/encryption.cpp)) and will now fire
  routinely on a lossy link — demote it or it becomes noise that masks real errors.
- **Delete `verifyNonceReplay()`** and both of its declarations (Decision C: the body at
  [:114-156](../src/encryption.cpp), [encryption.h:17](../src/encryption.h),
  [main.h:276](../src/main.h)). The build is the check here — the compiler will name any caller
  we missed.

### Step 5 — Verification
- **Host test** (Decision D): `tools/test_nonce_window.cpp` against `src/nonce_window.h`, run
  under UBSan/ASan. Full case list in Decision D — it covers the window state machine, *not*
  the `integrity_failures` behaviour, which is what hardware tests 1-2 below are for.
- **Build gate:** `pio run -e nrf52840custom -e esp32-s3-N16R8 -e esp32-c3-N16 -e esp32-c6-N4
  -e esp32-N4`. CI builds all 11.
- **Hardware** (py-opendisplay CLI + `tools/od-device-cli.py`):
  1. Forward-gap **within** the cap: skip 50 counters mid-session → next frame accepted,
     transfer continues, session survives (today: session destroyed after 3).
  2. Forward-gap **beyond** the cap: skip 100 (> `OD_NONCE_FORWARD_CAP`) → frames rejected as
     out-of-window, but **`integrity_failures` stays 0 and the session survives**. This is the
     D1 regression test and the one that matters most — a gap this large means the client has
     already aborted, and the device must be sitting clean and ready for its re-auth rather
     than wedged.
  3. True replay of an old counter → rejected, session survives.
  4. **Replay of the last frame of a session** (D3) → now REJECTED. Use a
     non-idempotent command (buzzer) so acceptance is observable.
  5. Forged/corrupt tag ×3 → session still cleared (unchanged behaviour, deliberately).
  6. Regression: full Spectra transfer and an E1004 ~960 KB upload complete untouched.

### Step 6 — Comment hygiene
- [communication.cpp:772-775](../src/communication.cpp) documents that the replay counter
  "already advanced at decrypt time … so drops/dupes never desync it". The invariant still
  holds (commit happens for every frame that *decrypts*, including ones the pipe handler
  discards) but the function name and the ordering claim are now wrong. Rewrite it in the
  same change, and state the derived bound explicitly: worst-case unseen run =
  `PIPE_MAX_W + MAX_PTO` = 35 (19 on `esp32-N4`) < `OD_NONCE_FORWARD_CAP` (64). That sentence
  is the load-bearing justification for the cap — anyone later raising `PIPE_MAX_W` or
  `MAX_PTO` must re-check it, so it belongs next to the code, not only here.

---

## Decisions

**All five are settled.** Nothing blocks implementation.

### Decision A — RESOLVED: forward cap **64**, derived from the client's own credit limit

The parent plan's `OD_NONCE_FORWARD_WINDOW = 4 * PIPE_MAX_W = 128`, and the review's
symmetrized ±128, were both multipliers picked for feel. Neither derived a bound. Here is the
bound.

The nonce gap is **not** the in-flight chunk depth — it is the run of consecutive transmissions
that never reach `decryptCommand`, since the client burns a counter per transmission whether or
not it lands ([device.py:747-760](../../py-opendisplay/src/opendisplay/device.py)). Three
sources feed it: frames lost on air; frames dropped at the command ring
([esp32_ble_callbacks.h:126-127](../src/esp32_ble_callbacks.h), which runs on the NimBLE host
task *before* decrypt); and PTO probes.

All three are capped by the client's credit limit. After the window is exhausted it **blocks
for an ACK** ([device.py:2705-2730](../../py-opendisplay/src/opendisplay/device.py)); on
timeout it resends exactly **one** chunk (`_send(window_base)`), never another window, and
`MAX_PTO = 3` then aborts the transfer. So:

```
worst-case unseen run = PIPE_MAX_W + MAX_PTO = 32 + 3 = 35     (19 on esp32-N4, W=16)
```

This is a **hard bound, not an estimate**: beyond it the transfer has aborted and the next
frame the device sees belongs to a new session at counter 0. Any received frame both advances
`last_seen_counter` and resets `pto_count`, so the gap collapses on any progress.

**`OD_NONCE_FORWARD_CAP = 64`** — ~1.8× the bound, covering both `PIPE_MAX_W` variants with one
constant.

**Why not more.** Forward width is not free margin: it *is* the magnitude of the `[M1]`
jam-forward DoS. One replayed genuine frame drags `last_seen_counter` forward by up to the cap
and strands every legitimate frame below the new window until the Phase 6 supervisor fires.
128 doubles that damage to buy headroom over a bound that is already provably 35. **Tight is
correct here.**

**Backward width is a separate constant and needs no such argument** — see Decision B. Under a
bitmap the two sides are fully decoupled: forward acceptance stores nothing, so a forward cap
wider than the backward window is harmless (the shift simply clears history that is now
out-of-window). The `[M1]` rule *"never forward-wider than the set can police"* was an artifact
of the value-ring representation and does not apply.

### Decision B — RESOLVED: sliding bitmap, **IPsec/DTLS shifting style**, `uint64_t[2]` = 16 B

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

| Backward window | Ring | Bitmap |
|---|---|---|
| 32 (today) | 512 B | 8 B |
| 127 (**chosen**) | 4,064 B | **16 B** |
| 255 | 8,160 B | 32 B |

`OD_NONCE_BACKWARD_BITS = 128` (`uint64_t[2]`, backward window 127) is generous for a
tolerance that is **never exercised in normal operation** — the client's counters are strictly
increasing and both transports preserve ordering — but at 16 B there is no reason to economize.
Net struct change is **−496 B** against today, versus **+1,536 B** for the ring plan. The
`esp32-N4` link headroom I measured for the ring (81,940 → 83,476 B of 327,680, SUCCESS both
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
- **Shift edges:** `diff` = 0, 1, 63, 64, 65, 127, 128, `OD_NONCE_FORWARD_CAP`, and
  `> OD_NONCE_FORWARD_CAP`.
- **Purity of `nonceCheck` (D2):** snapshot the state, call `nonceCheck` on every result class,
  `memcmp` the state afterwards. This is the single most valuable assertion in the file — it is
  the property that "the tag is the only thing that may advance replay state" rests on.
- **D3:** commit a counter, re-present the same counter → `NONCE_REPLAY`. Repeat at `diff == 0`
  specifically, which is the exempted case today.
- **Fresh session:** `last_seen = 0`, empty bitmap → counter 0 accepted exactly once, rejected
  on re-presentation. No `has_seen_counter` involved.
- **Wholesale slide:** forward jump ≥ `OD_NONCE_BACKWARD_BITS` → bitmap cleared; previously
  seen counters now return `OUT_OF_WINDOW`, **not** `REPLAY` (both reject, but conflating them
  would hide a genuine slide bug).
- **Differential/property test:** run a few thousand pseudo-random accept/replay/gap sequences
  against a naive `std::set` oracle that models "seen, within window". Cheap, and it covers
  the interleavings hand-written cases miss.
- **Counter arithmetic:** compute the delta as `(int64_t)(counter - last_seen)` rather than
  subtracting two casted `int64_t`s, and assert sane behaviour near `UINT64_MAX`. Unreachable
  in practice (2^63 frames) but free to get right.

**Not covered here, deliberately:** that a nonce failure leaves `integrity_failures` untouched
(D1) lives in `decryptCommand`, not in the window logic. That assertion belongs to Step 5's
hardware tests 1-2.

**CI:** add the two lines above as a step in `.github/workflows/main.yaml`. It runs in seconds,
needs no toolchain beyond the runner's stock `g++`, and gates every push alongside the 11
firmware builds.

### Decision E — RESOLVED: no wire change. Recorded for the future, **not actioned**

`decryptCommand` returning false yields an unencrypted `RESP_NACK`
([communication.cpp:700-701](../src/communication.cpp)) for both "lost your window" and
"tag failed". Phase 1 leaves that exactly as it is.

**Why it is acceptable to leave:** the whole point of Phase 1 is that a client which blindly
retransmits no longer wedges the device. With the session surviving and the forward cap set
above the client's real loss budget (Decision A), the client does not *need* to distinguish the
two cases — its existing retransmit/PTO machinery recovers on its own.

**Recorded for a future protocol revision** (do not implement in Phase 1, and do not let a
reviewer re-open it here): a distinct response code for "nonce out of window" would let a
client re-sync deliberately — abandon the in-flight window and re-authenticate — instead of
burning its `MAX_PTO` budget discovering the same thing by timeout. That is a strictly better
recovery, and worth doing *if* the wire is being revised for other reasons. It is not worth
doing on its own: it is a cross-repo change through `../opendisplay-protocol`, a `--push` to
all four firmware repos, and a coordinated py-opendisplay release, to save a few seconds on a
path Phase 1 already makes non-fatal.

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

## Files touched

| File | Change |
|---|---|
| `src/nonce_window.h` | **new** — dependency-free window state machine (Decision D) |
| `src/encryption_state.h` | ring → bitmap, **−496 B** |
| `src/encryption.cpp` | the substance: `nonceCheck`/`nonceCommit`, `decryptCommand` rewiring, `verifyNonceReplay` deleted |
| `src/encryption.h` | declaration removal (Decision C) |
| `src/main.h` | duplicate declaration removal (Decision C) |
| `src/communication.cpp` | comment only ([:772-775](../src/communication.cpp)) |
| `tools/test_nonce_window.cpp` | **new** — host test (Decision D) |
| `.github/workflows/main.yaml` | one step: compile + run the host test |

**No `platformio.ini` change**: the bitmap is smaller than what it replaces, so the `esp32-N4`
link headroom that gated the original proposal is not a consideration on any target.
**No protocol-header change** (Decision E). **No client-side change.**
