#ifndef SESSION_GUARD_H
#define SESSION_GUARD_H

#include <stdint.h>

#include "link_owner.h"

// One session teardown routine, shared by every path that ends a session --
// CONNECTION_POLICY R6. The value of a single routine depends entirely on every
// teardown actually reaching it: keeping two paths is how the direct-write
// watchdog once tore down a panel while leaving its pipe session live.
//
// Callers (the complete set as of Phase 2):
//
//   condition                                        dropLink   phase
//   ------------------------------------------------ ---------- -----
//   BLE disconnect serviced, owner's link gone        false      2
//   deep sleep, after linkMarkTerminal()              false      2
//   checkTransferTimeouts() fires                     true       2
//   serviceIdleTimeout() (BLE owner silent)           true       3
//   auth-abuse threshold, after the TX barrier        true       4
//
// NOT a caller: refusing a contender. Refusal disconnects the newcomer and does
// nothing else -- no abort, no cleanup flag, no release. The incumbent's session
// must be untouched by construction rather than by a guard that could be got
// wrong, and refusal/teardown sitting in the same handler is what makes this the
// case most likely to be confused in implementation.
//
// Loop task only, idempotent, and deferred by its callers while a refresh is in
// flight.
void abortToKnownState(const char* reason, bool dropLink, LinkId ownerId);

// `ownerId` is a PARAMETER, not a re-derivation, and this overload is why:
// ordinary callers want "whoever owns the slot right now", so they get a snapshot
// taken before the teardown starts. The terminal caller must NOT use this -- after
// linkMarkTerminal() the owner word reads terminal, so a re-derivation would act
// for the wrong identity; it passes the displaced identity instead.
void abortToKnownState(const char* reason, bool dropLink);

// Request termination of `handle`, then wait -- cooperatively and with a bound --
// until that link is actually down. Returns true if it went down within the bound.
//
// The predicate is the OWNER'S INSTANCE-TABLE ENTRY, not the aggregate connection
// count: R1 permits a refused contender to be transiently attached, so dropping
// the owner takes the count 2->1 and never to 0, and a count-based wait would sit
// out its full bound on a link that is already gone.
//
// Expiry is an early exit, not a failure. The abort runs regardless and the stale
// link is inert by construction: its writes fail the owner filter, its queued
// frames fail the dispatch tag check, and its late disconnect is inert on stale
// epoch. That guarantee is what let R3a drop the cross-pass DROPPING state.
bool bleDropAndWait(uint16_t handle, uint16_t epoch);

#endif  // SESSION_GUARD_H
