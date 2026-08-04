# Findings — moving the mbedTLS record buffers to PSRAM

Investigated 2026-08-04. Every `file:line` below was read and re-verified against `main` HEAD
`976f0c2`. Platform claims were checked against the pinned pioarduino S3 SDK
(`~/.platformio/packages/framework-arduinoespressif32-libs/esp32s3/`), Arduino 3.3.9 / IDF 5.5.4.
A second opinion on IDF/mbedTLS internals was taken from an external model; each of its claims is
marked below as verified or unverified.

## Question

`od_tls_reserve_records()` (`src/wifi_service.cpp:226-251`) reserves `OD_TLS_RECORD_SLOTS` (2) x
`OD_TLS_RECORD_SLOT_SIZE` (17,408 B) = **34,816 B** at boot with
`MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT` (`src/wifi_service.cpp:183-184, 235-236`), then installs a
global allocator hook, `mbedtls_platform_set_calloc_free(od_tls_calloc, od_tls_free)`. This is the
largest single app-owned block of internal DRAM, and internal DRAM — not flash, not PSRAM — is what
runs out on these boards.

Can the two slots move to `MALLOC_CAP_SPIRAM`?

## Verdict

**Yes, safe with caveats — on S3, which is the only target that compiles this code.** The one
concern that would have been a hard blocker (hardware crypto DMA cannot reach external PSRAM) does
**not** apply to ESP32-S3. The real caveats are a residual internal-DRAM dependency during AES, a
pre-existing thread-safety gap in the slot pool, and a plaintext-exposure trade-off.

## Verified against the pinned SDK

Hardware crypto is enabled and *is* DMA-driven, so this was the claim worth checking:

| Claim | Result |
|---|---|
| `CONFIG_MBEDTLS_HARDWARE_AES` / `_SHA` / `_MPI` | ✅ all three `=y` |
| `CONFIG_MBEDTLS_AES_USE_INTERRUPT` | ✅ `=y` |
| `SOC_PSRAM_DMA_CAPABLE` | ✅ `1` |
| `SOC_AES_GDMA`, `SOC_SHA_SUPPORT_DMA` | ✅ `(1)` |

S3 crypto DMA can therefore address external PSRAM. The IDF 5.5.4 AES/SHA ports gate on
`esp_ptr_dma_ext_capable()` and perform their own cache maintenance, so no manual
`esp_cache_msync()` belongs around mbedTLS calls.

**Scope is narrower than it looks, which removes a caveat.** TLS compiles only under
`OPENDISPLAY_HAS_WIFI` (= `TARGET_ESP32 && OPENDISPLAY_ENABLE_WIFI`, `src/wifi_service.h:32`), and
`-DOPENDISPLAY_ENABLE_WIFI` is set on exactly five envs, **all S3** — `esp32-s3-N16R8`,
`-N8R8`, `-N32R8`, `-N32R8-extuart`, `-N16R8-extuart` (`platformio.ini:173, 203, 232, 261, 377`).
The C3/C6 envs carry the flag *commented out*, "deliberately absent: no PSRAM here"
(`platformio.ini:310, 331, 352`). So the general warning against classic ESP32/C3/C6 — where
`SOC_PSRAM_DMA_CAPABLE` does not hold — cannot bite here.

> Method note: a raw `grep OPENDISPLAY_ENABLE_WIFI platformio.ini` piped through an
> env-attributing `awk` misreports C3/C6 as WiFi-enabled, because it matches the commented-out
> lines. Attribute env flags by resolving them, not by proximity.

## Caveats

**1. It does not fully decouple TLS from the internal heap.** mbedTLS hands AES *interior* pointers
into `in_buf`/`out_buf`, so aligning the 17 KB allocation does not make every crypto pointer
cacheline-aligned. Unaligned external output buffers are processed through a small temporary
internal DMA-capable buffer in chunks, so some internal DRAM is still required during AES, and AES
can still fail — cleanly — if that small allocation fails. **Unverified:** the external opinion put
the chunk ceiling at 1,600 B. Arduino ships prebuilt libs (headers only, no `.c`), so the constant
could not be confirmed locally. The *mechanism* is sound; treat the number as unconfirmed.

**2. The slot pool is not thread-safe, and the hook is global.** `s_tlsSlotBusy[]`
(`src/wifi_service.cpp:188`, set at `:196` and `:237`) is plain `bool` state with no critical
section. `mbedtls_platform_set_calloc_free()` redirects **every** mbedTLS allocation in the
firmware, including the CCM/CMAC paths in `encryption.cpp` — not just the LAN TLS server. "TLS runs
on the loop task" justifies the current code only while nothing else calls mbedTLS off-task. Any
allocation up to `OD_TLS_RECORD_SLOT_SIZE` can also consume a slot (`src/wifi_service.cpp:195`),
since the allocator cannot tell a record buffer from anything else. **This is pre-existing and
independent of PSRAM** — relocating the pool neither creates nor worsens it — but it should be
fixed first, so that a concurrency bug and a placement bug cannot be confused if something
regresses. `od_tls_calloc` should also reject `n * size` overflow, preserving `calloc` semantics.

**3. Plaintext crosses the external bus.** Record buffers hold plaintext after decryption and
before encryption. Unless PSRAM encryption is enabled, moving them exposes that traffic on the
external memory interface. A threat-model decision rather than a technical blocker, and likely
noise for a LAN display tag — but it should be a decision, not a side effect.

**4. Define the no-PSRAM behaviour explicitly.** A `MALLOC_CAP_SPIRAM` request returns `nullptr`
when PSRAM is absent or dead. Either refuse TLS with a log line, or fall back to internal slots
knowing that restores the original pressure — but do not install a pool holding null slots and then
describe TLS as PSRAM-backed. `odLanReserveRxBuffer()` (`src/wifi_service.cpp:253-278`) already
models the PSRAM-then-internal fallback worth copying, including its "no PSRAM on this board?"
warning.

> **Discrepancy found, unrelated to this change:** `src/wifi_service.h:61` and
> `src/wifi_service.cpp:275` both assert that `CONFIG_SPIRAM_IGNORE_NOTFOUND=1` makes a dead-PSRAM
> boot silent. That symbol is **not present in the pinned S3 sdkconfig**. Either it is set
> somewhere not found, or both comments are stale. It is load-bearing for the silent-fallback
> rationale, so it is worth resolving on its own.

## What to keep internal regardless

Slot pointers, busy state and any lock; `mbedtls_ssl_context`, config, entropy and DRBG state; keys
and PSK/session state; AES/SHA/MPI/ECC contexts; all small mbedTLS allocations (the existing
fallback-to-heap for anything that does not fit a slot is correct); crypto DMA descriptors and ISR
state, which IDF already places internally. Do not call mbedTLS, the BIO callbacks, or the
allocator hook from IRAM/cache-disabled context, and avoid custom cache-disabling flash operations
concurrently with TLS traffic.

## Recommendation

Sequence, least risk first:

1. **`pipeReorder` → PSRAM** (tracked separately): `PIPE_REORDER_SLOTS` (33) x `sizeof
   (PipeReorderSlot)` (252 B) = **8,316 B** — `src/structs.h:51, 92-97`,
   `src/display_service.cpp:678`. A file static in one `.cpp`, touched only on the loss-recovery
   path, so PSRAM latency never lands on the in-order streaming path. Contained; do it first.
2. **Fix the slot-pool locking and the overflow check** as its own commit. No behaviour change.
3. **Relocate the two record slots** to `MALLOC_CAP_SPIRAM`, with an explicit no-PSRAM policy.

Step 3 is the real win — 34,816 B, four times the reorder buffer, and the block implicated by the
DRAM-exhaustion coredump — but it carries the least risk when it lands last.
