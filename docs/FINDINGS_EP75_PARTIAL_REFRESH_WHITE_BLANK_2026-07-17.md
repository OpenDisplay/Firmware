# Findings — EP75 (7.5" 800×480) partial refresh blanks the surround to white

**Repo:** `Firmware` (branch `debug/v2.2-audit`)
**Date:** 2026-07-17
**Panel:** EP75 7.5" 800×480 **mono B/W** — UC8179-class controller (`BBEP_CHIP_UC81xx`). Reproducing SKU is almost certainly `EP75_800x480_GEN2` (config id `0x003B`), **not** plain `EP75_800x480` (`0x0014`) — see [§6](#6-which-ep75-sku-are-you-on).
**Symptom (confirmed on hardware):** sending a PARTIAL RECTANGLE update repaints the entire display **outside** the rectangle to **white**. Only the rectangle should change.
**Status:** Root cause confirmed at register level. **No code changed** — this is an investigation/decision doc. Fix options in [§5](#5-fix-options).
**Method:** Read of `src/display_service.cpp` partial paths + the vendored `bb_epaper` library under `.pio/libdeps/nrf52840custom/bb_epaper/src/` (`bb_ep.inl`, `bb_epaper.h`). Every claim cites `file:line`.

---

## 1. TL;DR

On this panel, the firmware's "partial refresh" is not a differential, region-limited partial — it is a **whole-panel drive-to-target** refresh:

- `bbepRefresh(&bbep, REFRESH_PARTIAL)` for the GEN2 EP75 either uses the OTP **full**-refresh LUT (when `pInitPart == NULL`) or a **non-hold** "partial" LUT (GEN2), both of which drive **every** pixel to the value in the NEW/DTM2 bank.
- The DRF (display refresh) is **never region-limited**: bb_epaper programs the UC8179 partial window (`PTL 0x90`) with "refresh whole screen" and issues `PTOU` (partial-out) *before* `DRF`, so the panel scans the whole gate/source range.
- The firmware white-fills **both** RAM banks outside the rectangle (`partial_prepare_panel_ram`), so the NEW/DTM2 bank outside the rect is **white**.

Whole-panel drive-to-target × white NEW bank outside the rect ⇒ **the entire surround is actively driven white.** That is the symptom.

The white-fill is only *exposed* here; it is harmless on a panel with a genuine differential-hold LUT (only `EP75_800x480` 0x0014 qualifies). The prior belief that "the generic `bbepRefresh` partial path is a true differential and is unaffected" is **false for this hardware**.

---

## 2. Controller and plane wiring (this part is correct)

**Controller family.** The EP75 panelDefs are `BBEP_CHIP_UC81xx` (UC8179-class) — `bb_ep.inl:3762-3766, 3780-3781, 3785, 3812`. UC8179 command set (`bb_epaper.h:413-447`): `PSR=0x00`, `PWR=0x01`, `PON=0x04`, **`DTM1=0x10` (old bank)**, `DRF=0x12`, **`DTM2=0x13` (new bank)**, LUT registers `VCOM=0x20 / WW=0x21 / BW=0x22 / WB=0x23 / BB=0x24 / VCOM2=0x25`, `CDI=0x50` (border), `TCON=0x60`, `TRES=0x61`, **`PTL=0x90` (partial window)**, **`PTIN=0x91` (partial-in)**, **`PTOU=0x92` (partial-out)**.

**Plane→command mapping.** For 1bpp EP75, `bbepStartWrite` / `bbepFill` map `PLANE_1 → DTM1 (0x10, OLD)` and `PLANE_0 → DTM2 (0x13, NEW)` — `bb_ep.inl:4145-4156, 4333-4360`. The firmware streams `PLANE_1` then `PLANE_0` into the rect window (`src/display_service.cpp:2808-2829`). So the two-bank/old-new wiring is right; the defect is entirely in the refresh waveform + window handling.

---

## 3. Root cause (register-level)

### 3.1 The refresh drives the whole panel to the NEW bank (no differential hold)

`bbepRefresh(&bbep, REFRESH_PARTIAL)` — `bb_ep.inl:4390-4416`:

- **`pInitPart == NULL` variants silently degrade to full/fast.** For `EP75_800x480_4GRAY` (0x0015), `_4GRAY_V2` (0x0016), `_4GRAY_GEN2` (0x003C), `EP73_800x480` (0x0022), `EP73_SPECTRA` (0x0023), `EP75R_800x480` (0x0026), `EP75YR` (0x0040), the panelDef `pInitPart` field is `NULL` (`bb_ep.inl:3764-3766, 3780-3781, 3785, 3812`). `bb_ep.inl:4391-4396` then falls back to `pInitFast` / `pInitFull` — the **OTP full-refresh LUT**, which drives every pixel to its NEW (DTM2) value.
- **`EP75_800x480_GEN2` (0x003B) has a `pInitPart`, but it is a drive-to-target LUT, not a hold LUT.** `epd75_init_partial_gen2` — `bb_ep.inl:859-890` — has `WW=0x80`, `BW=0x80`, `WB=0x40`, `BB=0x40`; **every** source/target combination applies drive voltage. It also begins with `EPD_RESET` (`bb_ep.inl:838`) → `bbepWakeUp` (`bb_ep.inl:4216-4217`), a hardware controller reset.

Contrast — a genuine differential-hold LUT (only plain `EP75_800x480` 0x0014): `epd75_init_sequence_partial` — `bb_ep.inl:702-762`, with `WW=0x00` and `BB=0x00` (unchanged pixels get **no** movement, `bb_ep.inl:720-751`) and `PSR=0x3f` (LUT-from-register). There, unchanged pixels physically hold and the white-fill is harmless.

### 3.2 The DRF is never region-limited

Even though UC8179 *can* limit the refresh to a window, bb_epaper defeats it:

- `bbepSetAddrWindow` emits `PTIN (0x91)` + `PTL (0x90)` but sets `PTL`'s last parameter byte to `1` = **"refresh whole screen"** — `bb_ep.inl:4054`.
- `bbepRefresh` sends `PTOU (0x92, partial-out)` **immediately before** `DRF (0x12)` — `bb_ep.inl:4414-4415` — with the literal source comment "update the entire panel, not just the last memory window."

So the gate/source scan covers the **whole panel** regardless of the RAM address window. (On SSD16xx this region-limiting is impossible; on UC8179 it *is* possible but is being switched off here.)

### 3.3 The firmware supplies the white

`partial_prepare_panel_ram` (`src/display_service.cpp:2848-2872`) white-fills both banks over the full panel at partial START (`bbepFill(&bbep, BBEP_WHITE, PLANE_1)` then `PLANE_0`, `:2866-2867`), skipped only for a full-frame rect. The subsequent stream overwrites only the rect window. So outside the rect, DTM2/NEW = white.

### 3.4 Composition

whole-panel scan (3.2) × drive-to-target LUT (3.1) × white NEW bank outside the rect (3.3) ⇒ **every pixel outside the rect is driven to white.** Inside the rect the client's real content lands correctly; outside, the panel is repainted white. Exactly the observed symptom.

---

## 4. Why the two earlier conclusions were wrong

1. **"The white-fill is correct because OLD==NEW==white ⇒ differential drives nothing."** This assumes a differential-hold waveform is active. On GEN2 EP75 it is not (3.1) — the LUT drives to target — so equal banks do **not** produce a hold; they produce a drive to the (white) target.
2. **"The generic `bbepRefresh(REFRESH_PARTIAL)` path is a true differential and is unaffected (only EP397/EP426 are broken)."** The generic UC81xx path is differential **only** when `pInitPart` is a hold-LUT, which across the entire EP75 family is true for **exactly one SKU** (`EP75_800x480` 0x0014). Every other EP75 variant is broken the way this doc describes. The EP397/EP426 blank documented in `FIRMWARE_NIMBLE_PORT_CODE_REVIEW`-adjacent notes has a *different* root cause (SSD16xx DISP_CTRL1 `0x21` left in OLD-plane-bypass); this EP75 case is a distinct bug in a distinct controller family.

### Related: the partial gate is too permissive

`handlePartialWriteStart` only rejects `getBitsPerPixel() != 1` (`src/display_service.cpp:1864-1870`); it does not check whether the panel actually has a differential-hold partial LUT. So GEN2 mono (1bpp, no hold LUT) slips through and reaches the broken refresh.

---

## 5. Fix options

UC8179 **can** do a real region-limited partial, so unlike the SSD16xx panels a proper fix exists — bb_epaper's refresh/window helpers just have to be bypassed.

### Option A — Real region-scan partial (correct, higher effort, needs hardware validation)

After streaming `DTM1 (old)` + `DTM2 (new)` for the rect, drive a **windowed** refresh with raw commands instead of `bbepRefresh`:

```
PTIN (0x91)                              // partial-in
PTL  (0x90)  x_start,x_end, y_start,y_end, 0x00   // last byte = 0x00 → scan WINDOW ONLY (not 0x01)
DRF  (0x12)                              // display refresh; then wait BUSY
PTOU (0x92)                              // partial-out
```

Key deltas vs current behavior:
- Do **not** call `bbepRefresh` (it sends `PTOU` before `DRF`, `bb_ep.inl:4414-4415`).
- Do **not** use `bbepSetAddrWindow`'s `PTL` last-byte=1 (`bb_ep.inl:4054`); use `0x00` so only the window is scanned.
- With window-scan, pixels outside the rect receive **no voltage** and physically hold — so this works even with the OTP full LUT, and the `bbepFill` white-fill at `display_service.cpp:2866-2867` becomes unnecessary and should be **removed** (it is only needed to make the full-panel drive "safe," which we are eliminating).

Caveats: requires per-SKU validation that the panel honors `PT_SCAN=0`; the in-window waveform may still flash (drive-to-target within the rect) unless a differential LUT is also loaded — acceptable, since the reported bug is the *surround*, not in-rect flashing. This touches flashable panel-drive code and must be eyeballed on hardware.

### Option B — Reject partial for non-differential EP75 variants (safe, minimal)

Mirror the existing Seeed / `getBitsPerPixel()!=1` guard in `handlePartialWriteStart`: reject partial (e.g. NACK `ERR_PARTIAL_UNSUPPORTED`) for `bbep.type` in `{EP75_800x480_GEN2, EP75_800x480_4GRAY, EP75_800x480_4GRAY_V2, EP75_800x480_4GRAY_GEN2, EP73_800x480, EP73_SPECTRA_800x480, EP75R_800x480, EP75YR_800x480}`, allowing partial only for panels with a differential-hold `pInitPart` (currently just `EP75_800x480` 0x0014). The client then falls back to a full-frame update. Low risk, no glitch, but loses the partial-update speed/flicker benefit.

### Option C — Sequence: B now, A later

Land the safe reject/fallback first so the display is correct immediately, then implement and validate the raw UC8179 region-scan (Option A) as a follow-up when hardware test time is available.

---

## 6. Which EP75 SKU are you on?

Plain `EP75_800x480` (`0x0014`) has a true differential-hold LUT (`bb_ep.inl:702-762`) and is **immune** to this bug. Because the symptom reproduces on your "mono B/W" panel, the configured `panel_ic_type` is almost certainly **`0x003B` (`EP75_800x480_GEN2`)**, whose GEN2 partial LUT drives-to-target (`bb_ep.inl:859-890`). Worth confirming the exact `globalConfig.displays[0].panel_ic_type` on the device (mapping table at `src/display_service.cpp:433-473`) before applying a SKU-scoped fix — if it *were* `0x0014` the bug would point somewhere else entirely.

---

## 7. Does bb_epaper support partial on EP75 at all?

Effectively **no**, except plain `EP75_800x480` (0x0014). Every other EP75/EP73 variant either has `pInitPart == NULL` (silently full/fast refresh) or a drive-to-target "partial" LUT (GEN2). So region/differential partial is unsupported across the family bar that one SKU; the firmware must either implement raw UC8179 region-scan (Option A) or reject partial for the rest (Option B).

---

## 8. Key references

Firmware — `src/display_service.cpp`:
- `partial_trigger_refresh` — `:2831-2846` (EP75 falls through to `bbepRefresh` at `:2844`)
- `partial_prepare_panel_ram` (white-fill) — `:2848-2872` (fills at `:2866-2867`)
- `partial_write_stream_bytes` (PLANE_1 then PLANE_0) — `:2808-2829`
- `panel_skips_reinit_on_partial_refresh` predicate (EP397/EP426 only) — `:2609-2626`
- `handlePartialWriteStart` bpp-only gate — `:1864-1870`
- panel id → type map — `:433-473`

bb_epaper — `.pio/libdeps/nrf52840custom/bb_epaper/src/`:
- `bbepRefresh` (pInitPart fallback + PTOU-before-DRF) — `bb_ep.inl:4390-4416` (fallback `:4391-4396`, `PTOU`/`DRF` `:4414-4415`)
- `bbepSetAddrWindow` (PTL last byte = 1) — `bb_ep.inl:4054`
- `bbepStartWrite` / `bbepFill` plane→command mapping — `bb_ep.inl:4145-4156, 4333-4360`
- EP75 0x0014 differential-hold partial LUT — `bb_ep.inl:702-762`
- EP75 GEN2 (0x003B) drive-to-target partial LUT + `EPD_RESET` — `bb_ep.inl:838, 859-890`
- EP75 panelDefs (`pInitPart == NULL` variants) — `bb_ep.inl:3762-3766, 3780-3781, 3785, 3812`
- UC8179 command constants — `bb_epaper.h:413-447`
