# IT8951 / ED103TC2 → bb_epaper Integration Plan

Decision-support document. **Report only** — no code was modified to produce it.

Scope: fold the Seeed_GFX-based IT8951 TCON path (10.3" **ED103TC2 1872×1404**, panel_ic
`OD_PANEL_IC_ED103TC2_1872X1404` = 3000 / `..._4GRAY` = 3001) into the primary **bb_epaper**
driver, so the vendored Seeed_GFX / TFT_eSPI fork can be dropped from the ESP32-S3 build.

All line numbers are as of this investigation:
- bb_epaper checkout: `/.pio/libdeps/esp32-s3-N16R8/bb_epaper/` (identical across esp32-s3 envs)
- Seeed_GFX: `/home/davelee/opendisplay/Firmware/lib/Seeed_GFX/`
- Firmware glue: `/home/davelee/opendisplay/Firmware/src/`

---

## A. Executive summary

**Feasible, and a natural fit — recommended.** OpenDisplay uses Seeed_GFX as nothing more than
(a) an IT8951 SPI transport and (b) a raw framebuffer it `memcpy`s pre-dithered pixels into
(`src/display_seeed_gfx.cpp`). It touches **zero** TFT_eSPI drawing/text/sprite/font/touch code.
The IT8951 command surface OpenDisplay actually exercises is tiny: full-frame packed-pixel load +
one `DPY_AREA` GC16 refresh + init(VCOM)/sleep/wake — a subset of the ~30 `tcon*` functions in
`Tcon.cpp`. bb_epaper already carries bitbank2's own dormant IT8951 scaffolding (chip enum,
register `#define`s, SPI primitives, an init table, a commented panel row, an `#ifdef FUTURE`
dispatch stub — all from commit `631e3c0`), so completing it is a clean upstreamable PR rather than
a fork. **Rough effort:** ~250–350 lines of new/ported C in bb_epaper across ~6 functions + 2
panel-table rows, plus deleting/collapsing the ~15 `#ifdef OPENDISPLAY_SEEED_GFX` sites in
`display_service.cpp` and retiring `display_seeed_gfx.{cpp,h}` down to a thin bb_epaper shim.
**Biggest risks:** (1) the existing bb_epaper IT8951 SPI stubs do **not** poll the **HRDY**
handshake and have **no read path** at all — both mandatory for real silicon; (2) per-unit **VCOM**
calibration (Seeed hardcodes `1400`/−1.40 V, bb_epaper's M5Paper stub uses `2300`/−2.30 V); (3)
packed-pixel **nibble/word order + X-mirroring** must be reproduced byte-for-byte. All three are
containable but demand a **physical ED103TC2 panel** to validate — there is no way to prove
correctness from source alone.

---

## B. Seeed_GFX interface surface — full catalog (e-paper-relevant)

### B.1 `EPaper` class — `lib/Seeed_GFX/Extensions/EPaper.{h,cpp}`
`EPaper : public TFT_eSprite` (owns a 1-bpp/4-bpp sprite framebuffer `_img8`).

| Method | File:line | Purpose |
|---|---|---|
| `EPaper()` ctor | EPaper.cpp:1 | `setColorDepth(1)`, `createSprite(w,h,1)` — allocates `_img8` |
| `begin(uint8_t wake=0)` | EPaper.cpp:8 | `init()` (RST toggle + `hostTconInit`) then `EPD_WAKEUP()`; `wake!=0` → `initFromSleep()` |
| `update()` | EPaper.cpp:41 | Full-frame push+refresh: `wake→SET_WINDOW→PUSH_NEW_COLORS→UPDATE→sleep` (1bpp or gray branch) |
| `update(x,y,w,h,data)` | EPaper.cpp:159 | Sub-region push via `pushImage` — **unused by OpenDisplay** |
| `updataPartial(x,y,w,h)` | EPaper.cpp:71 | Aligned partial window (16-px), `tconDisplayArea1bpp` — **unused** |
| `initGrayMode(uint8_t)` | EPaper.cpp:186 | Switch `_img8` to 4-bpp (grayLevel 16); recreates sprite |
| `deinitGrayMode()` | EPaper.cpp:208 | Switch `_img8` back to 1-bpp |
| `sleep()` | EPaper.cpp:224 | `EPD_SLEEP()` → `tconSleep()` (guarded by `_sleep`) |
| `wake()` | EPaper.cpp:232 | `EPD_SET_TEMP` + `EPD_WAKEUP`/`_GRAY` → `tconWake()` (guarded) |
| `drawBufferPixel / setTemp / getTemp / setHumi / getHumi` | EPaper.cpp:36,250-266 | **unused by OpenDisplay** |
| `getPointer()` (inherited `TFT_eSprite`) | Sprite.cpp:118 | Returns `_img8` raw framebuffer pointer |

### B.2 `EPD_*` macros — the EPaper→Tcon glue — `TFT_Drivers/ED103TC2_Defines.h`
These are the actual bridge that turns `EPaper` calls into `tcon*` commands:

| Macro | File:line | Expands to |
|---|---|---|
| `EPD_SET_WINDOW(x1,y1,x2,y2)` | :134 | `setTconWindowsData(x1,y1,x2,y2)` |
| `EPD_PUSH_NEW_COLORS(w,h,c)` | :145 | `tconLoad1bppImage(c,…,w,h,false)` |
| `EPD_PUSH_NEW_GRAY_COLORS(w,h,c)` | :156 | `tconLoadImage(c,…,w,h,false)` (4bpp) |
| `EPD_PUSH_OLD_COLORS(...)` | :173 | **no-op** |
| `EPD_UPDATE()` | :70 | `tconDisplayArea1bpp(…,0x02,0x00,0xff)` — GC16, BG=0 FG=255 |
| `EPD_UPDATE_GRAY()` | :77 | `tconDisplayArea(…,0x02)` — GC16 |
| `EPD_UPDATE_PARTIAL()` | :64 | `tconDisplayArea1bpp(…,0x01,…)` — DU — **unused** |
| `EPD_WAKEUP()` | :118 | `tconWake()` + `setTconTemp` |
| `EPD_SLEEP()` | :84 | `tconSleep()` |
| `EPD_SET_TEMP(t)` | :178 | `setTconTemp(t)` |
| `EPD_INIT()` / `OD_EPD_RST_TOGGLE()` | :92-116 | RST pulse (runtime pins) |
| init body `ED103TC2_Init.h` | :46-47 | RST pulse + `hostTconInit()` |
| wake body `ED103TC2_Init_Wake.h` | :33 | `hostTconInitFast()` |

### B.3 `Tcon` methods (added to `TFT_eSPI`) — `Extensions/Tcon.{h,cpp}`
| Method | Tcon.cpp:line | IT8951 command(s) issued |
|---|---|---|
| `tconWaitForReady()` | :22 | Poll HRDY (busy pin) until HIGH, w/ timeout |
| `tconSendWord / tconReceiveWord` | :48,53 | `spi.transfer16` (word granular) |
| `tconWriteCmdCode(cmd)` | :60 | preamble `0x6000` + cmd word (HRDY-gated) |
| `tconWirteData(d)` | :82 | preamble `0x0000` + data word |
| `tconWirteNData(buf,n)` | :100 | preamble `0x0000` + burst via `pushPixels[DMA]` (16 KB chunks) |
| `tconReadData()` | :144 | preamble `0x1000` + dummy + read word |
| `tconReadNData(buf,n)` | :164 | preamble `0x1000` + dummy + n-word burst read |
| `tconSendCmdArg(cmd,args,n)` | :188 | cmd + n data words |
| `tconReadReg / tconWriteReg` | :201,213 | `REG_RD`(0x10)/`REG_WR`(0x11) + addr(+val) |
| `tconLoadImgStart / …AreaStart / …End` | :223,245,261 | `LD_IMG`(0x20)/`LD_IMG_AREA`(0x21)/`LD_IMG_END`(0x22) |
| `tconSetImgBufBaseAddr(addr)` | :266 | write `LISAR`+2 / `LISAR` |
| `tconSetImgRotation(r)` | :237 | `LD_IMG` w/ rotation — **unused** |
| `tconHostAreaPackedPixelWrite(ld,area)` | :276 | set base addr → `LD_IMG_AREA` → mirror/pack rows → burst → `LD_IMG_END` |
| `tconDisplayArea(x,y,w,h,mode)` | :331 | `DPY_AREA`(0x34) + 5 args |
| `tconDisplayArea1bpp(...,bg,fg)` | :346 | X-mirror; set `UP1SR+2` bit2; set `BGVR`; `DPY_AREA`; wait; restore |
| `tconLoad1bppImage(buf,x,y,w,h,flip)` | :369 | X-mirror; load as 8bpp w/ width/8 → `tconHostAreaPackedPixelWrite` |
| `tconLoadImage(buf,x,y,w,h,flip)` | :393 | 4bpp → `tconHostAreaPackedPixelWrite` |
| `getTconInfo(buf)` | :416 | `GET_DEV_INFO`(0x0302) burst read → `I80TCONDevInfo` |
| `hostTconInit()` | :437 | `setTconVcom(1400)` → `getTconInfo` → enable `I80CPCR` packed mode |
| `hostTconInitFast()` | :453 | `getTconInfo` only (no VCOM, no I80CPCR) |
| `setTconWindowsData(x1,y1,x2,y2)` | :466 | store `_imgAreaInfo` (no bus traffic) |
| `getTconTemp / setTconTemp` | :474,482 | cmd `0x0040` |
| `getTconVcom / setTconVcom` | :491,498 | cmd `0x0039` (arg 0x02 = write) |
| `tconSleep / tconWake / tconStandby` | :506,511,516 | `SLEEP`(0x03)/`SYS_RUN`(0x01)/`STANDBY`(0x02) |
| `tconWaitForDisplayReady()` | :521 | poll `LUTAFSR` reg until 0 |

Everything else in `lib/Seeed_GFX/` (`Sprite.cpp` 82 KB, `Smooth_font.cpp`, `Button.cpp`,
`Touch.cpp`, all `TFT_Drivers/*` for TFT LCDs, the whole `TFT_eSPI` core) is TFT-LCD/graphics
machinery **entirely unused** by OpenDisplay except that `EPaper` inherits `TFT_eSprite` only to
get a malloc'd framebuffer + `getPointer()`.

---

## C. Portion USED by OpenDisplay Firmware ("must move to bb_epaper")

Only `src/display_seeed_gfx.cpp` links to Seeed_GFX. Its complete dependency set and the call
chains that reach real IT8951 traffic:

| `display_seeed_gfx.cpp` | → `EPaper` method | → `EPD_*` macro | → `tcon*` reached | IT8951 op |
|---|---|---|---|---|
| `seeed_gfx_epaper_begin` :104 | `initGrayMode(16)` / `deinitGrayMode` :110-113 | — | (sprite realloc) | switch 1/4-bpp buffer |
| ″ | `begin(0)` :115 | `init` body + `EPD_WAKEUP` | RST + `hostTconInit` + `tconWake`+`setTconTemp` | reset, VCOM, GetDevInfo, I80CPCR, wake |
| `seeed_gfx_direct_write_reset` :145 | `begin(0)` (cold/first) or `wake()` :155-157 | `EPD_WAKEUP`/`EPD_SET_TEMP` | `tconWake`,`setTconTemp` | wake / full init |
| `seeed_gfx_direct_write_chunk` :166 | `getPointer()` :168 + `memcpy` | — | — | fill framebuffer |
| `seeed_gfx_boot_write_row` :133 | `getPointer()` :134 + `memcpy` | — | — | fill one row |
| `seeed_gfx_direct_refresh` :179 | `update()` ×1–2 :180-183 | `SET_WINDOW`→`PUSH_NEW[_GRAY]_COLORS`→`UPDATE[_GRAY]` | `setTconWindowsData`, `tconLoad1bppImage`/`tconLoadImage`, `tconDisplayArea1bpp`/`tconDisplayArea`, `tconWaitForDisplayReady` | window, packed load, GC16 refresh, wait |
| `seeed_gfx_full_update` :119 | `update()` | (same as above) | (same) | (same) |
| `seeed_gfx_direct_sleep` / `_sleep_after_refresh` :186,129 | `sleep()` | `EPD_SLEEP` | `tconSleep` | sleep |

**Distinct IT8951 primitives OpenDisplay truly needs** (the port target):
`tconWaitForReady` (HRDY), `getTconInfo` (GetDevInfo), `setTconVcom`, `hostTconInit`
(VCOM+GetDevInfo+I80CPCR), reg read/write (for 1bpp mode + `LUTAFSR` polling),
`tconHostAreaPackedPixelWrite` (full-frame, both 8bpp-as-1bpp and 4bpp),
`tconDisplayArea` (GC16 gray) and `tconDisplayArea1bpp` (GC16 1bpp w/ BGVR),
`tconWaitForDisplayReady`, `tconWake`, `tconSleep`, `setTconTemp`. Plus RST toggle. **~14 ops.**

Waveform selection is a single mode index (`0x02` = GC16) passed to `DPY_AREA`; the waveforms
themselves live in the TCON's own flash and self-load — **no LUT bytes to port.** The `refresh_mode`
distinction in `seeed_gfx_direct_refresh` (:181) is merely "call GC16 once vs twice", not a
different waveform.

---

## D. Portion NOT used by Firmware (can be dropped)

Everything in B not in C. Confirmed unreachable from `display_seeed_gfx.cpp`:

- **All TFT_eSPI graphics/text/sprite/font/touch:** the entire `TFT_eSPI` core, `Sprite.cpp`
  (except the `malloc`+`getPointer` mechanics), `Smooth_font.cpp`, `Button.cpp`, `Touch.cpp`,
  every `TFT_Drivers/*` LCD driver. OpenDisplay never calls a drawing/text primitive.
- **Partial update:** `EPaper::updataPartial` (EPaper.cpp:71), `EPD_UPDATE_PARTIAL`
  (ED103TC2_Defines.h:64), `EPD_WAKEUP_PARTIAL`. OpenDisplay always does a full-frame
  `setTconWindowsData(0,0,w-1,h-1)` + full push.
- **Sub-region push:** `EPaper::update(x,y,w,h,data)` (EPaper.cpp:159), `pushImage`.
- **Rotation:** `tconSetImgRotation` (Tcon.cpp:237), rotate args — always `ROTATE_0`.
- **Read-back beyond what init/refresh need:** `tconReadNData`/`tconReadData` are needed **only**
  for `getTconInfo` and reg reads; the general read API and `MEM_BST_*` burst-memory commands are
  unused.
- **1bpp BGVR — REQUIRED (confirmed shipping 2026-07-24):** panel_ic **3000**
  (`OD_PANEL_IC_ED103TC2_1872X1404`, non-`_4GRAY`) **does use** the 1bpp BGVR path:
  `seeed_gfx_panel_is_4gray()` is false → `deinitGrayMode()` → `EPaper::update()` `_grayLevel==0`
  branch → `EPD_PUSH_NEW_COLORS` + `EPD_UPDATE` → `tconLoad1bppImage` + `tconDisplayArea1bpp`
  (which sets `UP1SR+2` bit2 and `BGVR`). **The product ships 1bpp**, so this is NOT droppable and
  NOT in section D — it is in scope (kept here only to record the resolved decision). Its
  consequence propagates: the `UP1SR+2` register **read-modify-write** makes the HRDY-gated **read
  path mandatory** (it is not merely a GetDevInfo convenience). See E.2, G, and H.
- **Temp/humidity helpers:** `setTemp/getTemp/setHumi/getHumi` (only `setTconTemp` on wake is used;
  the callback API is unused).
- **`hostTconInitFast`** (Tcon.cpp:453 / Init_Wake.h): the OpenDisplay wake path calls
  `EPaper::wake()`→`tconWake()` **not** `initFromSleep()`, and `seeed_gfx_direct_write_reset` forces
  a full `begin()` after any rail cut (`seeed_gfx_mark_hw_deinitialized`, cpp:190). So the
  "fast re-init after light sleep" path is effectively bypassed — port only the full `hostTconInit`.

---

## E. What must move to bb_epaper

### E.1 ED103TC2 panel definition(s)
Two `EPD_PANEL` rows (`bb_ep.inl` `panelDefs[]` @3738) + two `EP_PANEL_*` enum values
(`bb_epaper.h` @159, before `EP_PANEL_COUNT` @271; the old `EP47` enum value is **absent** — only a
commented `panelDefs` row survives @3779):

| Field | 1bpp (ic 3000) | 4gray (ic 3001) |
|---|---|---|
| width / height | 1872 / 1404 | 1872 / 1404 |
| x_offset | 0 | 0 |
| pInitFull | `it8951_ed103_init` (RST→VCOM 1400→GetDevInfo→I80CPCR) | same |
| pInitFast/Part | NULL | NULL |
| flags | 0 (B/W) | `BBEP_16GRAY` (0x0040) → 4-bpp buffer, `bbepAllocBuffer` @bb_ep_gfx.inl:1955 |
| chip_type | `BBEP_CHIP_IT8951` | `BBEP_CHIP_IT8951` |
| pColorLookup | `u8Colors_2clr` | `u8Colors_4gray` |

Analog/mode constants for both: **VCOM = 1400** (−1.40 V, from `hostTconInit` Tcon.cpp:440),
GC16 waveform = `DPY_AREA` mode **0x02**, pixel formats `IT8951_8BPP`(=3, used for 1bpp
transport) / `IT8951_4BPP`(=2), endian `IT8951_LDIMG_L_ENDIAN`, gray levels 2 or 16.
Note bb_epaper's `epd47_it8951_init` @1396 encodes VCOM **2300** (M5Paper) — **do not reuse**;
ED103TC2 needs 1400.

### E.2 IT8951 operations to port (source → target)

| New bb_epaper function | Ports from `Tcon.cpp` | Behavior |
|---|---|---|
| HRDY-gated cmd/data/read prims | `tconWriteCmdCode`:60,`tconWirteData`:82,`tconReadData`:144,`tconReadNData`:164 | Add HRDY wait around the **existing** stubs `bbepWriteIT8951Cmd/Data/CmdArgs` (arduino_io.inl:108-144) and add a **new read** primitive (stubs have none) |
| `bbepIT8951WaitHRDY` | `tconWaitForReady`:22 | Poll busy pin until HIGH w/ timeout (reuse `opnd_seeed_tcon_busy_timeout` semantics) |
| `bbepIT8951ReadReg/WriteReg` | `tconReadReg`:201,`tconWriteReg`:213 | `REG_RD`/`REG_WR` + addr(+val) |
| `bbepIT8951GetDevInfo` | `getTconInfo`:416 | `GET_DEV_INFO` burst read → panel W/H + img-buf base addr |
| `bbepIT8951SetVcom` | `setTconVcom`:498 | cmd 0x0039 arg 0x02 + vcom |
| `bbepIT8951Init` (host init) | `hostTconInit`:437 | RST → set VCOM 1400 → GetDevInfo → `I80CPCR=1` |
| `bbepIT8951LoadFull` (packed pixel) | `tconHostAreaPackedPixelWrite`:276 + `tconLoad1bppImage`:369 / `tconLoadImage`:393 | set `LISAR` base → `LD_IMG_AREA` → per-row X-mirror + word-pack → burst → `LD_IMG_END`. 1bpp: 8bpp transport, width/8, X-mirror; 4bpp: width as-is |
| `bbepIT8951Display` (GC16) | `tconDisplayArea`:331 / `tconDisplayArea1bpp`:346 | gray: `DPY_AREA` mode 2. 1bpp: X-mirror + set `UP1SR+2` bit2 + `BGVR`=(0<<8\|255) + `DPY_AREA` + wait + restore |
| `bbepIT8951WaitDisplay` | `tconWaitForDisplayReady`:521 | poll `LUTAFSR` until 0 |
| `bbepIT8951Sleep/Wake` | `tconSleep`:506/`tconWake`:511 | cmd `SLEEP`/`SYS_RUN` (+ `setTconTemp` on wake, Tcon.cpp:482) |

All register `#define`s already exist in `bb_epaper.h` @329-409 (`IT8951_*`); command opcodes too.
`I80TCONDevInfo` struct must be added (currently only in `Tcon.h`:24) — put it near the IT8951
`#define`s in `bb_epaper.h` or in `structs.h` (per repo rule, structs do not go in the protocol
header, but this is bb_epaper's own header, not the vendored protocol header, so it is fine there).

---

## F. Detailed integration plan

### F.1 Panel-table rows (bb_ep.inl:3738, bb_epaper.h enum)
Add `EP_ED103TC2_1872x1404` and `EP_ED103TC2_1872x1404_4GRAY` to the `EP_PANEL_*` enum
(bb_epaper.h, before `EP_PANEL_COUNT`:271), and the two `panelDefs[]` rows from E.1. Uncomment /
replace the dormant M5Paper row at bb_ep.inl:3779 or add fresh rows. Write `it8951_ed103_init` as a
byte sequence that `bbepSendCMDSequence` (bb_ep.inl:4202) can interpret — **but note** its opcode
model is single-byte SSD/UC commands; IT8951 needs 16-bit opcodes + HRDY, so the IT8951 init is
better done as a dedicated C function (`bbepIT8951Init`) invoked from the lifecycle branch rather
than shoe-horned into the `pInitFull` byte-table format. Set `pInitFull=NULL` and branch on
`chip_type==BBEP_CHIP_IT8951` in the lifecycle functions instead.

### F.2 `BBEP_CHIP_IT8951` branches to add (by function, bb_ep.inl)

| Function | Line | IT8951 branch behavior |
|---|---|---|
| `bbepWaitBusy` | 3957 | `busy_idle = HIGH` for IT8951 (HRDY ready = HIGH). Currently only UC81xx=HIGH else LOW (:3965) |
| `bbepWakeUp` | 3992 | RST pulse already generic; after reset, for IT8951 call `bbepIT8951Init` (VCOM+GetDevInfo+I80CPCR) — otherwise TCON unconfigured |
| `bbepSetAddrWindow` | 4006 | Complete the `#ifdef FUTURE` stub (:4016-4027): for full-frame OpenDisplay this collapses to storing the area; real `LD_IMG_AREA` happens inside the packed-write. Simplest: make it a no-op for IT8951 (window is implicit in the load) |
| `bbepWritePlane` | 5083 | Branch before the UC/SSD split (:5120): call `bbepIT8951LoadFull(pBBEP, plane)` (1bpp vs 4bpp per `BBEP_16GRAY`) and return |
| `bbepRefresh` | 4365 | Branch at top: for IT8951, `bbepIT8951Display` (GC16 mode 2, 1bpp-BGVR or gray) + `bbepIT8951WaitDisplay`; skip the UC/SSD `DISP_CTRL2`/`DRF` logic |
| `bbepSleep` | 4109 | Branch: IT8951 → `bbepIT8951Sleep` (cmd 0x03); `is_awake=0` |
| `bbepStartWrite` | 4140 | Not needed for IT8951 (load is monolithic); guard so it is a no-op |

That is **6–7 functions** getting a small IT8951 branch, plus the ~8 new IT8951 functions from E.2,
plus HRDY/read added to the 3 existing SPI stubs.

### F.3 Firmware-side changes (`src/`)
Replace the `EPaper`-based shim in `display_seeed_gfx.cpp` with bb_epaper calls, keeping the **same
public function names** so `display_service.cpp` needs no change beyond eventually collapsing its
`#ifdef`s:

| `display_seeed_gfx.cpp` fn | New bb_epaper implementation |
|---|---|
| `seeed_gfx_epaper_begin` :104 | `bbepSetPanelType(&bbep, EP_ED103TC2_… )`; `bbepInitIO(...)`; `bbepAllocBuffer` (or `setBuffer` to a PSRAM buffer); `bbepWakeUp` (→ runs `bbepIT8951Init`) |
| `getPointer()` :134,160,168 | `bbep.ucScreen` / `BBEPAPER::getBuffer()` (bb_epaper.cpp:421) |
| `seeed_gfx_direct_write_reset` :145 | first-boot/rail-cut → full `bbepWakeUp`+init; else nothing; `memset(ucScreen,0xFF,fb_byte_size())` |
| `seeed_gfx_direct_write_chunk` :166 | `memcpy` into `ucScreen + offset` (unchanged) |
| `seeed_gfx_direct_refresh` :179 | `bbepWritePlane(&bbep, PLANE_0, ...)` then `bbepRefresh(&bbep, REFRESH_FULL)` (×2 if mode 0) |
| `seeed_gfx_direct_sleep` :186 | `bbepSleep(&bbep, 1)` |
| `seeed_gfx_mark_hw_deinitialized` :190 | keep the plain-RAM flag; on next reset force full init |

Once the shim is pure bb_epaper, the ~15 `#ifdef OPENDISPLAY_SEEED_GFX` sites in
`display_service.cpp` (grep list: lines 19, 368, 418, 724, 753, 1579, 1677, 1964, 2093, 2147, 2332,
2452, 2644, 2826, 3170) **collapse**: the ED103TC2 path becomes just another
`seeed_driver_used()`-style panel_ic check that routes through the same `bbep` object the other
panels already use. Sites that today call `seeed_gfx_direct_*` can call the normal
`bbepWritePlane`/`bbepRefresh`/`bbepSleep` used by e.g. the E1004 path (already present at
:449-536). `seeed_driver_used()` (:723) can be renamed/merged into the generic panel dispatch.
`-DOPENDISPLAY_SEEED_GFX` and the `lib/Seeed_GFX/` tree are then deletable.

### F.4 Framebuffer ownership & sizing
Today `EPaper` (via `TFT_eSprite::createSprite`) mallocs `_img8`; `getPointer()` returns it. In
bb_epaper, `ucScreen` is the framebuffer, allocated by `bbepAllocBuffer` (bb_ep_gfx.inl:1951),
which routes **>98 000 bytes to `ps_malloc` (PSRAM)** automatically (:1965-1969). Sizes for
1872×1404 = **2 628 288 px**:

- **1bpp:** stride `(1872+7)/8 = 234` B/row × 1404 = **328 536 B ≈ 320.8 KiB (~329 KB)** →
  `bbepAllocBuffer` picks PSRAM (>98 000). ✔
- **4bpp (16-gray):** stride `1872/2 = 936` B/row × 1404 = **1 314 144 B ≈ 1.253 MiB (~1.31 MB)**
  → PSRAM, and `BBEP_16GRAY` makes `bbepAllocBuffer` size it as `(w>>1)*h` (:1955-1956). ✔

**PSRAM confirmed** on every ED103TC2 build env: `platformio.ini` has `-DBOARD_HAS_PSRAM` +
`board_build.psram_type=qspi_opi` for `esp32-s3-N16R8` (:47), `N8R8` (:72), `N32R8` (:124). The S3
carries 8 MB OPI PSRAM; a 1.31 MB framebuffer is comfortable. Packed-pixel transfer sizes match the
buffer sizes exactly (traced through `tconHostAreaPackedPixelWrite`: 1bpp → 117 words/row × 1404 =
328 536 B; 4bpp → 468 words/row × 1404 = 1 314 144 B).

### F.5 Upstream-vs-fork strategy
bb_epaper is vendored **upstream** (bitbank2). The IT8951 stub (chip enum, `IT8951_*` `#define`s,
`bbepWriteIT8951*`, `epd47_it8951_init`, the commented panel row, the `#ifdef FUTURE` dispatch) is
**bitbank2's own unfinished work** in a single 2024-12-16 commit `631e3c0`. Completing it is a
**natural upstream PR**, not a fork: add the HRDY/read primitives, the lifecycle branches, the
GetDevInfo/VCOM/packed-write/DPY_AREA functions, and generic IT8951 panel rows (M5Paper 540×960
**and** ED103TC2 1872×1404). Keep OpenDisplay-specific runtime-pin plumbing behind the existing
`OPENDISPLAY_SEEED_GFX_RUNTIME_PINS`-style guard so upstream stays board-agnostic. Until merged,
pin the vendored copy (as already done for `BBEP_T133A01` / E1004, platformio.ini:143).

---

## G. Feasibility with minimal architectural change

**It fits bb_epaper's table-driven model with a *parallel dispatch branch*, not a rewrite.**
bb_epaper's lifecycle is `init-table → bbepWritePlane (push local buffer) → bbepRefresh`. The
IT8951 maps onto this cleanly **at the seams**, because it too has (a) host-side framebuffer
(`ucScreen`), (b) a "push to controller frame memory" step (`bbepWritePlane`→packed-pixel load),
(c) a "refresh with a mode" step (`bbepRefresh`→`DPY_AREA` GC16). The difference is *inside* each
step: instead of a byte-command table + RAM window, IT8951 uses I80 command packets + HRDY + its
own frame memory. So each lifecycle function gets **one `if (chip_type==BBEP_CHIP_IT8951)` branch
that early-returns after doing the IT8951 equivalent** — the exact pattern already used for
`UC81xx` vs `SSD16xx` throughout (`bbepWaitBusy`:3965, `bbepSetAddrWindow`:4031, `bbepSleep`:4112,
`bbepStartWrite`:4145, `bbepWritePlane`:5120).

**Quantified:**
- Functions gaining an IT8951 branch: **6–7** (`bbepWaitBusy`, `bbepWakeUp`, `bbepSetAddrWindow`,
  `bbepWritePlane`, `bbepRefresh`, `bbepSleep`, +`bbepStartWrite` no-op).
- New/ported IT8951 functions: **~8** (HRDY wait, read primitive, reg rd/wr, GetDevInfo, SetVcom +
  host init, packed-pixel full-frame load, DPY_AREA display incl 1bpp-BGVR, wait-display).
- New/ported lines: **~250–350** C (the `tcon*` originals total ~520 lines but a third is
  unused: reads-beyond-init, rotation, partial, MEM_BST, temp/humi callbacks).
- Untouched: the entire SSD16xx/UC81xx code path, all 60+ existing panel rows, `bbepWriteImage*`,
  `bbepMakeLUTs`, the graphics layer — **no regression surface** for existing panels.

**Single biggest architectural friction: the SPI transport granularity + missing HRDY/read.**
bb_epaper's IT8951 stubs (`bbepWriteIT8951Cmd/Data/CmdArgs`, arduino_io.inl:108-144) fire
`SPI.transferBytes` **without any HRDY handshake and provide no read path**, whereas IT8951
mandates HRDY-gating before the preamble and before data, and GetDevInfo/reg-reads/`LUTAFSR`
polling are read bursts (Seeed polls HRDY per word, Tcon.cpp:22,151-184). **Contain it** by adding a
single `bbepIT8951WaitHRDY(pBBEP)` used inside a small set of transport helpers (cmd/data/read),
mirroring `tconWaitForReady`'s timeout guard (which OpenDisplay already surfaces via
`opnd_seeed_tcon_busy_timeout_*`, display_seeed_gfx.cpp:40-48) so a dead panel degrades gracefully
instead of hanging. For the bulk pixel push, HRDY is polled **once** before the burst (as Seeed
does, Tcon.cpp:107), so throughput is unaffected.

---

## H. Risk register

| Risk | Sev | Detail | Mitigation |
|---|---|---|---|
| **VCOM correctness** | High | IT8951 VCOM is a per-unit analog calibration (often on an FPC sticker). Seeed hardcodes **1400** (−1.40 V, Tcon.cpp:440); bb_epaper's M5Paper stub uses **2300** (bb_ep.inl:1399). Wrong VCOM → washed-out/ghosted/over-driven image. | Use 1400 for ED103TC2 (matches shipping Seeed path). Make VCOM a panel-table/config field, not a literal. Validate against a physical panel; consider reading factory VCOM if the TCON stores it. |
| **HRDY timing** | High | Existing stubs omit HRDY; IT8951 will corrupt/hang without it. Read bursts poll HRDY per word. | Add `bbepIT8951WaitHRDY` with the existing timeout+flag mechanism; poll before preamble/data and per read word; poll once before pixel burst. |
| **Packed-pixel nibble/word order + X-mirror** | High | `tconHostAreaPackedPixelWrite` reverses X per row (`width-1-i`, Tcon.cpp:307) and `tconLoad1bppImage`/`tconDisplayArea1bpp` additionally X-mirror the origin (`panelW-1-usX-usW+1`, :371,348). 4bpp nibble order is left-pixel=high-nibble (display_seeed_gfx.cpp:3-5). Any mismatch → mirrored or scrambled output. | Port the mirror/pack loop **verbatim**; keep the same `IT8951_LDIMG_L_ENDIAN` + `bswap` handling as the existing `bbepWriteIT8951CmdArgs` (arduino_io.inl:141). Diff first render pixel-for-pixel against the current Seeed build. |
| **1bpp BGVR register dance** | **High** (was Med) | **1bpp confirmed shipping (2026-07-24)** → this path is on the critical path, not optional. ic 3000 needs `UP1SR+2` bit2 set/restore + `BGVR` color table around `DPY_AREA` (Tcon.cpp:350-363), which requires **reg read-modify-write** → makes the HRDY-gated **read path mandatory** (no longer just for GetDevInfo). The existing bb_epaper stubs have **no read primitive at all** — this is net-new code, and the single largest correctness dependency. | Port `tconDisplayArea1bpp` exactly; implement + bench-verify the read/HRDY primitive **first**, before any refresh path. Diff first 1bpp render pixel-for-pixel vs. the current Seeed build. |
| **Buffer sizing / PSRAM** | Low | 329 KB (1bpp) / 1.31 MB (4bpp) must land in PSRAM. | `bbepAllocBuffer` auto-routes >98 KB to `ps_malloc` (bb_ep_gfx.inl:1965); all ED103TC2 envs set `BOARD_HAS_PSRAM` (platformio.ini:47,72,124). Verify alloc success; fall back to internal-DRAM is impossible at 1.3 MB → handle NULL. |
| **Hardware-validation dependency** | High | Correctness (VCOM, HRDY, mirroring, GC16 waveform index) cannot be proven from source; needs a real **ED103TC2 1872×1404** panel on an S3. | Gate the migration behind a bench bring-up: keep the Seeed path selectable until the bb_epaper path is validated on hardware side-by-side. |
| **Upstream acceptance** | Low | PR must stay board-neutral. | Keep runtime-pin/timeout glue behind guards; contribute generic M5Paper + ED103TC2 rows; until merged, pin the vendored bb_epaper (as done for E1004). |
| **Refresh-wait semantics** | Low | Today `seeed_gfx_wait_refresh` just `delay(300)` (display_seeed_gfx.cpp:123). bb_epaper uses `LUTAFSR` polling. | Port `tconWaitForDisplayReady`; drop the blind delay for a real poll (strict improvement). |

---

## I. Open questions (need a human / hardware)

1. **VCOM per unit:** Is −1.40 V correct for the specific ED103TC2/E1004 panels OpenDisplay ships,
   or does each unit carry an individual VCOM (sticker/OTP) that must be provisioned? Should VCOM
   become a `DisplayConfig`/factory field rather than a compile-time literal?
2. **1bpp vs 4gray in the field:** ~~Does any shipped product use panel_ic 3000 (1bpp)?~~
   **RESOLVED 2026-07-24 — 1bpp IS shipping.** The BGVR/`UP1SR` 1bpp path and its HRDY-gated read
   dependency are therefore **in scope and mandatory** — the port cannot be shrunk by dropping them.
   The 4bpp/4gray (ic 3001) path may or may not also ship, but does not reduce scope either way.
3. **HRDY pin identity:** Confirm the IT8951 HRDY line is wired to the same GPIO OpenDisplay maps as
   `busy_pin` / `opnd_seeed_runtime_busy` (default 13), and that ready = logic HIGH on this board.
4. **GC16-only, or is A2/DU wanted?** OpenDisplay's current path only ever issues GC16 (mode 0x02).
   Is a fast (A2) mode desired for partial/low-latency updates, or is GC16-only acceptable
   (matches today's behavior)?
5. **SPI clock:** Seeed drives the TCON via `TFT_eSPI` at its configured SPI freq; the bb_epaper
   E1004 path uses 8 MHz (`bbepInitIO(...,8000000)`, display_service.cpp:188). Is 8 MHz safe for the
   IT8951 GetDevInfo read burst on this wiring, or does it need a slower read clock?
6. **DMA:** Seeed's `tconWirteNData` optionally uses `pushPixelsDMA`. Should the bb_epaper port use
   ESP32 SPI DMA for the ~1.3 MB burst, or is `SPI.transferBytes` (as the stub uses) sufficient?
7. **Dual-controller?** The E1004 path handles a dual-CS split panel (`iCS2Pin`,
   display_service.cpp:187,215). Is any ED103TC2 variant dual-controller, or always single-CS?

---

## Appendix: complete bb_epaper function inventory (annotated for the IT8951 port)

bb_epaper has two layers: the **`BBEPAPER::` C++ class** (public API that
`display_service.cpp` calls) and the **`bbep*` C functions** (implementation the class
delegates to). Function counts and line refs are from the vendored checkout
(`.pio/libdeps/<env>/bb_epaper/`, upstream commit `c651b2a`).

**Legend for the port impact column:**
- **TOUCH** — existing function gets a new `BBEP_CHIP_IT8951` branch.
- **ADD** — net-new IT8951 helper (no existing equivalent).
- **USE** — called unchanged by the OpenDisplay IT8951 path.
- **—** — irrelevant to OpenDisplay (drawing/text/loaders never invoked; firmware
  `memcpy`s pre-dithered pixels into the buffer directly).

### A. `BBEPAPER::` public class API (71 methods, `bb_epaper.cpp`)

| Group | Methods | OpenDisplay/IT8951 |
|---|---|---|
| Lifecycle / panel | `begin` `initIO` `setPanelType` `testPanelType` `createVirtual` `getChip` `getFlags` `setFlags` `capabilities` `getLastError` | USE (`begin`,`initIO`,`setPanelType`) |
| Refresh / power | `refresh` `sleep` `wake` `wait` `isBusy` `startWrite` `setPasses` `hasFastRefresh` `hasPartialRefresh` `getRefreshTime` `dataTime` `opTime` `getCache` | USE (`refresh`,`sleep`,`wake`,`wait`) |
| Buffer / plane | `allocBuffer` `freeBuffer` `setBuffer` `getBuffer` `setPlane` `getPlane` `backupPlane` `writePlane` `writeRegion` `setAddrWindow` | USE (`allocBuffer`/`getBuffer` for the framebuffer + `writePlane`) |
| Geometry | `width` `height` `setRotation` `getRotation` `setCS` | USE (`width`,`height`,`setCS`) |
| Raw I/O | `writeCmd` `writeData` | USE (transitive) |
| Drawing (GFX) | `drawPixel` `drawLine` `drawRect`/`fillRect` `drawCircle`/`fillCircle` `drawEllipse`/`fillEllipse` `drawRoundRect`/`fillRoundRect` `fillScreen` `drawSprite` `stretchAndSmooth` | — |
| Text | `drawString` `print` `println` `write` `setCursor` `getCursorX`/`getCursorY` `setFont` `setFreeFont` `setItalic` `setTextColor` `setTextWrap` `getStringBox` | — |
| Image loaders | `loadBMP` `loadG` | — |
| Misc | `setDitherPattern` | — |

### B. Internal `bbep*` C functions

**B.1 Panel / lifecycle (`bb_ep.inl`)**

| Function | Line | Port impact | Note |
|---|---|---|---|
| `bbepSetPanelType` | 3828 | TOUCH (data) | add 2 ED103TC2 rows to `panelDefs[]` + 2 enum values |
| `bbepTestPanelType` | 4233 | — | |
| `bbepCreateVirtual` | 3902 | — | |
| `bbepSetDitherPattern` | 3880 | — | firmware pre-dithers |
| `bbepLightSleep` | 3938 | — | |
| `bbepWaitBusy` | 3957 | **TOUCH** | IT8951 needs HRDY-level poll, not the UC/SSD busy-pin ternary |
| `bbepIsBusy` | 3980 | **TOUCH** | same HRDY semantics |
| `bbepWakeUp` | 3992 | **TOUCH** | IT8951 `SYS_RUN` + `setTconTemp`, vs generic RST pulse |
| `bbepSetAddrWindow` | 4006 | **TOUCH** | IT8951 `LD_IMG_AREA` branch exists but is `#ifdef FUTURE` (bb_ep.inl:4017) — enable + finish |
| `bbepSleep` | 4109 | **TOUCH** | IT8951 `SLEEP` command |
| `bbepStartWrite` | 4140 | **TOUCH** | IT8951 packed-pixel preamble differs |
| `bbepMakeLUTs` | 4170 | **TOUCH (skip)** | IT8951 has no host LUTs — waveforms in TCON flash; branch is a no-op |
| `bbepSendCMDSequence` | 4202 | **TOUCH** | chip-agnostic byte-code walker today; IT8951 init needs read-back (GetDevInfo/VCOM) the flat table can't express → **the one genuinely new dispatch point** |
| `bbepFill` | 4253 | TOUCH (opt) | only if IT8951 fast-clear wanted; firmware overwrites full frame anyway |
| `bbepRefresh` | 4365 | **TOUCH** | IT8951 `DPY_AREA` GC16 (mode 0x02) + `LUTAFSR` wait |
| `bbepSetRotation` | 4444 | — | always ROTATE_0 |

**B.2 Plane / image write (`bb_ep.inl`)**

| Function | Line | Port impact |
|---|---|---|
| `bbepWritePlane` | 5083 | **TOUCH** — IT8951 full-frame packed-pixel `LD_IMG` burst |
| `bbepWriteRegion` | 5061 | — (partial region; OpenDisplay is full-frame) |
| `bbepWriteImage` / `bbepWriteHalf` / `bbepWriteImage1to4bpp` / `bbepWriteImage2bpp` / `bbepWriteImage4bpp` / `bbepWriteImage4bppDual` / `bbepWriteImage4bppSpecial` | 4464–4976 | — (SSD/UC plane encoders) |

**B.3 Low-level SPI I/O (per-platform; OpenDisplay uses `arduino_io.inl`)**

| Function | Port impact | Note |
|---|---|---|
| `bbepInitIO` `bbepSetCS2` `bbepWriteCmd` `bbepWriteData` `bbepCMD2` | USE | one definition each per platform (`arduino_io.inl`/`mem_io.inl`/`esphome_io.inl`/`rpi_io.inl`) |
| `bbepWriteIT8951Cmd` `bbepWriteIT8951Data` `bbepWriteIT8951CmdArgs` | **TOUCH** | existing stubs (arduino_io.inl:108–144), **no HRDY, no read path** — add HRDY wait |
| *(new)* `bbepReadIT8951Data` / `bbepReadIT8951Reg` | **ADD** | net-new read primitive — mandatory for GetDevInfo **and** the 1bpp `UP1SR` RMW |

**B.4 New IT8951 helpers to ADD (ported from Seeed `Tcon.cpp` — see §E.2)**

`bbepIT8951WaitHRDY` · `bbepIT8951ReadReg`/`WriteReg` · `bbepIT8951GetDevInfo` ·
`bbepIT8951SetVcom` · `bbepIT8951Init` · `bbepIT8951LoadFull` · `bbepIT8951Display` ·
`bbepIT8951WaitDisplay` · `bbepIT8951Sleep`/`Wake`  → all **ADD**.

**B.5 Buffer (`bb_ep_gfx.inl`)**

| Function | Line | Port impact |
|---|---|---|
| `bbepAllocBuffer` | 1951 | USE — routes 329 KB (1bpp) / 1.31 MB (4bpp) to `ps_malloc` (PSRAM) |

**B.6 IRRELEVANT to OpenDisplay (never called — firmware writes pixels directly)**

- **Pixel setters (`bb_ep_gfx.inl`):** `bbepSetPixel{2,3,4,16}Clr` · `bbepSetPixel4Gray` · `bbepSetPixel2ClrDither` · all `bbepSetPixelFast*` variants (`Fast2Clr/3Clr/4Clr/4ClrV2/16Clr/4Gray`).
- **Graphics primitives:** `bbepDrawLine` · `bbepEllipse` · `bbepRectangle` · `bbepRoundRect` · `bbepDrawSprite`.
- **Text / fonts:** `bbepWriteString` · `bbepWriteStringCustom` · `bbepSetCursor` · `bbepSetTextWrap` · `bbepGetStringBox` · `bbepUnicodeString` · `bbepUnicodeTo1252` · `bbepStretchAndSmooth`.
- **Image loaders:** `bbepLoadG5` · `bbepLoadG5_2Bit` · `bbepLoadBMP` · `bbepLoadBMP3`.

### C. Tally

| Bucket | Count |
|---|---|
| Existing `bbep*` functions that get an IT8951 branch (**TOUCH**) | 11 (`bbepWaitBusy`, `bbepIsBusy`, `bbepWakeUp`, `bbepSetAddrWindow`, `bbepSleep`, `bbepStartWrite`, `bbepMakeLUTs`(skip), `bbepSendCMDSequence`, `bbepRefresh`, `bbepWritePlane`, + IT8951 SPI stubs) |
| New IT8951 helpers (**ADD**) | ~10 (incl. the read primitive) |
| Existing functions used unchanged (**USE**) | class API + `bbepInitIO`/`bbepAllocBuffer`/SPI prims |
| Irrelevant, untouched (**—**) | ~40 (all GFX/text/pixel/loaders) + all SSD/UC plane encoders |

Net: the IT8951 work concentrates in ~11 TOUCH + ~10 ADD functions; the ~40-function GFX/text
surface and every existing panel stay untouched — confirming §G's "minimal architectural change"
verdict, with `bbepSendCMDSequence` (init read-back) as the single new dispatch point.
