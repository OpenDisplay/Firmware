/* ==========================================================================
 * opendisplay_structs.h  --  OpenDisplay config & message wire-payload contract
 * ==========================================================================
 *
 * PURPOSE
 *   Single source of truth for the LAYOUT of every payload that travels on the
 *   OpenDisplay BLE wire: the config-transfer TLV packets (CONFIG_WRITE /
 *   CONFIG_READ bodies), the fixed-size opcode message payloads (PIPE, partial
 *   write, auth, LED flash), and the BLE manufacturer-specific advertisement.
 *   Where opendisplay_protocol.h owns the FRAMING (opcodes, response/status
 *   bytes, errors, envelope), THIS file owns the CONTENTS -- the real C enums,
 *   bitfield definitions, and packed structs that describe those bytes.
 *
 *   A new engineer or an AI agent should be able to hand-decode a captured
 *   config blob or advertisement BYTE-BY-BYTE from THIS FILE ALONE: every field
 *   is documented in wire order with its meaning, valid values, units, default,
 *   and endianness. This header is also the CODEGEN SOURCE -- the machine-
 *   readable `@tag` annotations drive generation of the Swift / Python / JS
 *   type mirrors (see docs/shared-types-plan.md); codegen must carry the prose
 *   through as idiomatic doc comments in each target language.
 *
 *   OD_STRUCTS_VERSION  2.0   (MAJOR.MINOR spec marker for THIS file; see
 *                              VERSIONING POLICY below -- NOT sent on the wire)
 *   LAST CHANGED        2026-07-18
 *
 *   The two version schemes carried here are DISTINCT (see §6 Q2 of the plan):
 *     - OD_STRUCTS_VERSION_* : documents this spec file, like protocol.h's
 *       OD_PROTOCOL_VERSION. A compatibility marker; never transmitted.
 *     - OD_CONFIG_VERSION / OD_CONFIG_MINOR_VERSION : the ON-WIRE config-format
 *       version (currently 1 / 4). The OuterPacketHeader.version byte transmits
 *       the MAJOR; the minor is negotiated/inspected by peers.
 *
 * --------------------------------------------------------------------------
 * VERSIONING POLICY  (same shape as opendisplay_protocol.h, scoped to PAYLOAD
 *                     layout instead of framing)
 * --------------------------------------------------------------------------
 *   OD_STRUCTS_VERSION is MAJOR.MINOR and describes the payload SPEC in this
 *   file. It is a compatibility / documentation marker, NOT transmitted.
 *
 *   Bump MAJOR (x.0) for a BREAKING layout change -- one that makes a peer
 *   built against the previous version misread payload bytes:
 *     - moving, resizing, renumbering, or re-typing an existing field;
 *     - changing a field's endianness or a struct's total wire size;
 *     - removing or renumbering an enum value / bit / packet-type id;
 *     - changing the packed layout / offset of any field.
 *   Reset MINOR to 0 on every MAJOR bump.
 *
 *   Bump MINOR (2.x) for a BACKWARD-COMPATIBLE addition -- old peers keep
 *   working unchanged:
 *     - naming a byte carved out of a reserved[] block (the bytes were already
 *       must-be-zero, so old peers still interop) -- e.g. WifiConfig's server
 *       fields, the DisplayConfig cs_pin_2 promotion;
 *     - adding a NEW enum value, bitfield bit, or packet-type id that leaves
 *       existing ones untouched;
 *     - appending a NEW optional trailing field old parsers ignore.
 *
 *   NO bump for comment / prose clarifications, or renaming a field to its
 *   canonical spelling when the byte offset and size are unchanged.
 *
 *   Rule of thumb -- "could a peer on the previous version misread the bytes?"
 *     yes                         -> MAJOR
 *     no, but something new added -> MINOR
 *     neither                     -> no bump
 *
 * --------------------------------------------------------------------------
 * CHANGELOG  (newest first; entries accrue under "Unreleased" and roll into a
 *            new version heading on each bump -- see AGENT INSTRUCTIONS below)
 * --------------------------------------------------------------------------
 *   Unreleased (since 2.0)
 *     - Doc-only: fixed two comment shapes the codegen parser mis-read and added
 *       the CODEGEN AUTHORING RULES banner section to prevent recurrence. Split the
 *       combined BusFlags/PinBitmap @bits comment into one comment per group; folded
 *       OuterPacketHeader's trailing FOLLOWUP note inside its @doc quote. No wire
 *       change; regenerated the language mirrors.
 *     - Named the skipped/reserved bits inside two bitfield groups with explicit
 *       placeholder macros (no wire change; these bits stay reserved-must-be-0):
 *       TransmissionModes bit5/bit6 -> OD_TRANSMISSION_MODE_RESERVED_5/_6;
 *       MsdStatusBits bit3 -> OD_MSD_STATUS_RESERVED_3. Documentation only.
 *     - Add each payload-layout change here as it lands. On the next version bump,
 *       move these under a new "MAJOR.MINOR (YYYY-MM-DD)" heading.
 *
 *   2.0  (2026-07-18)
 *     - Initial canonical shared payload contract: the wire-payload counterpart
 *       to opendisplay_protocol.h, reconciling the per-repo opendisplay_structs.h
 *       / structs.h supersets and the website config.yaml (ble_proto v1.4) into
 *       one self-documenting header vendored into all firmware repos.
 *     - 15 config TLV packet structs + 2 framing structs + 9 message/advert
 *       structs, all packed with sizeof static-asserts; 19 value enums; the
 *       bitfield groups; and the loose wire constants (OD_CONFIG_VERSION pair,
 *       CRC constants, OD_PIN_UNUSED).
 *     - Reconciliations (superset of every source; total sizes unchanged):
 *       SystemConfig pwr_pin_2/3 named; ManufacturerData simple_config_* named;
 *       PowerOption charge/min_wake/screen_timeout tail named; DisplayConfig
 *       cs_pin_2 + full_update_mC + legacy_tag_type; SensorData i2c_addr_7bit +
 *       msd_data_start_byte; BinaryInputs input_pin_1..8 / pins_used +
 *       power_off_flags / power_off_hold_sec.
 *     - ColorScheme value 7 = SEVEN_COLOR (config.yaml "7color"); the former
 *       firmware COLOR_SCHEME_GRAY8 = 7 was a mistake and is dropped outright
 *       (no gray8 at any value).
 *     - WifiConfig extended AHEAD of firmware: server_host[64] + server_port
 *       (big-endian) promoted out of reserved[] (backward-compatible carve).
 *     - Message payloads (previously hand-parsed and drifting in Swift/JS) given
 *       a single machine-readable home: LedFlashPattern, MsdAdvertisement,
 *       PipeStartRequest/Response, PipePartialExt, PipeSack,
 *       PartialWriteStartHeader (all-big-endian geometry), AuthChallenge,
 *       AuthProof.
 *
 * --------------------------------------------------------------------------
 * AGENT INSTRUCTIONS -- changelog upkeep (perform on EVERY edit to this file)
 * --------------------------------------------------------------------------
 *   1. Set LAST CHANGED (top) to the date of your edit.
 *   2. Add a bullet describing the change under "Unreleased (since x.y)".
 *   3. Classify the change via the VERSIONING POLICY above: layout-breaking =>
 *      MAJOR, backward-compatible addition => MINOR, doc-only => no bump.
 *   4. VALIDATE before finishing: confirm the accumulated "Unreleased" entries
 *      are consistent with OD_STRUCTS_VERSION. When a bump is warranted, update
 *      OD_STRUCTS_VERSION_MAJOR / _MINOR / _STR, add a new "MAJOR.MINOR
 *      (YYYY-MM-DD)" heading, move the Unreleased entries beneath it (reset
 *      MINOR to 0 on a MAJOR bump), and leave "Unreleased" empty.
 *   5. If a change alters the ON-WIRE format, also bump OD_CONFIG_VERSION /
 *      OD_CONFIG_MINOR_VERSION and note it -- that pair is independent of the
 *      spec marker and is what peers actually negotiate.
 *   6. Never delete or rewrite historical entries -- the changelog is append-only.
 *   7. Every packed struct MUST keep its trailing
 *      OD_STATIC_ASSERT(sizeof(struct X) == N, ...); if you change a layout,
 *      change the asserted size to match (and treat it as a MAJOR change).
 *
 * CANONICAL LOCATION
 *   opendisplay-protocol/src/opendisplay_structs.h
 *
 *   VENDORED COPY IN FIRMWARE REPOS -- DO NOT EDIT THERE.
 *   Sync every copy byte-for-byte via tools/sync_protocol_header.py (same
 *   mechanism as opendisplay_protocol.h; same include guard so the vendored
 *   file is a drop-in replacement for each repo's existing opendisplay_structs.h).
 *
 * --------------------------------------------------------------------------
 * LANGUAGE / LINKAGE RULE  (INVERTED from opendisplay_protocol.h)
 * --------------------------------------------------------------------------
 *   opendisplay_protocol.h is macro-only. THIS header is the opposite: it
 *   intentionally CONTAINS real `enum`s and packed `struct`s -- that is its
 *   purpose (they are the codegen source). It must still compile clean as BOTH
 *   C99 and C++, the same gate as protocol.h:
 *       cc  -std=c99 -fsyntax-only src/opendisplay_structs.h
 *       c++          -fsyntax-only src/opendisplay_structs.h
 *   Constraints that keep it portable and drop-in:
 *     - Enums are PLAIN (not `enum class`); every enumerator has an EXPLICIT
 *       value and an `OD_` prefix (unprefixed spellings such as
 *       COLOR_SCHEME_MONO are live macros in the firmware repos, so an
 *       `OD_`-prefixed enumerator lets both coexist during migration).
 *     - Struct and field names match the firmware repos EXACTLY (DisplayConfig,
 *       instance_number, ...) so the vendored header is a drop-in.
 *     - NO functions. NO typedefs of RAM-only / in-memory types. NO
 *       repo-specific values (GPIO pin values, buffer sizes, GlobalConfig,
 *       EncryptionSession, ImageData, PipeWriteState, ButtonState). Those move
 *       to a repo-local header on adoption -- mirror of protocol.h's "no
 *       typedefs here" rule, inverted: "no in-memory / RAM-only types here".
 *     - This header #includes opendisplay_protocol.h and NEVER redeclares a name
 *       it already `#define`s. protocol.h defines OD_NFC_IC_* as MACROS; an
 *       `enum` member of the same spelling would be textually rewritten by the
 *       macro and fail to compile. So the nfc_ic_type field is @enum-bound to
 *       protocol.h's OD_NFC_IC_* rather than re-enumerated here; likewise pipe
 *       flags bind to protocol.h's PIPE_FLAG_*.
 *
 * --------------------------------------------------------------------------
 * BLOB-WIDE CONVENTIONS  (from config.yaml `meta`; hold for EVERY struct here)
 * --------------------------------------------------------------------------
 *   (1) BYTE ORDER is LITTLE-ENDIAN by default. Multi-byte fields are LE unless
 *       a field carries an `@endian be` tag. The big-endian exceptions in this
 *       header are: WifiConfig.server_port, and the ENTIRE PartialWriteStartHeader
 *       (0x76) geometry. (Note the PIPE 0x80 partial extension packs the same
 *       geometry LITTLE-endian -- see PipePartialExt.)
 *   (2) All wire structs are PACKED (__attribute__((packed)); no implicit
 *       padding between fields.
 *   (3) RESERVED fields MUST be written as 0 and are ignored by older parsers
 *       (this is what makes carving names out of reserved[] backward-compatible).
 *   (4) BITFIELDS number from bit 0 = LSB. Unused bits MUST be 0.
 *
 *   Two more file-wide notes:
 *     - OuterPacketHeader.length FOLLOWUP: the leading u16 `length` is populated
 *       inconsistently across encoders -- the website toolbox patches it to the
 *       real total, but some encoders leave it as a zero pad. The CRC is ALWAYS
 *       computed as if `length` were 0x0000 regardless. Do not rely on `length`
 *       until the contract is pinned; see OuterPacketHeader below.
 *     - Silicon-specific pin ENCODINGS (e.g. nRF54 (port<<4)|pin, Silabs 0xPN)
 *       are target-defined: every pin field here is a raw u8; its bit encoding
 *       is decided by the receiving firmware, not by this header.
 *     - The image dither-algorithm ids (DitherMode) are owned by epaper-dithering
 *       and are NOT wire values -- they appear in no packet here and are omitted.
 *
 * --------------------------------------------------------------------------
 * TAG CONVENTION  (machine-readable annotations for codegen; live in the SAME
 * trailing comment as the human prose. grep-able, line-anchored.)
 *   @packet 0xNN        this struct is config TLV packet-type 0xNN
 *   @message CMD_X      this struct is the fixed payload of opcode CMD_X
 *   @required           config packet MUST be present in a valid blob
 *   @repeatable [max=N] config packet MAY repeat, up to N instances (absent => 1)
 *   @enum <Name>        field's values are the named enum (may reference protocol.h)
 *   @bits <Group>       field is a bitfield of the named group
 *   @endian le|be       multi-byte field byte order (default le; be must be tagged)
 *   @reserved           must-be-zero forward-compat padding
 *   @unit <u>           physical unit of the value (px, mm, mC, ms, s, mAh, uA)
 *   @min/@max/@default  bounded-integer range / default value
 *   @since <cfgver>     on-wire config version the field/value first appeared in
 *   @changed "<note>"   wire-visible change history for the item
 *   @width <bytes>      intended on-wire width of an enum
 *   @external <comp>    enum mirrors / relates to an outside component
 *   @doc "<text>"       human prose (also the primary deliverable of this file)
 *
 * --------------------------------------------------------------------------
 * CODEGEN AUTHORING RULES  (AGENT INSTRUCTIONS -- keep tools/structs_model.py able
 * to parse this file; the stdlib parser hard-errors or MIS-ATTRIBUTES on anything
 * outside this shape, and the Python/JS/... mirrors are GENERATED from here)
 * --------------------------------------------------------------------------
 *   A. One shape per construct, one declaration per line (no `uint8_t a, b;`):
 *        - enum Name { OD_X = <int literal>, ... };   explicit values, OD_-prefixed.
 *        - struct Name { <uintN_t> f; <uintN_t> f[<int literal>]; ... }
 *              __attribute__((packed));   fixed-width scalars/arrays ONLY -- NO C
 *              bitfields (uint8_t x:3), unions, nested structs, or macro-sized arrays.
 *        - #define OD_..._NAME (1u << N)   for bit flags (mask/shift consts allowed).
 *   B. Every packed struct ENDS with OD_STATIC_ASSERT(sizeof(struct X) == N, ...);
 *      the parser sums the field widths and validates them against N -- that assert
 *      IS the layout oracle (it replaces a compiler front-end). Keep it exact.
 *   C. Each @bits group gets its OWN group comment. Do NOT describe two @bits groups
 *      in one comment block: the parser attaches one comment per group, so a shared
 *      block mis-labels the later group. (Regression fixed 2026-07-18.)
 *   D. ALL prose that must reach the generated docs goes INSIDE the `@doc "..."`
 *      quotes. Text AFTER the closing quote (trailing notes, `FOLLOWUP:` lines) is
 *      DROPPED by the parser -- fold such notes into the quoted @doc. (Fixed 2026-07-18.)
 *   E. Never redeclare a name opendisplay_protocol.h `#define`s; bind fields to it
 *      with @enum instead (see LANGUAGE / LINKAGE RULE above).
 *   F. AFTER ANY EDIT to values/layout/docs, regenerate the mirrors and confirm no
 *      drift (do this in the same change, like protocol.h's gen tools):
 *        python3 tools/gen_python_structs.py --write   # then commit the .py
 *        python3 tools/gen_python_structs.py --check    # CI gate; exit 1 on drift
 *      (add the same for future JS/Swift emitters as they land).
 * ========================================================================== */

#ifndef OPENDISPLAY_STRUCTS_H
#define OPENDISPLAY_STRUCTS_H

#include <stdint.h>
#include <stdbool.h>
#include "opendisplay_protocol.h"  /* framing: CMD_*, RESP_*, OD_NFC_IC_*, PIPE_FLAG_*, ... */

/* Payload-spec revision, MAJOR.MINOR. Documents THIS file; NOT sent on the wire.
 * Distinct from the on-wire OD_CONFIG_VERSION pair defined in SECTION 1. */
#define OD_STRUCTS_VERSION_MAJOR       2u
#define OD_STRUCTS_VERSION_MINOR       0u
#define OD_STRUCTS_VERSION_STR         "2.0"

/* --------------------------------------------------------------------------
 * Portable compile-time size check. Defined once here; every packed struct is
 * followed by OD_STATIC_ASSERT(sizeof(struct X) == N, "..."). The C99 fallback
 * uses a __LINE__-uniquified typedef name so repeated asserts do not collide
 * (a fixed typedef name warns "redefinition of typedef is a C11 feature" under
 * -std=c99, which is the gate used for this header).
 * -------------------------------------------------------------------------- */
#if defined(__cplusplus)
  #define OD_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
  #define OD_STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#else
  #define OD_STATIC_ASSERT_CAT_(a, b) a##b
  #define OD_STATIC_ASSERT_CAT(a, b)  OD_STATIC_ASSERT_CAT_(a, b)
  #define OD_STATIC_ASSERT(expr, msg) \
      typedef char OD_STATIC_ASSERT_CAT(od_static_assert_, __LINE__)[(expr) ? 1 : -1]
#endif

/* ==========================================================================
 * SECTION 1 -- CONFIG TRANSFER FRAMING
 * ==========================================================================
 * A full config transfer is one OUTER packet:
 *     [OuterPacketHeader][single_packet]...[single_packet][crc:2 LE]
 * where each single_packet is:
 *     [SinglePacketHeader][payload]
 * The payload is a fixed-size struct chosen by SinglePacketHeader.id (see the
 * ConfigPacketType enum in SECTION 2). The whole outer packet is what a
 * CONFIG_WRITE (opcode 0x0041, opendisplay_protocol.h) transmits and a
 * CONFIG_READ (0x0040) returns.
 * ========================================================================== */

/* On-wire config-format version. UNLIKE OD_STRUCTS_VERSION / OD_PROTOCOL_VERSION,
 * these ARE transmitted / negotiated: OuterPacketHeader.version carries the
 * MAJOR byte. Frozen at 1.4 by this header (the app's bundled config.yaml is at
 * minor 3 and must catch up). */
#define OD_CONFIG_VERSION              1u   /* @doc "outer-packet major version byte" */
#define OD_CONFIG_MINOR_VERSION        4u   /* @doc "config-format minor; backward-compatible additions" */

/* CRC over the outer packet. CRC16-CCITT, poly 0x1021, init 0xFFFF, computed
 * over length+version+packets AS IF the 2-byte length field were 0x0000 (the
 * toolbox computes the CRC before patching length), with the 2-byte CRC field
 * itself excluded. This "length treated as zero" quirk is mandatory to match. */
#define OD_CONFIG_CRC_POLY             0x1021u /* @doc "CRC16-CCITT polynomial" */
#define OD_CONFIG_CRC_INIT             0xFFFFu /* @doc "CRC16-CCITT initial value" */

/* The pervasive "pin not present" sentinel. A pin field set to 0xFF means the
 * pin is absent / unused (some fields also treat 0 as unused -- noted per field).
 * Wire-meaningful, so canonical here (was GPIO_PIN_UNUSED in the repos). */
#define OD_PIN_UNUSED                  0xFFu   /* @doc "pin field value meaning 'no pin'" */

/** @packet outer  @doc "Wraps the whole transfer: a length+version header, then a
 *  sequence of single_packet entries, then a trailing CRC (NOT part of this
 *  struct -- it is the last 2 bytes of the outer packet). FOLLOWUP: `length` is
 *  populated inconsistently (the website toolbox patches it to the real total incl.
 *  CRC; some encoders leave it a zero pad -> [00 00][version]...). The CRC is always
 *  computed as if `length` were 0x0000. Verify whether any receiver relies on
 *  `length` before firmware depends on it." */
struct OuterPacketHeader {
    uint16_t length;  /**< @endian le @doc "total outer-packet length INCLUDING the trailing CRC. Inconsistently populated -- may be a real total or a zero pad; the CRC is computed as if this were 0x0000 either way." */
    uint8_t  version; /**< @enum OD_CONFIG_VERSION @doc "config-format MAJOR version = OD_CONFIG_VERSION (1). The minor is not carried in the header." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct OuterPacketHeader) == 3, "OuterPacketHeader wire size");

/** @packet single  @doc "Header preceding each config payload inside the outer
 *  packet. The payload that follows is a fixed-size struct whose size and layout
 *  are determined entirely by `id` (see ConfigPacketType); there is no per-packet
 *  length field -- sizeof the selected struct IS the payload length." */
struct SinglePacketHeader {
    uint8_t number; /**< @doc "0-based sequential packet index within the outer packet." */
    uint8_t id;     /**< @enum ConfigPacketType @doc "packet-type id selecting the payload struct." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct SinglePacketHeader) == 2, "SinglePacketHeader wire size");

/* ==========================================================================
 * SECTION 2 -- CONFIG PACKET-TYPE IDS
 * ==========================================================================
 * The value of SinglePacketHeader.id. Canonical replacement for the per-repo
 * CONFIG_PKT_* macros and the comment-only table in opendisplay_protocol.h
 * SECTION 1. Each id's payload struct, required-ness, and repeatability are
 * documented on the struct itself in SECTION 4 (via @packet / @required /
 * @repeatable), not duplicated here.
 * ========================================================================== */

/** @enum ConfigPacketType  @width 1
 *  @doc "Config TLV packet-type identifiers (SinglePacketHeader.id)." */
enum ConfigPacketType {
    OD_PKT_SYSTEM         = 0x01, /**< @doc "system_config (required, singleton)" */
    OD_PKT_MANUFACTURER   = 0x02, /**< @doc "manufacturer_data (required, singleton)" */
    OD_PKT_POWER          = 0x04, /**< @doc "power_option (required, singleton)" */
    OD_PKT_DISPLAY        = 0x20, /**< @doc "display (repeatable, max 4)" */
    OD_PKT_LED            = 0x21, /**< @doc "led (repeatable, max 4)" */
    OD_PKT_SENSOR         = 0x23, /**< @doc "sensor_data (repeatable, max 4)" */
    OD_PKT_DATA_BUS       = 0x24, /**< @doc "data_bus (repeatable, max 4)" */
    OD_PKT_BINARY_INPUT   = 0x25, /**< @doc "binary_inputs (repeatable, max 4)" */
    OD_PKT_WIFI           = 0x26, /**< @doc "wifi_config (singleton)" */
    OD_PKT_SECURITY       = 0x27, /**< @doc "security_config (singleton)" */
    OD_PKT_TOUCH          = 0x28, /**< @doc "touch_controller (repeatable, max 4)" */
    OD_PKT_BUZZER         = 0x29, /**< @doc "buzzer (repeatable, max 4). Canonical name is BUZZER (matches Silabs), resolving the Silabs BUZZER vs NRF54/yaml PASSIVE_BUZZER split; config.yaml's packet name (passive_buzzer) is regenerated to buzzer." */
    OD_PKT_NFC            = 0x2A, /**< @doc "nfc_config (repeatable, max 2)" */
    OD_PKT_FLASH          = 0x2B, /**< @doc "flash_config (repeatable, max 2)" */
    OD_PKT_DATA_EXTENDED  = 0x2C  /**< @doc "data_extended (singleton)" */
};

/* ==========================================================================
 * SECTION 3 -- SHARED SCALAR ENUMS
 * ==========================================================================
 * Tiny enums reused across several packets. OD_PIN_UNUSED lives in SECTION 1.
 * ========================================================================== */

/** @enum ActiveLevel  @width 1
 *  @doc "GPIO active-level polarity. Bound from NfcConfig.field_detect_active,
 *  NfcConfig.power_active, FlashConfig.power_active." */
enum ActiveLevel {
    OD_ACTIVE_LOW  = 0, /**< @doc "signal asserted / present when the pin is LOW" */
    OD_ACTIVE_HIGH = 1  /**< @doc "signal asserted / present when the pin is HIGH" */
};

/* ==========================================================================
 * SECTION 4 -- CONFIG PAYLOAD PACKETS  (in packet-id order)
 * ==========================================================================
 * Each block below = the packet's value enums, then its bitfield groups, then
 * the packed payload struct + a sizeof assert. Read top-to-bottom, in wire
 * order, to decode a whole config blob. All layouts are the reconciled SUPERSET
 * of the firmware repos and config.yaml (ble_proto v1.4): every field any source
 * promoted out of reserved[] is named; reserved[] shrinks; total sizes are
 * unchanged from the on-wire format.
 * ========================================================================== */

/* -----------------------------------------------------------------------
 * 0x01  system_config
 * ----------------------------------------------------------------------- */

/** @enum ICType  @width 2  @doc "Host MCU/IC of the device (SystemConfig.ic_type)." */
enum ICType {
    OD_IC_TYPE_NRF52840              = 1, /**< @doc "nRF52840-based boards" */
    OD_IC_TYPE_ESP32S3               = 2, /**< @doc "ESP32-S3-based boards" */
    OD_IC_TYPE_ESP32C3               = 3, /**< @doc "ESP32-C3-based boards" */
    OD_IC_TYPE_ESP32C6               = 4, /**< @doc "ESP32-C6-based boards" */
    OD_IC_TYPE_NRF52811              = 5, /**< @doc "nRF52811-based boards" */
    OD_IC_TYPE_EFR32BG22C222F352GM40 = 6, /**< @doc "Silicon Labs EFR32BG22 boards" */
    OD_IC_TYPE_NRF54L15              = 7, /**< @doc "Seeed XIAO nRF54L15 (Zephyr)" */
    OD_IC_TYPE_NRF54LM20             = 8  /**< @doc "Seeed XIAO nRF54LM20A (Zephyr)" */
};

/** @enum ManufacturerId  @width 2  @doc "Device manufacturer (ManufacturerData.manufacturer_id).
 *  Board-name tables (board_type) are intentionally NOT enumerated here -- they
 *  live only in config.yaml; board_type stays a raw u8 field." */
enum ManufacturerId {
    OD_MANUFACTURER_DIY         = 0, /**< @doc "self-built / DIY devices" */
    OD_MANUFACTURER_SEEED       = 1, /**< @doc "Seeed Studio" */
    OD_MANUFACTURER_WAVESHARE   = 2, /**< @doc "Waveshare Electronics" */
    OD_MANUFACTURER_SOL         = 3, /**< @doc "SOL" */
    OD_MANUFACTURER_OPENDISPLAY = 4  /**< @doc "OpenDisplay (free giveaways only)" */
};

/* SystemConfig.communication_modes @bits CommunicationModes (bits 3-7 reserved). */
#define OD_COMM_MODE_BLE               (1u << 0) /* @doc "BLE transfer supported" */
#define OD_COMM_MODE_OEPL              (1u << 1) /* @doc "OEPL (OpenEPaperLink) transfer supported" */
#define OD_COMM_MODE_WIFI              (1u << 2) /* @doc "WiFi transfer supported" */

/* SystemConfig.device_flags @bits DeviceFlags (bits 5-7 reserved). */
#define OD_DEVICE_FLAG_PWR_PIN         (1u << 0) /* @doc "device has external power management for the display etc." */
#define OD_DEVICE_FLAG_XIAO_INIT       (1u << 1) /* @doc "apply Seeed XIAO low-power init" */
#define OD_DEVICE_FLAG_WS_PP_INIT      (1u << 2) /* @doc "apply Waveshare PhotoPainter low-power init" */
#define OD_DEVICE_FLAG_PWR_LATCH       (1u << 3) /* @doc "self-holding MOSFET latch on pwr_pin_2; optional long-press shutdown button on pwr_pin_3" */
#define OD_DEVICE_FLAG_PWR_LATCH_DFF   (1u << 4) /* @doc "74AHC1G79 D-FF latch: pwr_pin_2=D (hold), pwr_pin_3=CP (clock); command 0x0052 releases latch" */

/** @struct SystemConfig  @packet 0x01  @required
 *  @doc "Host IC identity, communication capabilities, and power-management pins.
 *  Required singleton. 22 bytes." */
struct SystemConfig {
    uint16_t ic_type;             /**< @enum ICType @endian le @doc "host MCU/IC of this device." */
    uint8_t  communication_modes; /**< @bits CommunicationModes @doc "supported transfer modes (bitfield)." */
    uint8_t  device_flags;        /**< @bits DeviceFlags @doc "misc device / power-latch flags (bitfield)." */
    uint8_t  pwr_pin;             /**< @doc "primary power-management pin; 0xFF = not present." @default 0xFF */
    uint8_t  reserved[15];        /**< @reserved @doc "future use; must be 0." */
    uint8_t  pwr_pin_2;           /**< @doc "aux enable / battery-latch hold / pwr_latch_dff D input; 0 or 0xFF = firmware default." */
    uint8_t  pwr_pin_3;           /**< @doc "aux enable / battery-latch shutdown button / pwr_latch_dff CP clock; 0 or 0xFF = none/default." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct SystemConfig) == 22, "SystemConfig wire size");

/* -----------------------------------------------------------------------
 * 0x02  manufacturer_data
 * ----------------------------------------------------------------------- */

/** @struct ManufacturerData  @packet 0x02  @required
 *  @doc "Manufacturer id, board identity, and the 'simple config' preset indices
 *  the website UI applied. Required singleton. 22 bytes." */
struct ManufacturerData {
    uint16_t manufacturer_id;              /**< @enum ManufacturerId @endian le @doc "device manufacturer." */
    uint8_t  board_type;                   /**< @doc "board identifier; meaning depends on manufacturer_id. Value table lives in config.yaml only (NOT enumerated here) -- raw u8." */
    uint8_t  board_revision;               /**< @doc "board revision number." */
    uint16_t simple_config_driver_index;   /**< @endian le @doc "simple-config driver preset index (1-based; 0 = not set)." */
    uint16_t simple_config_display_index;  /**< @endian le @doc "simple-config display preset index (1-based; 0 = not set)." */
    uint16_t simple_config_power_index;    /**< @endian le @doc "simple-config power preset index (1-based; 0 = not set)." */
    uint8_t  simple_config_configured_at[6]; /**< @endian le @unit s @doc "48-bit little-endian Unix time (seconds) when simple config was last applied." */
    uint8_t  reserved[6];                  /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct ManufacturerData) == 22, "ManufacturerData wire size");

/* -----------------------------------------------------------------------
 * 0x04  power_option
 * ----------------------------------------------------------------------- */

/** @enum PowerMode  @width 1  @doc "Power source (PowerOption.power_mode)." */
enum PowerMode {
    OD_POWER_MODE_BATTERY = 1, /**< @doc "battery powered" */
    OD_POWER_MODE_USB     = 2, /**< @doc "USB powered" */
    OD_POWER_MODE_SOLAR   = 3  /**< @doc "solar powered" */
};

/** @enum CapacityEstimator  @width 1
 *  @doc "Battery chemistry model for state-of-charge estimation (PowerOption.capacity_estimator)." */
enum CapacityEstimator {
    OD_CAPACITY_EST_LI_ION           = 1, /**< @doc "1S Li-ion" */
    OD_CAPACITY_EST_LIFEPO4          = 2, /**< @doc "1S LiFePO4" */
    OD_CAPACITY_EST_SUPERCAP         = 3, /**< @doc "2S supercapacitor (4.5 V max, 3.6 V cutoff)" */
    OD_CAPACITY_EST_LITHIUM_PRIMARY  = 4, /**< @doc "lithium primary (non-rechargeable)" */
    OD_CAPACITY_EST_SEEED_LI_ION     = 5  /**< @doc "1S Li-ion, Seeed reTerminal E-series discharge curve" */
};

/* PowerOption.sleep_flags @bits SleepFlags (bits 1-7 reserved).
 * Firmware ships bit0; config.yaml still lists it reserved_0 (yaml is behind). */
#define OD_SLEEP_FLAG_BUTTON_WAKE_DISABLE   (1u << 0) /* @doc "opt out of button wake from timer deep sleep (buttons sharing wake-hostile pads)" */

/* PowerOption.battery_sense_flags @bits BatterySenseFlags (bits 1-7 reserved).
 * Firmware ships bit0; config.yaml still lists it reserved_0 (yaml is behind). */
#define OD_BATTERY_SENSE_FLAG_ENABLE_INVERTED (1u << 0) /* @doc "battery-sense enable is active-low (e.g. XIAO ~READ_BAT on P0.14)" */

/* PowerOption.charger_flags @bits ChargerFlags (bits 2-7 reserved). */
#define OD_CHARGER_FLAG_ENABLE_ACTIVE_LOW   (1u << 0) /* @doc "charge-enable (/CE) is active-low" */
#define OD_CHARGER_FLAG_STATE_ACTIVE_LOW    (1u << 1) /* @doc "charge-state (BQ25616 STAT) is active-low: charging when LOW" */

/** @struct PowerOption  @packet 0x04  @required
 *  @doc "Power source, battery model, sleep/advertising timing, battery-sense and
 *  charger pins, and EPD keep-alive. Required singleton. 30 bytes." */
struct PowerOption {
    uint8_t  power_mode;               /**< @enum PowerMode @doc "power source type." */
    uint8_t  battery_capacity_mah[3];  /**< @endian le @unit mAh @doc "battery capacity, 24-bit LE; 0 = unknown." */
    uint16_t sleep_timeout_ms;         /**< @endian le @unit ms @doc "nominal awake/advertising time before sleep (actual may be shorter)." */
    uint8_t  tx_power;                  /**< @doc "BLE transmit-power setting (platform-specific units)." */
    uint8_t  sleep_flags;              /**< @bits SleepFlags @doc "sleep-related flags (bitfield)." */
    uint8_t  battery_sense_pin;        /**< @doc "ADC pin measuring battery voltage; 0xFF = none." @default 0xFF */
    uint8_t  battery_sense_enable_pin; /**< @doc "pin enabling the battery-sense divider; 0xFF = none." @default 0xFF */
    uint8_t  battery_sense_flags;      /**< @bits BatterySenseFlags @doc "battery-sense flags (bitfield)." */
    uint8_t  capacity_estimator;       /**< @enum CapacityEstimator @doc "battery chemistry model." */
    uint16_t voltage_scaling_factor;   /**< @endian le @doc "ADC voltage scaling / divider factor (implementation-specific)." */
    uint32_t deep_sleep_current_ua;    /**< @endian le @unit uA @doc "board deep-sleep current excl. MCU; 0 = unknown." */
    uint16_t deep_sleep_time_seconds;  /**< @endian le @unit s @doc "timer deep-sleep duration (ESP32 on battery); 0 = disabled." */
    uint8_t  charge_enable_pin;        /**< @doc "BQ25616 charge-enable GPIO; 0 or 0xFF = unused." @default 0xFF */
    uint8_t  charge_state_pin;         /**< @doc "BQ25616 charge-state input GPIO; 0 or 0xFF = unused." @default 0xFF */
    uint8_t  charger_flags;            /**< @bits ChargerFlags @doc "charger polarity flags (bitfield)." */
    uint16_t min_wake_time_seconds;    /**< @endian le @unit s @doc "min awake window after first boot / button wake; 0 = default 120 s." */
    uint8_t  screen_timeout_seconds;   /**< @unit s @min 0 @max 30 @doc "EPD keep-alive: seconds the panel stays powered after a refresh before shutdown (values >30 clamped; 0 = power off immediately; forced 0 when an AXP2101 PMIC is present)." */
    uint8_t  reserved[4];              /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PowerOption) == 30, "PowerOption wire size");

/* -----------------------------------------------------------------------
 * 0x20  display   (external-related enums PanelIC + ColorScheme inline)
 * ----------------------------------------------------------------------- */

/** @enum DisplayTechnology  @width 1  @doc "Display technology (DisplayConfig.display_technology)." */
enum DisplayTechnology {
    OD_DISPLAY_TECH_UNDEFINED  = 0, /**< @doc "undefined / unspecified" */
    OD_DISPLAY_TECH_E_PAPER    = 1, /**< @doc "e-paper / ESL" */
    OD_DISPLAY_TECH_LCD        = 2, /**< @doc "LCD" */
    OD_DISPLAY_TECH_LED_MATRIX = 3  /**< @doc "LED-matrix display" */
};

/** @enum Rotation  @width 1  @doc "Physical panel rotation (DisplayConfig.rotation)." */
enum Rotation {
    OD_ROTATION_0   = 0, /**< @doc "0 degrees" */
    OD_ROTATION_90  = 1, /**< @doc "90 degrees" */
    OD_ROTATION_180 = 2, /**< @doc "180 degrees" */
    OD_ROTATION_270 = 3  /**< @doc "270 degrees" */
};

/** @enum PartialUpdateSupport  @width 1  @doc "Panel partial-update capability (DisplayConfig.partial_update_support)." */
enum PartialUpdateSupport {
    OD_PARTIAL_UPDATE_NONE       = 0, /**< @doc "only full updates supported" */
    OD_PARTIAL_UPDATE_SUPPORTED  = 1, /**< @doc "partial updates supported" */
    OD_PARTIAL_UPDATE_FULL_FRAME = 2  /**< @doc "partial supported but a full-frame stream is required" */
};

/** @enum ColorScheme  @width 1
 *  @external mirror: epaper-dithering packages/rust/core/src/palettes.rs (integer
 *  values MUST match; the crate declares itself a firmware mirror). informed-by:
 *  bb_epaper. The value->plane/packing meaning is OpenDisplay-owned here.
 *  @doc "display color/plane scheme (DisplayConfig.color_scheme); drives plane
 *  count and pixel packing." */
enum ColorScheme {
    OD_COLOR_SCHEME_MONO         = 0,   /**< @doc "1bpp black/white" */
    OD_COLOR_SCHEME_BWR          = 1,   /**< @doc "black/white/red" */
    OD_COLOR_SCHEME_BWY          = 2,   /**< @doc "black/white/yellow" */
    OD_COLOR_SCHEME_BWRY         = 3,   /**< @doc "black/white/red/yellow" */
    OD_COLOR_SCHEME_BWGBRY       = 4,   /**< @doc "Spectra 6-color (black/white/green/blue/red/yellow)" */
    OD_COLOR_SCHEME_GRAY4        = 5,   /**< @doc "2bpp 4-level gray" */
    OD_COLOR_SCHEME_GRAY16       = 6,   /**< @doc "4bpp 16-level gray" */
    OD_COLOR_SCHEME_SEVEN_COLOR  = 7,   /**< @doc "7-color (Spectra/ACeP 7, config.yaml '7color')" @changed "2.0: value 7 is SEVEN_COLOR; the former COLOR_SCHEME_GRAY8=7 was a mistake (gray8 is not a real scheme) and is removed -- not renumbered." */
    OD_COLOR_SCHEME_BWGBRY_SPLIT = 8,   /**< @doc "Spectra 6 nibbles, left-half plane then right-half plane (dual-CS panels, no device framebuffer)" */
    OD_COLOR_SCHEME_RGB565       = 100, /**< @doc "RGB565 (non-epaper)" */
    OD_COLOR_SCHEME_RGB888       = 101, /**< @doc "RGB888 (non-epaper)" */
    OD_COLOR_SCHEME_RGB16BPC     = 102  /**< @doc "16 bits per channel (non-epaper)" */
};

/** @enum PanelIC  @width 2
 *  @external bb_epaper (0-76 names track bb_epaper EP* panel identifiers); the
 *  1000-1030 range comes from the M3 / EPD-nRF5 driver line; 3000+ from the
 *  FastEPD IT8951 / OpenDisplay runtime. Wire values are OpenDisplay-OWNED; each
 *  firmware maps value -> its bb_epaper EP* constant via a repo-local
 *  opendisplay_epd_map.c (NOT part of this header; bb_epaper never dictates the
 *  wire values). @doc "display controller / panel type (DisplayConfig.panel_ic_type).
 *  Only representative values are named here; the full list lives in config.yaml." */
enum PanelIC {
    OD_PANEL_IC_EP_PANEL_UNDEFINED       = 0   , /**< @doc "undefined / unknown panel" */
    OD_PANEL_IC_EP42_400X300             = 1   , /**< @doc "WFT0420CZ15 4.2\" B/W" */
    OD_PANEL_IC_EP42B_400X300            = 2   , /**< @doc "DEPG0420BN / GDEY042T81 4.2\" B/W" */
    OD_PANEL_IC_EP213_122X250            = 3   , /**< @doc "Waveshare 2.13\"" */
    OD_PANEL_IC_EP213B_122X250           = 4   , /**< @doc "GDEY0213B74 (Inky pHAT 2.13\" B/W newer)" */
    OD_PANEL_IC_EP293_128X296            = 5   , /**< @doc "128x296 panel" */
    OD_PANEL_IC_EP294_128X296            = 6   , /**< @doc "Waveshare newer 2.9\" 1-bit 128x296" */
    OD_PANEL_IC_EP295_128X296            = 7   , /**< @doc "harvested Solum 2.9\" BW ESLs" */
    OD_PANEL_IC_EP295_128X296_4GRAY      = 8   , /**< @doc "4-gray variant of EP295" */
    OD_PANEL_IC_EP266_152X296            = 9   , /**< @doc "GDEY0266T90" */
    OD_PANEL_IC_EP102_80X128             = 10  , /**< @doc "GDEW0102T4" */
    OD_PANEL_IC_EP27B_176X264            = 11  , /**< @doc "GDEY027T91" */
    OD_PANEL_IC_EP29R_128X296            = 12  , /**< @doc "tricolor 2.9 panel" */
    OD_PANEL_IC_EP122_192X176            = 13  , /**< @doc "GDEM0122T61" */
    OD_PANEL_IC_EP154R_152X152           = 14  , /**< @doc "1.54\" B/W/R" */
    OD_PANEL_IC_EP42R_400X300            = 15  , /**< @doc "400x300 tricolor" */
    OD_PANEL_IC_EP42R2_400X300           = 16  , /**< @doc "GDEQ042Z21" */
    OD_PANEL_IC_EP37_240X416             = 17  , /**< @doc "GDEY037T03" */
    OD_PANEL_IC_EP37B_240X416            = 18  , /**< @doc "CROWPANEL 3.7\"" */
    OD_PANEL_IC_EP213_104X212            = 19  , /**< @doc "InkyPHAT 2.13 B/W" */
    OD_PANEL_IC_EP75_800X480             = 20  , /**< @doc "GDEY075T7 (older version)" */
    OD_PANEL_IC_EP75_800X480_4GRAY       = 21  , /**< @doc "GDEW075T7 (older) 4-gray mode" */
    OD_PANEL_IC_EP75_800X480_4GRAY_V2    = 22  , /**< @doc "GDEY075T7 older panel, darker grays" */
    OD_PANEL_IC_EP29_128X296             = 23  , /**< @doc "Pimoroni Badger2040" */
    OD_PANEL_IC_EP29_128X296_4GRAY       = 24  , /**< @doc "Pimoroni Badger2040 4-gray" */
    OD_PANEL_IC_EP213R_122X250           = 25  , /**< @doc "Inky pHAT 2.13 B/W/R" */
    OD_PANEL_IC_EP154_200X200            = 26  , /**< @doc "Waveshare 2.0\" B/W" */
    OD_PANEL_IC_EP154B_200X200           = 27  , /**< @doc "DEPG01540BN" */
    OD_PANEL_IC_EP266YR_184X360          = 28  , /**< @doc "GDEY0266F51" */
    OD_PANEL_IC_EP29YR_128X296           = 29  , /**< @doc "GDEY029F51" */
    OD_PANEL_IC_EP29YR_168X384           = 30  , /**< @doc "GDEY029F51H" */
    OD_PANEL_IC_EP583_648X480            = 31  , /**< @doc "DEPG0583BN" */
    OD_PANEL_IC_EP296_128X296            = 32  , /**< @doc "Waveshare 2.9\" 128x296 B/W V2" */
    OD_PANEL_IC_EP26R_152X296            = 33  , /**< @doc "Solum 2.6\" B/W/R" */
    OD_PANEL_IC_EP73_800X480             = 34  , /**< @doc "GEDY073D46 (slower, EOL 7-color)" */
    OD_PANEL_IC_EP73_SPECTRA_800X480     = 35  , /**< @doc "Spectra 6/7-color 800x480" */
    OD_PANEL_IC_EP74R_640X384            = 36  , /**< @doc "640x384 panel" */
    OD_PANEL_IC_EP583R_600X448           = 37  , /**< @doc "4-bits-per-pixel 5.83\" panel" */
    OD_PANEL_IC_EP75R_800X480            = 38  , /**< @doc "Waveshare 800x480 3-color" */
    OD_PANEL_IC_EP426_800X480            = 39  , /**< @doc "Waveshare 4.26\" B/W 800x480" */
    OD_PANEL_IC_EP426_800X480_4GRAY      = 40  , /**< @doc "4.26\" 2-bit grayscale mode" */
    OD_PANEL_IC_EP29R2_128X296           = 41  , /**< @doc "Adafruit 2.9\" Tricolor FeatherWing" */
    OD_PANEL_IC_EP41_640X400             = 42  , /**< @doc "EInk ED040TC1 SPI UC81xx" */
    OD_PANEL_IC_EP81_SPECTRA_1024X576    = 43  , /**< @doc "Spectra 8.1\" 1024x576 6-color" */
    OD_PANEL_IC_EP7_960X640              = 44  , /**< @doc "ED070EC1" */
    OD_PANEL_IC_EP213R2_122X250          = 45  , /**< @doc "UC8151 3-color 2.13\"" */
    OD_PANEL_IC_EP29Z_128X296            = 46  , /**< @doc "SSD1680 (CrowPanel 2.9\")" */
    OD_PANEL_IC_EP29Z_128X296_4GRAY      = 47  , /**< @doc "SSD1680 4-gray" */
    OD_PANEL_IC_EP213Z_122X250           = 48  , /**< @doc "SSD1680 CrowPanel 2.13\"" */
    OD_PANEL_IC_EP213Z_122X250_4GRAY     = 49  , /**< @doc "CrowPanel 2.13\" 4-gray mode" */
    OD_PANEL_IC_EP154Z_152X152           = 50  , /**< @doc "CrowPanel 1.54\"" */
    OD_PANEL_IC_EP579_792X272            = 51  , /**< @doc "CrowPanel 5.79\"" */
    OD_PANEL_IC_EP213YR_122X250          = 52  , /**< @doc "GDEY0213F52" */
    OD_PANEL_IC_EP37YR_240X416           = 53  , /**< @doc "GDEM037F51" */
    OD_PANEL_IC_EP35YR_184X384           = 54  , /**< @doc "GDEM035F51" */
    OD_PANEL_IC_EP397YR_800X480          = 55  , /**< @doc "GDEM0397F81" */
    OD_PANEL_IC_EP154YR_200X200          = 56  , /**< @doc "GDEM0154F51H" */
    OD_PANEL_IC_EP266YR2_184X360         = 57  , /**< @doc "GDEY0266F52H" */
    OD_PANEL_IC_EP42YR_400X300           = 58  , /**< @doc "GDEM042F52" */
    OD_PANEL_IC_EP75_800X480_GEN2        = 59  , /**< @doc "GEDY075-D2 (Waveshare/Xiao V2 panels)" */
    OD_PANEL_IC_EP75_800X480_4GRAY_GEN2  = 60  , /**< @doc "GDEY075T7-D2 (newer) 4-gray mode" */
    OD_PANEL_IC_EP215YR_160X296          = 61  , /**< @doc "Waveshare 2.15\" 4-color" */
    OD_PANEL_IC_EP1085_1360X480          = 62  , /**< @doc "GDEM1085T51" */
    OD_PANEL_IC_EP31_240X320             = 63  , /**< @doc "GDEQ031T10 LilyGo T-Deck Pro" */
    OD_PANEL_IC_EP75YR_800X480           = 64  , /**< @doc "EP75YR 800x480 7-color" */
    OD_PANEL_IC_EP133A_SPECTRA_1200X1600 = 66  , /**< @doc "T133A01 Spectra 6 13.3\" dual-controller (Seeed reTerminal E1004). config.yaml catches up (it skips 65-66 today); value 65 stays reserved." @since 1.4 */
    OD_PANEL_IC_EP154_200X200_4GRAY      = 67  , /**< @doc "Waveshare 1.54\" 4-gray" */
    OD_PANEL_IC_EP42B_400X300_4GRAY      = 68  , /**< @doc "DEPG0420BN / GDEY042T81 4-gray" */
    OD_PANEL_IC_EP397_800X480            = 69  , /**< @doc "GDEM0397T81P" */
    OD_PANEL_IC_EP397_800X480_4GRAY      = 70  , /**< @doc "GDEM0397T81P 4-gray" */
    OD_PANEL_IC_EP368_792X528            = 71  , /**< @doc "xteink X3 3.68\" B/W" */
    OD_PANEL_IC_EP368_792X528_4GRAY      = 72  , /**< @doc "xteink X3 3.68\" 4-gray" */
    OD_PANEL_IC_EP213ZZ_122X250          = 73  , /**< @doc "LilyGo T3S3 2.13\"" */
    OD_PANEL_IC_EP40_SPECTRA_400X600     = 74  , /**< @doc "GDEP040E01 Spectra 6 4\" 400x600" */
    OD_PANEL_IC_EP27_176X264             = 75  , /**< @doc "Badger2350 B/W 2.7\"" */
    OD_PANEL_IC_EP27_176X264_4GRAY       = 76  , /**< @doc "Badger2350 4-gray 2.7\"" */
    OD_PANEL_IC_UC8176_420_BW            = 1000, /**< @doc "UC8176 4.2\" B/W (M3 / EPD-nRF5 driver line, values 1000-1030)" */
    OD_PANEL_IC_SSD1619_420_BWR          = 1001, /**< @doc "SSD1619 4.2\" B/W/R" */
    OD_PANEL_IC_UC8176_420_BWR           = 1002, /**< @doc "UC8176 4.2\" B/W/R" */
    OD_PANEL_IC_SSD1619_420_BW           = 1003, /**< @doc "SSD1619 4.2\" B/W" */
    OD_PANEL_IC_JD79668_420_BWRY         = 1004, /**< @doc "JD79668 4.2\" B/W/R/Y" */
    OD_PANEL_IC_UC8179_750_BW            = 1005, /**< @doc "UC8179 7.5\" B/W" */
    OD_PANEL_IC_UC8179_750_BWR           = 1006, /**< @doc "UC8179 7.5\" B/W/R" */
    OD_PANEL_IC_UC8159_750_LOW_BW        = 1007, /**< @doc "UC8159 7.5\" Low Power B/W" */
    OD_PANEL_IC_UC8159_750_LOW_BWR       = 1008, /**< @doc "UC8159 7.5\" Low Power B/W/R" */
    OD_PANEL_IC_SSD1677_750_HD_BW        = 1009, /**< @doc "SSD1677 7.5\" HD B/W" */
    OD_PANEL_IC_SSD1677_750_HD_BWR       = 1010, /**< @doc "SSD1677 7.5\" HD B/W/R" */
    OD_PANEL_IC_JD79665_750_BWRY         = 1011, /**< @doc "JD79665 7.5\" B/W/R/Y" */
    OD_PANEL_IC_JD79665_583_BWRY         = 1012, /**< @doc "JD79665 5.83\" B/W/R/Y" */
    OD_PANEL_IC_UC8151_029_BW            = 1013, /**< @doc "UC8151 2.9\" 168x384 B/W" */
    OD_PANEL_IC_UC8151_029_BWR           = 1014, /**< @doc "UC8151 2.9\" 168x384 B/W/R" */
    OD_PANEL_IC_SSD1619_029_BW           = 1015, /**< @doc "SSD1619 2.9\" 168x384 B/W" */
    OD_PANEL_IC_SSD1619_029_BWR          = 1016, /**< @doc "SSD1619 2.9\" 168x384 B/W/R" */
    OD_PANEL_IC_SSD1619_016_BW           = 1017, /**< @doc "SSD1619 1.6\" 200x200 B/W" */
    OD_PANEL_IC_SSD1619_016_BWR          = 1018, /**< @doc "SSD1619 1.6\" 200x200 B/W/R" */
    OD_PANEL_IC_SSD1619_022_BW           = 1019, /**< @doc "SSD1619 2.2\" 240x320 B/W" */
    OD_PANEL_IC_SSD1619_022_BWR          = 1020, /**< @doc "SSD1619 2.2\" 240x320 B/W/R" */
    OD_PANEL_IC_SSD1619_026_BW           = 1021, /**< @doc "SSD1619 2.6\" 296x152 B/W" */
    OD_PANEL_IC_SSD1619_026_BWR          = 1022, /**< @doc "SSD1619 2.6\" 296x152 B/W/R" */
    OD_PANEL_IC_UC8151_027_BW            = 1023, /**< @doc "UC8151 2.7\" 200x300 B/W" */
    OD_PANEL_IC_UC8151_027_BWR           = 1024, /**< @doc "UC8151 2.7\" 200x300 B/W/R" */
    OD_PANEL_IC_UC43_430_BW              = 1025, /**< @doc "UC variant 4.3\" 152x522 B/W" */
    OD_PANEL_IC_UC43_430_BWR             = 1026, /**< @doc "UC variant 4.3\" 152x522 B/W/R" */
    OD_PANEL_IC_SSD1619_013_BW           = 1027, /**< @doc "SSD1619 1.3\" 144x200 B/W" */
    OD_PANEL_IC_SSD1619_013_BWR          = 1028, /**< @doc "SSD1619 1.3\" 144x200 B/W/R" */
    OD_PANEL_IC_SSD1619_022_LITE_BW      = 1029, /**< @doc "SSD1619 M3 Lite 2.2\" 250x128 B/W" */
    OD_PANEL_IC_SSD1619_022_LITE_BWR     = 1030, /**< @doc "SSD1619 M3 Lite 2.2\" 250x128 B/W/R" */
    OD_PANEL_IC_ED103TC2_1872X1404       = 3000, /**< @doc "E Ink ED103TC2 + IT8951 (10.3\", 1872x1404, 1bpp; FastEPD IT8951 path, values 3000+)" */
    OD_PANEL_IC_ED103TC2_1872X1404_4GRAY = 3001 /**< @doc "same panel as 3000; 4bpp (16-level gray via FastEPD)" */
};

/* DisplayConfig.transmission_modes @bits TransmissionModes (bits 5-6 reserved --
 * named as placeholders below; bit7 is defined above the gap). */
#define OD_TRANSMISSION_MODE_STREAMING_DECOMPRESSION (1u << 0) /* @doc "streaming zlib inflate, 512-byte DEFLATE window (requires zip). Canonical name resolves the Firmware/Silabs ZIPXL vs NRF54/yaml streaming_decompression split." */
#define OD_TRANSMISSION_MODE_ZIP                     (1u << 1) /* @doc "zip-compressed transfer (full window)" */
#define OD_TRANSMISSION_MODE_G5                      (1u << 2) /* @doc "Group 5 compression (not yet implemented)" */
#define OD_TRANSMISSION_MODE_DIRECT_WRITE            (1u << 3) /* @doc "direct-write mode (bufferless)" */
#define OD_TRANSMISSION_MODE_PIPE_WRITE              (1u << 4) /* @doc "PIPE-write mode: faster transfer via sliding-window pipeline" */
#define OD_TRANSMISSION_MODE_RESERVED_5              (1u << 5) /* @reserved @doc "reserved; must be 0 (placeholder name for a future transmission mode)" */
#define OD_TRANSMISSION_MODE_RESERVED_6              (1u << 6) /* @reserved @doc "reserved; must be 0 (placeholder name for a future transmission mode)" */
#define OD_TRANSMISSION_MODE_CLEAR_ON_BOOT           (1u << 7) /* @doc "Clear screen on boot. Synonymous with config.yaml's no_boot_text / NO_BOOT_TEXT -- same bit, same behavior (clearing on boot is why no boot text shows)." */

/** @struct DisplayConfig  @packet 0x20  @repeatable max=4
 *  @doc "Per-panel configuration: geometry, controller, SPI pins, color scheme,
 *  and supported transmission modes. Up to 4 instances. 46 bytes." */
struct DisplayConfig {
    uint8_t  instance_number;        /**< @doc "0-based display index (unique per display block)." */
    uint8_t  display_technology;     /**< @enum DisplayTechnology @doc "panel technology." */
    uint16_t panel_ic_type;          /**< @enum PanelIC @endian le @doc "display controller / panel type." */
    uint16_t pixel_width;            /**< @endian le @unit px @doc "panel pixel width." */
    uint16_t pixel_height;           /**< @endian le @unit px @doc "panel pixel height." */
    uint16_t active_width_mm;        /**< @endian le @unit mm @doc "active area width." */
    uint16_t active_height_mm;       /**< @endian le @unit mm @doc "active area height." */
    uint16_t legacy_tag_type;        /**< @endian le @doc "OEPL legacy tag type (optional; 0 if unused). Canonical snake_case of config.yaml legacy_tagtype / firmware tag_type." */
    uint8_t  rotation;               /**< @enum Rotation @doc "physical rotation." */
    uint8_t  reset_pin;              /**< @doc "panel reset GPIO; 0xFF = none." @default 0xFF */
    uint8_t  busy_pin;               /**< @doc "panel busy-status GPIO; 0xFF = none." @default 0xFF */
    uint8_t  dc_pin;                 /**< @doc "data/command select; doubles as SPI MISO on OpenDisplay-runtime IT8951 / ED103 panels." */
    uint8_t  cs_pin;                 /**< @doc "SPI chip-select; 0xFF = none." @default 0xFF */
    uint8_t  data_pin;               /**< @doc "data-out pin (SPI MOSI / data line)." */
    uint8_t  partial_update_support; /**< @enum PartialUpdateSupport @doc "partial-update capability." */
    uint8_t  color_scheme;           /**< @enum ColorScheme @doc "color/plane scheme." */
    uint8_t  transmission_modes;     /**< @bits TransmissionModes @doc "supported image/data transmission modes (bitfield)." */
    uint8_t  clk_pin;                /**< @doc "SPI SCLK pin." */
    uint8_t  cs_pin_2;               /**< @doc "second CS for panels split over 2 drivers (dual-controller Spectra); 0xFF = none. Canonical name (firmware called it reserved_pin_2)." @since 1.4 @default 0xFF */
    uint8_t  reserved_pin_3;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t  reserved_pin_4;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t  reserved_pin_5;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t  reserved_pin_6;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t  reserved_pin_7;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t  reserved_pin_8;         /**< @reserved @doc "spare pin; must be 0." */
    uint16_t full_update_mC;         /**< @endian le @unit mC @doc "energy consumed by a full refresh; 0 = unknown." */
    uint8_t  reserved[13];           /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct DisplayConfig) == 46, "DisplayConfig wire size");

/* -----------------------------------------------------------------------
 * 0x21  led
 * ----------------------------------------------------------------------- */

/** @enum LedType  @width 1  @doc "LED wiring type (LedConfig.led_type)." */
enum LedType {
    OD_LED_TYPE_RGB           = 0, /**< @doc "RGB LED (3 channels)" */
    OD_LED_TYPE_SINGLE        = 1, /**< @doc "single LED" */
    OD_LED_TYPE_RY            = 2, /**< @doc "red/yellow pair" */
    OD_LED_TYPE_FOUR_SEPARATE = 3  /**< @doc "four separate LEDs" */
};

/* LedConfig.led_flags @bits LedFlags (bits 4-7 reserved). */
#define OD_LED_FLAG_LED1_INVERT        (1u << 0) /* @doc "invert LED channel 1 polarity" */
#define OD_LED_FLAG_LED2_INVERT        (1u << 1) /* @doc "invert LED channel 2 polarity" */
#define OD_LED_FLAG_LED3_INVERT        (1u << 2) /* @doc "invert LED channel 3 polarity" */
#define OD_LED_FLAG_LED4_INVERT        (1u << 3) /* @doc "invert LED channel 4 polarity" */

/** @struct LedConfig  @packet 0x21  @repeatable max=4
 *  @doc "LED channel pins + invert flags. Up to 4 instances. 22 bytes. NOTE: the
 *  15-byte reserved[] tail also carries the runtime LED flash pattern (bytes
 *  0..11) written by CMD_LED_ACTIVATE 0x0073 -- see struct LedFlashPattern." */
struct LedConfig {
    uint8_t instance_number; /**< @doc "0-based LED-block index." */
    uint8_t led_type;        /**< @enum LedType @doc "LED wiring type." */
    uint8_t led_1_r;         /**< @doc "channel 1 (red) pin; 0xFF = unused." */
    uint8_t led_2_g;         /**< @doc "channel 2 (green) pin; 0xFF = unused." */
    uint8_t led_3_b;         /**< @doc "channel 3 (blue) pin; 0xFF = unused." */
    uint8_t led_4;           /**< @doc "channel 4 pin (if present); 0xFF = unused." */
    uint8_t led_flags;       /**< @bits LedFlags @doc "per-channel invert flags (bitfield)." */
    uint8_t reserved[15];    /**< @reserved @doc "future use; bytes 0..11 hold the runtime LedFlashPattern (see 0x0073). Must be 0 in a fresh config." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct LedConfig) == 22, "LedConfig wire size");

/* -----------------------------------------------------------------------
 * 0x23  sensor_data
 * ----------------------------------------------------------------------- */

/** @enum SensorType  @width 2  @doc "Sensor / PMIC type (SensorData.sensor_type)." */
enum SensorType {
    OD_SENSOR_TYPE_TEMPERATURE = 1, /**< @doc "generic temperature sensor" */
    OD_SENSOR_TYPE_HUMIDITY    = 2, /**< @doc "generic humidity sensor" */
    OD_SENSOR_TYPE_AXP2101     = 3, /**< @doc "AXP2101 power-management IC" */
    OD_SENSOR_TYPE_SHT40       = 4, /**< @doc "Sensirion SHT40 (I2C); writes 3 LE bytes into MSD dynamic data (21 bits: 0.1C, 0.1% RH)" */
    OD_SENSOR_TYPE_BQ27220     = 5  /**< @doc "TI BQ27220 fuel gauge (I2C 0x55); voltage in MSD bytes 14-15, one packed dynamic byte (bits 0-6 SOC%, bit7 charging)" */
};

/** @struct SensorData  @packet 0x23  @repeatable max=4
 *  @doc "A sensor bound to a data bus, with its MSD publish slot. Up to 4
 *  instances. 30 bytes." */
struct SensorData {
    uint8_t  instance_number;    /**< @doc "0-based sensor-block index." */
    uint16_t sensor_type;        /**< @enum SensorType @endian le @doc "sensor type." */
    uint8_t  bus_id;             /**< @doc "data_bus instance this sensor lives on." */
    uint8_t  i2c_addr_7bit;      /**< @doc "I2C 7-bit address; 0 or 0xFF = per-sensor default (SHT40 0x44, BQ27220 0x55)." @default 0xFF */
    uint8_t  msd_data_start_byte; /**< @doc "first dynamicreturndata index this sensor publishes into. SHT40 uses 3 bytes (0/0xFF = default 7); BQ27220 uses 1 packed byte (0xFF = do not publish)." @default 0xFF */
    uint8_t  reserved[24];       /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct SensorData) == 30, "SensorData wire size");

/* -----------------------------------------------------------------------
 * 0x24  data_bus
 * ----------------------------------------------------------------------- */

/** @enum BusType  @width 1  @doc "Bus protocol (DataBus.bus_type). Canonical
 *  replacement for the per-repo OD_BUS_TYPE_I2C macro." */
enum BusType {
    OD_BUS_TYPE_I2C = 1, /**< @doc "I2C bus" */
    OD_BUS_TYPE_SPI = 2  /**< @doc "SPI bus" */
};

/* DataBus.bus_flags @bits BusFlags -- all bits reserved today (must be 0). */

/* DataBus.pullups / .pulldowns @bits PinBitmap: bit N-1 = pin N (pin_1..pin_7);
 * bit 7 reserved. See the PinBitmap convention note above BinaryInputs. */

/** @struct DataBus  @packet 0x24  @repeatable max=4
 *  @doc "A shared I2C/SPI bus definition (pins, speed, pull config) other packets
 *  reference by instance. Up to 4 instances. 30 bytes." */
struct DataBus {
    uint8_t  instance_number; /**< @doc "0-based bus-block index; referenced by SensorData.bus_id, TouchController.bus_id, etc." */
    uint8_t  bus_type;        /**< @enum BusType @doc "bus protocol." */
    uint8_t  pin_1;           /**< @doc "pin 1 (SCL for I2C)." */
    uint8_t  pin_2;           /**< @doc "pin 2 (SDA for I2C)." */
    uint8_t  pin_3;           /**< @doc "pin 3 (aux / SPI)." */
    uint8_t  pin_4;           /**< @doc "pin 4 (aux)." */
    uint8_t  pin_5;           /**< @doc "pin 5 (aux)." */
    uint8_t  pin_6;           /**< @doc "pin 6 (aux)." */
    uint8_t  pin_7;           /**< @doc "pin 7 (aux)." */
    uint32_t bus_speed_hz;    /**< @endian le @unit Hz @doc "bus clock speed." */
    uint8_t  bus_flags;       /**< @bits BusFlags @doc "bus flags (all reserved today; must be 0)." */
    uint8_t  pullups;         /**< @bits PinBitmap @doc "internal pull-ups, bit N-1 = pin N (pin_1..pin_7); bit 7 reserved." */
    uint8_t  pulldowns;       /**< @bits PinBitmap @doc "internal pull-downs, bit N-1 = pin N (pin_1..pin_7); bit 7 reserved." */
    uint8_t  reserved[14];    /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct DataBus) == 30, "DataBus wire size");

/* -----------------------------------------------------------------------
 * 0x25  binary_inputs
 * ----------------------------------------------------------------------- *
 * PinBitmap convention (shared shape): a per-pin bitmap byte where bit index
 * i maps to pin i+1 (bit 0 = pin 1 ... bit 7 = pin 8). Used by pins_used,
 * invert, pullups, pulldowns, power_off_flags below and by DataBus pull config.
 * --------------------------------------------------------------------------- */

/** @enum InputType  @width 1  @doc "Binary input hardware type (BinaryInputs.input_type)." */
enum InputType {
    OD_INPUT_TYPE_BUTTON = 1, /**< @doc "momentary button" */
    OD_INPUT_TYPE_SWITCH = 2  /**< @doc "toggle switch" */
};

/** @enum InputDisplayAs  @width 1  @doc "How a UI should render the input (BinaryInputs.display_as)." */
enum InputDisplayAs {
    OD_INPUT_DISPLAY_AS_BUTTON = 1, /**< @doc "present as a button" */
    OD_INPUT_DISPLAY_AS_SWITCH = 2  /**< @doc "present as a switch" */
};

/** @struct BinaryInputs  @packet 0x25  @repeatable max=4
 *  @doc "Up to 8 button/switch pins per block, with per-pin used/invert/pull and
 *  long-press power-off config. Up to 4 instances. 30 bytes. Field names are the
 *  canonical snake_case (config.yaml inputpinN -> input_pin_N, isused -> pins_used;
 *  firmware called pins reserved_pin_N)." */
struct BinaryInputs {
    uint8_t instance_number;        /**< @doc "0-based input-block index." */
    uint8_t input_type;             /**< @enum InputType @doc "input hardware type." */
    uint8_t display_as;             /**< @enum InputDisplayAs @doc "how to render the input." */
    uint8_t input_pin_1;            /**< @doc "input pin 1 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_2;            /**< @doc "input pin 2 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_3;            /**< @doc "input pin 3 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_4;            /**< @doc "input pin 4 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_5;            /**< @doc "input pin 5 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_6;            /**< @doc "input pin 6 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_7;            /**< @doc "input pin 7 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t input_pin_8;            /**< @doc "input pin 8 GPIO; 0xFF = not configured." @default 0xFF */
    uint8_t pins_used;              /**< @bits PinBitmap @doc "which pins are active (bit N-1 = input_pin_N)." */
    uint8_t invert;                 /**< @bits PinBitmap @doc "per-pin logic invert (bit N-1 = input_pin_N)." */
    uint8_t pullups;                /**< @bits PinBitmap @doc "per-pin internal pull-up (bit N-1 = input_pin_N)." */
    uint8_t pulldowns;              /**< @bits PinBitmap @doc "per-pin internal pull-down (bit N-1 = input_pin_N)." */
    uint8_t button_data_byte_index; /**< @min 0 @max 10 @doc "dynamicreturndata index this block reports button state into (bounded integer, not an enum). 0xFF = not published." @default 0xFF */
    uint8_t power_off_flags;        /**< @bits PinBitmap @doc "bit N-1 = input_pin_N triggers long-press power-off (latched devices only; ignored otherwise)." */
    uint8_t power_off_hold_sec;     /**< @unit s @doc "hold time before power-off; 0 = default 3 s (latched devices only)." */
    uint8_t reserved[12];           /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct BinaryInputs) == 30, "BinaryInputs wire size");

/* -----------------------------------------------------------------------
 * 0x26  wifi_config
 * ----------------------------------------------------------------------- */

/** @enum WifiEncryptionType  @width 1  @doc "WiFi security (WifiConfig.encryption_type)." */
enum WifiEncryptionType {
    OD_WIFI_ENC_NONE = 0, /**< @doc "open network (no encryption)" */
    OD_WIFI_ENC_WEP  = 1, /**< @doc "WEP" */
    OD_WIFI_ENC_WPA  = 2, /**< @doc "WPA" */
    OD_WIFI_ENC_WPA2 = 3, /**< @doc "WPA2" */
    OD_WIFI_ENC_WPA3 = 4  /**< @doc "WPA3" */
};

/** @struct WifiConfig  @packet 0x26
 *  @doc "WiFi credentials plus optional upload server host/port. Singleton. 160
 *  bytes. This header is AHEAD of current firmware: server_host + server_port are
 *  promoted out of the former reserved[95] (a backward-compatible carve -- the
 *  bytes were must-be-zero, so old peers still interop; only a peer that writes a
 *  real port must honor the big-endian server_port). NOTE server_port is the ONE
 *  big-endian field in this otherwise little-endian struct." */
struct WifiConfig {
    uint8_t  ssid[32];        /**< @doc "WiFi SSID, null-terminated and zero-padded to 32 bytes." */
    uint8_t  password[32];    /**< @doc "WiFi password, null-terminated and zero-padded (empty for open networks)." */
    uint8_t  encryption_type; /**< @enum WifiEncryptionType @doc "network security type." */
    uint8_t  server_host[64]; /**< @doc "upload server URL/hostname, null-terminated and zero-padded to 64 bytes; empty = none. Promoted out of reserved[]." @since 1.4 */
    uint16_t server_port;     /**< @endian be @doc "upload server TCP port, BIG-ENDIAN (the single BE field here). 0 = default/none. Promoted out of reserved[] at former indices 64-65." @since 1.4 */
    uint8_t  reserved[29];    /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct WifiConfig) == 160, "WifiConfig wire size");

/* -----------------------------------------------------------------------
 * 0x27  security_config
 * ----------------------------------------------------------------------- */

/* SecurityConfig.flags @bits SecurityFlags (bits 6-7 reserved). */
#define OD_SECURITY_FLAG_REWRITE_ALLOWED    (1u << 0) /* @doc "allow unauthenticated config writes while encryption is enabled" */
#define OD_SECURITY_FLAG_SHOW_KEY_ON_SCREEN (1u << 1) /* @doc "show the encryption key on screen (future feature)" */
#define OD_SECURITY_FLAG_RESET_PIN_ENABLED  (1u << 2) /* @doc "reset pin enabled (must be set for the reset pin to work)" */
#define OD_SECURITY_FLAG_RESET_PIN_POLARITY (1u << 3) /* @doc "reset-pin polarity: 0 = LOW triggers, 1 = HIGH triggers" */
#define OD_SECURITY_FLAG_RESET_PIN_PULLUP   (1u << 4) /* @doc "enable pull-up on the reset pin" */
#define OD_SECURITY_FLAG_RESET_PIN_PULLDOWN (1u << 5) /* @doc "enable pull-down on the reset pin" */

/** @struct SecurityConfig  @packet 0x27
 *  @doc "Application-layer encryption master key and reset-pin config. Singleton.
 *  64 bytes. The AEAD envelope and auth handshake are specified in
 *  opendisplay_protocol.h; this packet only stores the pre-shared key + policy." */
struct SecurityConfig {
    uint8_t  encryption_enabled;      /**< @doc "0 = disabled (all commands work unauthenticated); 1 = enabled (sensitive commands require a session). If encryption_key is all-zero, encryption is off regardless." */
    uint8_t  encryption_key[16];      /**< @doc "AES-128 master key (16 bytes)." */
    uint16_t session_timeout_seconds; /**< @endian le @unit s @doc "session timeout; 0 = no timeout (persists until disconnect)." */
    uint8_t  flags;                   /**< @bits SecurityFlags @doc "security policy flags (bitfield)." */
    uint8_t  reset_pin;               /**< @doc "reset GPIO; when enabled and triggered, securely erases config and reboots." @default 0xFF */
    uint8_t  reserved[43];            /**< @reserved @doc "future use; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct SecurityConfig) == 64, "SecurityConfig wire size");

/* -----------------------------------------------------------------------
 * 0x28  touch_controller
 * ----------------------------------------------------------------------- */

/** @enum TouchIcType  @width 2  @doc "Touch controller IC (TouchController.touch_ic_type).
 *  Canonical replacement for the per-repo TOUCH_IC_* macros." */
enum TouchIcType {
    OD_TOUCH_IC_NONE  = 0, /**< @doc "disabled / no touch" */
    OD_TOUCH_IC_GT911 = 1  /**< @doc "Goodix GT911 (I2C)" */
};

/* TouchController.flags @bits TouchFlags (bits 3-7 reserved). */
#define OD_TOUCH_FLAG_INVERT_X         (1u << 0) /* @doc "invert X using panel width" */
#define OD_TOUCH_FLAG_INVERT_Y         (1u << 1) /* @doc "invert Y using panel height" */
#define OD_TOUCH_FLAG_SWAP_XY          (1u << 2) /* @doc "swap X and Y before invert/clip" */

/** @struct TouchController  @packet 0x28  @repeatable max=4
 *  @doc "Capacitive touch controller on I2C (e.g. GT911). Publishes the first
 *  contact (5 bytes) into MSD dynamic data. Up to 4 instances. 32 bytes." */
struct TouchController {
    uint8_t  instance_number;      /**< @doc "0-based touch-block index." */
    uint16_t touch_ic_type;        /**< @enum TouchIcType @endian le @doc "touch controller IC." */
    uint8_t  bus_id;               /**< @doc "data_bus instance for I2C; 0xFF means bus 0." */
    uint8_t  i2c_addr_7bit;        /**< @doc "I2C 7-bit address (GT911 0x5D or 0x14); 0 or 0xFF = auto-detect." @default 0xFF */
    uint8_t  int_pin;              /**< @doc "interrupt GPIO (active low); 0xFF = poll only." @default 0xFF */
    uint8_t  rst_pin;              /**< @doc "reset GPIO; 0xFF = skip hardware reset (probe only)." @default 0xFF */
    uint8_t  display_instance;     /**< @doc "DisplayConfig index used for coordinate clip / invert reference." */
    uint8_t  flags;                /**< @bits TouchFlags @doc "coordinate mapping flags (bitfield)." */
    uint8_t  poll_interval_ms;     /**< @unit ms @doc "minimum time between polls; 0 = default 25 ms." */
    uint8_t  touch_data_start_byte; /**< @min 0 @max 6 @doc "start index in dynamicreturndata for the 5-byte touch block (avoid overlap with binary_inputs)." */
    uint8_t  enable_pin;           /**< @doc "optional touch-panel power-enable GPIO (active high); 0 or 0xFF = unused." @default 0xFF */
    uint8_t  reserved[20];         /**< @reserved @doc "future use (e.g. GT911 register profile); must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct TouchController) == 32, "TouchController wire size");

/* -----------------------------------------------------------------------
 * 0x29  buzzer  (config.yaml packet name: passive_buzzer)
 * ----------------------------------------------------------------------- */

/* BuzzerConfig.flags @bits BuzzerFlags (bits 1-7 reserved). */
#define OD_BUZZER_FLAG_ENABLE_ACTIVE_HIGH (1u << 0) /* @doc "enable pin is active-high when set; otherwise active-low" */

/** @struct BuzzerConfig  @packet 0x29  @repeatable max=4
 *  @doc "Buzzer (passive piezo, PWM-driven). Up to 4 instances. 32 bytes. The tone
 *  PATTERN is not stored here -- it is carried variable-length by CMD_BUZZER
 *  0x0077 ([instance][outer_repeats][n_patterns] then per-pattern n_steps x
 *  (freq:u8, duration:u8)); that payload stays prose-documented in
 *  opendisplay_protocol.h (it is variable-length, so not modeled as a struct)." */
struct BuzzerConfig {
    uint8_t instance_number; /**< @doc "0-based buzzer-block index." */
    uint8_t drive_pin;       /**< @doc "GPIO driving the PWM / square wave into the buzzer (via transistor). Pin encoding is target-defined (e.g. nRF54 (port<<4)|pin)." */
    uint8_t enable_pin;      /**< @doc "optional enable (e.g. FET); 0xFF = unused." @default 0xFF */
    uint8_t flags;           /**< @bits BuzzerFlags @doc "buzzer flags (bitfield)." */
    uint8_t duty_percent;    /**< @min 1 @max 100 @doc "PWM duty cycle; 0 = default 50." */
    uint8_t reserved[27];    /**< @reserved @doc "future use (incl. internal frequency/time scaling); must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct BuzzerConfig) == 32, "BuzzerConfig wire size");

/* -----------------------------------------------------------------------
 * 0x2A  nfc_config
 * ----------------------------------------------------------------------- */

/* NfcConfig.nfc_ic_type binds to opendisplay_protocol.h's OD_NFC_IC_* MACROS
 * (OD_NFC_IC_AUTO 0, OD_NFC_IC_TNB132M 1) -- NOT re-enumerated here: an enum
 * member of the same spelling would be rewritten by the macro and fail to
 * compile. See the LANGUAGE / LINKAGE RULE in the banner. */

/** @enum NfcFieldDetectMode  @width 1  @doc "How NFC field presence is sensed (NfcConfig.field_detect_mode)." */
enum NfcFieldDetectMode {
    OD_NFC_FIELD_DETECT_DISABLED   = 0, /**< @doc "field-detect logic disabled" */
    OD_NFC_FIELD_DETECT_GPIO_LEVEL = 1, /**< @doc "GPIO level sampled / polled" */
    OD_NFC_FIELD_DETECT_IRQ_LATCHED = 2  /**< @doc "interrupt-latched (reserved for future use)" */
};

/* NfcConfig.flags @bits NfcFlags (bits 1-7 reserved). */
#define OD_NFC_FLAG_ENABLED            (1u << 0) /* @doc "must be set to enable NFC init (no fallback init if clear)" */

/** @struct NfcConfig  @packet 0x2A  @repeatable max=2
 *  @doc "NFC IC selection, data bus, and optional field-detect GPIO that maps a
 *  button-like state into the BLE advertisement. Up to 2 instances. 32 bytes." */
struct NfcConfig {
    uint8_t instance_number;        /**< @doc "0-based NFC-block index." */
    uint8_t nfc_ic_type;            /**< @enum OD_NFC_IC (see opendisplay_protocol.h: OD_NFC_IC_AUTO 0, OD_NFC_IC_TNB132M 1) @doc "NFC IC / flow selector." */
    uint8_t bus_instance;           /**< @doc "data_bus instance (I2C) to use." */
    uint8_t flags;                  /**< @bits NfcFlags @doc "NFC flags (bitfield); bit0 must be set to enable." */
    uint8_t field_detect_pin;       /**< @doc "field-detect input GPIO; 0xFF = disabled." @default 0xFF */
    uint8_t field_detect_mode;      /**< @enum NfcFieldDetectMode @doc "field-detect sensing mode." */
    uint8_t field_detect_active;    /**< @enum ActiveLevel @doc "active level of field_detect_pin." */
    uint8_t field_detect_debounce_ms; /**< @unit ms @doc "debounce time; 0 = no debounce." */
    uint8_t power_pin;              /**< @doc "NFC rail enable pin; 0xFF = use data_bus pin_3 if present, else none." @default 0xFF */
    uint8_t power_active;           /**< @enum ActiveLevel @doc "power-pin polarity (reserved in current firmware)." */
    uint8_t power_on_delay_ms;      /**< @unit ms @doc "delay after enabling power before NFC access." */
    uint8_t power_off_delay_ms;     /**< @unit ms @doc "delay before power-off/park (reserved in current firmware)." */
    uint8_t adv_button_byte_index;  /**< @min 0 @max 10 @doc "dynamicreturndata index for the field-detect button-like state." */
    uint8_t adv_button_button_id;   /**< @doc "3-bit button id (stored in the lower bits) for the advertised state." */
    uint8_t reserved_pin_1;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t reserved_pin_2;         /**< @reserved @doc "spare pin; must be 0." */
    uint8_t reserved[16];           /**< @reserved @doc "future NFC IC-specific extensions; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct NfcConfig) == 32, "NfcConfig wire size");

/* -----------------------------------------------------------------------
 * 0x2B  flash_config
 * ----------------------------------------------------------------------- */

/** @enum FlashIcType  @width 1  @doc "External-flash IC type (FlashConfig.flash_ic_type)." */
enum FlashIcType {
    OD_FLASH_IC_AUTO = 0 /**< @doc "generic SPI-flash deep-sleep command flow" */
};

/* FlashConfig.flags @bits FlashFlags (bits 1-7 reserved). */
#define OD_FLASH_FLAG_ENABLED          (1u << 0) /* @doc "must be set to enable flash pin config / deep-sleep sequencing" */

/** @struct FlashConfig  @packet 0x2B  @repeatable max=2
 *  @doc "External SPI flash deep-sleep pin sequencing. Up to 2 instances. 32
 *  bytes. Ignored when the enabled flag is clear." */
struct FlashConfig {
    uint8_t instance_number;   /**< @doc "0-based flash-block index." */
    uint8_t flash_ic_type;     /**< @enum FlashIcType @doc "flash IC type (reserved for future)." */
    uint8_t bus_instance;      /**< @doc "reserved for future bus binding." */
    uint8_t flags;             /**< @bits FlashFlags @doc "flash flags (bitfield); bit0 must be set to enable." */
    uint8_t mosi_pin;          /**< @doc "SPI MOSI pin (encoding target-defined, e.g. 0xPN)." */
    uint8_t sck_pin;           /**< @doc "SPI SCK pin." */
    uint8_t cs_pin;            /**< @doc "SPI CS pin." */
    uint8_t power_pin;         /**< @doc "optional power-enable pin (reserved in current firmware)." @default 0xFF */
    uint8_t power_active;      /**< @enum ActiveLevel @doc "power-pin polarity (reserved in current firmware)." */
    uint8_t power_on_delay_ms; /**< @unit ms @doc "power-on delay before first command (reserved)." */
    uint8_t power_off_delay_ms; /**< @unit ms @doc "power-off delay after command (reserved)." */
    uint8_t mode;              /**< @doc "flash mode selector (reserved for future)." */
    uint8_t reserved[20];      /**< @reserved @doc "future flash extensions; must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct FlashConfig) == 32, "FlashConfig wire size");

/* -----------------------------------------------------------------------
 * 0x2C  data_extended
 * ----------------------------------------------------------------------- */

/** @struct DataExtended  @packet 0x2C
 *  @doc "Extended device-identity strings. Singleton. 288 bytes = 9 fixed 32-byte
 *  fields. Each field is a null-terminated, zero-padded UTF-8 string; default is
 *  the empty string (all zeros)." */
struct DataExtended {
    uint8_t manufacturer_name[32]; /**< @doc "manufacturer name (32-byte UTF-8, null-terminated, zero-padded)." */
    uint8_t model_name[32];        /**< @doc "model name." */
    uint8_t serial_number[32];     /**< @doc "serial number." */
    uint8_t friendly_name[32];     /**< @doc "human-friendly device name." */
    uint8_t device_location[32];   /**< @doc "physical / logical location." */
    uint8_t device_id[32];         /**< @doc "unique device identifier string." */
    uint8_t custom_string_1[32];   /**< @doc "user-defined string 1." */
    uint8_t custom_string_2[32];   /**< @doc "user-defined string 2." */
    uint8_t custom_string_3[32];   /**< @doc "user-defined string 3." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct DataExtended) == 288, "DataExtended wire size");

/* ==========================================================================
 * SECTION 5 -- MESSAGE PAYLOAD STRUCTS  (fixed-size opcode payloads)
 * ==========================================================================
 * The single machine-readable home for the fixed-layout bodies of the opcodes
 * defined in opendisplay_protocol.h. protocol.h's @request/@response blocks are
 * the narrative; these structs are the byte offsets (one home per layout ->
 * they cannot drift apart). Variable-length / streaming payloads (pipe DATA
 * chunks, the 0x0077 buzzer pattern list) are NOT modeled here -- they stay
 * prose in protocol.h.
 *
 * IMPORTANT endianness split: PIPE (0x80/0x81) fields are LITTLE-endian; the
 * 0x76 PartialWriteStartHeader geometry is BIG-endian. The two carry the SAME
 * geometry fields in opposite byte orders -- see PipePartialExt vs
 * PartialWriteStartHeader.
 * ========================================================================== */

/* LedFlashPattern.mode_brightness low nibble = mode; 1 = flash (0 = off/idle).
 * The pipe flags (PipeStartRequest.flags) bind to opendisplay_protocol.h's
 * PIPE_FLAG_COMPRESSED / PIPE_FLAG_PARTIAL macros -- not redefined here. */

/** @struct LedFlashPattern  @message CMD_LED_ACTIVATE  @endian le
 *  @doc "12-byte LED flash program carried in CMD_LED_ACTIVATE 0x0073 after the
 *  led_instance byte; firmware persists it into LedConfig.reserved[0..11]. Odd
 *  nibble packing is OEPL-compatible -- do not 'fix' it. Three color/loop stages
 *  (1,2,3) each contribute a color byte, a packed loop-delay/count byte, and an
 *  inter-loop delay byte." */
struct LedFlashPattern {
    uint8_t mode_brightness;    /**< @bits @doc "low nibble = mode (1 = flash, 0 = off); high nibble = brightness minus 1 (0..15 -> brightness 1..16)." */
    uint8_t color1;             /**< @doc "stage-1 color/channel byte." */
    uint8_t loop1_delay_count;  /**< @bits @doc "high nibble = stage-1 loop delay (x100 ms); low nibble = stage-1 loop count." */
    uint8_t inter_loop_delay1;  /**< @unit ms @doc "stage-1 inter-loop delay (x100 ms); 0 = none." */
    uint8_t color2;             /**< @doc "stage-2 color/channel byte." */
    uint8_t loop2_delay_count;  /**< @bits @doc "high nibble = stage-2 loop delay (x100 ms); low nibble = stage-2 loop count." */
    uint8_t inter_loop_delay2;  /**< @unit ms @doc "stage-2 inter-loop delay (x100 ms); 0 = none." */
    uint8_t color3;             /**< @doc "stage-3 color/channel byte." */
    uint8_t loop3_delay_count;  /**< @bits @doc "high nibble = stage-3 loop delay (x100 ms); low nibble = stage-3 loop count." */
    uint8_t inter_loop_delay3;  /**< @unit ms @doc "stage-3 inter-loop delay (x100 ms); 0 = none." */
    uint8_t group_repeats;      /**< @doc "group repeat count minus 1 (value 255 = repeat forever)." */
    uint8_t reserved;           /**< @reserved @doc "must be 0." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct LedFlashPattern) == 12, "LedFlashPattern wire size");

/** @struct PipeStartRequest  @message CMD_PIPE_WRITE_START  @endian le
 *  @doc "10-byte PIPE (0x0080) START header that follows the 2 opcode bytes
 *  (host->device). If flags bit1 (PIPE_FLAG_PARTIAL) is set, a 12-byte
 *  PipePartialExt is appended (22 bytes total after the opcode). NOTE: the plan
 *  inventory listed this as 8 bytes, but the field list and protocol.h's
 *  '10-byte header (22 when partial)' both give 10 -- 10 is correct." */
struct PipeStartRequest {
    uint8_t  version;          /**< @doc "PIPE protocol version = PIPE_VERSION (1)." */
    uint8_t  flags;            /**< @bits PIPE_FLAG (see opendisplay_protocol.h: bit0 PIPE_FLAG_COMPRESSED, bit1 PIPE_FLAG_PARTIAL)." */
    uint8_t  req_window;       /**< @min 1 @max 32 @doc "requested sliding-window size (frames)." */
    uint8_t  req_ack_every;    /**< @min 1 @max 32 @doc "requested ACK cadence (frames per SACK)." */
    uint16_t client_max_frame; /**< @endian le @doc "largest on-wire frame the client will send." */
    uint32_t total_size;       /**< @endian le @doc "decompressed byte total to transfer (partial: plane_size x 2)." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PipeStartRequest) == 10, "PipeStartRequest wire size");

/** @struct PipePartialExt  @message CMD_PIPE_WRITE_START  @endian le
 *  @doc "12-byte partial-region extension appended to PipeStartRequest when
 *  PIPE_FLAG_PARTIAL is set. LITTLE-endian twin of the 0x76 PartialWriteStartHeader
 *  geometry (which is big-endian)." */
struct PipePartialExt {
    uint32_t old_etag; /**< @endian le @doc "etag currently displayed; must match or the START is NACKed." */
    uint16_t x;        /**< @endian le @unit px @doc "region left edge." */
    uint16_t y;        /**< @endian le @unit px @doc "region top edge." */
    uint16_t w;        /**< @endian le @unit px @doc "region width." */
    uint16_t h;        /**< @endian le @unit px @doc "region height." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PipePartialExt) == 12, "PipePartialExt wire size");

/** @struct PipeStartResponse  @message CMD_PIPE_WRITE_START  @endian le
 *  @doc "6-byte PIPE START ACK data (device->host): the device echoes its
 *  negotiated maxima (min-rule applies against the client's request)." */
struct PipeStartResponse {
    uint8_t  version;       /**< @doc "PIPE protocol version = PIPE_VERSION (1)." */
    uint8_t  max_window;    /**< @doc "device-granted maximum window size." */
    uint8_t  max_ack_every; /**< @doc "device-granted maximum ACK cadence." */
    uint16_t max_frame;     /**< @endian le @doc "device-granted maximum frame size." */
    uint8_t  resp_flags;    /**< @bits @doc "bit0 = selective-repeat supported; bit1 = partial accepted." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PipeStartResponse) == 6, "PipeStartResponse wire size");

/** @struct PipeSack  @message CMD_PIPE_WRITE_DATA  @endian le
 *  @doc "5-byte selective-ACK data in the PIPE 0x0081 ACK frame (device->host).
 *  highest_seen is implicitly acked; ack_mask bit i (LSB first) acks chunk
 *  (highest_seen - 1 - i)." */
struct PipeSack {
    uint8_t  highest_seen; /**< @doc "highest received seq (mod 256), implicitly acked." */
    uint32_t ack_mask;     /**< @endian le @doc "selective-ACK bitmask below highest_seen (bit i = seq highest_seen-1-i)." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PipeSack) == 5, "PipeSack wire size");

/** @struct PartialWriteStartHeader  @message CMD_PARTIAL_WRITE_START  @endian be
 *  @doc "17-byte header for CMD_PARTIAL_WRITE_START 0x0076, following the 2 opcode
 *  bytes (host->device). ENTIRELY BIG-ENDIAN -- the showcase for the per-message
 *  endianness split: the PIPE 0x80 path carries the same geometry LITTLE-endian
 *  (see PipePartialExt). x and width must be multiples of 8; old_etag must equal
 *  the currently displayed etag." */
struct PartialWriteStartHeader {
    uint8_t  flags;    /**< @doc "partial-write flags (see protocol.h 0x76 @request)." */
    uint32_t old_etag; /**< @endian be @doc "etag currently displayed; must match." */
    uint32_t new_etag; /**< @endian be @doc "etag to store after this write." */
    uint16_t x;        /**< @endian be @unit px @doc "region left edge (multiple of 8)." */
    uint16_t y;        /**< @endian be @unit px @doc "region top edge." */
    uint16_t width;    /**< @endian be @unit px @doc "region width (multiple of 8)." */
    uint16_t height;   /**< @endian be @unit px @doc "region height." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct PartialWriteStartHeader) == 17, "PartialWriteStartHeader wire size");

/** @struct AuthChallenge  @message CMD_AUTHENTICATE  @endian le
 *  @doc "21-byte CMD_AUTHENTICATE 0x0050 step-1 response DATA (device->host),
 *  after the [status][echo] framing: the device's nonce + id the client signs.
 *  This handshake is ALWAYS plaintext (bootstraps the session)." */
struct AuthChallenge {
    uint8_t  status;          /**< @doc "auth status = AUTH_STATUS_CHALLENGE (0x00) on success (see protocol.h)." */
    uint8_t  server_nonce[16]; /**< @doc "device nonce; part of the CMAC input." */
    uint32_t device_id;       /**< @endian le @doc "device id; part of the CMAC input." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct AuthChallenge) == 21, "AuthChallenge wire size");

/** @struct AuthProof  @message CMD_AUTHENTICATE
 *  @doc "32-byte CMD_AUTHENTICATE 0x0050 step-2 request payload (host->device):
 *  the client nonce and the AES-CMAC over (server_nonce || client_nonce ||
 *  device_id)." */
struct AuthProof {
    uint8_t client_nonce[16]; /**< @doc "client nonce; part of the CMAC input." */
    uint8_t mac[16];          /**< @doc "AES-CMAC(master_key, server_nonce || client_nonce || device_id)." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct AuthProof) == 32, "AuthProof wire size");

/* ==========================================================================
 * SECTION 6 -- BLE MANUFACTURER-SPECIFIC ADVERTISEMENT (MSD)
 * ==========================================================================
 * The 16-byte manufacturer-specific data record broadcast in BLE advertisements
 * AND returned by CMD_READ_MSD 0x0044. The 11-byte `dynamic` area holds
 * config-driven slots (button / touch / SHT40 / BQ27220) at the indices those
 * config packets specify (SensorData.msd_data_start_byte,
 * BinaryInputs.button_data_byte_index, TouchController.touch_data_start_byte).
 * Verified against OD App Models/AdvertisementData.swift.
 * ========================================================================== */

/* MsdAdvertisement.status @bits MsdStatusBits. bit0 is the 9th (MSB) bit of the
 * 10-bit battery voltage; bits 4-7 are a free-running main-loop nibble counter. */
#define OD_MSD_STATUS_BATTERY_VOLTAGE_BIT8 (1u << 0) /* @doc "high bit of the 10-bit battery voltage (units of 10 mV); combine with battery_voltage_low" */
#define OD_MSD_STATUS_REBOOT_FLAG          (1u << 1) /* @doc "device rebooted since last read" */
#define OD_MSD_STATUS_CONNECTION_REQUESTED (1u << 2) /* @doc "device is requesting a connection" */
#define OD_MSD_STATUS_RESERVED_3           (1u << 3) /* @reserved @doc "reserved; must be 0 (placeholder name for a future status flag; sits between the flags and the bits 4-7 counter)" */
#define OD_MSD_STATUS_MAIN_LOOP_COUNTER_SHIFT 4u     /* @doc "bits 4-7: free-running main-loop nibble counter (liveness)" */
#define OD_MSD_STATUS_MAIN_LOOP_COUNTER_MASK  0xF0u  /* @doc "mask for the bits 4-7 main-loop counter nibble" */

/** @struct MsdAdvertisement  @message CMD_READ_MSD  @endian le
 *  @doc "16-byte OpenDisplay manufacturer-specific data. Broadcast in BLE
 *  advertisements and returned by CMD_READ_MSD 0x0044 (device->host). Battery
 *  voltage is 10 bits (10 mV units): the low 8 bits are battery_voltage_low, the
 *  9th bit is status bit0; the 10th is implied 0 / reserved by firmware." */
struct MsdAdvertisement {
    uint16_t company_id;         /**< @endian le @doc "BLE company identifier." */
    uint8_t  dynamic[11];        /**< @doc "config-driven dynamic area: button/touch/SHT40/BQ27220 slots at indices set by the relevant config packets." */
    uint8_t  chip_temperature;   /**< @unit C @doc "MCU chip temperature (signed degrees C, firmware-defined offset)." */
    uint8_t  battery_voltage_low; /**< @unit 10mV @doc "low 8 bits of the 10-bit battery voltage (10 mV units); high bit in status bit0." */
    uint8_t  status;             /**< @bits MsdStatusBits @doc "status flags + battery voltage MSB + main-loop counter nibble." */
} __attribute__((packed));
OD_STATIC_ASSERT(sizeof(struct MsdAdvertisement) == 16, "MsdAdvertisement wire size");

#endif /* OPENDISPLAY_STRUCTS_H */
