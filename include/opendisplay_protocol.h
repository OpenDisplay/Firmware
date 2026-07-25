/* ==========================================================================
 * opendisplay_protocol.h  --  OpenDisplay BLE wire-protocol contract
 * ==========================================================================
 *
 * PURPOSE
 *   Single source of truth for every value that travels on the OpenDisplay
 *   BLE wire: command opcodes, response/status bytes, auth states, error
 *   codes, and the sub-protocol constants for NFC and the sliding-window
 *   image PIPE. A new engineer or an AI agent should be able to implement a
 *   fully-correct client from THIS FILE ALONE, without reading firmware.
 *
 *   OD_PROTOCOL_VERSION 2.2   (MAJOR.MINOR; see VERSIONING POLICY below)
 *   LAST CHANGED        2026-07-22
 *
 * --------------------------------------------------------------------------
 * VERSIONING POLICY
 * --------------------------------------------------------------------------
 *   The version is MAJOR.MINOR and describes the SPEC in THIS file. It is a
 *   compatibility / documentation marker and is NOT transmitted on the wire.
 *   (The on-wire, negotiated PIPE_VERSION used by the 0x0080 sub-protocol is a
 *   separate field, versioned independently.)
 *
 *   Bump MAJOR (x.0) for a BREAKING wire change -- one that makes a peer built
 *   against the previous version misread bytes or lose interoperability:
 *     - changing the value or meaning of an existing opcode, response/status
 *       byte, auth state, or error code (e.g. the NFC 0x0082 -> 0x0083 move
 *       that defines 2.0);
 *     - changing an existing message's request/response layout, field size,
 *       endianness, or a framing rule (opcode width, envelope, auth gating);
 *     - removing or renumbering any existing command / response / error code.
 *   Reset MINOR to 0 on every MAJOR bump.
 *
 *   Bump MINOR (2.x) for a BACKWARD-COMPATIBLE addition -- old peers keep
 *   working unchanged, new peers gain capability:
 *     - adding a NEW command opcode, response/status byte, error code, NFC
 *       sub-command, or NFC record type that leaves existing ones untouched;
 *     - appending a NEW optional, length-discriminated field to an existing
 *       message that old parsers safely ignore;
 *     - widening a documented limit / range in a compatible direction.
 *
 *   NO bump for changes that do not alter wire behavior -- comment / spec
 *   clarifications, or adding an @targets entry when a firmware merely starts
 *   implementing an already-defined opcode. Record those in git history and
 *   reserve version bumps for real spec changes.
 *
 *   Rule of thumb -- "could a peer on the previous version misread the wire?"
 *     yes                         -> MAJOR
 *     no, but something new added -> MINOR
 *     neither                     -> no bump
 *
 * --------------------------------------------------------------------------
 * CHANGELOG  (newest first; entries accrue under "Unreleased" and roll into a
 *            new version heading on each bump -- see AGENT INSTRUCTIONS below)
 * --------------------------------------------------------------------------
 *   Unreleased (since 2.2)
 *     - LAN framing: cap the WIRE frame ([len:2 LE] prefix + payload) at a new
 *       constant OD_LAN_MAX_FRAME (4096); OD_LAN_MAX_PAYLOAD is now DERIVED as
 *       OD_LAN_MAX_FRAME - 2 = 4094 (was a standalone 4096, which made the max
 *       wire frame an awkward 4098). Senders MUST keep payloads <= 4094 and
 *       receivers MUST reject len > 4094. Breaking only for senders that framed
 *       4095/4096-byte payloads (none deployed); max DIRECT_WRITE chunk data on
 *       LAN is now OD_LAN_MAX_PAYLOAD - 2 = 4092.
 *     - BLE framing: add OD_BLE_MAX_FRAME (256) as the named BLE transport
 *       ceiling -- the ATT MTU, counting opcode(1) + handle(2) + value, so the
 *       usable single-write value is 253. This is the BLE peer of
 *       OD_LAN_MAX_FRAME and codifies a limit the wire already had: PIPE_MAX_FRAME
 *       (244) is the frame ceiling, and nRF peripherals are hard-capped at ATT MTU
 *       247 (payload 244) by the SoftDevice. Peers SHOULD declare their GATT value
 *       length and preferred MTU at OD_BLE_MAX_FRAME rather than the 512 ATT
 *       maximum, so an oversize write draws ATT error 0x0D instead of being
 *       silently dropped. Breaking only for peers that wrote >253 bytes in a
 *       single GATT write (none deployed -- HA's GATT client caps at 244 and
 *       rejects larger writes outright).
 *     - Add each further wire-spec change here as it lands. On the next version
 *       bump, move these under a new "MAJOR.MINOR (YYYY-MM-DD)" heading.
 *
 *   2.2  (2026-07-22)
 *     - MINOR: new SECTION 9 -- NETWORK TRANSPORT (LAN). Documents the WiFi/LAN
 *       transport as a first-class, implement-from-this-file-alone contract:
 *       framed TCP ([len:2 LE][payload], payload 1..OD_LAN_MAX_PAYLOAD) over a
 *       plaintext OR TLS-PSK channel selected by isEncryptionEnabled(). Adds the
 *       OD_LAN_* constants (OD_LAN_TCP_PORT 2446u, OD_LAN_TLS_PORT 2447u,
 *       OD_LAN_MAX_PAYLOAD 4096u, OD_LAN_MDNS_SERVICE "_opendisplay._tcp",
 *       OD_LAN_READ_TIMEOUT_S 30u). No new opcodes, no struct change: the
 *       plaintext port comes from WifiConfig.server_port (default OD_LAN_TCP_PORT)
 *       and the TLS port is DERIVED as server_port + 1. Additive -- BLE peers on
 *       2.1 are unaffected (no wire bytes on the existing BLE path change).
 *
 *   2.1  (2026-07-18)
 *     - MINOR: new CMD_POWER_OFF (0x0052) -- explicit hard rail-cut via the
 *       D-FF power latch; wakes ONLY on a physical button. Fire-and-forget
 *       (ACK queued best-effort, usually dies before the rail is cut). Adds
 *       RESP_POWER_OFF (0x52) and the scoped OD_ERR_POWER_OFF_* namespace
 *       (SECTION 4d).
 *     - CMD_DEEP_SLEEP is opcode 0x0053. It documents its optional one-shot
 *       [seconds:2 BE] wake-timer payload (originally shipped on 0x0052,
 *       Firmware PR #97) and gains the scoped OD_ERR_DEEP_SLEEP_* NACK
 *       namespace (SECTION 4c: 0x00 unsupported, 0x01 disabled, 0x02
 *       not-battery). Client rule: 0x01/0x02 mean "rejected, still awake".
 *     - BREAKING vs shipped firmware: CMD_DEEP_SLEEP moved 0x0052 -> 0x0053,
 *       swapped with CMD_POWER_OFF so power-off takes the lower number. Since
 *       0x0052 = deep-sleep shipped in Firmware PR #97, a peer on the old
 *       mapping now hits the wrong command. Corrected in place within the
 *       (unreleased) 2.1 line rather than a MAJOR bump (amended 2026-07-19).
 *     - BEHAVIORAL: target design for CMD_DEEP_SLEEP (0x0053) on D-FF-latch
 *       ESP32 changes from power-off to timer-wake deep sleep (latch held, like
 *       non-latch boards); the hard rail-cut is CMD_POWER_OFF (0x0052).
 *     - Doc-only: LANGUAGE / LINKAGE RULE now requires macro VALUES to stay
 *       simple (literal or reference to a prior macro) so the header stays
 *       parseable by tools/gen_python_protocol.py. No wire bytes change.
 *
 *   2.0  (2026-07-15)
 *     - Initial shared protocol contract: one self-documenting header vendored
 *       into all firmware repos (Firmware, NRF54, Silabs, NRF52811).
 *     - BREAKING: NFC endpoint moved 0x0082 -> 0x0083 to resolve the collision
 *       with CMD_PIPE_WRITE_END; clean cutover, no legacy alias. This move
 *       defines the 2.x line.
 *     - Union superset of every repo's commands / responses / auth states /
 *       error codes, with per-opcode tagged spec blocks.
 *     - Established MAJOR.MINOR versioning policy, this changelog, and the
 *       LAST CHANGED field.
 *     - Opcode-scoped, prefixed NACK error namespaces (OD_ERR_PARTIAL_* for
 *       0x76, OD_ERR_PIPE_START_* for the 0x80 START, NFC_ERR_* for 0x83) and
 *       the scoping rule that data[0] is always decoded in the scope of the
 *       echoed opcode -- there is no global error enum, and byte values are
 *       reused across namespaces with different meanings. Names/comments only;
 *       no wire bytes change (these constants are new in 2.0, unconsumed).
 *
 * --------------------------------------------------------------------------
 * AGENT INSTRUCTIONS -- changelog upkeep (perform on EVERY edit to this file)
 * --------------------------------------------------------------------------
 *   1. Set LAST CHANGED (top) to the date of your edit.
 *   2. Add a bullet describing the change under "Unreleased (since x.y)".
 *   3. Classify the change via the VERSIONING POLICY above: breaking => MAJOR,
 *      backward-compatible addition => MINOR, doc-only => no bump.
 *   4. VALIDATE before finishing: confirm the accumulated "Unreleased" entries
 *      are consistent with OD_PROTOCOL_VERSION. If ANY entry is breaking, MAJOR
 *      must be bumped; if any adds wire capability, at least MINOR must be
 *      bumped. When a bump is warranted: update OD_PROTOCOL_VERSION_MAJOR /
 *      _MINOR / _STR, add a new "MAJOR.MINOR (YYYY-MM-DD)" heading, move the
 *      Unreleased entries beneath it (reset MINOR to 0 on a MAJOR bump), and
 *      leave "Unreleased (since x.y)" empty.
 *   5. Never delete or rewrite historical entries -- the changelog is append-only.
 *
 * CANONICAL LOCATION
 *   opendisplay-protocol/src/opendisplay_protocol.h
 *
 *   VENDORED COPY IN FIRMWARE REPOS -- DO NOT EDIT THERE.
 *   Sync every copy byte-for-byte via tools/sync_protocol_header.py:
 *       Firmware/include/opendisplay_protocol.h
 *       Firmware_NRF54/src/opendisplay_protocol.h
 *       Firmware_Silabs/opendisplay_protocol.h
 *       Firmware_NRF/opendisplay_protocol.h
 *   CI runs `sync_protocol_header.py --check` (a hash compare) in each repo.
 *
 * LANGUAGE / LINKAGE RULE
 *   MACRO-ONLY. Every constant is a `#define`, `u`-suffixed. Macros have no
 *   linkage, so this header is safe to `#include` from C and C++ translation
 *   units with NO `extern "C"` block. Do not add typedefs, enums, structs, or
 *   functions here -- those are per-repo and belong in opendisplay_structs.h /
 *   opendisplay_constants.h. Keep the include guard OPENDISPLAY_PROTOCOL_H so
 *   the vendored copies are drop-in replacements.
 *   KEEP MACRO VALUES SIMPLE: an integer literal (0x...u / ...u), a string
 *   literal, or a reference to a macro already defined above. No expressions
 *   (e.g. `(1u << 3)`), casts, or multi-token values. This keeps the header
 *   parseable by tools/gen_python_protocol.py, which mirrors these constants
 *   into src/opendisplay_protocol.py (the Python single-source equivalent of
 *   the byte-for-byte firmware vendoring) and hard-errors on any other shape.
 *
 * --------------------------------------------------------------------------
 * UNIVERSAL FRAMING
 * --------------------------------------------------------------------------
 *   REQUEST (host -> device):
 *       [cmd_hi][cmd_lo][payload...]
 *     - Opcode is 2 bytes, BIG-ENDIAN (e.g. 0x0041 -> bytes 0x00 0x41).
 *     - Payload layout is per-command (see each @request below). Multi-byte
 *       payload fields are LITTLE-ENDIAN unless a block says "BE".
 *
 *   RESPONSE / NOTIFICATION (device -> host):
 *       [status][cmd_echo][data...]
 *     - status   : 0x00 = ACK (RESP_ACK)
 *                  0xFF = NACK (RESP_NACK); a NACK's data[0] is often an
 *                         error code (OD_ERR_PARTIAL_* / OD_ERR_PIPE_START_* /
 *                         NFC_ERR_* / handler-local).
 *                  0xFE = auth required (RESP_AUTH_REQUIRED); sent for any
 *                         gated command when security is on but no session.
 *     - cmd_echo : the LOW byte of the command (RESP_* mirror the low byte of
 *                  the matching CMD_*; e.g. CMD_CONFIG_WRITE 0x0041 echoes
 *                  RESP_CONFIG_WRITE 0x41).
 *
 *   NACK ERROR-CODE SCOPING RULE (READ THIS)
 *     - The NACK error code in data[0] is ALWAYS interpreted in the SCOPE of the
 *       echoed opcode (byte 1). There is deliberately NO single global error
 *       enum: each opcode's handler owns its own prefixed namespace, so the same
 *       byte value carries DIFFERENT meanings under different opcodes.
 *     - Concrete overlap: a NACK byte 0x03 means OD_ERR_PARTIAL_RECT_OOB under a
 *       0x76 NACK ([0xFF][0x76][0x03][0x00]) but OD_ERR_PIPE_START_SIZE_MISMATCH
 *       under a 0x80 NACK ([0xFF][0x80][0x03][0x00]). Decode data[0] only after
 *       you know byte 1. See SECTION 4 for the full per-opcode error namespaces.
 *     - Some frames are unsolicited device->host notifications (refresh
 *       success/timeout); they use the same [status][echo] shape.
 *
 *   ENCRYPTED ENVELOPE (when a security session is active):
 *       [cmd:2 BE][nonce:16][ciphertext][tag:12]
 *     - Inner plaintext (what the ciphertext protects): [len:1][payload]
 *       where len is the payload byte count.
 *     - AAD (additional authenticated data) = the 2 plaintext opcode bytes.
 *     - AEAD = AES-128-CCM, 12-byte tag (ENCRYPTION_TAG_SIZE), 16-byte nonce
 *       (ENCRYPTION_NONCE_SIZE); the CCM 13-byte nonce is nonce[3..15].
 *     - ALWAYS-PLAINTEXT commands/responses (never encrypted, so a client can
 *       bootstrap a session): AUTHENTICATE (0x50) and FIRMWARE_VERSION (0x43);
 *       on Firmware_Silabs, READ_MSD (0x44) is also plaintext.
 *
 *   AUTH GATING
 *     - When security is enabled, EVERY command except AUTHENTICATE requires a
 *       live authenticated session; otherwise the device replies
 *       [0xFE][cmd_echo] (RESP_AUTH_REQUIRED) and drops the request.
 *
 *   CHUNK BUDGETS (max DATA bytes per frame, image/direct-write path)
 *     - Unencrypted : 230 bytes.
 *     - Encrypted   : 154 bytes  (packet = cmd2 + nonce16 + len1 + 154 + tag12
 *                     = 185 <= MTU-derived ceiling). Config chunks are capped
 *                     separately at CONFIG_CHUNK_SIZE (200).
 *
 * --------------------------------------------------------------------------
 * FIRMWARE TARGET LEGEND (used by @targets below)
 * --------------------------------------------------------------------------
 *   Firmware   : combined nRF52840 + ESP32 (PlatformIO). Implements PIPE
 *                (0x0080/0x0081/0x0082) and 0x0045/0x0075/0x0076/0x0077.
 *                NO NFC. Here 0x0082 == PIPE_WRITE_END.
 *   NRF54      : Firmware_NRF54 (nRF54, Zephyr). Implements NFC (0x0083) and
 *                0x0045/0x0075/0x0076/0x0077. NO PIPE 0x0080/0x0081.
 *   Silabs     : Firmware_Silabs (EFR32 Flex). Config r/w/chunk, fw-ver, MSD,
 *                auth, reboot, DFU, deep-sleep, direct-write, LED activate/stop,
 *                NFC (0x0083). NO 0x0045 / 0x0076 / 0x0077 / PIPE.
 *   NRF52811   : Firmware_NRF (nRF52811, minimal). Config r/w/chunk (NO 0x0045),
 *                direct-write, LED_ACTIVATE (NO LED_STOP), reboot, fw-ver, MSD,
 *                auth, DFU, deep-sleep. NO NFC / PIPE / partial / buzzer.
 *
 * --------------------------------------------------------------------------
 * TAG CONVENTION (every opcode block below is uniform + line-anchored so that
 * `grep '@opcode'`-style tooling and agents can parse it)
 *   @opcode  @name  @dir       identity + direction
 *   @request                   byte-by-byte layout (offsets, sizes, endianness)
 *   @response                  each distinct status/echo/data frame
 *   @errors                    NACK / error codes this opcode can return
 *   @state                     sequencing, preconditions, timeouts
 *   @limits                    payload / chunk budgets
 *   @targets                   which firmwares implement it
 *   @changed / @since          history (wire-visible changes)
 *   @collision                 only where a byte value is reused
 * ========================================================================== */

#ifndef OPENDISPLAY_PROTOCOL_H
#define OPENDISPLAY_PROTOCOL_H

/* Wire-protocol revision, MAJOR.MINOR. See VERSIONING POLICY in the banner for
 * when to bump which number. This marker describes the spec and is NOT sent on
 * the wire (the negotiated 0x0080 PIPE_VERSION is a separate field). */
#define OD_PROTOCOL_VERSION_MAJOR      2u
#define OD_PROTOCOL_VERSION_MINOR      2u
#define OD_PROTOCOL_VERSION_STR        "2.2"

/* ==========================================================================
 * SECTION 1 -- COMMAND OPCODES (16-bit, big-endian on the wire)
 * ==========================================================================
 * Config-packet-type bytes (the payload's first byte for CONFIG_WRITE TLVs)
 * are NOT defined here -- they live in each repo's opendisplay_structs.h /
 * opendisplay_constants.h and diverge per target. Comment-only reference:
 *     0x01 system        0x02 manufacturer   0x04 power
 *     0x20 display       0x21 led            0x23 sensor
 *     0x24 data_bus      0x25 binary_inputs  0x26 wifi
 *     0x27 security      0x28 touch          0x29 passive_buzzer
 *     0x2A nfc           0x2B flash          0x2C data_extended
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * @opcode: 0x000F   @name: CMD_REBOOT   @dir: host->device
 * @request:  [0x00][0x0F]  (no payload)
 * @response: none on most targets (immediate reset, BLE link drops). Silabs
 *            may ACK before resetting; treat a dropped connection as success.
 * @errors:   none
 * @state:    if security enabled, session required (except on the no-ACK path).
 * @limits:   -
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_REBOOT                     0x000Fu

/* --------------------------------------------------------------------------
 * @opcode: 0x0040   @name: CMD_CONFIG_READ   @dir: host->device
 * @request:  [0x00][0x40]  (no payload)
 * @response: one OR MORE notifications (chunked), each:
 *              [0x00][0x40][chunk_number:2 LE][total_len:2 LE on chunk 0 only][data...]
 *            chunk_number counts from 0; total_len (uint16 LE) is present ONLY
 *            on chunk 0 and gives the full config byte length. Empty config
 *            yields a single [0x00][0x40][0x00 0x00][0x00 0x00].
 * @errors:   [0xFF][0x40][0x00][0x00] on storage-init failure.
 * @state:    session required when security enabled.
 * @limits:   each frame's data <= MAX_RESPONSE_DATA_SIZE (100) minus its header.
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_CONFIG_READ                0x0040u

/* --------------------------------------------------------------------------
 * @opcode: 0x0041   @name: CMD_CONFIG_WRITE   @dir: host->device
 * @request:  single-frame (payload <= 200):  [0x00][0x41][config_data...]
 *            chunked (payload  > 200):        [0x00][0x41][total:2 LE][first 200 data bytes]
 *              -> the first frame MUST carry a FULL 200 data bytes (payload =
 *                 2 + 200 = 202); a short first chunk makes firmware take the
 *                 single-frame path and NACK the following 0x42 chunks.
 *              -> remaining bytes follow as CMD_CONFIG_CHUNK (0x0042) frames.
 * @response: [0x00][0x41] on accept (or after last chunk); [0xFF][0x41] on error.
 * @errors:   NACK on malformed TLV, size overflow, or bad chunk sequencing.
 * @state:    session required when security enabled. Device computes
 *            expectedChunks = ceil(total / 200).
 * @limits:   CONFIG_CHUNK_SIZE 200; first chunk payload CONFIG_CHUNK_SIZE_WITH_PREFIX 202.
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_CONFIG_WRITE               0x0041u

/* --------------------------------------------------------------------------
 * @opcode: 0x0042   @name: CMD_CONFIG_CHUNK   @dir: host->device
 * @request:  [0x00][0x42][chunk_data...]  (up to 200 bytes; continuation of a
 *            chunked CONFIG_WRITE started at 0x0041).
 * @response: [0x00][0x42] per accepted chunk; final chunk commits + reloads
 *            config. [0xFF][0x42] on overflow / no active write / bad total.
 * @errors:   NACK if no active chunked write, or received > declared total.
 * @state:    must follow a chunked 0x0041 START; count bounded by MAX_CONFIG_CHUNKS.
 * @limits:   <= CONFIG_CHUNK_SIZE (200) data bytes.
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_CONFIG_CHUNK               0x0042u

/* --------------------------------------------------------------------------
 * @opcode: 0x0043   @name: CMD_FIRMWARE_VERSION   @dir: host->device
 * @request:  [0x00][0x43]  (no payload)
 * @response: [0x00][0x43][major:1][minor:1][... optional git/SHA string]
 * @errors:   none
 * @state:    ALWAYS PLAINTEXT (never encrypted) so version is readable pre-auth.
 * @limits:   -
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_FIRMWARE_VERSION           0x0043u

/* --------------------------------------------------------------------------
 * @opcode: 0x0044   @name: CMD_READ_MSD   @dir: host->device
 * @request:  [0x00][0x44]  (no payload)
 * @response: [0x00][0x44][msd_bytes:16]  (manufacturer-specific data record).
 * @errors:   none
 * @state:    on Silabs this response is PLAINTEXT even with security enabled.
 * @limits:   16 data bytes.
 * @targets:  NRF54 | Silabs | NRF52811   (Firmware handles it but see repo)
 * -------------------------------------------------------------------------- */
#define CMD_READ_MSD                   0x0044u

/* --------------------------------------------------------------------------
 * @opcode: 0x0045   @name: CMD_CONFIG_CLEAR   @dir: host->device
 * @request:  [0x00][0x45]  (no payload)
 * @response: [0x00][0x45] on success; [0xFF][0x45] on failure.
 * @errors:   NACK if config storage erase fails.
 * @state:    session required when security enabled.
 * @limits:   -
 * @targets:  Firmware | NRF54 | Silabs      (NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_CONFIG_CLEAR               0x0045u

/* --------------------------------------------------------------------------
 * @opcode: 0x0050   @name: CMD_AUTHENTICATE   @dir: host->device
 * @request:  STEP 1 (request challenge):  [0x00][0x50][0x00]
 *            STEP 2 (prove key):          [0x00][0x50][client_nonce:16][mac:16]
 *              mac = AES-CMAC(master_key, server_nonce || client_nonce || device_id)
 * @response: STEP 1 challenge (23 bytes): [0x00][0x50][status:1][server_nonce:16][device_id:4]
 *              status = AUTH_STATUS_CHALLENGE (0x00) on success.
 *            STEP 2:  [0x00][0x50][AUTH_STATUS_SUCCESS] on match (session opens);
 *                     [0x00][0x50][AUTH_STATUS_FAILED]  on wrong key;
 *                     [0x00][0x50][AUTH_STATUS_*]       for the other states.
 * @errors:   AUTH_STATUS_NOT_CONFIG (no key set), AUTH_STATUS_RATE_LIMIT,
 *            AUTH_STATUS_ALREADY, AUTH_STATUS_ERROR.
 * @state:    STEP 2 must arrive within 30 s of the challenge or it is rejected.
 *            Rate limit: >=10 attempts within any 60 s window -> RATE_LIMIT.
 *            ALWAYS PLAINTEXT (this is how a session is bootstrapped).
 * @limits:   step-2 command is 34 bytes total (2 opcode + 16 + 16).
 * @targets:  Firmware | NRF54 | Silabs | NRF52811  (where security is enabled)
 * -------------------------------------------------------------------------- */
#define CMD_AUTHENTICATE               0x0050u

/* --------------------------------------------------------------------------
 * @opcode: 0x0051   @name: CMD_ENTER_DFU   @dir: host->device
 * @request:  [0x00][0x51]  (no payload)
 * @response: none on nRF (sets Nordic GPREGRET magic, jumps to bootloader,
 *            BLE drops). Silabs may ACK [0x00][0x51] before entering DFU.
 * @errors:   none
 * @state:    session required when security enabled.
 * @limits:   -
 * @targets:  NRF54 | Silabs | NRF52811   (nRF/Silabs bootloaders)
 * -------------------------------------------------------------------------- */
#define CMD_ENTER_DFU                  0x0051u

/* --------------------------------------------------------------------------
 * @opcode: 0x0052   @name: CMD_POWER_OFF   @dir: host->device
 * @request:  [0x00][0x52]  (bare). Any trailing payload is RESERVED and
 *            ignored today; senders SHOULD send none.
 * @response: FIRE-AND-FORGET (same acknowledgement model as CMD_DEEP_SLEEP
 *            0x0053):
 *              [0x00][0x52]              ACK is QUEUED on latch HW but usually
 *                                        dies in the TX buffer before the rail
 *                                        is cut (~100 ms later). Expected, NOT
 *                                        a bug; senders MUST NOT wait for it --
 *                                        treat transmit as "delivered".
 *              [0xFF][0x52][0x00][0x00]  no power latch on this target.
 *              [0xFE][0x52]              auth required (no live session).
 * @errors:   NACK data[0] scoped to opcode 0x52 (OD_ERR_POWER_OFF_*, see
 *            SECTION 4d): 0x00 UNSUPPORTED (no D-FF latch). A target that
 *            refuses 0x52 may still accept 0x53 deep sleep -- do not conflate
 *            "power-off unsupported" with "cannot sleep".
 * @state:    session required when security enabled. Cuts the power rail via
 *            the D-FF latch; the device wakes ONLY on a physical button press
 *            (fresh cold boot). No timer, no wake interval -- power-off is
 *            absolute.
 * @limits:   trailing payload reserved/ignored.
 * @targets:  Firmware (ESP32, D-FF power-latch HW ONLY). All other targets
 *            (non-latch/mains ESP32, Silabs, nRF) NACK [0xFF][0x52][0x00].
 * @since:    new in 2.1 -- splits the hard rail-cut out of CMD_DEEP_SLEEP so
 *            latch HW gets timed-wake sleep via 0x0053 and an explicit,
 *            capability-gated hard-off via 0x0052.
 * @changed:  opcode is 0x0052 -- swapped with CMD_DEEP_SLEEP within the
 *            (unreleased) 2.1 line so power-off takes the lower number. NOTE:
 *            0x0052 was CMD_DEEP_SLEEP in shipped firmware (PR #97); a peer
 *            still on the old mapping will hit deep-sleep here.
 * -------------------------------------------------------------------------- */
#define CMD_POWER_OFF                  0x0052u

/* --------------------------------------------------------------------------
 * @opcode: 0x0053   @name: CMD_DEEP_SLEEP   @dir: host->device
 * @request:  [0x00][0x53]                 -> sleep now, configured cadence.
 *            [0x00][0x53][seconds:2 BE]   -> sleep now; wake timer = seconds
 *                                            for THIS cycle only (one-shot,
 *                                            never persisted).
 *              - seconds is a BIG-ENDIAN uint16 (deliberate, documented
 *                exception to the LE payload default). Range 1..65535 s
 *                (~18.2 h; the wire width is the ceiling).
 *              - seconds == 0x0000 or empty payload = "no override" (use
 *                configured deep_sleep_time_seconds); NOT an error.
 *              - len == 1 is malformed -> logged, treated as no override
 *                (not rejected). Bytes beyond the 2 seconds bytes are ignored
 *                (forward compatibility).
 * @response: FIRE-AND-FORGET. ACK/NACK are OPTIONAL / best-effort; a sender
 *            MUST NOT expect, block on, or infer success/failure from any
 *            frame. The SUCCESS path emits NO frame: the device enters deep
 *            sleep and BLE drops -- treat disconnect/silence as "delivered".
 *              [0xFF][0x53][0x01][0x00]  deep sleep disabled in config.
 *              [0xFF][0x53][0x02][0x00]  mains-powered; refuses to sleep.
 *              [0xFF][0x53][0x00][0x00]  unsupported target (nRF; may instead
 *                                        stay silent and log).
 *              [0xFE][0x53]              auth required (no live session).
 * @errors:   NACK data[0] scoped to opcode 0x53 (OD_ERR_DEEP_SLEEP_*, see
 *            SECTION 4c): 0x00 UNSUPPORTED, 0x01 DISABLED
 *            (deep_sleep_time_seconds == 0), 0x02 NOT_BATTERY (power_mode
 *            != 1). CRITICAL client rule: 0x01/0x02 mean "rejected -- device
 *            still AWAKE and reachable"; ONLY 0x00 means the target lacks deep
 *            sleep. Never conflate them.
 * @state:    session required when security enabled. The override is consumed
 *            at sleep entry; the next wake (and every later one) reverts to the
 *            configured cadence. An aborted sleep discards the override.
 * @limits:   optional [seconds:2 BE]; ceiling 65535 s (wire width). Recommended
 *            client-side floor >= 10 s (wake-storm guard).
 * @targets:  Firmware (ESP32: honors duration, incl. D-FF-latch HW -- target
 *            design, see @changed; reTerminal E1001/2/3 have no latch and
 *            already plain timer-sleep) | Silabs (ACKs, IGNORES payload, enters
 *            EM4) | nRF targets: unsupported.
 * @changed:  BREAKING vs shipped firmware -- opcode moved 0x0052 -> 0x0053
 *            (swapped with CMD_POWER_OFF within the unreleased 2.1 line). The
 *            [seconds:2 BE] payload shipped on 0x0052 (Firmware PR #97), so a
 *            peer still on 0x0052 will now hit CMD_POWER_OFF. Also target
 *            design: D-FF-latch ESP32 timer-sleeps (latch HELD) instead of
 *            powering off; the hard rail-cut is CMD_POWER_OFF (0x0052). The
 *            request/response byte layout is otherwise UNCHANGED.
 * @since:    [seconds:2 BE] one-shot payload shipped in-2.x on 0x0052 (Firmware
 *            PR #97); documented, moved to 0x0053, and error namespace added
 *            in 2.1.
 * -------------------------------------------------------------------------- */
#define CMD_DEEP_SLEEP                 0x0053u

/* --------------------------------------------------------------------------
 * @opcode: 0x0070   @name: CMD_DIRECT_WRITE_START   @dir: host->device
 * @request:  UNCOMPRESSED: [0x00][0x70]  (no size, no data; all bytes via 0x71).
 *            COMPRESSED:   [0x00][0x70][uncompressed_size:4 LE][first zlib bytes...]
 *              -> remaining zlib bytes stream via CMD_DIRECT_WRITE_DATA (0x71).
 * @response: [0x00][0x70] (RESP_DIRECT_WRITE_START_ACK) on accept.
 * @errors:   [0xFF][0x70] (RESP_DIRECT_WRITE_ERROR path) on setup failure.
 * @state:    begins a full-frame image session; a new START aborts any prior one.
 * @limits:   START plaintext <= 200 bytes (encrypted: fit within 154 budget).
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_DIRECT_WRITE_START         0x0070u

/* --------------------------------------------------------------------------
 * @opcode: 0x0071   @name: CMD_DIRECT_WRITE_DATA   @dir: host->device
 * @request:  [0x00][0x71][image_data...]  (chunk of the image / zlib stream)
 * @response: [0x00][0x71] (RESP_DIRECT_WRITE_DATA_ACK) per chunk.
 * @errors:   [0xFF][0x71] on write/decompress/overflow; for a partial (0x76)
 *            session the NACK data byte is an OD_ERR_PARTIAL_* code.
 * @state:    must follow a 0x70 or 0x76 START; also carries partial-stream data.
 * @limits:   <= 230 bytes (unencrypted) / <= 154 (encrypted).
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_DIRECT_WRITE_DATA          0x0071u

/* --------------------------------------------------------------------------
 * @opcode: 0x0072   @name: CMD_DIRECT_WRITE_END   @dir: host->device
 * @request:  [0x00][0x72][refresh:1]  (+ optional [new_etag:4 BE]; presence by
 *            length). refresh: 0=FULL, 1=FAST/PARTIAL.
 * @response: [0x00][0x72] (RESP_DIRECT_WRITE_END_ACK, data complete) THEN an
 *            unsolicited refresh notification:
 *              [0x00][0x73] RESP_DIRECT_WRITE_REFRESH_SUCCESS (panel updated), or
 *              [0x00][0x74] RESP_DIRECT_WRITE_REFRESH_TIMEOUT (refresh timed out).
 *            [0xFF][0x72] if the transfer was incomplete (missing bytes / zlib flush).
 * @errors:   NACK on incompleteness; refresh reported via 0x73/0x74.
 * @state:    finalizes the 0x70 session and triggers the panel refresh.
 * @limits:   -
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * -------------------------------------------------------------------------- */
#define CMD_DIRECT_WRITE_END           0x0072u

/* --------------------------------------------------------------------------
 * @opcode: 0x0073   @name: CMD_LED_ACTIVATE   @dir: host->device
 * @request:  [0x00][0x73][led_instance:1][flash_config:12]
 * @response: [0x00][0x73] (RESP_LED_ACTIVATE_ACK).
 * @errors:   [0xFF][0x73] on bad instance / config.
 * @state:    session required when security enabled.
 * @limits:   flash_config is a fixed 12-byte typed payload.
 * @targets:  Firmware | NRF54 | Silabs | NRF52811
 * @collision: the RESPONSE byte 0x73 is reused -- see RESP_LED_ACTIVATE_ACK vs
 *            RESP_DIRECT_WRITE_REFRESH_SUCCESS below (disambiguate by direction).
 * -------------------------------------------------------------------------- */
#define CMD_LED_ACTIVATE               0x0073u

/* --------------------------------------------------------------------------
 * @opcode: 0x0075   @name: CMD_LED_STOP   @dir: host->device
 * @request:  [0x00][0x75][led_instance:1]  (stop the LED pattern)
 * @response: [0x00][0x75] (RESP_LED_STOP_ACK).
 * @errors:   [0xFF][0x75] on bad instance.
 * @state:    session required when security enabled.
 * @limits:   -
 * @targets:  Firmware | NRF54 | Silabs      (NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_LED_STOP                   0x0075u

/* --------------------------------------------------------------------------
 * @opcode: 0x0076   @name: CMD_PARTIAL_WRITE_START   @dir: host->device
 * @request:  [0x00][0x76][flags:1][old_etag:4 BE][new_etag:4 BE]
 *                       [x:2 BE][y:2 BE][width:2 BE][height:2 BE][initial stream...]
 *            17-byte fixed header, ALL BIG-ENDIAN (note: PIPE 0x80's partial
 *            extension packs the same geometry LITTLE-endian instead). Remaining
 *            stream bytes follow via 0x71 DATA; the transfer ends with 0x72.
 * @response: [0x00][0x76] on accept (then 0x71 ACKs, then 0x72 END + 0x73/0x74).
 * @errors:   [0xFF][0x76][OD_ERR_PARTIAL_*][0x00]  (namespace scoped to 0x76):
 *              0x01 OD_ERR_PARTIAL_ETAG_MISMATCH, 0x03 OD_ERR_PARTIAL_RECT_OOB,
 *              0x04 OD_ERR_PARTIAL_RECT_ALIGN,     0x05 OD_ERR_PARTIAL_FLAGS,
 *              0x06 OD_ERR_PARTIAL_STREAM,         0x07 OD_ERR_PARTIAL_UNSUPPORTED.
 *            (0x02 unused.) Any geometry/etag NACK clears the device's
 *            displayed_etag. See SECTION 4; do NOT emit OD_ERR_PIPE_START_* here.
 * @state:    requires 1bpp panel; x and width must be multiples of 8; old_etag
 *            must equal the currently displayed etag.
 * @limits:   START plaintext <= 200 bytes.
 * @targets:  Firmware | NRF54      (NOT Silabs, NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_PARTIAL_WRITE_START        0x0076u

/* --------------------------------------------------------------------------
 * @opcode: 0x0077   @name: CMD_BUZZER   @dir: host->device
 * @request:  [0x00][0x77][buzzer_instance:1][outer_repeats:1][n_patterns:1][patterns...]
 * @response: [0x00][0x77] (RESP_BUZZER_ACK).
 * @errors:   [0xFF][0x77] on bad instance / config.
 * @state:    session required when security enabled.
 * @limits:   -
 * @targets:  Firmware | NRF54      (NOT Silabs, NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_BUZZER                     0x0077u

/* --------------------------------------------------------------------------
 * @opcode: 0x0080   @name: CMD_PIPE_WRITE_START   @dir: host->device
 * @request:  10-byte header (22 when partial):
 *              [0x00][0x80][ver:1][flags:1][req_window:1][req_ack_every:1]
 *                          [client_max_frame:2 LE][total_size:4 LE]
 *              --- appended iff flags bit1 (PIPE_FLAG_PARTIAL) ---
 *                          [old_etag:4 LE][x:2 LE][y:2 LE][w:2 LE][h:2 LE]
 *              ver = PIPE_VERSION (1); flags bit0 = PIPE_FLAG_COMPRESSED (zlib),
 *              bit1 = PIPE_FLAG_PARTIAL. total_size = decompressed byte total
 *              (partial: plane_size * 2). All pipe fields are LITTLE-endian.
 * @response: [0x00][0x80][ver:1][max_window:1][max_ack_every:1]
 *                        [max_frame:2 LE][resp_flags:1]
 *              resp_flags bit0 = selective-repeat supported, bit1 = partial
 *              accepted. Device echoes its negotiated maxima (min-rule applies).
 * @errors:   [0xFF][0x80][OD_ERR_PIPE_START_*][0x00]  (namespace scoped to the
 *            0x80 START, DISTINCT from the 0x76 OD_ERR_PARTIAL_* set):
 *              0x01 OD_ERR_PIPE_START_BAD_HEADER   (bad length or version),
 *              0x02 OD_ERR_PIPE_START_UNKNOWN_FLAG,
 *              0x03 OD_ERR_PIPE_START_SIZE_MISMATCH,
 *              0x05 OD_ERR_PIPE_START_ETAG_MISMATCH        (partial),
 *              0x06 OD_ERR_PIPE_START_PARTIAL_UNSUPPORTED,
 *              0x07 OD_ERR_PIPE_START_RECT_INVALID          (partial).
 *            (0x04 unused.) See SECTION 4; do NOT emit OD_ERR_PARTIAL_* here --
 *            0x03/0x05/0x06/0x07 mean different things in the two namespaces.
 * @state:    a new START aborts any in-flight transfer; seq resets to 0.
 * @limits:   window/ack_every 1..32; frame <= PIPE_MAX_FRAME (244).
 * @targets:  Firmware      (NOT NRF54, NOT Silabs, NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_PIPE_WRITE_START           0x0080u

/* --------------------------------------------------------------------------
 * @opcode: 0x0081   @name: CMD_PIPE_WRITE_DATA   @dir: host->device (and the
 *          device->host ACK/NACK opcode)
 * @request:  [0x00][0x81][seq:1][data...]
 *              seq = chunk index mod 256 (reset to 0 by each 0x80 START).
 *              When encrypted, [seq][data] is the inner plaintext payload.
 * @response: SACK ACK (7 bytes): [0x00][0x81][highest_seen:1][ack_mask:4 LE]
 *              mask bit i (LSB first) = chunk (highest_seen-1-i) received;
 *              highest_seen is implicitly acked. ACK cadence = negotiated N.
 *            NACK (8 bytes, FATAL): [0xFF][0x81][err:1][highest_seen:1][ack_mask:4 LE]
 *              err: 0x02 decompress error, 0x03 write/size error, 0x04 protocol.
 * @errors:   see NACK above; after a NACK, further 0x81 frames are discarded
 *            until the next 0x80 START or disconnect.
 * @state:    out-of-order frames buffered in a 33-slot reorder queue; window W.
 * @limits:   data <= frame_eff - PIPE_FRAME_OVERHEAD (3); frame_eff <= 244.
 * @targets:  Firmware      (NOT NRF54, NOT Silabs, NOT NRF52811)
 * -------------------------------------------------------------------------- */
#define CMD_PIPE_WRITE_DATA            0x0081u

/* --------------------------------------------------------------------------
 * @opcode: 0x0082   @name: CMD_PIPE_WRITE_END   @dir: host->device
 * @request:  [0x00][0x82][refresh:1]  (+ optional [new_etag:4 BE]; presence by
 *            length). Full-frame: refresh 0=FULL,1=FAST. Partial-negotiated:
 *            0=FULL,1=FAST,2/absent=PARTIAL.
 * @response: a tail-flush SACK [0x00][0x81]... first, then
 *              [0x00][0x82] end-ACK, then the refresh notification
 *              [0x00][0x73] success or [0x00][0x74] timeout.
 *            [0xFF][0x82] if incomplete (a hole remains / byte count short).
 * @errors:   NACK [0xFF][0x82] on incompleteness; refresh via 0x73/0x74.
 * @state:    client must not send END until every chunk is acked.
 * @limits:   -
 * @targets:  Firmware      (NOT NRF54, NOT Silabs, NOT NRF52811)
 * @collision: 0x0082 is PIPE_WRITE_END here. NFC moved OFF 0x0082 to 0x0083 in
 *            protocol v2 precisely to end the historical opcode clash.
 * -------------------------------------------------------------------------- */
#define CMD_PIPE_WRITE_END             0x0082u

/* --------------------------------------------------------------------------
 * @opcode: 0x0083   @name: CMD_NFC_ENDPOINT   @dir: host->device
 * @request:  [0x00][0x83][sub:1][sub-specific payload...]  where sub =
 *              NFC_SUB_READ        0x00 : (no more payload) read the tag.
 *              NFC_SUB_WRITE       0x01 : [rec_type:1][len:2 BE][payload:len]
 *                                          single-shot write (<= ~120 B app data).
 *              NFC_SUB_WRITE_START 0x10 : [rec_type:1][total_len:2 BE]
 *                                          begin a chunked write (buffer <= 512 B).
 *              NFC_SUB_WRITE_DATA  0x11 : [chunk...]  (append; needs active START)
 *              NFC_SUB_WRITE_END   0x12 : (no payload) commit; received==total.
 *            rec_type is one of OD_NFC_REC_* (below).
 * @response: NOTE the 3rd byte is an NFC SUB-STATUS, a DIFFERENT field from the
 *            opcode echo -- it happens to reuse values 0x80/0x81/0x82:
 *              READ ok : [0x00][0x83][NFC_STATUS_READ_DATA 0x80][rec_type:1][len:2 BE][data:len]
 *              WRITE ok (single / chunk END): [0x00][0x83][NFC_STATUS_WRITE_COMMITTED 0x81]
 *              WRITE_START / WRITE_DATA ok  : [0x00][0x83][NFC_STATUS_CHUNK_ACCEPTED 0x82]
 * @errors:   [0xFF][0x83][0xFF][NFC_ERR_*] (4-byte NACK; 4th byte is the code):
 *              0x01 malformed/short, 0x02 read failed, 0x03 tag write failed,
 *              0x04 unknown sub-command, 0x05 invalid rec_type,
 *              0x06 bad total_len (0 or >512), 0x07 chunk without active START
 *              (or wrong connection), 0x08 chunk overflow past total_len,
 *              0x09 END with received != total_len.
 * @state:    chunked write is per-connection: START binds the connection; DATA
 *            from another connection -> 0x07. Buffer size 512 bytes.
 * @limits:   single-write app data threshold ~120 B; chunk buffer 512 B.
 * @targets:  NRF54 | Silabs      (NOT Firmware, NOT NRF52811)
 * @since:    v2
 * @changed:  moved 0x0082 -> 0x0083 in protocol v2 to resolve the collision
 *            with CMD_PIPE_WRITE_END. Old 0x0082 NFC frames are now rejected
 *            (clean cutover, no alias). The NFC sub-status value 0x82
 *            ("chunk accepted") is UNRELATED to the retired 0x0082 opcode --
 *            do not confuse the two.
 * -------------------------------------------------------------------------- */
#define CMD_NFC_ENDPOINT               0x0083u

/* ==========================================================================
 * SECTION 2 -- RESPONSE / STATUS BYTES (8-bit)
 * ==========================================================================
 * A response frame is [status][cmd_echo][data]. The status byte is one of the
 * three shared values; cmd_echo is the low byte of the command, and the RESP_*
 * echo constants below mirror that low byte.
 * ========================================================================== */

/* Shared status byte (frame position 0). */
#define RESP_ACK                       0x00u   /* success */
#define RESP_NACK                      0xFFu   /* failure; data[0] often an error code */
#define RESP_AUTH_REQUIRED             0xFEu   /* gated command sent without a live session */

/* Command-echo bytes (frame position 1) = low byte of the matching CMD_*. */
#define RESP_CONFIG_READ               0x40u
#define RESP_CONFIG_WRITE              0x41u
#define RESP_CONFIG_CHUNK              0x42u
#define RESP_FIRMWARE_VERSION          0x43u
#define RESP_MSD_READ                  0x44u
#define RESP_CONFIG_CLEAR              0x45u
#define RESP_AUTHENTICATE              0x50u
#define RESP_ENTER_DFU                 0x51u
#define RESP_POWER_OFF                 0x52u
#define RESP_DEEP_SLEEP                0x53u

/* Direct-write family (Firmware / NRF52811 + shared). */
#define RESP_DIRECT_WRITE_START_ACK        0x70u
#define RESP_DIRECT_WRITE_DATA_ACK         0x71u
#define RESP_DIRECT_WRITE_END_ACK          0x72u
#define RESP_DIRECT_WRITE_REFRESH_SUCCESS  0x73u   /* unsolicited device->host: panel refreshed OK */
#define RESP_DIRECT_WRITE_REFRESH_TIMEOUT  0x74u   /* unsolicited device->host: refresh timed out */
#define RESP_DIRECT_WRITE_ERROR            0xFFu   /* direct-write NACK status (== RESP_NACK) */

/* LED / buzzer acks. */
#define RESP_LED_ACTIVATE_ACK          0x73u
#define RESP_LED_STOP_ACK              0x75u
#define RESP_BUZZER_ACK                0x77u

/* NFC endpoint echo byte (was 0x82u in protocol v1; see CMD_NFC_ENDPOINT). */
#define RESP_NFC_ENDPOINT              0x83u

/* @collision: RESPONSE byte 0x73 has two meanings, disambiguated by direction
 *   and context:
 *     RESP_LED_ACTIVATE_ACK             -- a REPLY to CMD_LED_ACTIVATE, framed
 *                                          [status][0x73][...]; solicited.
 *     RESP_DIRECT_WRITE_REFRESH_SUCCESS -- an UNSOLICITED device->host refresh
 *                                          notification following a 0x72/0x82 END.
 *   Same byte value, distinct semantics; a client keys off which command it
 *   just issued (LED activate vs image END) to interpret the 0x73. */

/* ==========================================================================
 * SECTION 3 -- AUTHENTICATE STATUS (3rd byte of a 0x50 response)
 * ========================================================================== */
#define AUTH_STATUS_CHALLENGE          0x00u
#define AUTH_STATUS_SUCCESS            AUTH_STATUS_CHALLENGE  /* alias: step-2 success == challenge value */
#define AUTH_STATUS_FAILED             0x01u   /* wrong key */
#define AUTH_STATUS_ALREADY            0x02u   /* already authenticated */
#define AUTH_STATUS_NOT_CONFIG         0x03u   /* encryption not configured */
#define AUTH_STATUS_RATE_LIMIT         0x04u   /* >=10 attempts / 60 s */
#define AUTH_STATUS_ERROR              0xFFu   /* generic error */

/* ==========================================================================
 * SECTION 4 -- OPCODE-SCOPED NACK ERROR NAMESPACES
 * ==========================================================================
 * NACK error bytes are opcode-SCOPED, never global: data[0] of a NACK is only
 * meaningful once you know the echoed opcode (byte 1). This section holds the
 * distinct, PREFIXED error namespaces -- one per handler -- so a byte value can
 * be reused with different meanings and stay unambiguous on the wire. The NFC
 * namespace lives in SECTION 5 (NFC_ERR_*, the 4th byte of
 * [0xFF][0x83][0xFF][err]); it is cross-referenced but NOT redefined here.
 *
 * SCOPING RULE
 *   The 0x76 (OD_ERR_PARTIAL_*) and 0x80-START (OD_ERR_PIPE_START_*) sets below
 *   DELIBERATELY REUSE byte values 0x03, 0x05, 0x06 and 0x07 with DIFFERENT
 *   meanings. They must NEVER be cross-applied: the echoed opcode is the only
 *   thing that disambiguates them. E.g. byte 0x03 == OD_ERR_PARTIAL_RECT_OOB
 *   under a 0x76 NACK but OD_ERR_PIPE_START_SIZE_MISMATCH under a 0x80 NACK.
 *
 * IMPLEMENTATION GUIDANCE (for firmware handlers adopting these constants)
 *   - A handler for CMD_PARTIAL_WRITE_START (0x76), and the 0x71/0x72 frames of
 *     that partial session, MUST emit ONLY OD_ERR_PARTIAL_* codes.
 *   - A handler for CMD_PIPE_WRITE_START (0x80) MUST emit ONLY
 *     OD_ERR_PIPE_START_* codes.
 *   - The NFC handler (CMD_NFC_ENDPOINT 0x83) MUST emit ONLY NFC_ERR_* codes
 *     (SECTION 5).
 *   - The CMD_POWER_OFF handler (0x52) MUST emit ONLY OD_ERR_POWER_OFF_*
 *     codes (SECTION 4d); the CMD_DEEP_SLEEP handler (0x53) ONLY
 *     OD_ERR_DEEP_SLEEP_* codes (SECTION 4c).
 *   A code from one namespace must NEVER appear in another handler's NACK. The
 *   overlapping raw byte values are intentional and safe ONLY because the echoed
 *   opcode scopes them -- pick the constant by which opcode you are answering.
 * --------------------------------------------------------------------------
 * 4a. PARTIAL-WRITE errors -- scope: CMD_PARTIAL_WRITE_START (0x76) and its
 *     0x71 DATA / 0x72 END frames.  NACK frame: [0xFF][0x76][err][0x00].
 *     (0x02 is intentionally unused in this set.)
 * -------------------------------------------------------------------------- */
#define OD_ERR_PARTIAL_ETAG_MISMATCH   0x01u   /* old_etag != displayed etag */
#define OD_ERR_PARTIAL_RECT_OOB        0x03u   /* rectangle out of panel bounds */
#define OD_ERR_PARTIAL_RECT_ALIGN      0x04u   /* x / width not a multiple of 8 */
#define OD_ERR_PARTIAL_FLAGS           0x05u   /* bad / unsupported flags */
#define OD_ERR_PARTIAL_STREAM          0x06u   /* stream / length error */
#define OD_ERR_PARTIAL_UNSUPPORTED     0x07u   /* partial write unsupported (e.g. not 1bpp) */

/* --------------------------------------------------------------------------
 * 4b. PIPE-START errors -- scope: CMD_PIPE_WRITE_START (0x80) ONLY.
 *     NACK frame: [0xFF][0x80][err][0x00].  DISTINCT namespace from 4a: the
 *     shared byte values (0x03/0x05/0x06/0x07) mean different things here.
 *     (0x04 is unused in this set.)  Note: the in-flight 0x81 PIPE_WRITE_DATA
 *     NACK carries its OWN handler-local err byte (see CMD_PIPE_WRITE_DATA
 *     @response) and is not part of this START namespace.
 * -------------------------------------------------------------------------- */
#define OD_ERR_PIPE_START_BAD_HEADER          0x01u   /* bad length or version */
#define OD_ERR_PIPE_START_UNKNOWN_FLAG        0x02u   /* unknown flags bit set */
#define OD_ERR_PIPE_START_SIZE_MISMATCH       0x03u   /* total_size inconsistent */
#define OD_ERR_PIPE_START_ETAG_MISMATCH       0x05u   /* partial: old_etag mismatch */
#define OD_ERR_PIPE_START_PARTIAL_UNSUPPORTED 0x06u   /* partial mode not supported */
#define OD_ERR_PIPE_START_RECT_INVALID        0x07u   /* partial: invalid rectangle */

/* --------------------------------------------------------------------------
 * 4c. DEEP-SLEEP errors -- scope: CMD_DEEP_SLEEP (0x0053) ONLY.
 *     NACK frame: [0xFF][0x53][err][0x00].  Byte values 0x00..0x02 are reused
 *     from other namespaces -- scoped by the echoed opcode, as always. Note
 *     0x00 IS a valid error code here (a NACK's data[0], never a success
 *     marker). CLIENT RULE: 0x01/0x02 mean "rejected, device still AWAKE and
 *     reachable"; ONLY 0x00 means deep sleep is unsupported on the target.
 *     Never conflate the two classes.
 * -------------------------------------------------------------------------- */
#define OD_ERR_DEEP_SLEEP_UNSUPPORTED  0x00u   /* target has no deep sleep (nRF) */
#define OD_ERR_DEEP_SLEEP_DISABLED     0x01u   /* deep_sleep_time_seconds == 0 in config */
#define OD_ERR_DEEP_SLEEP_NOT_BATTERY  0x02u   /* power_mode != 1 (mains-powered) */

/* --------------------------------------------------------------------------
 * 4d. POWER-OFF errors -- scope: CMD_POWER_OFF (0x0052) ONLY.
 *     NACK frame: [0xFF][0x52][err][0x00].
 * -------------------------------------------------------------------------- */
#define OD_ERR_POWER_OFF_UNSUPPORTED   0x00u   /* no D-FF power latch on this target */

/* ==========================================================================
 * SECTION 5 -- NFC SUB-PROTOCOL (rides CMD_NFC_ENDPOINT 0x0083)
 * ========================================================================== */

/* NFC sub-command (request payload byte 0). */
#define NFC_SUB_READ                   0x00u
#define NFC_SUB_WRITE                  0x01u   /* single-shot write */
#define NFC_SUB_WRITE_START            0x10u   /* begin chunked write */
#define NFC_SUB_WRITE_DATA             0x11u   /* append chunk */
#define NFC_SUB_WRITE_END              0x12u   /* commit chunked write */

/* NFC record types (rec_type field). Wire bytes an agent must ground on --
 * kept here (not in per-repo constants) for that reason. */
#define OD_NFC_REC_TEXT                0u
#define OD_NFC_REC_URI                 1u
#define OD_NFC_REC_WELL_KNOWN_RAW      2u
#define OD_NFC_REC_MIME                3u
#define OD_NFC_REC_RAW_NDEF            4u

/* NFC IC (tag chip) selector. */
#define OD_NFC_IC_AUTO                 0u
#define OD_NFC_IC_TNB132M              1u

/* NFC response SUB-STATUS (3rd byte of a 0x83 response, AFTER the
 * [status][echo] pair). This is a DIFFERENT field from the opcode echo; the
 * value 0x82 here means "chunk accepted" and is UNRELATED to the retired
 * 0x0082 NFC opcode. */
#define NFC_STATUS_READ_DATA           0x80u   /* [.. 0x80][rec_type][len:2 BE][data] */
#define NFC_STATUS_WRITE_COMMITTED     0x81u   /* single write / chunk END committed */
#define NFC_STATUS_CHUNK_ACCEPTED      0x82u   /* WRITE_START or WRITE_DATA accepted */

/* NFC NACK error codes (4th byte of [0xFF][0x83][0xFF][err]). */
#define NFC_ERR_MALFORMED              0x01u   /* malformed / short frame */
#define NFC_ERR_READ_FAILED            0x02u   /* tag read failed */
#define NFC_ERR_TAG_WRITE_FAILED       0x03u   /* tag write failed */
#define NFC_ERR_UNKNOWN_SUBCMD         0x04u   /* unknown sub-command byte */
#define NFC_ERR_INVALID_REC_TYPE       0x05u   /* rec_type not in OD_NFC_REC_* */
#define NFC_ERR_BAD_TOTAL_LEN          0x06u   /* total_len == 0 or > 512 */
#define NFC_ERR_CHUNK_NO_START         0x07u   /* DATA/END without active START (or wrong connection) */
#define NFC_ERR_CHUNK_OVERFLOW         0x08u   /* chunk would exceed total_len */
#define NFC_ERR_END_LEN_MISMATCH       0x09u   /* END with received != total_len */

/* ==========================================================================
 * SECTION 6 -- PIPE SLIDING-WINDOW WIRE CONSTANTS (0x0080..0x0082)
 * ========================================================================== */
#define PIPE_VERSION                   0x01u   /* carried in 0x80 request/response ver field */
#define PIPE_FLAG_COMPRESSED           0x01u   /* flags bit0: streamed bytes are zlib-compressed */
#define PIPE_FLAG_PARTIAL              0x02u   /* flags bit1: partial-region refresh */
#define PIPE_MAX_FRAME                 244u    /* max on-wire frame size (GATT write ceiling) */
#define PIPE_FRAME_OVERHEAD            3u      /* plaintext 0x81 header: cmd(2) + seq(1) */
#define PIPE_ACK_MASK_BITS             32u     /* SACK ack_mask width in bits */
#define OD_PIPE_MAX_PAYLOAD            244u    /* max payload buffer used by pipe/NFC responses */

/* ==========================================================================
 * SECTION 7 -- CHUNK / SIZE BUDGETS
 * ========================================================================== */
#define OD_BLE_MAX_FRAME               256u    /* max BLE ATT MTU: opcode(1) + handle(2) + value, inclusive; usable single-write value is OD_BLE_MAX_FRAME - 3 = 253 (BLE peer of OD_LAN_MAX_FRAME) */
#define CONFIG_CHUNK_SIZE              200u    /* max config data bytes per chunk */
#define CONFIG_CHUNK_SIZE_WITH_PREFIX  202u    /* first chunked CONFIG_WRITE payload: total(2) + 200 */
#define MAX_CONFIG_CHUNKS              20u     /* upper bound on config chunks in a write */
#define MAX_RESPONSE_DATA_SIZE         100u    /* max bytes in a single notification frame */

/* ==========================================================================
 * SECTION 8 -- ENCRYPTION ENVELOPE (wire-visible sizes)
 * ==========================================================================
 * Envelope: [cmd:2 BE][nonce:16][ciphertext][tag:12]; inner plaintext is
 * [len:1][payload]; AAD = the 2 opcode bytes; AEAD = AES-128-CCM.
 * NOTE: this AES-CCM envelope applies to the BLE transport (and plaintext LAN).
 * On the TLS-PSK LAN channel it MUST NOT be applied -- see SECTION 9 rule 4
 * (origin-gated decrypt: the dispatcher decides by the frame's ORIGIN transport,
 * not the global isEncryptionEnabled() flag).
 * ========================================================================== */
#define BLE_CMD_HEADER_SIZE            2u      /* opcode bytes (also the AAD length) */
#define ENCRYPTION_NONCE_SIZE          16u     /* envelope nonce (CCM nonce = bytes 3..15) */
#define ENCRYPTION_TAG_SIZE            12u     /* CCM authentication tag length */

/* ==========================================================================
 * SECTION 9 -- NETWORK TRANSPORT (LAN)
 * ==========================================================================
 * The OpenDisplay command set can also ride a LAN (WiFi) TCP transport, in
 * addition to BLE. This section is the implement-from-this-file-alone contract
 * for that transport. It is ADDITIVE: it introduces no new opcodes and no
 * struct fields -- the same SECTION 1 commands / SECTION 2 responses travel
 * inside a framed TCP stream. BLE-only peers are unaffected.
 *
 * FRAME FORMAT
 *   Each application message is length-prefixed on the TCP byte stream:
 *       [len:2 LE][payload]
 *     - len     : 16-bit LITTLE-ENDIAN byte count of the payload that follows.
 *     - payload : one command/response frame, EXACTLY as on BLE but WITHOUT the
 *                 GATT MTU segmentation -- i.e. [cmd_hi][cmd_lo][payload...] for
 *                 a request, [status][cmd_echo][data...] for a response.
 *     - The complete WIRE frame (2-byte prefix + payload) MUST NOT exceed
 *       OD_LAN_MAX_FRAME (4096); equivalently, valid payload length is
 *       1..OD_LAN_MAX_PAYLOAD (= OD_LAN_MAX_FRAME - 2 = 4094). len == 0 is
 *       invalid; len > OD_LAN_MAX_PAYLOAD MUST be rejected and the connection
 *       dropped. Capping the WIRE frame (not the payload) at a power of two is
 *       deliberate: receiver buffers sized in wire bytes divide evenly.
 *   A reader MUST handle partial and coalesced TCP reads: read exactly 2 bytes
 *   for len, then exactly len bytes for the payload.
 *
 * NORMATIVE RULES
 *   (1) MODE / PORT SELECTION -- one channel, chosen by the firmware predicate
 *       isEncryptionEnabled() (SecurityConfig.encryption_enabled == 1 AND a
 *       non-zero 16-byte master key):
 *         - false -> PLAINTEXT framed TCP on the configured WifiConfig.server_port
 *                    (default OD_LAN_TCP_PORT if server_port == 0) ONLY.
 *         - true  -> TLS-PSK on the DERIVED port server_port + 1
 *                    (default OD_LAN_TLS_PORT == OD_LAN_TCP_PORT + 1) ONLY.
 *       A flag == 1 with a blank/all-zero key is NOT "encrypted" -- it falls back
 *       to plaintext. The two modes are MUTUALLY EXCLUSIVE: exactly one port is
 *       served at a time. Clients learn the live port from the mDNS SRV record
 *       (see rule 6) or authoritatively via CONFIG_READ (0x26): read server_port,
 *       add 1 for the TLS port.
 *   (2) NO PIPE ON LAN -- the sliding-window image PIPE (0x0080 / 0x0081 / 0x0082,
 *       SECTION 6) MUST NOT be used on this transport; TCP already provides
 *       ordered, reliable, flow-controlled delivery. Stream image data via
 *       DIRECT_WRITE with chunks of at most OD_LAN_MAX_PAYLOAD - 2 data bytes.
 *   (3) ONE CLIENT AT A TIME -- the device serves a single network client; a new
 *       connection EVICTS the prior one. Clients MAY stay persistently connected;
 *       the server drops a client only after OD_LAN_READ_TIMEOUT_S with no
 *       traffic. Any valid command resets the idle timer.
 *   (4) TLS-PSK / NO DOUBLE ENCRYPTION -- on the encrypted channel the PSK is
 *       DERIVED from the SecurityConfig master key via the existing CMAC KDF
 *       (the same key derivation used elsewhere); prefer an ECDHE-PSK
 *       ciphersuite. The TLS handshake IS the authentication: the app-layer
 *       AUTHENTICATE (0x50) exchange is NOT used on this port. There is NO double
 *       encryption -- INSIDE the TLS tunnel, frames are PLAINTEXT opcode frames
 *       and the app-layer AES-CCM envelope (SECTION 8) MUST NOT be applied. The
 *       dispatcher MUST gate app-layer decrypt/encrypt on the frame's ORIGIN
 *       transport (TLS-origin bypasses the AEAD envelope; BLE-origin keeps it),
 *       NOT on the global isEncryptionEnabled() flag.
 *   (5) IDENTITY -- the BLE MAC is the cross-transport identity anchor. The BLE
 *       manufacturer-specific data (MSD) is VOLATILE telemetry, NOT identity.
 *       Correlate a discovered LAN device to its BLE identity via the `mac` mDNS
 *       TXT key (rule 6); the IP address is only a locator, never an identity.
 *   (6) DISCOVERY -- mDNS / DNS-SD. Service OD_LAN_MDNS_SERVICE
 *       ("_opendisplay._tcp"; FQDN "_opendisplay._tcp.local."). TXT record keys:
 *         mac : REQUIRED. The advertised BLE address, lowercase colon-separated
 *               (e.g. "aa:bb:cc:dd:ee:ff") -- this is the ADVERTISED BLE address,
 *               NOT the eFuse / WiFi MAC.
 *         tls : REQUIRED. "0" or "1"; mirrors isEncryptionEnabled() (1 = the
 *               served channel is TLS-PSK on server_port + 1).
 *         fw  : RECOMMENDED. Firmware version "major.minor".
 *         cm  : RECOMMENDED. communication_modes, 2 lowercase hex digits.
 *         id  : OPTIONAL. device_id, 8 hex digits.
 *         pv  : OPTIONAL. protocol version (e.g. "2.2").
 *         msd : EXISTING. 28 lowercase hex digits of volatile telemetry; throttle
 *               republish to >= 400 ms.
 *       The SRV record port MUST equal the ACTIVE mode's port: server_port when
 *       tls == 0, server_port + 1 when tls == 1. On a config change that flips
 *       isEncryptionEnabled(), the device MUST re-register the service on the new
 *       port (and update the `tls` TXT) so clients converge on the live channel.
 * ========================================================================== */
#define OD_LAN_TCP_PORT                2446u   /* DEFAULT plaintext port; live port = WifiConfig.server_port (this default if 0) */
#define OD_LAN_TLS_PORT                2447u   /* DEFAULT TLS-PSK port; DERIVED live = server_port + 1 (== OD_LAN_TCP_PORT + 1) */
#define OD_LAN_MAX_FRAME               4096u   /* max WIRE frame: [len:2 LE] prefix + payload, inclusive */
#define OD_LAN_MAX_PAYLOAD             4094u   /* max payload bytes after the prefix; == OD_LAN_MAX_FRAME - 2 */
#define OD_LAN_MDNS_SERVICE            "_opendisplay._tcp"  /* DNS-SD service; FQDN "_opendisplay._tcp.local." */
#define OD_LAN_READ_TIMEOUT_S          30u     /* idle timeout: drop a client after this many seconds of no traffic */

#endif /* OPENDISPLAY_PROTOCOL_H */
