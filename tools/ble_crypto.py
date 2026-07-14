"""
BLE session encryption for OpenDisplay's 0x0050 authenticate handshake.

Mirrors src/encryption.cpp: AES-128-CMAC challenge/response, session key/id
derivation, and AES-CCM (13-byte nonce, 12-byte tag) command/response framing.

Requires the `cryptography` package (pip install cryptography).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from typing import Any

SESSION_LABEL = b"OpenDisplay session"


def _require_cryptography() -> Any:
    try:
        import cryptography  # noqa: F401
    except ImportError:
        print("ERROR: encrypted BLE requires cryptography - run: pip install cryptography", file=sys.stderr)
        sys.exit(1)
    return cryptography


def aes_cmac(key: bytes, message: bytes) -> bytes:
    _require_cryptography()
    from cryptography.hazmat.primitives import cmac
    from cryptography.hazmat.primitives.ciphers import algorithms

    c = cmac.CMAC(algorithms.AES(key))
    c.update(message)
    return c.finalize()


def aes_ecb_encrypt_block(key: bytes, block: bytes) -> bytes:
    if len(block) != 16:
        raise ValueError("aes_ecb_encrypt_block: block must be exactly 16 bytes")
    _require_cryptography()
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

    encryptor = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
    return encryptor.update(block) + encryptor.finalize()


def aes_ccm_encrypt(key: bytes, nonce: bytes, plaintext: bytes, ad: bytes, tag_length: int = 12) -> tuple[bytes, bytes]:
    _require_cryptography()
    from cryptography.hazmat.primitives.ciphers.aead import AESCCM

    ct_and_tag = AESCCM(key, tag_length=tag_length).encrypt(nonce, plaintext, ad)
    return ct_and_tag[:-tag_length], ct_and_tag[-tag_length:]


def aes_ccm_decrypt(key: bytes, nonce: bytes, ciphertext: bytes, tag: bytes, ad: bytes) -> bytes:
    _require_cryptography()
    from cryptography.hazmat.primitives.ciphers.aead import AESCCM

    return AESCCM(key, tag_length=len(tag)).decrypt(nonce, ciphertext + tag, ad)


def derive_session_key(master_key: bytes, client_nonce: bytes, server_nonce: bytes, device_id: bytes) -> bytes:
    cmac_input = SESSION_LABEL + b"\x00" + device_id + client_nonce + server_nonce + b"\x00\x80"
    intermediate = aes_cmac(master_key, cmac_input)
    ecb_input = (1).to_bytes(8, "big") + intermediate[:8]
    return aes_ecb_encrypt_block(master_key, ecb_input)


def derive_session_id(session_key: bytes, client_nonce: bytes, server_nonce: bytes) -> bytes:
    return aes_cmac(session_key, client_nonce + server_nonce)[:8]


def compute_challenge_response(master_key: bytes, server_nonce: bytes, client_nonce: bytes, device_id: bytes) -> bytes:
    return aes_cmac(master_key, server_nonce + client_nonce + device_id)


def compute_server_response(session_key: bytes, server_nonce: bytes, client_nonce: bytes, device_id: bytes) -> bytes:
    return aes_cmac(session_key, server_nonce + client_nonce + device_id)


def encrypt_command(session_key: bytes, session_id: bytes, counter: int, cmd_hi: int, cmd_lo: int, payload: bytes) -> tuple[bytes, int]:
    if len(payload) > 255:
        raise ValueError("encrypt_command: payload exceeds 255 bytes")
    nonce_full = session_id + counter.to_bytes(8, "big")
    ccm_nonce = nonce_full[3:]
    ad = bytes([cmd_hi, cmd_lo])
    plaintext = bytes([len(payload)]) + payload
    ciphertext, tag = aes_ccm_encrypt(session_key, ccm_nonce, plaintext, ad)
    wire = bytes([cmd_hi, cmd_lo]) + nonce_full + ciphertext + tag
    return wire, counter + 1


def decrypt_response(session_key: bytes, data: bytes) -> bytes:
    if len(data) < 2 + 16 + 1 + 12:
        raise ValueError("decrypt_response: frame too short to be an encrypted response")
    cmd = data[:2]
    nonce_full = data[2:18]
    tag = data[-12:]
    ciphertext = data[18:-12]
    ccm_nonce = nonce_full[3:]
    plaintext = aes_ccm_decrypt(session_key, ccm_nonce, ciphertext, tag, cmd)
    payload_len = plaintext[0]
    return cmd + plaintext[1 : 1 + payload_len]


def response_needs_decryption(data: bytes) -> bool:
    if len(data) < 2:
        return False
    if data[0] == 0xFF:
        return False
    command = (data[0] << 8) | data[1]
    if command in (0x0050, 0x0043):
        return False
    if len(data) < 2 + 16 + 1 + 12:
        # Shorter than any valid CCM frame (2B cmd + 16B nonce + >=1B ciphertext +
        # 12B tag) - can only be a plaintext status/ack, e.g. the final config-write
        # ack sent after reloadConfigAfterSave() clears the session server-side.
        return False
    return True


@dataclass
class BleSession:
    master_key: bytes
    session_key: bytes | None = field(default=None)
    session_id: bytes | None = field(default=None)
    counter: int = 0
    authenticated: bool = False

    def encrypt(self, cmd_hi: int, cmd_lo: int, payload: bytes) -> bytes:
        assert self.session_key is not None and self.session_id is not None
        wire, self.counter = encrypt_command(self.session_key, self.session_id, self.counter, cmd_hi, cmd_lo, payload)
        return wire

    def decrypt(self, data: bytes) -> bytes:
        assert self.session_key is not None
        return decrypt_response(self.session_key, data)

    def needs_decryption(self, data: bytes) -> bool:
        return self.authenticated and response_needs_decryption(data)
