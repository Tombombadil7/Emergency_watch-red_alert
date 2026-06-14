from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import hashes, serialization
import hashlib

def sign_files(private_key_pem: bytes, *file_paths: str, out_sig: str):
    """Hash all files concatenated in order, sign with ECDSA P-256, write raw 64-byte sig."""
    key = serialization.load_pem_private_key(private_key_pem, password=None)

    h = hashlib.sha256()
    for path in file_paths:
        with open(path, "rb") as f:
            h.update(f.read())
    digest = h.digest()  # 32 bytes

    # Sign the raw digest (not re-hashed — use Prehashed)
    from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature
    sig_der = key.sign(digest, ec.ECDSA(hashes.Prehashed()))  # DER format
    r, s = decode_dss_signature(sig_der)

    # Write as raw 64 bytes (32 R + 32 S) — no DER overhead
    raw_sig = r.to_bytes(32, "big") + s.to_bytes(32, "big")
    with open(out_sig, "wb") as f:
        f.write(raw_sig)

# Geo: two files, one sig
sign_files(pem, "geo_regions.bin", "geo_zones.bin", out_sig="geo.sig")

# OTA: one file, one sig
sign_files(pem, "firmware.bin", out_sig="firmware.sig")