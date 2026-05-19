#!/usr/bin/env python3
"""Generate SslCert.h from _smb.pfx (run gen_ssl_header after creating PFX)."""
from pathlib import Path
from cryptography.hazmat.primitives.serialization import (
    Encoding,
    NoEncryption,
    PrivateFormat,
    pkcs12,
)

ROOT = Path(__file__).resolve().parent.parent / "SuperMarketBot-IOT"
PFX = ROOT / "_smb.pfx"
OUT = ROOT / "SslCert.h"

def main() -> None:
    pfx = PFX.read_bytes()
    key, cert, _ = pkcs12.load_key_and_certificates(pfx, b"smb")
    cert_pem = cert.public_bytes(Encoding.PEM).decode()
    key_pem = key.private_bytes(
        Encoding.PEM, PrivateFormat.TraditionalOpenSSL, NoEncryption()
    ).decode()
    body = f"""#ifndef SSL_CERT_H
#define SSL_CERT_H

static const char SMB_SSL_CERT[] PROGMEM = R"PEM(
{cert_pem})PEM";

static const char SMB_SSL_KEY[] PROGMEM = R"PEM(
{key_pem})PEM";

#endif
"""
    OUT.write_text(body, encoding="utf-8")
    print(f"Wrote {OUT} ({len(body)} bytes)")

if __name__ == "__main__":
    main()
