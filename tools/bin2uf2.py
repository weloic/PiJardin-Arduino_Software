#!/usr/bin/env python3
"""Convert a raw .bin firmware image into a UF2 file for drag-and-drop flashing.

The XIAO SAMD21 bootloader exposes a USB mass-storage drive ("Arduino") after a
double-tap of the reset pad; copying a .uf2 onto it flashes the board with no
tooling installed at all. This script turns PlatformIO's firmware.bin into that
.uf2 -- see "Standalone firmware image" in README.md.

Usage: python bin2uf2.py firmware.bin firmware.uf2 [base_addr_hex] [family_hex]

Defaults match the XIAO SAMD21: base 0x2000 (application start, just above the
bootloader) and family 0x68ed2b88 (SAMD21). The base MUST stay 0x2000 -- writing
the image at 0x0000 would overwrite the bootloader.
"""
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
FLAG_FAMILY_ID = 0x00002000

SAMD21_FAMILY = 0x68ED2B88
SAMD21_APP_BASE = 0x2000


def convert(data, base=SAMD21_APP_BASE, family=SAMD21_FAMILY):
    """Return the UF2 blob for `data`, as 512-byte blocks of 256-byte payloads."""
    blocks = (len(data) + 255) // 256
    out = bytearray()
    for i in range(blocks):
        chunk = data[i * 256:(i + 1) * 256]
        out += struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0, UF2_MAGIC_START1, FLAG_FAMILY_ID,
            base + i * 256,  # target address of this block
            256,             # payload size
            i, blocks,
            family,
        )
        out += chunk.ljust(476, b"\x00")  # payload padded to the 476-byte data field
        out += struct.pack("<I", UF2_MAGIC_END)
    return bytes(out)


def main(argv):
    if len(argv) < 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    src, dst = argv[1], argv[2]
    base = int(argv[3], 16) if len(argv) > 3 else SAMD21_APP_BASE
    family = int(argv[4], 16) if len(argv) > 4 else SAMD21_FAMILY
    with open(src, "rb") as f:
        data = f.read()
    blob = convert(data, base, family)
    with open(dst, "wb") as f:
        f.write(blob)
    print(f"{src} ({len(data)} B) -> {dst} "
          f"({len(blob)} B, {len(blob) // 512} blocks, base 0x{base:04X})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
