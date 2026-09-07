#!/usr/bin/env python3
"""Flash an RP2040 board by copying a .uf2 onto its bootloader drive.

This is the pump board's `upload_command` -- see [env:pump] in platformio.ini.
It exists because the default `picotool` protocol does not work on a stock
Windows install: picotool drives the RP2040's PICOBOOT interface (interface 1 of
the BOOTSEL composite device), for which Windows ships no driver, so the upload
dies with

    Device ... appears to be a RP2040 device in BOOTSEL mode, but picotool was
    unable to connect. You may need to install a driver via Zadig.

Interface 0 of that same device is ordinary USB mass storage, which Windows has
always supported -- it is the RPI-RP2 drive. Copying a .uf2 onto it is the
flashing method Raspberry Pi documents first, it needs no driver, and it is what
this does. Installing WinUSB with Zadig is the alternative; it is avoided here
because it is a manual step on every machine that ever builds this firmware, and
pointing Zadig at the wrong interface breaks the drive as well.

Usage:  flash_uf2.py <firmware.uf2> [serial_port]

With the board already in BOOTSEL (hold BOOT while plugging in) the drive is
there and it just copies. With the board running the firmware, it opens the
serial port at 1200 baud and closes it again -- the arduino-pico core treats
that as "reboot into the bootloader", which is the same trick the Arduino IDE
uses -- then waits for the drive and copies. So no button press is needed for a
reflash, only for the very first one or to recover a board whose firmware no
longer enumerates.
"""
import os
import shutil
import string
import sys
import time

BOOTSEL_VID = 0x2E8A  # Raspberry Pi; PID 0x0003 in BOOTSEL, 0x000A running
DRIVE_WAIT_S = 20.0


def find_bootsel_drive():
    """Return the path of a mounted RP2040 bootloader volume, or None.

    Identified by INFO_UF2.TXT rather than the volume label: the file is part of
    the bootloader's contract and names the board, whereas a label can be
    anything once someone has reformatted a stray USB stick to RPI-RP2.
    """
    candidates = []
    if os.name == "nt":
        candidates = [f"{d}:\\" for d in string.ascii_uppercase]
    else:
        for base in ("/media", "/run/media", "/Volumes"):
            if os.path.isdir(base):
                for entry in os.listdir(base):
                    path = os.path.join(base, entry)
                    candidates.append(path)
                    if os.path.isdir(path):  # /media/<user>/<volume>
                        candidates += [os.path.join(path, s) for s in os.listdir(path)]

    for path in candidates:
        info = os.path.join(path, "INFO_UF2.TXT")
        try:
            if os.path.isfile(info):
                with open(info, "r", errors="replace") as fh:
                    if "RP2" in fh.read():
                        return path
        except OSError:
            continue  # empty card reader, permission denied, disconnected mid-scan
    return None


def touch_1200(port=None):
    """Ask a running board to reboot into BOOTSEL, by opening its port at 1200 baud.

    Returns the port that was touched, or None if there was no port to touch.

    Whether the touch *worked* is deliberately not decided here. The board resets
    the instant it sees the 1200 baud open, so on Windows the port disappears
    underneath pyserial and it reports

        Cannot configure port ... A device which does not exist was specified

    which is indistinguishable from a real failure and is in fact the success
    path -- measured: that exception, board in BOOTSEL, drive mounted. So every
    exception is swallowed and main() polls for the bootloader drive instead.
    The drive is direct evidence; the exception is not evidence at all.
    """
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        return None

    if port is None:
        matches = [p.device for p in list_ports.comports() if p.vid == BOOTSEL_VID]
        if len(matches) != 1:
            return None
        port = matches[0]

    try:
        # Opening and closing at 1200 baud IS the signal; nothing is written.
        serial.Serial(port, 1200).close()
    except Exception:  # noqa: BLE001 -- see the docstring; this proves nothing
        pass
    return port


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    uf2 = sys.argv[1]
    # An empty argv[2] counts as "not given": PlatformIO substitutes $UPLOAD_PORT
    # with an empty string when no --upload-port was passed, and treating that as
    # a port name would skip the auto-detection below and then fail to open "".
    port = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None

    if not os.path.isfile(uf2):
        sys.exit(f"no such image: {uf2}  (run `pio run -e pump` first)")

    drive = find_bootsel_drive()
    if drive is None:
        print("No bootloader drive; rebooting the board into BOOTSEL...")
        touched = touch_1200(port)
        if touched is None:
            print("  (no RP2040 serial port to reset -- hoping for a manual BOOTSEL)")
        else:
            print(f"  1200 baud touch on {touched}, waiting for the drive...")

        deadline = time.time() + DRIVE_WAIT_S
        while time.time() < deadline:
            drive = find_bootsel_drive()
            if drive:
                break
            time.sleep(0.25)

        if drive is None:
            sys.exit(
                f"no bootloader drive appeared within {DRIVE_WAIT_S:.0f}s.\n"
                f"Hold the BOOT button while plugging the board in, then re-run."
            )

    print(f"Copying {os.path.basename(uf2)} -> {drive}")
    try:
        shutil.copy(uf2, drive)
    except OSError as exc:
        # The bootloader reboots the moment the last block lands, so the handle
        # can die during the final flush. That is the success path, not an error.
        if getattr(exc, "errno", None) not in (5, 22, 32):
            raise
        print(f"  (drive disconnected during copy: {exc.strerror} -- normal)")

    print("Done. The board reboots into the new firmware; a serial port")
    print("reappears a second or two later.")


if __name__ == "__main__":
    main()
