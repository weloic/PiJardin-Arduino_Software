#!/usr/bin/env python3
"""Bench tool for calibrating the pump sensor board against a real installation.

The pump firmware ships with placeholder thresholds (see "Calibration" in
docs/pump.md) and cannot know the right ones: the counts-to-volts relationship
lives in the ZMPT101B's gain pot, so the numbers only exist once the module is
wired to the actual pump with the pot at its actual position. This drives the
board's NDJSON protocol over USB serial so that measurement is a few commands
instead of hand-typed JSON and 400 numbers read off a terminal.

Requires pyserial:  pip install pyserial

    python tools/pump_tune.py status                 # identity, defaults, limits
    python tools/pump_tune.py probe                  # one window + quality checks
    python tools/pump_tune.py plot                   # waveform, for setting the pot
    python tools/pump_tune.py watch                  # live readings, ~1/s
    python tools/pump_tune.py calibrate              # guided; emits the #define block
    python tools/pump_tune.py raw '{"id":1,"cmd":"status"}'

The port is auto-detected when exactly one candidate is present; otherwise pass
--port COM5 (Windows) or --port /dev/ttyACM0.

NOTE ON RESET: the XIAO has native USB, so opening the port at 9600 does NOT
reset the board the way DTR does on a classic Arduino. The {"type":"ready"}
banner therefore usually will NOT appear on connect -- that is normal, not a
fault. This tool proves liveness with a `status` round-trip instead of waiting
for a banner that already went out before it got here.
"""
import argparse
import json
import statistics
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

BAUD = 9600  # matches Serial.begin(9600) and monitor_speed in platformio.ini
ADC_MAX = 4095

# USB vendor IDs worth preferring when guessing the port. The pump board is a
# XIAO RP2040, which enumerates under Raspberry Pi's VID with the arduino-pico
# core; Seeed's own VID is here because the well sensor (a XIAO SAMD21) uses it
# and may well be plugged into the same laptop. Neither is load-bearing -- the
# fallback below accepts any USB serial port, and the role check in main()
# catches a wrong guess with a clear message rather than a confusing reading.
BOARD_VIDS = (0x2E8A, 0x2886)  # Raspberry Pi, Seeed
MID_SCALE = 2048


class Board:
    """One open serial link to a sensor board, speaking newline-delimited JSON."""

    def __init__(self, port, timeout=5.0):
        self.ser = serial.Serial(port, BAUD, timeout=timeout)
        self.port = port
        self._id = 0
        # Native-USB boards enumerate before the sketch is ready to read; a short
        # settle avoids losing the first request into a port that is open but not
        # yet listening.
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def request(self, cmd, **params):
        """Send one request and return the response whose id matches.

        Replies are matched on the echoed id rather than "next line in", so a
        late reply to an earlier request -- or a stray banner after a reset --
        cannot be mistaken for the answer to this one.
        """
        self._id += 1
        want = self._id
        line = json.dumps({"id": want, "cmd": cmd, **params}, separators=(",", ":"))
        self.ser.write((line + "\n").encode())
        self.ser.flush()

        deadline = time.time() + 10.0
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            try:
                msg = json.loads(raw.decode(errors="replace"))
            except json.JSONDecodeError:
                continue  # boot banner fragment, or line noise
            if msg.get("type") == "resp" and msg.get("id") == want:
                return msg
        raise TimeoutError(f"no reply to id {want} ({cmd}) on {self.port}")

    def close(self):
        self.ser.close()


def find_port(explicit=None):
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    known = [p for p in ports if p.vid in BOARD_VIDS]
    candidates = known or [p for p in ports if p.vid is not None]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        sys.exit(
            "no USB serial ports found.\n"
            "  - An RP2040 in BOOTSEL mode presents a DRIVE (RPI-RP2), not a serial\n"
            "    port. If you see that drive, the board is in the bootloader: unplug\n"
            "    and replug WITHOUT holding BOOT, or flash it first.\n"
            "  - Otherwise check the cable actually carries data; charge-only USB-C\n"
            "    cables enumerate nothing at all."
        )
    listing = "\n".join(f"  {p.device}  {p.description}" for p in candidates)
    sys.exit(f"several ports found, pass --port:\n{listing}")


# --- Reporting ----------------------------------------------------------------

def check(resp, mains_hz):
    """Return the list of things wrong with this reading, worst first.

    These are exactly the traps docs/pump.md warns about; having them checked
    mechanically is the point of the tool, because every one of them produces a
    number that looks fine on its own.
    """
    problems = []

    if resp.get("status") == "error":
        problems.append(f"ERROR {resp.get('code')} {resp.get('field', '')}".strip())
        return problems

    bias = resp.get("bias_counts", 0)
    if resp.get("bias_ok") is False:
        problems.append(
            f"bias {bias:.0f} outside the plausible window -- module unpowered, "
            f"output shorted, or the A0 wire is off. NOT a stopped pump."
        )
    elif abs(bias - MID_SCALE) > 250:
        problems.append(f"bias {bias:.0f} is well off mid-scale ({MID_SCALE}); check the supply")

    if resp.get("n_clipped", 0) > 0:
        problems.append(
            f"{resp['n_clipped']} clipped samples -- turn the gain pot DOWN. "
            f"rms_counts understates the real signal until this is 0."
        )

    if resp.get("truncated"):
        problems.append(
            f"window truncated to {resp.get('cycles_eff', 0):.1f} of "
            f"{resp.get('cycles')} cycles -- lower rate_hz or cycles"
        )

    interval = resp.get("interval_us", 0)
    late = resp.get("max_late_us", 0)
    if interval and late > interval * 0.5:
        problems.append(
            f"max_late_us {late} vs interval_us {interval} -- analogRead cannot keep "
            f"up at this rate, spacing is not uniform and freq_hz is unreliable"
        )

    # Only meaningful once there is a signal at all; 0 is the honest pump-off result.
    freq = resp.get("freq_hz", 0)
    if resp.get("rms_counts", 0) > 50 and freq > 0 and abs(freq - mains_hz) > 2.0:
        problems.append(
            f"freq_hz {freq:.2f} is not {mains_hz:.0f} Hz -- this looks like induced "
            f"hum or pickup, not the pump's feed"
        )

    return problems


def summarise(resp, mains_hz, prefix=""):
    if resp.get("status") == "error":
        print(f"{prefix}status=error  code={resp.get('code')} "
              f"field={resp.get('field', '-')}  bias={resp.get('bias_counts', 0):.0f}")
    else:
        vrms = f"  vrms={resp['vrms']:.1f}V" if "vrms" in resp else ""
        print(
            f"{prefix}state={resp.get('state', '?'):<9} "
            f"rms={resp.get('rms_counts', 0):8.2f}  "
            f"bias={resp.get('bias_counts', 0):7.1f}  "
            f"min/max={resp.get('min_counts', 0)}/{resp.get('max_counts', 0)}  "
            f"clip={resp.get('n_clipped', 0)}  "
            f"freq={resp.get('freq_hz', 0):6.2f}Hz  "
            f"late={resp.get('max_late_us', 0)}us{vrms}"
        )
    for p in check(resp, mains_hz):
        print(f"    ! {p}")


def ascii_plot(samples, height=19, width=76):
    """Min/max envelope plot of the waveform, one column per bucket.

    Min/max per bucket rather than every k-th sample on purpose: decimating a
    50 Hz sine by picking single points aliases it into a slower-looking sine,
    and worse, it can step straight over a flat top and hide the clipping you
    opened the plot to look for. An envelope cannot.
    """
    if not samples:
        print("(no samples -- pass --dump-n above 0)")
        return

    n = len(samples)
    per_col = max(1, -(-n // width))  # ceil, so the plot never overruns `width`
    cols = []
    for c in range(0, n, per_col):
        bucket = samples[c:c + per_col]
        cols.append((min(bucket), max(bucket)))

    lo, hi = min(c[0] for c in cols), max(c[1] for c in cols)
    span = max(hi - lo, 1)

    def row_of(v):
        return int((hi - v) / span * (height - 1))

    grid = [[" "] * len(cols) for _ in range(height)]
    for x, (cmin, cmax) in enumerate(cols):
        top, bottom = row_of(cmax), row_of(cmin)
        # Bridge to the previous column's range as well, or a steep edge draws as
        # disconnected dashes and the waveform stops looking like one.
        if x > 0:
            prev_top, prev_bottom = row_of(cols[x - 1][1]), row_of(cols[x - 1][0])
            top, bottom = min(top, prev_bottom), max(bottom, prev_top)
        for y in range(top, bottom + 1):
            grid[y][x] = "*"

    print(f"  {'counts':>6}")
    for y, row in enumerate(grid):
        label = f"{hi - y * span / (height - 1):6.0f}"
        print(f"  {label} |{''.join(row)}")
    print(f"  {'':6} +{'-' * len(cols)}")
    print(f"  {'':6}  {n} samples, oldest first, {per_col} per column")

    # Rails are absolute, so say so regardless of the autoscale above.
    if lo <= 4 or hi >= ADC_MAX - 4:
        print("  ! touching a rail -- the peaks are cut off; turn the pot DOWN")
    # Target headroom mirrors the max_counts 3000-3800 window in docs/pump.md:
    # a clear signal that still cannot clip on a slightly high mains day.
    headroom = min(lo, ADC_MAX - hi)
    if headroom < 250:
        verdict = "too little -- turn the pot DOWN"
    elif headroom > 1100:
        verdict = "signal is small -- turn the pot UP"
    else:
        verdict = "good"
    print(f"  headroom to the nearest rail: {headroom} counts ({verdict})")


# --- Commands -----------------------------------------------------------------

def cmd_status(board, args):
    resp = board.request("status")
    for k, v in resp.items():
        print(f"  {k:<20} {v}")


def cmd_probe(board, args):
    resp = board.request("sampling", dump_n=0, **sample_args(args))
    summarise(resp, args.mains_hz)
    if not check(resp, args.mains_hz):
        print("    all quality checks pass")


def cmd_plot(board, args):
    resp = board.request("sampling", dump_n=args.dump_n, **sample_args(args))
    summarise(resp, args.mains_hz)
    print()
    ascii_plot(resp.get("samples", []))


def cmd_watch(board, args):
    print("Ctrl-C to stop. Switch the pump on and off and watch rms move.\n")
    try:
        while True:
            resp = board.request(args.cmd, **sample_args(args))
            summarise(resp, args.mains_hz, prefix=time.strftime("%H:%M:%S  "))
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")


def collect(board, args, n, label):
    """Take n windows and return their rms figures, printing each as it lands."""
    rms = []
    for i in range(n):
        resp = board.request("sampling", dump_n=0, **sample_args(args))
        if resp.get("status") == "error":
            print(f"  [{label} {i+1}/{n}] ERROR {resp.get('code')} -- fix this first")
            for p in check(resp, args.mains_hz):
                print(f"    ! {p}")
            sys.exit(1)
        rms.append(resp["rms_counts"])
        summarise(resp, args.mains_hz, prefix=f"  [{label} {i+1}/{n}] ")
        time.sleep(0.2)
    return rms


def cmd_calibrate(board, args):
    n = args.repeat
    print(__doc__.split("\n")[0])
    print(f"\nTaking {n} windows in each state. Both thresholds are derived from the")
    print("WORST case seen, not the average -- the point is a margin that holds.\n")

    input("1. Switch the pump OFF, wait for it to settle, then press Enter... ")
    off = collect(board, args, n, "off")

    print()
    input("2. Switch the pump ON, wait for it to settle, then press Enter... ")
    on = collect(board, args, n, "on")

    rms_off = max(off)   # worst-case noise floor
    rms_on = min(on)     # worst-case signal
    print(f"\n  noise floor (worst of {n}): {rms_off:.2f} counts  "
          f"[median {statistics.median(off):.2f}]")
    print(f"  signal      (worst of {n}): {rms_on:.2f} counts  "
          f"[median {statistics.median(on):.2f}]")

    if rms_on <= rms_off * 3:
        print("\n  ! The two states are not separated. Either the pot gain is far too low,")
        print("  ! or -- check this first -- the module is on the WRONG SIDE of the")
        print("  ! contactor and is seeing house mains in both states. Nothing in this")
        print("  ! tool can tell those apart; go and look at the wiring.")
        sys.exit(1)

    off_counts = max(5.0 * rms_off, 20.0)
    on_counts = rms_on / 3.0
    if on_counts <= off_counts:
        # Separated, but not by enough for the 5x/÷3 rule to leave a gap. Split the
        # difference geometrically rather than emitting an inverted pair, which the
        # firmware would reject as bad_param anyway.
        mid = (rms_off * rms_on) ** 0.5
        off_counts, on_counts = mid * 0.7, mid * 1.4
        print("\n  ! Margin is tight; thresholds placed geometrically between the states.")
        print("  ! Raise the gain pot and re-run to get a wider uncertain band.")

    print("\n--- Paste into src/pump/pump_sensor.cpp -------------------------------")
    print(f"#define DEFAULT_ON_COUNTS {on_counts:.1f}f")
    print(f"#define DEFAULT_OFF_COUNTS {off_counts:.1f}f")
    print("-----------------------------------------------------------------------")
    print(f"\nMeasured {time.strftime('%Y-%m-%d')}: rms_off={rms_off:.2f} (worst of {n}), "
          f"rms_on={rms_on:.2f} (worst of {n}),")
    print(f"bias={statistics.median(off):.0f} ... put these numbers in the commit message.")

    print("\nVerify them WITHOUT reflashing first -- the board takes both per request:")
    print(f'  python tools/pump_tune.py watch --on-counts {on_counts:.1f} '
          f'--off-counts {off_counts:.1f}')
    print("Toggle the pump a few times; every reading should be a clean on/off,")
    print("never uncertain. Only then bake them in and re-flash.")


def cmd_raw(board, args):
    board.ser.write((args.line + "\n").encode())
    board.ser.flush()
    deadline = time.time() + 10.0
    while time.time() < deadline:
        raw = board.ser.readline()
        if raw:
            print(raw.decode(errors="replace").rstrip())
            return
    print("(no reply within 10 s)")


def sample_args(args):
    """The measurement parameters, omitting any the user did not override.

    Omitted means the board uses its own default, and echoes back what it used --
    which is what you want while the defaults are still the thing under test.
    """
    out = {"cycles": args.cycles, "mains_hz": args.mains_hz, "rate_hz": args.rate_hz}
    if args.on_counts is not None:
        out["on_counts"] = args.on_counts
    if args.off_counts is not None:
        out["off_counts"] = args.off_counts
    if args.counts_per_volt is not None:
        out["counts_per_volt"] = args.counts_per_volt
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port; auto-detected when unambiguous")
    ap.add_argument("--cycles", type=int, default=10)
    ap.add_argument("--mains-hz", type=float, default=50.0)
    ap.add_argument("--rate-hz", type=int, default=2000)
    ap.add_argument("--on-counts", type=float, help="override the board's on threshold")
    ap.add_argument("--off-counts", type=float, help="override the board's off threshold")
    ap.add_argument("--counts-per-volt", type=float, help="supply to get vrms in the reply")

    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("status").set_defaults(func=cmd_status)
    sub.add_parser("probe").set_defaults(func=cmd_probe)

    p = sub.add_parser("plot")
    p.add_argument("--dump-n", type=int, default=200)
    p.set_defaults(func=cmd_plot)

    p = sub.add_parser("watch")
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--cmd", default="sampling", choices=["sampling", "read_pump"])
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("calibrate")
    p.add_argument("--repeat", type=int, default=5, help="windows per state")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("raw")
    p.add_argument("line", help="a complete JSON request line")
    p.set_defaults(func=cmd_raw)

    args = ap.parse_args()
    port = find_port(args.port)
    board = Board(port)
    try:
        # Prove the link before anything that would blame the sensor for a dead port.
        if args.command != "raw":
            ident = board.request("status")
            if ident.get("role") != "pump":
                sys.exit(f"{port} is a '{ident.get('role')}' board, not the pump sensor")
            print(f"# {port}  role={ident['role']}  fw={ident['fw']}  "
                  f"proto={ident['proto']}  up={ident['uptime_ms']}ms\n")
        args.func(board, args)
    finally:
        board.close()


if __name__ == "__main__":
    main()
