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
import os
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

# Mirrors FREQ_MIN_RMS in src/pump/pump_sensor.cpp. Kept here only to tell the
# user when the firmware's value no longer suits their installation -- the board
# does not report it, so this copy can drift; check it if the advice looks odd.
FREQ_MIN_RMS_DEFAULT = 10.0

# The uncertain band must span at least this ratio for the 5x / div-3 rule to be
# worth using. Below it the rule degenerates: it needs roughly 15x separation
# before `signal/3` clears `5 x floor` at all, so just past that point the two
# thresholds come out almost equal. Measured: floor 12.93, signal 199.20 (15.4x)
# produced off=64.7 and on=66.4 -- a 1.7-count band, which is a single threshold
# wearing a disguise. The `on <= off` inversion test does not catch it, because
# 66.4 really is greater than 64.7.
MIN_BAND_RATIO = 1.6


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

    # Clipping that happens BELOW the ADC rails, which n_clipped cannot see.
    #
    # The stock ZMPT101B is specified for 5-30 V and its LM358 cannot drive its
    # output closer than ~1.3 V to its positive rail. Run from 3V3 it therefore
    # flattens the top of the wave at ~2.0 V (~2480 counts) while the bottom half
    # swings freely -- a distorted reading with n_clipped:0 and both rails
    # untouched. Measured on this project's hardware: bias 2032, max 2476
    # (+444), min 1480 (-552), rms 394 where a clean sine of that peak would
    # give 352.
    #
    # Two independent symptoms, because either alone has innocent explanations:
    # mains itself is often slightly flat-topped (odd harmonics from rectifier
    # loads), but that distorts both halves EQUALLY, so asymmetry is what
    # separates an op-amp running out of headroom from an ugly grid.
    rms = resp.get("rms_counts", 0)
    lo, hi = resp.get("min_counts", 0), resp.get("max_counts", 0)
    pos, neg = hi - bias, bias - lo
    peak = (hi - lo) / 2.0
    if rms > 50 and peak > 0 and max(pos, neg) > 0:
        asym = abs(pos - neg) / max(pos, neg)
        fullness = rms / peak  # 0.707 for a sine, 1.0 for a square
        if asym > 0.15 and fullness > 0.76:
            volts = hi / 4095.0 * 3.3
            target = int(MID_SCALE + 0.8 * (hi - MID_SCALE))
            problems.append(
                f"ASYMMETRIC CLIPPING: +{pos:.0f}/-{neg:.0f} counts about the bias "
                f"({asym * 100:.0f}% lopsided) and rms/peak {fullness:.2f} vs 0.707 "
                f"for a sine. The top is being flattened at {hi} counts (~{volts:.2f} V), "
                f"well below the ADC rail -- so n_clipped cannot see it.\n"
                f"      This is the LM358 out of headroom on a 3.3 V supply.\n"
                f"      TURNING THE POT: a multi-turn pot gives no clue which way is "
                f"'down', and max_counts is PINNED at the ceiling so it will not move "
                f"whichever way you turn. Judge by these instead:\n"
                f"        rms_counts must FALL   (rising = wrong way)\n"
                f"        bias_counts must climb back toward {MID_SCALE} "
                f"(now {bias:.0f}; it sags because the clipped top drags the mean down)\n"
                f"      Aim for max_counts near {target} with this warning gone. Watch it "
                f"live while turning:  pump_tune.py watch\n"
                f"      Or fix it properly: VCC to 5V, OUT divided 2:1 -- see docs/pump.md."
            )
        elif asym > 0.15 and fullness > 0.72:
            problems.append(
                f"waveform is lopsided about the bias: +{pos:.0f}/-{neg:.0f} counts, "
                f"and rms/peak {fullness:.2f} is starting to climb above the 0.707 of a "
                f"sine. Approaching the flattening described in docs/pump.md -- do not "
                f"raise the gain further."
            )
        # No warning for asymmetry with fullness at or below ~0.72. A real mains
        # waveform is not a textbook sine -- harmonics from every rectifier on the
        # circuit lean it, commonly 15-20%, and NO gain setting removes that.
        #
        # An earlier version warned on asymmetry alone and told the user to
        # "reduce the gain until the two swings match", which is unreachable
        # advice: measured on this installation, winding the gain down took rms
        # 262 -> 206 while the lean went 15% -> 18%. Two things separate the
        # benign case from clipping, and both were visible in that data:
        #   - direction: flattening SUPPRESSES one side, so the clipped side is
        #     the SMALLER excursion. Here the larger side was the positive one.
        #   - fullness: flattening pushes rms/peak ABOVE 0.707 (0.81 when this
        #     module was truly clipping). Benign asymmetry sits at or below it.

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
        # Two very different causes, and one reading cannot separate them, so
        # name both rather than asserting the alarming one. A window that
        # straddles the contactor opening or closing is part quiet and part
        # mains, which yields an in-between RMS and a frequency that is an
        # artefact of the join -- measured on this installation: 52 counts at
        # 31 Hz and 149 counts at 43 Hz, each immediately before a clean state
        # change either side. That is the sampling window doing its job, not a
        # fault. Persistent pickup shows the same shape but does NOT resolve
        # into a clean on/off on the next reading.
        problems.append(
            f"freq_hz {freq:.2f} is not {mains_hz:.0f} Hz. If the readings either side "
            f"are clean on/off, this window simply straddled a pump transition -- "
            f"expected, and what the 'uncertain' state exists for. If it persists "
            f"while the pump is steady, it is induced hum or pickup rather than the "
            f"pump's feed."
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


def ascii_plot(samples, bias=None, ceiling=None, height=19, width=76):
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
    # Gain advice, but ONLY once we know the ADC rails are the real limit.
    #
    # "Lots of headroom, turn the pot up" is exactly wrong when the ceiling is
    # the op-amp rather than the ADC: the module is already flattening the top
    # of the wave hundreds of counts below the rail, and more gain flattens it
    # further while this measure keeps reporting room to spare. Measured on a
    # ZMPT101B at 3V3: 1480 counts of apparent headroom while the positive peaks
    # were already compressed 20%. So say nothing about gain until the waveform
    # is symmetric enough to trust the rails as the limit.
    lopsided = False
    if bias is not None:
        pos, neg = hi - bias, bias - lo
        if max(pos, neg) > 0:
            lopsided = abs(pos - neg) / max(pos, neg) > 0.15

    if ceiling is not None:
        # --ceiling declares that something below the ADC rail is the real limit,
        # which is the situation when an LM358 module is deliberately run at 3V3
        # (see docs/pump.md). Without it this function measures room that the
        # signal cannot actually use and advises raising a gain that is already
        # too high -- the exact mistake that produced the flattened capture in
        # the docs. Margins here are small by nature, hence the tighter bands.
        margin = min(ceiling - hi, lo)
        if margin < 0 and lopsided:
            # Once the top is hard-clipped, `hi` sits ON the ceiling and stops
            # responding to the pot, so this margin freezes at a small negative
            # number and reports the same thing however far the gain is wound --
            # measured: -11, -11, -12, -12 across four turns that tripled the
            # RMS. Saying "turn it down" against a frozen number reads as "the
            # pot does nothing". Point at the figures that do still move.
            print(f"  max_counts ({hi}) is PINNED at the ceiling -- this margin cannot")
            print(f"  improve, and does not move however far you turn the pot. Judge by")
            print(f"  rms_counts and bias_counts instead; see the warning above.")
            return
        if margin < 0:
            verdict = f"OVER the {ceiling}-count ceiling -- reduce the gain"
        elif margin < 50:
            verdict = "too tight; drift will clip it -- turn the pot DOWN"
        elif margin > 250:
            verdict = "room to raise the gain a little"
        else:
            verdict = "good"
        print(f"  margin to the declared {ceiling}-count ceiling: {margin} counts ({verdict})")
    elif lopsided:
        # Report, do not prescribe. Whether a lean is benign harmonic content or
        # the onset of flattening needs the RMS, which check() has and this does
        # not -- so this states the measurement and leaves the diagnosis there.
        # Two functions independently deciding what the user should do to the pot
        # is how this tool ended up printing contradictory instructions.
        pos, neg = hi - bias, bias - lo
        print(f"  swing about the bias: +{pos:.0f} / -{neg:.0f} counts "
              f"({abs(pos - neg) / max(pos, neg) * 100:.0f}% lopsided) -- see any")
        print(f"  warning above for whether that matters.")
        print(f"  {min(lo, ADC_MAX - hi)} counts to the nearest ADC rail. If this module")
        print(f"  cannot reach the rail (an LM358 at 3V3 tops out near 2460), that figure")
        print(f"  is not a gain guide -- re-run with --ceiling COUNTS for advice against")
        print(f"  the real limit.")
    else:
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
    ascii_plot(resp.get("samples", []), bias=resp.get("bias_counts"),
               ceiling=args.ceiling)


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
    """Take n windows and return the full responses, printing each as it lands.

    The whole response rather than just the RMS: the frequency of the pump-OFF
    trace decides what the floor actually is, and therefore whether the gain can
    do anything about it. See cmd_calibrate.
    """
    out = []
    for i in range(n):
        resp = board.request("sampling", dump_n=0, **sample_args(args))
        if resp.get("status") == "error":
            print(f"  [{label} {i+1}/{n}] ERROR {resp.get('code')} -- fix this first")
            for p in check(resp, args.mains_hz):
                print(f"    ! {p}")
            sys.exit(1)
        out.append(resp)
        summarise(resp, args.mains_hz, prefix=f"  [{label} {i+1}/{n}] ")
        time.sleep(0.2)
    return out


def floor_is_coupled_mains(off_resps, mains_hz):
    """True if the pump-off floor is mains leaking through, not ADC noise.

    The distinction decides whether more gain can widen the margin. ADC noise is
    generated after the amplifier, so turning the gain up lifts the signal and
    leaves the noise where it is -- the ratio improves. Mains coupled in ahead of
    the amplifier (a contactor snubber, an indicator lamp, capacitive pickup
    between conductors) is amplified by exactly the same factor as the signal, so
    the ratio is fixed by the installation and no pot setting changes it.

    They are told apart by frequency: the firmware only derives one above
    FREQ_MIN_RMS and never guesses, so an off-state trace reporting the mains
    frequency has real mains in it. Measured on this installation: pump off,
    rms 26.3 counts, freq 50.12 Hz across all five windows.
    """
    hits = sum(1 for r in off_resps
               if abs(r.get("freq_hz", 0.0) - mains_hz) < 2.0)
    return hits > len(off_resps) / 2


def cmd_calibrate(board, args):
    n = args.repeat
    print(__doc__.split("\n")[0])
    print(f"\nTaking {n} windows in each state. Both thresholds are derived from the")
    print("WORST case seen, not the average -- the point is a margin that holds.\n")

    input("1. Switch the pump OFF, wait for it to settle, then press Enter... ")
    off_resps = collect(board, args, n, "off")

    print()
    input("2. Switch the pump ON, wait for it to settle, then press Enter... ")
    on_resps = collect(board, args, n, "on")

    off = [r["rms_counts"] for r in off_resps]
    on = [r["rms_counts"] for r in on_resps]
    coupled = floor_is_coupled_mains(off_resps, args.mains_hz)

    rms_off = max(off)   # worst-case floor
    rms_on = min(on)     # worst-case signal
    print(f"\n  floor  (worst of {n}): {rms_off:.2f} counts  "
          f"[median {statistics.median(off):.2f}]")
    print(f"  signal (worst of {n}): {rms_on:.2f} counts  "
          f"[median {statistics.median(on):.2f}]")
    print(f"  separation: {rms_on / rms_off:.1f}x")
    if coupled:
        print(f"  the floor carries mains at ~{args.mains_hz:.0f} Hz, so it is coupling")
        print(f"  through the installation rather than ADC noise -- see below")

    # FREQ_MIN_RMS decides whether the firmware derives a frequency at all. A
    # floor above it means the board reports one for a STOPPED pump, and unless
    # that floor is coupled mains the number is derived from noise. Measured
    # here: floor 12.9 against the 10.0 default, yielding 307/425/393/328/341 Hz
    # across five windows -- wandering like that is the signature of no real
    # periodicity, which is exactly what this guard exists to suppress.
    if rms_off >= FREQ_MIN_RMS_DEFAULT:
        want = max(20, int(round(rms_off * 3 / 10.0)) * 10)
        freqs = ", ".join(f"{r.get('freq_hz', 0):.0f}" for r in off_resps)
        print(f"\n  ! FREQ_MIN_RMS in the firmware is {FREQ_MIN_RMS_DEFAULT:.1f}, BELOW this "
              f"floor of {rms_off:.2f}, so the")
        if coupled:
            print(f"  ! board reports a frequency with the pump stopped. Here that is real "
                  f"coupled\n  ! mains rather than fiction, so it is informative -- but "
                  f"raising FREQ_MIN_RMS to\n  ! {want}.0f would keep the off state quiet if "
                  f"you would rather it said nothing.")
        else:
            print(f"  ! board derives one from NOISE with the pump stopped: {freqs} Hz across "
                  f"the\n  ! five windows. Wandering like that means there is no real "
                  f"periodicity there.\n  ! Set FREQ_MIN_RMS to {want}.0f in "
                  f"src/pump/pump_sensor.cpp (~3x this floor).")

    if rms_on <= rms_off * 3:
        print("\n  ! The two states are not separated. Either the pot gain is far too low,")
        print("  ! or -- check this first -- the module is on the WRONG SIDE of the")
        print("  ! contactor and is seeing house mains in both states. Nothing in this")
        print("  ! tool can tell those apart; go and look at the wiring.")
        sys.exit(1)

    off_counts = max(5.0 * rms_off, 20.0)
    on_counts = rms_on / 3.0
    if on_counts < off_counts * MIN_BAND_RATIO:
        # Separated, but not by enough for the 5x/÷3 rule to leave a gap. Split the
        # difference geometrically rather than emitting an inverted pair, which the
        # firmware would reject as bad_param anyway.
        mid = (rms_off * rms_on) ** 0.5
        off_counts, on_counts = mid * 0.7, mid * 1.4
        print("\n  ! Margin is tight for the 5x/div-3 rule; thresholds placed")
        print(f"  ! geometrically instead, each about {rms_on / (mid * 1.4):.1f}x clear of "
              f"its state.")
        if coupled:
            # Do NOT suggest more gain here. The floor is mains coupled in ahead
            # of the module's amplifier, so the pot multiplies it by the same
            # factor as the signal and the ratio does not move -- it only walks
            # the wave back into clipping. Measured here: 204/26 = 7.8x, fixed.
            print("  ! DO NOT raise the gain to widen this: the floor is coupled mains,")
            print("  ! amplified by the same pot as the signal, so the ratio will not")
            print("  ! change -- you would only clip again. The separation is set by the")
            print("  ! installation. To improve it, reduce the coupling: check for an RC")
            print("  ! snubber or indicator lamp across the contactor, and route the")
            print("  ! sense wires away from the pump's feed.")
        else:
            print("  ! The floor looks like ADC noise rather than coupled mains, so")
            print("  ! raising the gain would widen the ratio -- but only if the")
            print("  ! waveform stays unclipped. Re-run `plot` after any pot change.")

    print("\n--- Paste into src/pump/pump_sensor.cpp -------------------------------")
    print(f"#define DEFAULT_ON_COUNTS {on_counts:.1f}f")
    print(f"#define DEFAULT_OFF_COUNTS {off_counts:.1f}f")
    print("-----------------------------------------------------------------------")
    # `off` is the RMS list; the bias lives on the responses. An earlier version
    # took the median of `off` here and labelled it "bias", printing a plausible
    # two-digit number into the line meant to be pasted into the commit message.
    bias_med = statistics.median([r["bias_counts"] for r in off_resps])
    print(f"\nMeasured {time.strftime('%Y-%m-%d')}: rms_off={rms_off:.2f}, "
          f"rms_on={rms_on:.2f} (both worst of {n}), separation {rms_on / rms_off:.1f}x,")
    print(f"bias={bias_med:.0f}"
          + (f", floor is coupled mains at ~{args.mains_hz:.0f} Hz" if coupled else "")
          + " ... put these numbers in the commit message.")

    # sys.executable, not a bare "python": PlatformIO's bundled interpreter is
    # usually the only one with pyserial installed and is typically not on PATH
    # at all, so a copy-pasted "python ..." fails with a Microsoft Store stub.
    if os.name == "nt":
        invocation = f'& "{sys.executable}" tools/pump_tune.py'
    else:
        invocation = f"{sys.executable} tools/pump_tune.py"
    print("\nVerify them WITHOUT reflashing first -- the board takes both per request:")
    print(f'  {invocation} watch --on-counts {on_counts:.1f} '
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


def add_common_opts(p, on_subparser):
    """Options that work either BEFORE or AFTER the subcommand.

    Declared on both the main parser and every subparser, on purpose. argparse
    resolves a subparser's arguments after the main parser's, so an option
    present in both would have its pre-subcommand value silently overwritten by
    the subparser's default -- and declared on only one side, the other ordering
    is a hard "unrecognized arguments" error. Since `calibrate` prints a ready-
    made command with these flags in it, one of those orderings being rejected
    means the tool hands the user a command it then refuses to run.

    default=SUPPRESS on the subparser copies is what makes it work: those
    arguments set nothing at all unless actually typed, so whichever side the
    user put them on wins and the other keeps out of the way.
    """
    d = (lambda _: argparse.SUPPRESS) if on_subparser else (lambda v: v)
    p.add_argument("--port", default=d(None),
                   help="serial port; auto-detected when unambiguous")
    p.add_argument("--cycles", type=int, default=d(10))
    p.add_argument("--mains-hz", type=float, default=d(50.0))
    p.add_argument("--rate-hz", type=int, default=d(2000))
    p.add_argument("--on-counts", type=float, default=d(None),
                   help="override the board's on threshold")
    p.add_argument("--off-counts", type=float, default=d(None),
                   help="override the board's off threshold")
    p.add_argument("--counts-per-volt", type=float, default=d(None),
                   help="supply to get vrms in the reply")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    add_common_opts(ap, on_subparser=False)

    _sub = ap.add_subparsers(dest="command", required=True)

    class sub:  # noqa: N801 -- keeps the add_parser calls below unchanged
        @staticmethod
        def add_parser(name, **kw):
            p = _sub.add_parser(name, **kw)
            add_common_opts(p, on_subparser=True)
            return p

    sub.add_parser("status").set_defaults(func=cmd_status)
    sub.add_parser("probe").set_defaults(func=cmd_probe)

    p = sub.add_parser("plot")
    p.add_argument("--dump-n", type=int, default=200)
    p.add_argument(
        "--ceiling", type=int, metavar="COUNTS",
        help="highest count the module can actually reach, when that is lower "
             "than the ADC rail -- e.g. ~2450 for an LM358 module run at 3V3. "
             "Gain advice is measured against this instead of 4095.")
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
