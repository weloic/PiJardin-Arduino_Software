# Pump sensor firmware (`pump`)

Detects whether the garden pump is running, by looking for mains AC on the pump's own feed.

- **Board:** Seeed Studio XIAO **RP2040** — ⚠️ *not* the SAMD21 the well sensor uses. Same package,
  different chip; check the silkscreen before flashing.
- **Sensor:** ZMPT101B AC voltage module (0–250 V, single phase).
- **Source:** [src/pump/pump_sensor.cpp](../src/pump/pump_sensor.cpp)
- **Env:** `pump` — `pio run -e pump -t upload`
- **Firmware / protocol:** `fw 1.0.0`, `proto 1`

The first `pio run -e pump` downloads the RP2040 platform and toolchain from GitHub — it needs
network and takes several minutes. Subsequent builds are seconds.

Voltage presence is a more honest signal than the alternatives: either the contactor is closed and
there are 230 V on the motor, or there are not. A current threshold has to be calibrated against
the motor's actual draw and drifts as the motor ages; a flow switch reports the *water*, which is
what you wanted to infer, not what you measured.

See the repo [README](../README.md) for the shared NDJSON envelope, the toolchain and flashing.

---

## ⚠️ Two ways to destroy this, both silent

### 1. Powering the module from 5 V

**Power the ZMPT101B from the XIAO's `3V3` pad. Not `5V`.**

The module biases its analog output at Vcc/2 and swings around it, so on 5 V it idles at 2.5 V and
peaks near 5 V. The RP2040's GPIOs are **not 5 V tolerant** and its ADC reference is 3.3 V. That
arrangement clips every reading *and* stresses the pin past its rating. On 3V3 the module idles near
mid-scale (~1.65 V, ~2048 counts) and physically cannot overshoot.

### 2. Wiring it across the wrong side of the contactor

**The module must sit across the pump's SWITCHED feed** — downstream of the contactor, relay or
switch that starts the motor:

```
  mains ──┬── [ contactor ] ──┬── PUMP MOTOR
          │                   │
          │                   └── ZMPT101B  ✅ sees 230 V only while the pump runs
          │
          └────────────────────── ZMPT101B  ❌ sees 230 V always; reports "on" forever
```

Wired across the incoming mains instead, it reads a healthy 230 V whether the pump is running or
stopped, and this firmware will report `"state":"on"` forever. **The board cannot detect this
mistake** — a correct reading and a useless one are identical from its side. This is the one failure
mode with no diagnostic, so verify it at install time by switching the pump off and confirming the
reading actually drops.

> ⚠️ 230 V mains. The ZMPT101B's input side is at line potential. Isolate before working on it, and
> if the pump's feed is not already on a circuit you can safely break into, have an electrician make
> the connection.

## Wiring

| Signal | Pin | Note |
| --- | --- | --- |
| ZMPT101B `OUT` | `A0` | GPIO26 = ADC0 on the RP2040 |
| ZMPT101B `VCC` | `3V3` | **not 5V** — see above |
| ZMPT101B `GND` | `GND` | |
| Status LED | `LED_BUILTIN` | GPIO17, the red LED; lit while a command is served |

The XIAO RP2040's user LEDs are **active low** — the pin is pulled low to light them. The firmware
uses `LED_ON`/`LED_OFF` rather than `HIGH`/`LOW` for exactly this reason; drive it the intuitive way
round and the LED is on when idle and dark when working, which reads as a hung board.

### Flashing

```
pio run -e pump -t upload
```

That is all, including on a board that is already running the firmware — no BOOT button, no driver
install. If the board is running, the uploader resets it into the bootloader first.

**Why this board does not use `picotool`.** The default RP2040 upload protocol drives the PICOBOOT
interface of the BOOTSEL device, for which **Windows ships no driver**, so a stock machine fails
with:

```
Device ... appears to be a RP2040 device in BOOTSEL mode, but picotool was unable
to connect. You may need to install a driver via Zadig.
```

The same bootloader also exposes plain USB mass storage — the `RPI-RP2` drive — which every OS
already supports, and copying a `.uf2` there is the method Raspberry Pi documents first. So
[env:pump] sets `upload_protocol = custom` and points it at
[tools/flash_uf2.py](../tools/flash_uf2.py), which does the copy. Zadig would work too, but it is a
manual step on every machine that ever builds this firmware, and aimed at the wrong interface it
breaks the drive as well.

The RP2040 build emits `.pio/build/pump/firmware.uf2` directly, so
[tools/bin2uf2.py](../tools/bin2uf2.py) — which is SAMD21-specific — is not used for this board.

**Manual fallback,** for a board whose firmware no longer enumerates: hold **BOOT** while plugging
in, then copy `.pio/build/pump/firmware.uf2` onto the `RPI-RP2` drive.

> While in BOOTSEL the board presents **only a drive, no serial port**. If `pump_tune.py` or the
> serial monitor reports no port and you can see an `RPI-RP2` drive, that is why — replug without
> holding BOOT, or just run `-t upload`.

The module's **multi-turn potentiometer** sets its output gain. It ships at an arbitrary position
and must be adjusted — see [Calibration](#calibration-required).

## How a reading is taken

Per request, with no state kept between requests:

1. **Sample** `A0` at a paced, uniform rate (default 2 kHz) for a whole number of mains cycles
   (default 10 → 200 ms at 50 Hz). The pacing is deliberate: a free-running loop overruns the
   sample buffer long before the window closes, which truncates it to a *fractional* number of
   cycles and biases the RMS by a few percent in a direction that depends on the phase the window
   happened to start at.
2. **Bias** = the mean of the window. Measured, never assumed to be mid-scale: the module derives
   its bias from its own supply and sets it with a pot, so the real idle level drifts with both.
3. **RMS** of the bias-removed signal, in ADC counts, computed in two passes over the buffered
   samples. Not via the one-pass `E[x²] − E[x]²` identity — with a bias near 2048 counts and an
   "off" signal of a few counts those two terms agree to five significant figures and subtracting
   them cancels away the answer. Measured error of the one-pass form in `float`: **52 % at low RMS**,
   0 % at high. The failure lands precisely on the reading that decides off-versus-on.
4. **Frequency** from rising crossings of the bias, with a hysteresis band scaled to the RMS.
   Derived from the span between the first and last crossing rather than crossings-per-window,
   because the window's edges fall at an arbitrary phase. Accurate to **≤0.25 % over 45–65 Hz** and
   independent of starting phase. Skipped entirely below ~10 counts RMS: noise crosses any band
   that small repeatedly, and a measured 3 counts of noise otherwise yields a confident, entirely
   fictional 245 Hz.
5. **Verdict** — `rms_counts` against two thresholds.

### Why `state` is three-valued, not a boolean

`"on"` / `"off"` / `"uncertain"`. A reading between `off_counts` and `on_counts` is a real outcome,
and there is no honest boolean for it. Resolving it needs history — the previous readings, how long
the pump has been in a state, whether it is mid-startup — and **the board has none**: it is
stateless by design, like the well sensor. So it reports the band it landed in and the Pi, which
does have history, applies any hysteresis or debouncing.

Consequence worth stating plainly: **a pump that switches faster than the Pi polls will be missed.**
That is a property of polling and the board cannot paper over it.

### Why a sensor fault is not "off"

A dead module and a stopped pump both produce a low RMS. They are separated by the **bias**: a live
module idles near mid-scale whatever the pump is doing, because that bias comes from its own supply.
A bias far from mid-scale means the board is not looking at a working module — unpowered, output
shorted, or (most likely in the field) the signal wire off, leaving the ADC input floating.

That is reported as `sensor_fault`, never as `"off"`. Same reasoning the well sensor uses to keep a
dead HC-SR04 apart from one that simply found no echo: a hardware fault reported as an ordinary
measurement is the worst available failure mode, because nothing downstream can tell.

---

## Calibration (required)

**`DEFAULT_ON_COUNTS` (200) and `DEFAULT_OFF_COUNTS` (80) in the source are placeholders.** They
have to be, because the counts-to-volts relationship lives in the module's gain pot and no firmware
can know where it is set. Left uncalibrated they will misreport.

### The short way

[`tools/pump_tune.py`](../tools/pump_tune.py) drives the protocol from a PC on the bench and does
all of the below, including the arithmetic and the quality checks (`pip install pyserial`):

```
python tools/pump_tune.py probe        # one window, with every trap in this page checked
python tools/pump_tune.py plot         # ASCII waveform -- turn the pot while watching it
python tools/pump_tune.py calibrate    # guided off/on, prints the two #define lines
```

`calibrate` derives the thresholds from the **worst** window seen in each state rather than the
average, and refuses to emit anything if the two states are not separated — which is, among other
things, what a module on the wrong side of the contactor looks like. The manual procedure below is
the same thing by hand; use it if you would rather not install pyserial.

### By hand

1. **Flash and connect.** `pio run -e pump -t upload`, then open the serial monitor at 9600.
2. **Pump OFF** — set the noise floor:
   ```json
   {"id":1,"cmd":"sampling","dump_n":200}
   ```
   Note `rms_counts` and check `bias_counts` is near 2048 with `bias_ok:true`. Typical: a few counts.
3. **Pump ON** — set the gain:
   ```json
   {"id":2,"cmd":"sampling","dump_n":200}
   ```
   - `n_clipped` must be **0**. If not, turn the pot **down** and repeat — flat-topped peaks mean
     the RMS is an underestimate.
   - Aim for `max_counts` around 3000–3800 (a clear signal with headroom), i.e. `rms_counts` in the
     high hundreds.
   - `freq_hz` should read within a few tenths of your mains frequency. If it does not, you are
     looking at pickup rather than the feed.
   - Plot the `samples` array if you want to see it: a clean sine, well inside 0–4095.
4. **Set the thresholds.** With `rms_off` and `rms_on` from steps 2 and 3, put them far apart —
   these are `off_counts` and `on_counts` in [pump_sensor.cpp](../src/pump/pump_sensor.cpp):
   ```
   off_counts ≈ max(5 × rms_off, 20)      e.g. rms_off =   3  ->  off_counts =  20
   on_counts  ≈ rms_on / 3                e.g. rms_on  = 700  ->  on_counts  = 233
   ```
   The gap between them is the `"uncertain"` band, and it is doing real work — make it wide. Both
   can be overridden per request, so test candidate values before editing the source.
5. **Check the noise floor against `FREQ_MIN_RMS`** (10.0 in the source). That constant is what
   stops a frequency being derived from pure noise. The RP2040's SAR ADC has a documented
   differential-nonlinearity problem — a few codes it will never return — which shows up as extra
   apparent noise on a quiet input, so this board may sit higher than a SAMD21 would. If the
   pump-off `rms_counts` from step 2 is anywhere near 10, raise `FREQ_MIN_RMS` to about 3× it;
   otherwise the guard is not guarding and `freq_hz` goes back to being invented from noise.
6. **Verify both states** with `read_pump`, then rebuild with the values baked in.

Record the numbers you measured in the commit message. The well sensor's `ack_timeout_us` comment
is the precedent: a constant that was guessed and then measured, with the measurement written down,
so the next person knows which it is.

---

## Protocol

Shared envelope with the well sensor: one JSON object per line, `\n`-terminated, every reply
echoing the request `id`. Unknown request fields are ignored; a field of the **wrong type** is a
`bad_param`, while an out-of-range value is **clamped** and the effective value echoed back.

### Boot banner

```json
{"type":"ready","proto":1,"fw":"1.0.0","role":"pump"}
```

`role` distinguishes the boards. Read it rather than inferring the board from `proto`, which is
numbered per firmware.

### Requests

| `cmd` | Purpose |
| --- | --- |
| `read_pump` | The verdict. Gated on sensor plausibility. |
| `sampling` | Diagnostics: same figures plus the raw waveform. Not gated. |
| `status` | Firmware identity, defaults and limits. |

Parameters, all optional, accepted by `read_pump` and `sampling`:

| Field | Default | Range | Meaning |
| --- | --- | --- | --- |
| `cycles` | 10 | 1–60 | whole mains cycles to cover |
| `mains_hz` | 50.0 | 40–70 | nominal mains frequency; sizes the window |
| `rate_hz` | 2000 | 500–20000 | sampling rate |
| `on_counts` | 200.0 | 0–4095 | RMS at or above which the pump is `on` |
| `off_counts` | 80.0 | 0–4095 | RMS at or below which the pump is `off` |
| `bias_min` | 1024 | 0–4095 | low edge of the plausible idle window |
| `bias_max` | 3072 | 0–4095 | high edge of the plausible idle window |
| `counts_per_volt` | — | > 0 | supply to get `vrms`; omitted by default |
| `dump_n` | 200 | 0–400 | raw samples to return (`sampling` only) |

`cycles × rate_hz / mains_hz` must fit the 1200-sample buffer or the window is cut short and
`truncated:true` is returned. At the defaults it is 400.

### Response — `read_pump`

```json
{"id":7,"type":"resp","proto":1,"status":"ok","state":"on",
 "rms_counts":707.14,"bias_counts":2047.8,"min_counts":1048,"max_counts":3048,
 "n_clipped":0,"freq_hz":50.0,"n_rise":9,
 "n_samples":400,"window_us":200013,"interval_us":500,"max_late_us":12,
 "truncated":false,"cycles_eff":10.0,"adc_bits":12,
 "cycles":10,"mains_hz":50.0,"rate_hz":2000,
 "on_counts":200.0,"off_counts":80.0,"bias_min":1024,"bias_max":3072}
```

| Field | Meaning |
| --- | --- |
| `state` | `on` / `off` / `uncertain` — the answer |
| `rms_counts` | RMS of the bias-removed signal; what the thresholds compare against |
| `bias_counts` | measured idle level; the sensor-fault check |
| `min_counts`, `max_counts` | window extremes, in raw counts |
| `n_clipped` | samples at a rail. Non-zero ⇒ `rms_counts` **understates**; turn the gain down. Harmless for the verdict — clipped still unambiguously means "on" |
| `freq_hz`, `n_rise` | crossing-derived frequency. `0` = none established (the ordinary pump-off result), never a guess |
| `max_late_us` | worst arrival lateness against the sampling schedule. Approaching `interval_us` means `analogRead` could not keep up, the spacing was not uniform, and `freq_hz` is the first casualty |
| `truncated`, `cycles_eff` | whether the window covered whole cycles, and how many it really covered |
| `vrms`, `unit`, `counts_per_volt` | **only** when `counts_per_volt` was supplied |

### Why volts are omitted by default

The counts-to-volts factor lives in the module's gain pot, so the board cannot know it. Reporting a
plausible voltage derived from a guessed constant is worse than reporting none, because nothing
downstream can tell it was invented. Supply `counts_per_volt` and you get `vrms`; otherwise the
field is absent. This is the same rule the well sensor applies to temperature (see
[README → Future: environment sensing](../README.md#future-environment-sensing)).

For on/off detection you do not need volts at all — `rms_counts` is sufficient and needs no
calibration beyond the two thresholds.

### Response — `sampling`

Everything above, plus `bias_ok` (the plausibility check as a flag rather than an error) and:

```json
{"samples":[2048,2154,2258,...],"dump_n":200}
```

Raw counts, oldest first, **contiguous from the start of the window** — not decimated across it.
Sampling every k-th point of a 50 Hz sine aliases into a slower-looking sine, which is exactly the
kind of plausible-but-wrong picture you would then calibrate against.

`sampling` is deliberately **not gated** on plausibility: diagnosing a sensor fault is the main
reason to call it, so refusing to show the samples that prove the fault would defeat the purpose.

### Errors

```json
{"id":7,"type":"resp","proto":1,"status":"error","code":"sensor_fault", ...}
```

| `code` | Meaning |
| --- | --- |
| `sensor_fault` | `bias_counts` outside `[bias_min, bias_max]`. The module is not reporting, so the pump's state is **unknown** — not `off`. Retrying will not help |
| `bad_param` | a field of the wrong type, or `on_counts < off_counts`, or `bias_min >= bias_max`, or `counts_per_volt <= 0`. `field` names it |
| `bad_request` | line was not valid JSON. `"id":null` |
| `bad_id` | `id` missing or not an integer. `"id":null` |
| `unknown_cmd` | unrecognised `cmd` |
| `line_too_long` | request exceeded 192 characters; the reader drains to the newline and recovers |

`sensor_fault` responses still carry the full measurement and context, so one logged line explains
itself.

---

## Testing after a flash

Serial monitor at 9600 baud, reset the board.

1. **Banner** → `{"type":"ready","proto":1,"fw":"1.0.0","role":"pump"}`.
2. `{"id":1,"cmd":"status"}` → `ok` with the defaults and limits (`adc_bits:12`, `max_samples:1200`,
   `rate_hz_default:2000`, `line_max:192`).
3. **Sampling quality** — `{"id":2,"cmd":"sampling","dump_n":0}` with the pump running:
   - `max_late_us` should be a small fraction of `interval_us` (500 µs at defaults). If it is
     comparable, lower `rate_hz` — the pacing is not being met and `freq_hz` degrades first.
   - `truncated:false` and `cycles_eff` ≈ `cycles`.
   - `freq_hz` within a few tenths of 50. **This is the check that the module is seeing real mains**
     rather than induced hum.
4. **Both states** — `{"id":3,"cmd":"read_pump"}` with the pump on, then off. Expect `"on"` then
   `"off"`, with `rms_counts` differing by a wide margin. **If it reads `"on"` both times, the
   module is on the wrong side of the contactor** — see the wiring warning above; nothing else in
   this checklist will catch that.
5. **Sensor fault, the important one** — **unplug the `A0` signal wire**. Expect `sensor_fault`, and
   check it is *not* `"state":"off"`: a disconnected sensor must not look like a stopped pump.
   Reconnect and confirm it recovers.
   - Also unplug the module's `3V3`: also `sensor_fault`.
   - Run `sampling` in each case and read `bias_counts` / `bias_ok` to see why.
6. **The uncertain band is reachable** — `{"id":4,"cmd":"read_pump","on_counts":4000}` on a running
   pump → `"uncertain"`, not `"off"`. Confirms the three-valued path works rather than collapsing.
7. **Clipping is detected** — turn the pot up until `n_clipped > 0`, confirm `read_pump` still
   returns `"on"` (clipping does not break the verdict), then turn it back down and re-run the
   [calibration](#calibration-required).
8. **Parameters:**
   - `{"id":5,"cmd":"read_pump","cycles":999}` → clamped, echoes `cycles:60`.
   - `{"id":6,"cmd":"read_pump","cycles":"ten"}` → `bad_param`, `"field":"cycles"`.
   - `{"id":7,"cmd":"read_pump","on_counts":50,"off_counts":100}` → `bad_param`,
     `"field":"on_counts"`.
   - `{"id":8,"cmd":"read_pump","cycles":60,"rate_hz":20000}` → `truncated:true` (24000 samples
     requested, 1200 available) with `cycles_eff` well under 60.
   - `{"id":9,"cmd":"read_pump","counts_per_volt":3.1}` → adds `vrms` and `unit:"V"`; omit it and
     both disappear.
9. **Errors:** send `hello` → `bad_request` with `"id":null`. Send `{"cmd":"status"}` → `bad_id`.
   Send `{"id":10,"cmd":"nope"}` → `unknown_cmd`. Paste a line over 192 characters →
   `line_too_long`, then confirm the **next** valid request still answers.
10. **Reader:** two requests back-to-back with no delay → two correlated responses, no lost line.
    An empty line → no reply at all.

## Pi-side integration

Not yet written. The board is a drop-in structural match for `read_puit.py` — same framing, same
`id` echo, same error envelope — so the Pi-side reader should be that module's shape with
`read_pump` in place of `read_puit`.

Two things specific to this board:

- **`uncertain` needs a policy.** The obvious one is to hold the last known state and only switch
  on N consecutive agreeing readings. That decision belongs on the Pi, which has the history.
- **`sensor_fault` must not be recorded as "pump off".** It means unknown. Logging it as `off`
  produces a clean-looking time series that says the pump never ran.

## Known limitations

- **Polling latency.** A 200 ms window plus the Pi's poll interval bounds how fast a transition can
  be seen. Short cycles will be missed entirely.
- **Voltage present ≠ water moving.** This detects that the motor is energised. A pump that is
  energised but airlocked, dry, or with a closed valve reads `"on"`.
- **Single phase.** One ZMPT101B on one conductor. A three-phase pump losing one phase reads `"on"`
  if the monitored phase is still live.
- **Wrong-side-of-contactor wiring is undetectable.** Restated because it is the only failure here
  with no diagnostic signature at all.
