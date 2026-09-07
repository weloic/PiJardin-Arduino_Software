# Pump sensor firmware (`pump`)

Records **when the garden pump starts and stops**, by watching for mains AC on the pump's own feed.

- **Board:** Seeed Studio XIAO **RP2040** — ⚠️ *not* the SAMD21 the well sensor uses. Same package,
  different chip; check the silkscreen before flashing.
- **Sensor:** ZMPT101B AC voltage module (0–250 V, single phase).
- **Source:** [src/pump/pump_sensor.cpp](../src/pump/pump_sensor.cpp)
- **Env:** `pump` — `pio run -e pump -t upload`
- **Firmware / protocol:** `fw 2.0.0`, `proto 2`

The first `pio run -e pump` downloads the RP2040 platform and toolchain from GitHub — it needs
network and takes several minutes. Subsequent builds are seconds.

Voltage presence is a more honest signal than the alternatives: either the contactor is closed and
there are 230 V on the motor, or there are not. A current threshold has to be calibrated against
the motor's actual draw and drifts as the motor ages; a flow switch reports the *water*, which is
what you wanted to infer, not what you measured.

See the repo [README](../README.md) for the shared NDJSON envelope, the toolchain and flashing.

---

## The board watches; the Pi records what it is told

**`proto 2` is a push protocol.** The board measures a 200 ms window continuously — not on request
— runs a state machine over it, and **emits an event line the moment the state changes**. It also
emits a heartbeat every 60 s, and it keeps the last 32 transitions in RAM so a Pi that was down can
ask for what it missed.

Polling cannot do this job. The Pi would have to ask every few seconds forever, would still miss
any cycle shorter than its interval, and — the part that actually breaks it — would have no way to
know what it missed while it was rebooting or being redeployed. The board is the only thing
watching continuously, so the board is the only thing that can decide, remember and report.

### The principle that reversed

`proto 1` argued the exact opposite, and the reasoning is worth keeping rather than quietly
deleting: the board was stateless, there was **no latching and no hysteresis**, a reading between
the thresholds was reported as `"uncertain"` for the Pi (which had history) to resolve, and the
docs said plainly that *"a pump that switches faster than the Pi polls will be missed; that is a
property of polling and the board cannot paper over it."*

Every word of that was correct **for a polled board**. What changed is the question, not the
reasoning. Once the job is *recording transitions* rather than *answering "is it on now?"*, the
board is the only side with history, so **latching, hysteresis and debounce move here**. The band
between `off_counts` and `on_counts` stops being a verdict and becomes the thing the hysteresis is
made of: a reading inside it **holds** the current state.

**Statelessness survives everywhere it still applies, and is not weakened:**

- **Thresholds and parameters are compile-time constants.** They can be overridden per request for
  *one measurement* (that is how calibration works) but **never for the detector**. Nothing the Pi
  sends changes how the board decides.
- **Nothing is persisted to flash.** A reboot starts from `unknown`, never from a remembered guess
  about a pump nobody was watching. `uptime_ms` is how the Pi knows a reboot happened, so `since_ms`
  is never extrapolated across one.
- **Every response still echoes the effective values it used.**

The only history the board keeps is what it has *watched* since boot.

`read_pump` still exists and still returns `"uncertain"`, because it is still a single unlatched
measurement — that is what [`tools/pump_tune.py`](../tools/pump_tune.py) calibrates against, and
making it return the latched verdict instead would break calibration.

---

## Wiring

| Signal | Pin | Note |
| --- | --- | --- |
| ZMPT101B `OUT` | `A0` | GPIO26 = ADC0 — **direct, no divider** |
| ZMPT101B `VCC` | `3V3` | **not 5V** — see below |
| ZMPT101B `GND` | `GND` | |
| Status LED | `PIN_LED_R/G/B` | GPIO17/16/25, the 3-in-1 user LED; shows the **detector state** by colour — see below |
| NeoPixel | GPIO12, GPIO11 | unused, **held dark** — see below |

```
  ZMPT101B VCC ──── XIAO 3V3
  ZMPT101B GND ──── XIAO GND
  ZMPT101B OUT ──── XIAO A0
```

### Status LED

The LED answers the question you actually have standing in front of the board: **is the pump
running?** It says it in **colour**, on the 3-in-1 user LED.

| Colour | Pattern | Meaning |
| --- | --- | --- |
| 🟢 green | solid | pump **`on`** |
| 🟢 green | 60 ms blip every 3 s | pump **`off`**, board alive |
| 🔵 blue | blink, 2 Hz | **undecided** — `unknown` at boot, or a change part way through its debounce |
| 🔴 red | blink, 10 Hz | **`fault`** — the module is not reporting |
| ⚫ dark | — | **the board is not running** |

Exactly one die is ever lit. Two at once would be a third colour and a fourth thing to learn.

#### ⚠️ All four LEDs are now driven, including the ones that are not used

The XIAO RP2040 carries **four user-visible LEDs**: the three dice of the user LED (red GPIO17,
green GPIO16, blue GPIO25) and a WS2812 "NeoPixel" (data GPIO12, power GPIO11). Up to fw 2.0.0 this
firmware drove only the red die.

**Leaving the rest uninitialised does not leave them off.** An uninitialised pin is an *input*, and
these LEDs are wired to 3V3 through their anodes — a floating cathode leaks enough to sit lit or
glowing. The result was three or four LEDs on at once, and the one that meant something lost among
them. Every LED the firmware does not use is now explicitly driven off at boot, and the NeoPixel is
held unpowered *and* with its data line low, so it cannot latch noise as a colour.

The NeoPixel is deliberately not used as the indicator: it needs a library and ~30 µs of bit-banging
with interrupts off per update, and three plain GPIOs say the same thing without going near the
timing this firmware's sampling depends on.

#### Why the two awkward-looking choices

**`fault` is red, not dark.** Dark is what `off` would otherwise be, and leaving them the same
reintroduces — on the one output a person actually looks at — exactly the conflation that the bias
plausibility window, the `sensor_fault` code and the `fault` state all exist to prevent.

**`off` is a blip, not dark.** Same reason the protocol has a [heartbeat](#events-board--pi-unsolicited):
silence has to mean one thing. A dark LED must mean *this board is not running*, and it cannot mean
that if it also means *the pump is stopped* — the state the board will legitimately sit in for most
of the year. The blip is short enough not to compete with anything and long enough to see. It is one
constant (`LED_ALIVE_FLASH_MS`) if you would rather it were dark.

**A reading in the hysteresis band does not blink.** The board holds its state through that band on
purpose and is entirely certain of the answer it is giving; blinking there would advertise doubt the
design does not have, and make a correctly-working detector look flaky. Blue means *not committed* —
no state yet, or one being confirmed. The old per-window `"uncertain"` verdict is still on
`read_pump` for anyone who wants the raw view.

> **This replaced the fw 1.x busy indicator**, which lit the LED while a command was being served.
> That made sense when the board idled between requests. It now measures continuously, so "lit while
> working" would mean "lit always" — and a `sampling` call would hold it solid for 1.2 s looking
> exactly like a running pump. One indicator cannot say both things.

The XIAO RP2040's user LEDs are **active low** — the pin is pulled low to light a die. The firmware
uses `LED_ON`/`LED_OFF` rather than `HIGH`/`LOW` for exactly this reason; drive them the intuitive
way round and every colour above inverts *and* all three light at once, which is the muddle this
replaced.

The module's **multi-turn potentiometer** sets its output gain. It ships at an arbitrary position
and must be adjusted — see [Calibration](#calibration-required).

### The 3V3 front end, and what it costs

**This arrangement is out of specification on purpose, and it works.** Every threshold in the
firmware was measured through it. It is worth understanding *why* it works, because the margin is
real rather than comfortable and there is a failure mode it creates.

- **The ZMPT101B is specified for +5 V to +30 V.** 3V3 is below that. It works here only because
  the gain pot is set low enough that the signal never reaches the region where the module breaks
  down. That is a working point, not a guarantee.
- **The LM358 cannot drive within ~1.3 V of its positive rail.** On 3V3 that puts its output
  ceiling near **2.0 V ≈ 2480 counts** — measured on this exact hardware at high gain, where the
  positive peaks topped out at **2476** and stopped responding to the pot at all. With the bias at
  **2052**, that leaves about **430 counts of usable headroom above the bias** against about
  **2050 below it**. The swing is asymmetric and **the positive peak is the binding constraint**;
  the ADC's 4095 rail never comes into it.
- **At the calibrated pot setting the running signal is rms 199.2 counts**, so the peak is ≈ **283
  counts** (rms/peak 0.703 — essentially a clean sine). That is about **66 % of the available
  headroom**. That is the real margin, and it is why the [headroom check](#the-asym-headroom-check)
  exists.

> ### ⚠️ Do not "fix" this with 5 V or a divider
>
> Putting the module on 5 V and dividing its output 2:1 would give the LM358 its proper rail and a
> symmetric swing. It would also **invalidate every count in the firmware** — `DEFAULT_ON_COUNTS`,
> `DEFAULT_OFF_COUNTS` and `FREQ_MIN_RMS` were all measured through the front end above. Changing
> the front end means re-running the [calibration](#calibration-required), not editing a comment.
>
> And `OUT` straight from a 5 V-powered module to `A0` would **destroy the pin**: the RP2040 is not
> 5 V tolerant and its ADC reference is 3.3 V.

### The `asym` headroom check

The 3V3 supply creates one failure mode, and `n_clipped` **structurally cannot see it**. The LM358
runs out of headroom and flattens the top of the wave at ~2480 counts while the bottom half swings
freely. `n_clipped` only counts samples within 4 counts of 0 or 4095, so it **stays 0 through the
entire failure**. Nothing else in the numbers gives it away — the RMS just quietly understates the
signal.

The firmware detects it by **asymmetry about the measured bias**, reported on every reading and
every event line:

```
asym = (max_counts - bias) / (bias - min_counts)
```

| `asym` | Meaning |
| --- | --- |
| ~1.00 | clean sine — the positive and negative swings match |
| **< 0.90** | **flagged** — the top is being flattened; turn the pot **down** |
| 0.80 | the measured over-gain case: `(2476−2032) / (2032−1480)` |

A ratio, not a hardcoded ceiling, because the ratio is **self-calibrating**: it stays valid if the
3V3 rail sags, the pot moves, or the module is replaced. A fixed "2480" would survive none of those.

Only evaluated when `rms_counts ≥ FREQ_MIN_RMS` (40.0). On a pump-off noise floor `min` and `max`
are two or three counts of DNL-inflated noise either side of the bias, and their ratio is a coin
toss. Flagged windows are counted in **`n_headroom`**, reported in `status` and every heartbeat.

**It is a warning and never a fault.** A soft-clipped waveform still unambiguously means the pump
is on, and refusing to report a running pump because its waveform is ugly would be a worse failure
than the one being reported. What it means is that the pot wants turning down — and it is early
warning that a **mains overvoltage will start clipping**: 230 V +10 % = 253 V pushes the peak from
~283 to ~311 counts, against ~430 of headroom.

**Soft clipping does not affect `freq_hz`.** The Schmitt band sits at 0.25 × rms around the bias,
far below the flattened peak, so the crossings it counts are nowhere near the compressed region.

### ⚠️ Wiring it across the wrong side of the contactor

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

---

## How a reading is taken

One **window**, taken continuously by the detector and on demand by `read_pump` / `sampling`:

1. **Sample** `A0` at a paced, uniform rate (default 2 kHz) for a whole number of mains cycles
   (default 10 → 200 ms at 50 Hz, 400 samples). The pacing is deliberate: a free-running loop
   overruns the sample buffer long before the window closes, which truncates it to a *fractional*
   number of cycles and biases the RMS by a few percent in a direction that depends on the phase the
   window happened to start at.
2. **Bias** = the mean of the window. Measured, never assumed to be mid-scale: the module derives
   its bias from its own supply and sets it with a pot, so the real idle level drifts with both.
3. **RMS** of the bias-removed signal, in ADC counts, computed in two passes over the buffered
   samples. Not via the one-pass `E[x²] − E[x]²` identity — with a bias near 2048 counts and an
   "off" signal of a few counts those two terms agree to five significant figures and subtracting
   them cancels away the answer. Measured error of the one-pass form in `float`: **52 % at low RMS**,
   0 % at high. The failure lands precisely on the reading that decides off-versus-on.
4. **Headroom** — [`asym`](#the-asym-headroom-check), the swing above the bias over the swing below
   it. Only evaluated above `FREQ_MIN_RMS`.
5. **Frequency** from rising crossings of the bias, with a hysteresis band scaled to the RMS.
   Derived from the span between the first and last crossing rather than crossings-per-window,
   because the window's edges fall at an arbitrary phase. Accurate to **≤0.25 % over 45–65 Hz** and
   independent of starting phase. Skipped entirely below `FREQ_MIN_RMS` (40.0 counts): noise crosses
   any band that small repeatedly, and a measured 3 counts of noise otherwise yields a confident,
   entirely fictional 245 Hz.
6. **Verdict** — `rms_counts` against the two thresholds, then the state machine.

**The detector always uses the defaults.** `cycles` stays at 10 rather than being shortened for
latency: the calibrated floor (12.93 counts) and signal (199.20 counts) were both measured at a
10-cycle window, and a narrower one integrates less noise away and moves the floor the thresholds
were placed against. Latency is bought with the debounce, not by making the measurement worse.

## The state machine

Four states. **`unknown` is the boot state** — the board cannot know what the pump was doing before
it was watching, and it does not guess.

```
                  ┌──────────────────────────────────────┐
                  │             unknown                  │  boot; never a guess
                  └───────┬──────────────────────┬───────┘
                          │                      │
        rms ≥ on_counts   │                      │  rms ≤ off_counts
        AND freq ≈ mains  │                      │
                     ┌────▼────┐            ┌────▼────┐
                     │   on    │◄──────────►│   off   │
                     └────┬────┘            └────┬────┘
                          │                      │
                          └──────►┌───────┐◄─────┘
                                  │ fault │  bias outside [bias_min, bias_max]
                                  └───────┘
```

Every arrow requires **3 consecutive agreeing windows**.

| Rule | What it does |
| --- | --- |
| **Hysteresis** | `rms ≥ on_counts` → *on*; `rms ≤ off_counts` → *off*; **in between, hold**. The old `"uncertain"` band is now what the hysteresis is made of. |
| **Frequency gate** | To declare **on**, `freq_hz` must also be within **5 Hz** of `mains_hz`. A high RMS at the wrong frequency is induced hum or a wiring fault, not a running pump. Such a window **holds** the current state and is counted in `n_freq_reject` — there is deliberately no fifth state for it. |
| **`fault`** | Entered when the bias leaves `[bias_min, bias_max]`. A **real transition** with a real event, and it is **never reported as `off`** — a dead module and a stopped pump both read a low RMS, and the bias is the only thing separating them. |
| **Debounce** | A candidate state must hold for **3 consecutive windows (600 ms)** before it is declared. Absorbs contactor bounce, motor inrush, and a window that straddles the switching instant. |
| **Blind-spot reset** | After servicing a `read_pump` or `sampling` request the debounce counter is **reset**. Those commands take the ADC for up to 1.2 s, and a blind spot must not be able to manufacture a phantom transition. |

### Resolution floor: ~600–800 ms

200 ms window × 3 debounce windows = **600 ms minimum**, plus up to one window of phase, plus
whatever a request in flight cost. **A pump cycle shorter than roughly 600–800 ms is not resolved**,
here or anywhere downstream. The event timestamps are in milliseconds; that does not make them
accurate to milliseconds.

This is the deliberate trade against the `proto 1` behaviour: that version had no floor at all
because it did not resolve transitions in the first place.

---

## Protocol

Shared envelope with the well sensor: one JSON object per line, `\n`-terminated. **Responses** echo
the request `id`; **events** are unsolicited and carry no `id`. Unknown request fields are ignored;
a field of the **wrong type** is a `bad_param`, while an out-of-range value is **clamped** and the
effective value echoed back.

Three line types come *from* the board:

| `type` | When | Correlate by |
| --- | --- | --- |
| `ready` | once, at boot | — |
| `resp` | one per request | `id` |
| `event` | unsolicited: transitions and heartbeats | `seq` |

> **A reader must tolerate an `event` line arriving at any moment**, including between a request and
> its response. Match responses on `type == "resp"` and the echoed `id`, never on "next line in".

### Boot banner

```json
{"type":"ready","proto":2,"fw":"2.0.0","role":"pump"}
```

`role` distinguishes the boards. Read it rather than inferring the board from `proto`, which is
numbered per firmware.

Best-effort, like every unsolicited line: the XIAO's native USB does **not** reset the board when
the port is opened, so the banner has usually gone out before any host is listening. **`status` is
the real handshake** — do not wait for a banner.

### Events (board → Pi, unsolicited)

**Transition** — emitted immediately on a debounced state change:

```json
{"type":"event","proto":2,"role":"pump","ev":"pump","seq":412,"state":"on","ms":8134221,
 "rms_counts":199.20,"freq_hz":50.00,"asym":0.99,"prev_state":"off","prev_ms":1840500}
```

**Heartbeat** — every 60 s regardless of state:

```json
{"type":"event","proto":2,"role":"pump","ev":"hb","seq":413,"state":"on","ms":8194221,
 "rms_counts":198.40,"freq_hz":50.00,"asym":0.99,"since_ms":60000,"dropped":0,
 "n_freq_reject":0,"n_headroom":0}
```

| Field | Meaning |
| --- | --- |
| `seq` | counter, **monotonic per boot**, incremented on **every** event line — transitions *and* heartbeats. A gap is how the Pi detects a lost line |
| `state` | `on` / `off` / `fault` / `unknown` |
| `ms` | board `millis()` at the event. For a transition this is when the state was declared |
| `prev_state`, `prev_ms` | the state being left, and the `ms` at which *it* began (`pump` only) |
| `since_ms` | how long the current state has held, in ms (`hb` only) |
| `dropped` | event lines the TX path had no room for — see below (`hb` only) |
| `n_freq_reject` | windows loud enough for `on` at the wrong frequency (`hb` only) |
| `n_headroom` | windows flagged by the [`asym`](#the-asym-headroom-check) check (`hb` only) |

All durations are computed with **unsigned arithmetic**, so the `millis()` wrap at ~49.7 days is
handled on the board, once, and never by the Pi.

**The heartbeat is not filler.** Without it the Pi cannot distinguish "pump off for six hours" from
"board dead for six hours", and that distinction is the whole point of recording this.

#### `dropped` — the board never blocks to talk

If the host is enumerated but not reading, the USB TX buffer fills and `Serial.write()` stalls — and
**a board stalled in `Serial.write()` has stopped watching the pump**. So every unsolicited line is
written only when it is already known to fit; otherwise it is **dropped** and `dropped` is
incremented. `seq` still advances, so the loss is visible.

Dropping costs nothing but timeliness: **the transition is in the ring buffer either way**, and
[`history`](#requests) hands it back. Detection is never hostage to the link.

### Requests

| `cmd` | Purpose |
| --- | --- |
| `read_pump` | One unlatched measurement + verdict. Gated on sensor plausibility. |
| `sampling` | Diagnostics: same figures plus the raw waveform. Not gated. |
| `status` | Identity, limits, defaults **and the detector's live state**. The handshake and the resync primitive. |
| `history` | Replay of missed transitions. |

Parameters, all optional, accepted by `read_pump` and `sampling`. **They affect that one
measurement only — never the detector.**

| Field | Default | Range | Meaning |
| --- | --- | --- | --- |
| `cycles` | 10 | 1–60 | whole mains cycles to cover |
| `mains_hz` | 50.0 | 40–70 | nominal mains frequency; sizes the window |
| `rate_hz` | 2000 | 500–20000 | sampling rate |
| `on_counts` | 71.1 | 0–4095 | RMS at or above which the pump is `on` |
| `off_counts` | 35.5 | 0–4095 | RMS at or below which the pump is `off` |
| `bias_min` | 1024 | 0–4095 | low edge of the plausible idle window |
| `bias_max` | 3072 | 0–4095 | high edge of the plausible idle window |
| `counts_per_volt` | — | > 0 | supply to get `vrms`; omitted by default |
| `dump_n` | 200 | 0–400 | raw samples to return (`sampling` only) |

`cycles × rate_hz / mains_hz` must fit the 1200-sample buffer or the window is cut short and
`truncated:true` is returned. At the defaults it is 400.

Because the detector holds the ADC continuously, a request is parsed **between** windows — a reply
can be up to ~200 ms late. That is the intended trade: a late reply costs nothing, a missed window
costs a transition.

### Response — `read_pump`

```json
{"id":7,"type":"resp","proto":2,"status":"ok","state":"on",
 "rms_counts":199.2,"bias_counts":2052.4,"min_counts":1769,"max_counts":2335,
 "n_clipped":0,"asym":0.99,"freq_hz":50.0,"n_rise":9,
 "n_samples":400,"window_us":200013,"interval_us":500,"max_late_us":12,
 "truncated":false,"cycles_eff":10.0,"adc_bits":12,
 "cycles":10,"mains_hz":50.0,"rate_hz":2000,
 "on_counts":71.1,"off_counts":35.5,"bias_min":1024,"bias_max":3072}
```

| Field | Meaning |
| --- | --- |
| `state` | `on` / `off` / `uncertain` — **this window only, unlatched**. Not the detector's state; that is on the event lines and in `status` |
| `rms_counts` | RMS of the bias-removed signal; what the thresholds compare against |
| `bias_counts` | measured idle level; the sensor-fault check |
| `min_counts`, `max_counts` | window extremes, in raw counts |
| `n_clipped` | samples at an **ADC rail**. On this front end it stays 0 even while the wave is flattened — use `asym` |
| `asym` | `(max−bias)/(bias−min)`. ~1.0 is clean; **< 0.90 means the LM358 is out of headroom**. Meaningless below `FREQ_MIN_RMS` |
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
calibration beyond the two thresholds. On this 3V3 front end `vrms` would in any case be wrong
whenever `asym` is flagged, because the compression is not linear.

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

### Response — `status`

Everything `proto 1` returned (`fw`, `role`, `uptime_ms`, `adc_bits`, `adc_max`, `max_samples`,
`cycles_default`, `max_cycles`, `mains_hz_default`, `rate_hz_default`, `max_rate_hz`,
`on_counts_default`, `off_counts_default`, `bias_min_default`, `bias_max_default`, `max_dump_n`,
`line_max`) **plus**:

| Field | Meaning |
| --- | --- |
| `state` | the detector's current state |
| `since_ms` | how long it has held it |
| `seq` | the last `seq` the board issued |
| `dropped` | event lines the board could not write |
| `n_freq_reject`, `n_headroom` | the two warning counters |
| `detect_cycles` | window width the detector uses (10) |
| `debounce` | consecutive agreeing windows required (3) |
| `hb_ms` | heartbeat interval (60000) |
| `history_max` | ring buffer size (32) — **size your expectations from this, not from a hardcoded 32** |
| `freq_min_rms`, `freq_match_hz`, `asym_min` | the detector's other constants, published so nothing downstream keeps a copy that can drift |

**`status` is the resync primitive.** On every connect the Pi reads `state` + `since_ms` and
back-dates a change that happened while it was away. `uptime_ms` is what tells it the board
rebooted — so `since_ms` is never extrapolated across a reboot the Pi did not see. `seq` tells it
whether it missed event lines; `history` gets them back.

### Request — `history`

```json
Pi  -> {"id":9,"cmd":"history","after_seq":412}
MCU -> {"id":9,"type":"resp","proto":2,"status":"ok",
        "events":[{"seq":415,"state":"on","ms":8134221},
                  {"seq":419,"state":"off","ms":8320145}],
        "truncated":false,"more":false}
```

A 32-entry RAM ring buffer holding `{seq, state, ms}` for every **`pump`** event — heartbeats
consume a `seq` but are not recorded, because a replay has no use for them.

| Field | Meaning |
| --- | --- |
| `after_seq` | request: return events with `seq` **greater than** this. Omit for everything still buffered |
| `events` | chronological, oldest first |
| `truncated` | **true when `after_seq` is older than the oldest entry still buffered** — some transitions are permanently gone, and the Pi must not read a short list as completeness |
| `more` | true when the reply hit the 8-event cap; call again with the highest `seq` received |

**This is what makes a Pi reboot lossless.** Without it, a pump cycle that both starts *and* ends
while the Pi is down disappears with no trace at all — the daily runtime is quietly short and
nothing indicates it.

Nothing is persisted, so the ring is empty after a board reboot. That is not a gap the Pi has to
guess at: `uptime_ms` says the board restarted, and `unknown` says it does not know what it missed.

### Errors

```json
{"id":7,"type":"resp","proto":2,"status":"error","code":"sensor_fault", ...}
```

| `code` | Meaning |
| --- | --- |
| `sensor_fault` | `bias_counts` outside `[bias_min, bias_max]`. The module is not reporting, so the pump's state is **unknown** — not `off`. Retrying will not help. The detector enters the `fault` state for the same reason |
| `bad_param` | a field of the wrong type, or `on_counts < off_counts`, or `bias_min >= bias_max`, or `counts_per_volt <= 0`, or a non-integer `after_seq`. `field` names it |
| `bad_request` | line was not valid JSON. `"id":null` |
| `bad_id` | `id` missing or not an integer. `"id":null` |
| `unknown_cmd` | unrecognised `cmd` |
| `line_too_long` | request exceeded 192 characters; the reader drains to the newline and recovers |

`sensor_fault` responses still carry the full measurement and context, so one logged line explains
itself.

### Watchdog

The RP2040's hardware watchdog is armed at boot with an 8 s timeout, before the banner is printed.
The board now runs unattended and continuously and **nothing can restart it remotely** — the Pi
cannot reset it, and recovery would otherwise mean a UF2 reflash or a walk to the well. A hung
detector is silent data loss, which is the failure this firmware exists to prevent.

8 s is comfortably longer than the slowest legitimate blocking stretch (a 60-cycle window at 40 Hz
is 1.5 s; a `sampling` reply with 400 raw counts is a couple of kB to a host that is reading). It is
deliberately **not** fed from inside a stalled `Serial.write()`: a solicited reply that cannot drain
in 8 s means the host stopped reading mid-reply, and a reset is the right answer to that.

**Consequence for the Pi:** a reboot shows up as `uptime_ms` going backwards and `state` returning
to `unknown`. Treat it as a gap, not as a transition.

---

## Calibration (required)

`DEFAULT_ON_COUNTS` and `DEFAULT_OFF_COUNTS` cannot be shipped as universal values: the
counts-to-volts relationship lives in the module's gain pot and no firmware can know where it is
set. The values currently in the source are **measured against this installation**, not placeholders
and not guesses:

```
Calibrated 2026-08-28, tools/pump_tune.py, 3V3 front end:

  floor  12.93 counts (worst of 5, pump stopped)   -> off_counts 35.5   2.7x clear
  signal 199.20 counts (worst of 5, pump running)  -> on_counts  71.1   2.8x clear
  separation 15.4x;  bias 2052;  rms/peak 0.703 (a sine is 0.707)
```

The thresholds are placed **geometrically** between the two states rather than by the 5× / ÷3 rule
below, which needs about 20× separation before it leaves a usable gap — at 15.4× it put the two
thresholds 1.7 counts apart, which is a single threshold wearing a disguise.

`FREQ_MIN_RMS` is **40.0**, also measured: this installation's pump-off floor is 8.4–14.2 counts,
which cleared the original 10.0 guard, so the board was deriving a frequency from ADC noise and
reporting it with a straight face — 274, 290, 307, 341, 425 Hz across consecutive windows. 40.0 is
~3× the observed floor; it leaves the 199-count running signal untouched and returns an honest
`freq_hz` of 0 when the pump is stopped.

**Re-run the calibration if** the pot moves, the module is replaced, the 3V3 rail changes, or
anything on that circuit changes.

### The short way

[`tools/pump_tune.py`](../tools/pump_tune.py) drives the protocol from a PC on the bench and does
all of the below, including the arithmetic and the quality checks (`pip install pyserial`):

```
python tools/pump_tune.py probe        # one window, with every trap in this page checked
python tools/pump_tune.py plot         # ASCII waveform -- turn the pot while watching it
python tools/pump_tune.py listen       # events + heartbeats as they arrive, with seq-gap flags
python tools/pump_tune.py history      # replay the buffered transitions
python tools/pump_tune.py calibrate    # guided off/on, prints the two #define lines
```

`calibrate` derives the thresholds from the **worst** window seen in each state rather than the
average, and refuses to emit anything if the two states are not separated — which is, among other
things, what a module on the wrong side of the contactor looks like.

### Setting the gain pot

`plot` measures the margin against the **LM358 ceiling**, not the ADC rail, because on this front
end the rail is not the limit:

```
python tools/pump_tune.py plot          # --ceiling defaults to 2480 for the 3V3 rig
```

Turn the pot **down** until the `asym` warning clears and the margin reads `good`. Aim for the
middle of the band rather than the edge — the ceiling moves with supply and temperature, and a pot
set right against it will start clipping in a warm plant room, or on a 253 V day.

> **A multi-turn pot gives no clue which way is "down", and `max_counts` is pinned at the ceiling
> so it will not move whichever way you turn.** Judge by `rms_counts` (must fall) and `asym` (must
> climb toward 1.0) instead. `pump_tune.py watch` shows both live.

### By hand

1. **Flash and connect.** `pio run -e pump -t upload`, then open the serial monitor at 9600.
   Expect a heartbeat every 60 s and an event within a second or so of any pump change — that alone
   proves the detector is running.
2. **Pump OFF** — set the noise floor:
   ```json
   {"id":1,"cmd":"sampling","dump_n":200}
   ```
   Note `rms_counts` and check `bias_counts` is near 2048 with `bias_ok:true`.
3. **Pump ON** — set the gain:
   ```json
   {"id":2,"cmd":"sampling","dump_n":200}
   ```
   - **`asym` must be ≥ 0.90.** If it is not, turn the pot **down** and repeat — the top of the wave
     is being flattened and `rms_counts` understates the real signal. `n_clipped` will **not** tell
     you this on a 3V3 module.
   - Aim for `max_counts` around 2300–2350 — a clear signal with headroom against the ~2480
     ceiling, **not** against 4095.
   - `freq_hz` should read within a few tenths of your mains frequency. If it does not, you are
     looking at pickup rather than the feed.
4. **Set the thresholds** from `rms_off` and `rms_on`. With ≥ 20× separation the classic rule works:
   ```
   off_counts ≈ max(5 × rms_off, 20)
   on_counts  ≈ rms_on / 3
   ```
   Below that, place them geometrically: `mid = √(rms_off × rms_on)`, then `off = 0.7 × mid`,
   `on = 1.4 × mid`. That is what produced 35.5 / 71.1. Both can be overridden per request, so test
   candidate values before editing the source.
5. **Check whether the pump-off floor is noise or coupled mains.** If the off-state windows report
   a `freq_hz` at the mains frequency, the floor is not ADC noise — it is mains leaking through with
   the contactor open, typically an RC snubber or indicator lamp across the contacts, or capacitive
   pickup between the sense wires and the feed. Measured on this installation at an earlier pot
   setting: **26.3 counts at 50.12 Hz**, against 204 counts running — a fixed 7.8× separation.

   The consequence is counter-intuitive and worth stating plainly: **raising the gain cannot widen
   that ratio.** Coupled mains enters ahead of the module's amplifier, so the pot multiplies it by
   exactly the same factor as the signal. Turning the gain up only walks the waveform back into
   clipping. The separation is a property of the installation; to improve it, reduce the coupling.
6. **Check the floor against `FREQ_MIN_RMS`.** If the pump-off `rms_counts` from step 2 is anywhere
   near it, raise it to about 3× that floor; otherwise the guard is not guarding and `freq_hz` goes
   back to being invented from noise.
7. **Verify both states**, then rebuild with the values baked in and confirm the *events* land, not
   just the readings.

Record the numbers you measured in the commit message. The well sensor's `ack_timeout_us` comment
is the precedent: a constant that was guessed and then measured, with the measurement written down,
so the next person knows which it is.

---

## Testing after a flash

Serial monitor at 9600 baud (or `pump_tune.py listen`), reset the board.

1. **Banner** → `{"type":"ready","proto":2,"fw":"2.0.0","role":"pump"}`. May be missed — native USB
   does not reset on port open. Not a fault; use `status`.
   - **Look at the board.** Exactly **one** LED should be doing anything: the user LED, blue at
     2 Hz. If the NeoPixel or a second colour is also lit, the firmware on the board predates this
     change — everything unused is driven off at boot.
2. `{"id":1,"cmd":"status"}` → `ok` with the defaults and limits (`adc_bits:12`, `max_samples:1200`,
   `rate_hz_default:2000`, `line_max:192`) **and** the detector block (`state`, `since_ms`, `seq`,
   `debounce:3`, `hb_ms:60000`, `history_max:32`).
3. **The detector is running** — leave the port open with the pump steady. Within ~1 s of boot a
   `pump` event should declare `off` (or `on`) out of `unknown`, and a `hb` line should appear every
   60 s with `seq` incrementing by 1 each time. **A gap in `seq` means a line was lost** — check
   `dropped` in the next heartbeat.
4. **A real transition** — switch the pump on. Expect an `ev:"pump"` line with `"state":"on"`,
   `prev_state:"off"`, within roughly 600–800 ms. Switch it off and expect the mirror image.
   Check `prev_ms` matches the `ms` of the previous transition.
   - **Watch the LED while you do it.** Blue at 2 Hz for the ~600 ms the change is being debounced,
     then solid green (on) or the green blip (off). At reset it is blue until the first state is
     declared. This is the whole state machine visible without a serial port.
5. **Debounce holds** — flick the pump on and straight back off in under half a second. Expect
   **no** event. That is the resolution floor doing its job, not a miss.
6. **Sampling quality** — `{"id":2,"cmd":"sampling","dump_n":0}` with the pump running:
   - `max_late_us` should be a small fraction of `interval_us` (500 µs at defaults).
   - `truncated:false` and `cycles_eff` ≈ `cycles`.
   - `asym` ≥ 0.90.
   - `freq_hz` within a few tenths of 50. **This is the check that the module is seeing real mains**
     rather than induced hum.
7. **Both states** — `{"id":3,"cmd":"read_pump"}` with the pump on, then off. Expect `"on"` then
   `"off"`. **If it reads `"on"` both times, the module is on the wrong side of the contactor** —
   nothing else in this checklist will catch that.
8. **Sensor fault, the important one** — **unplug the `A0` signal wire**. Expect `sensor_fault` from
   `read_pump`, and within ~600 ms an event with `"state":"fault"` — *not* `"off"`: a disconnected
   sensor must not look like a stopped pump. **The LED must go red at 10 Hz, not dark** — same rule,
   on the output you can see from across the room. Reconnect and confirm it recovers with another
   event.
   - Also unplug the module's `3V3`: also `fault`.
   - Run `sampling` in each case and read `bias_counts` / `bias_ok` to see why.
9. **History** — after a few transitions, `{"id":4,"cmd":"history"}` → the buffered events oldest
   first. Then `{"id":5,"cmd":"history","after_seq":<a recent seq>}` → only the newer ones,
   `truncated:false`. Force more than 32 transitions and confirm an old `after_seq` comes back
   `truncated:true` rather than quietly short.
10. **Never blocks** — open the port with a program that does not read (or `Ctrl-S` a terminal),
    leave it 5 minutes, then read again. The board must still be alive and answering, and the next
    heartbeat's `dropped` must be non-zero with `seq` showing the gap.
11. **Headroom warning is reachable** — turn the pot up until `asym` drops below 0.90 on a running
    pump. Confirm the state stays `on` (an ugly waveform is still a running pump) and `n_headroom`
    climbs, then turn it back down and re-run the [calibration](#calibration-required).
12. **Parameters:**
    - `{"id":6,"cmd":"read_pump","cycles":999}` → clamped, echoes `cycles:60`.
    - `{"id":7,"cmd":"read_pump","cycles":"ten"}` → `bad_param`, `"field":"cycles"`.
    - `{"id":8,"cmd":"read_pump","on_counts":50,"off_counts":100}` → `bad_param`,
      `"field":"on_counts"`.
    - `{"id":9,"cmd":"read_pump","cycles":60,"rate_hz":20000}` → `truncated:true` with `cycles_eff`
      well under 60.
    - `{"id":10,"cmd":"read_pump","counts_per_volt":3.1}` → adds `vrms` and `unit:"V"`.
    - **A per-request override must not move the detector**: `{"id":11,"cmd":"read_pump",
      "on_counts":4000}` on a running pump → `"uncertain"` in the *response*, while `status` still
      reports `"state":"on"`.
13. **Errors:** send `hello` → `bad_request` with `"id":null`. Send `{"cmd":"status"}` → `bad_id`.
    Send `{"id":12,"cmd":"nope"}` → `unknown_cmd`. Paste a line over 192 characters →
    `line_too_long`, then confirm the **next** valid request still answers.
14. **Reader:** two requests back-to-back with no delay → two correlated responses, no lost line,
    even if an event line lands between them.

## Pi-side integration

Not yet written. The board is a structural match for `read_puit.py` for the *request* path — same
framing, same `id` echo, same error envelope — but the event path has no precedent in this project
and is where the work is.

The shape it needs:

- **Hold the port open and read continuously.** Every line that parses as `type:"event"` is a fact
  to record. This is a listener, not a poller.
- **On connect, `status` first.** Take `state` + `since_ms` and back-date the transition it implies;
  compare `uptime_ms` against the last one seen to detect a reboot.
- **Then `history` with the last `seq` recorded.** Anything it returns happened while the Pi was
  away. `truncated:true` means some are gone for good — record the gap explicitly rather than
  interpolating over it.
- **Watch `seq` for gaps** on every event line, and call `history` when one appears.
- **`fault` must not be recorded as "pump off".** It means unknown. Logging it as `off` produces a
  clean-looking time series that says the pump never ran.
- **`unknown` is not a state to plot.** It means the board has not decided yet.
- **Silence is a signal.** No heartbeat for appreciably more than 60 s means the link or the board
  is down — not that the pump is off.

## Known limitations

- **Resolution floor ~600–800 ms.** Set by the 200 ms window and the 3-window debounce. A cycle
  shorter than that is not resolved. Deliberate: the debounce is what absorbs contactor bounce and
  motor inrush.
- **Voltage present ≠ water moving.** This detects that the motor is energised. A pump that is
  energised but airlocked, dry, or with a closed valve reads `"on"`.
- **Single phase.** One ZMPT101B on one conductor. A three-phase pump losing one phase reads `"on"`
  if the monitored phase is still live.
- **Wrong-side-of-contactor wiring is undetectable.** Restated because it is the only failure here
  with no diagnostic signature at all.
- **History is RAM only.** 32 transitions, lost on reboot. Persisting it to flash was rejected: a
  reboot must start from `unknown`, and flash wear on a board that may transition thousands of times
  a season is a poor trade for a gap the Pi can already see from `uptime_ms`.
- **`asym` cannot separate a flattening op-amp from an ugly grid** on its own. Real mains carries
  odd harmonics from every rectifier on the circuit, which flatten *both* halves; the asymmetry test
  is what distinguishes them, and a genuinely lopsided supply would fool it. `pump_tune.py` also
  checks rms/peak fullness for that reason.
