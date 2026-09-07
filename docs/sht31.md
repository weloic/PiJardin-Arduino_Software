# SHT31 bench rig (`sht31`)

Firmware for qualifying a **Sensirion SHT31** temperature/humidity probe on the desk, before
it is fitted at the well head. Source: [src/sht31/sht31_sensor.cpp](../src/sht31/sht31_sensor.cpp).

> ⚠️ **This is not a deployed board.** It exists to answer one question about a newly bought
> probe — *is it wired right, does it answer, and are the numbers real?* — on whatever XIAO
> happens to be free. The probe's real home is the **well sensor**, per
> [README.md § Future: environment sensing](../README.md#future-environment-sensing).
> Expect this environment to be deleted, or kept as a regression rig, once the read is folded
> into `src/puit/`.

It runs on a **XIAO RP2040** (the spare board), not the SAMD21 the well sensor uses. Nothing
here depends on that: the probe is I²C and the firmware uses no chip-specific facility beyond
`Wire` and the RP2040's LEDs. The measurement path is written to be lifted into `src/puit/`
unchanged.

## Contents

- [Wiring](#wiring)
- [Cable length, pull-ups and bus speed](#cable-length-pull-ups-and-bus-speed)
- [Bringup in five minutes](#bringup-in-five-minutes)
- [Status LED](#status-led)
- [Serial protocol](#serial-protocol)
- [Commands](#commands) — `read_env`, `stream`, `heater`, `selftest`, `scan`, `bus`, `lines`,
  `status`, `reset`
- [`lines` as a cable tester](#lines-as-a-cable-tester)
- [What counts as a valid sample](#what-counts-as-a-valid-sample)
- [Error codes](#error-codes)
- [Build & flash](#build--flash)
- [Testing after a flash](#testing-after-a-flash)
- [What moves into `puit` later](#what-moves-into-puit-later)

**Not in this file:** wiring the probe to a board other than this rig, its measured electrical
characteristics, and **verifying that the temperature is accurate** (settling time, the ice point,
drying after a wet test) — all in [docs/sht31-hookup.md](sht31-hookup.md), which depends on no code
from this repo.

## Wiring

| SHT31 breakout | XIAO RP2040 | Note |
|---|---|---|
| `VIN` / `VCC` | **`3V3` pad** | **never 5V** — see below |
| `GND` | `GND` | |
| `SDA` | `D4` (GPIO6) | i²c1 SDA |
| `SCL` | `D5` (GPIO7) | i²c1 SCL |
| `ADR` | `GND`, or leave unconnected | → address **`0x44`** (the firmware default) |
| `ADR` | `VIN` | → address **`0x45`** — pass `"addr":69` on every command |

> **To wire this probe to a board other than the bench rig**, see
> [docs/sht31-hookup.md](sht31-hookup.md) — the conductor map, per-board pin tables, the measured
> electrical characteristics, and a board-agnostic read with no dependency on this firmware.

### Cabled probes

A sealed probe on a flying lead has no silkscreen, so the colours are all you have — and **the
colours lie**. The wiring that actually works on the 5-conductor probe used here:

| Colour | What it really is | Connect to |
|---|---|---|
| red | VCC | `3V3` |
| yellow | **SDA** | `D4` |
| green | **SCL** | `D5` |
| black ×2 | GND / shield | **both** to `GND` |

> ⚠️ **Yellow is SDA and green is SCL on this probe — the opposite of the convention every tutorial
> quotes** (yellow = SCL, green = SDA). Believing the convention cost a full bringup session here.
> Assume nothing from the colours: wire your best guess, `scan`, and if it comes back empty **swap
> the two data wires and scan again**. They are open-drain 3.3 V lines at both ends, so the swap
> risks nothing — it is the cheapest test in this document and it is the one that finally worked.

Both blacks are on `GND` in the working configuration. Whether that is required was never isolated:
one of them is the cable shield and the other the sensor's ground return, and grounding both is
correct either way. If the second black is instead the SHT31's `ADDR` pin, grounding it selects
0x44 — which is another reason the "both to GND" move is the right first attempt rather than a
guess to be narrowed down later.

**Only the red/black pair is dangerous to get wrong.** Reversing VCC and GND can destroy the
sensor. SDA and SCL are open-drain 3.3 V lines at both ends, so swapping *those* damages nothing —
the bus just fails to work and `scan` returns `"found":[]`. So verify power with a meter and settle
the data pair by trying it.

**Telling the two blacks apart.** Gauge is not reliable: a foil shield's drain wire is *thinner*
than the signal conductors, while a braided shield twisted into a pigtail comes out *fatter*. The
decisive test, unpowered, is that **the shield is open to every other conductor** while GND is not —
GND shows a ~0.4–0.8 V diode drop to each coloured wire in diode mode (the die's ESD diodes), and in
resistance mode it charges the module's decoupling cap, so the reading drifts upward instead of
sitting at `OL`. Continuity between the two blacks means the shield is bonded to GND inside the
probe and the distinction is moot.

## Cable length, pull-ups and bus speed

> ⚠️ **This is the failure that costs an afternoon.** A probe on a long cable with no external
> pull-ups NACKs every address, which is **indistinguishable from an absent sensor** — `scan`
> returns `"found":[]` and `status` returns `"reg_error":"nack"` in both cases.

I²C budgets **400 pF for the whole bus**. Shielded cable runs roughly **100 pF/m** per conductor to
the shield, so the cable alone spends the entire budget somewhere around 3–4 m. The falling edge is
driven by the sensor, so only the **rise** is at risk, and the rise is `R × C`:

| Pull-up | τ = R×C at 300 pF (3 m) | Rise to V_IH (~1.2 τ) | Fits a 100 kHz clock high (~4 µs)? |
|---|---|---|---|
| **~65 kΩ — the RP2040's internal ones** | 19.5 µs | ~23 µs | **no** |
| 10 kΩ | 3.0 µs | ~3.6 µs | no (spec limit is 1 µs) |
| 4.7 kΩ | 1.4 µs | ~1.7 µs | marginal |
| **2.2 kΩ** | 0.66 µs | ~0.8 µs | **yes** |

`Wire.begin()` in the arduino-pico core enables the RP2040's internal pull-ups (`gpio_pull_up` on
both lines), so the bus is never floating — but at ~65 kΩ those are only good for a sensor on short
jumpers. **On a cable of a metre or more, fit 2.2 kΩ from SDA to 3V3 and 2.2 kΩ from SCL to 3V3**,
at the XIAO end. (2.2 kΩ sinks 1.5 mA at 3.3 V; the SHT31 and the RP2040 both drive 4 mA.) Note this
supersedes the 4.7 kΩ usually quoted for a bare module on a breadboard — that value is marginal
once there is 3 m of cable on the line.

Also **ground the shield at the MCU end** on a long run. A floating shield still loads both lines
*and* couples them to each other, so a clock edge crosstalks into SDA; grounding one end makes the
capacitance a defined quantity and kills the coupling. One end only — grounding both invites a loop.

### Proving it is the edges and not the wiring

Bus speed is settable at runtime precisely so this is a test rather than a theory. A slow clock
stretches the time available for the rise: at 10 kHz the clock is high for ~50 µs, which even a
23 µs rise fits.

```json
{"id":1,"cmd":"scan","sweep":true}
```

That probes the bus at 400/200/100/50/20/10 kHz and reports each rung:

| Sweep result | Reading |
|---|---|
| found only at the **slow** end | **rise time** — pull-ups too weak for the cable. Fit 2.2 kΩ and it will work at 100 kHz |
| found at **every** speed | the bus is fine; whatever is wrong is elsewhere |
| found at **no** speed | **not an edge problem** — go to `lines` next, then power, GND, SDA/SCL swapped, or a dead probe |
| **every address** answers at some speed | SDA is stuck low — a short, or a slave holding the bus |

The ladder runs down to 2 kHz, which is 50× slower than the default, so "found at no speed" is a
statement about the wiring rather than about how far the ladder happened to reach.

The sweep restores the session speed when it finishes; it is a probe, not a setting.

If it turns out the resistors are not to hand, `{"cmd":"bus","i2c_hz":10000}` will get you a working
sensor to play with today. It is a diagnosis, not a configuration — it is deliberately **not**
persisted, so a reset returns to 100 kHz rather than quietly hiding the fault it revealed.

> ⚠️ **3V3, not 5V — and not only because of the RP2040.** The bare SHT31 die is a 2.4–5.5 V
> part, so "it's a 5 V-tolerant sensor" is a half-truth that destroys boards. Most breakouts
> regulate to 3.3 V *and pull SDA/SCL up to their own rail*: powered from 5 V, that breakout
> drives **5 V into GPIO6/7**, and the RP2040 is not 5 V tolerant. Powering everything from
> `3V3` is the only arrangement that is safe whatever the breakout does.

**Pull-ups.** Most breakouts (Adafruit, Sparkfun, the common blue Chinese modules) carry 10 kΩ
pull-ups on board. A bare sensor or a potted probe on a lead often does not, and a bus with
pull-ups too weak for its cable looks *exactly* like a bus with no sensor — `scan` returns nothing.
Values, and how to tell that case apart from a genuinely absent sensor, are in
[Cable length, pull-ups and bus speed](#cable-length-pull-ups-and-bus-speed).

**Lead length.** The firmware runs the bus at **100 kHz** (not the 400 kHz the sensor supports)
because the sensor's own 15 ms conversion dominates the transaction anyway, so a faster bus would
buy nothing and cost edge margin. Anything longer than short jumpers needs external pull-ups —
[Cable length, pull-ups and bus speed](#cable-length-pull-ups-and-bus-speed) has the arithmetic and
the values. If you see `crc_error`, that is the bus telling you the leads or the pull-ups are
marginal; it is *not* a broken sensor.

## Bringup in five minutes

Flash, open the serial monitor at 9600 baud, and go in this order. Each step rules out a
different thing, so a failure at step *n* means nothing after it is worth trying.

1. **Look at the LED before touching the keyboard.** Green blipping every 3 s = the probe is
   answering with valid CRCs, and you are done with the wiring. Red flashing = it is not; go to
   step 2. (The LED is meaningful without the monitor because on native USB the boot banner is
   usually gone before the host attaches.)
2. `{"id":1,"cmd":"scan"}` — what is on the bus?
   - `"found":[68]` → the sensor is at 0x44, wiring good.
   - `"found":[69]` → it is at 0x45: the `ADR` pin is tied high. Add `"addr":69` to everything
     below, or move the pin.
   - `"found":[]` → nothing acknowledged. **Swap the two data wires and scan again** — that is one
     move, it risks nothing, and on the probe used here it was the answer. If both orders come back
     empty, run `{"id":1,"cmd":"scan","sweep":true}`: a bus whose edges are too slow looks exactly
     like an empty one, and the sweep tells the two apart in one command (see
     [Cable length, pull-ups and bus speed](#cable-length-pull-ups-and-bus-speed)). Only if the
     sweep finds nothing at any speed, in either wire order, is it worth suspecting power, GND, or
     the probe itself.
3. `{"id":2,"cmd":"status"}` — `"sensor_present":true` plus the sensor's own status register.
4. `{"id":3,"cmd":"read_env"}` — three samples, and `"sample_status":"VVV"`.
   A room reads roughly 18–25 °C and 35–60 %RH; `temp_spread` should be a few hundredths of a
   degree.
5. `{"id":4,"cmd":"stream","on":true}` — a line a second. **Breathe on the probe**: RH should
   jump 20–30 points within a second or two and fall back over ten. That is the sensor
   responding to the world rather than repeating a constant.
6. `{"id":5,"cmd":"heater","on":true}` — the definitive test, below.

### The heater test — the one that actually proves the sensor is real

A module that is faulty, counterfeit, or wired to something else entirely can still return a
plausible-looking constant. The SHT31 has an **internal heater** that a real one cannot fail to
respond to — and one command runs the test and judges it:

```json
{"id":6,"cmd":"selftest"}
```

`"verdict":"pass"` means temperature rose **and** humidity fell: the probe is measuring the real
world. On a potted probe expect `weak` or a modest `d_temp_c` — read `snr`, which expresses the rise
in units of the sensor's own measured noise. See [`selftest`](#selftest) for why judging this by eye
does not work.

> The heater biases what it measures — that is what it is for. Every measurement reply carries
> `"heater":true|false` so a diagnostic reading can never be mistaken for an ambient one after
> the fact, and it **auto-switches off** (10 s default, 30 s ceiling) because forgetting it is
> the obvious mistake to make at a bench at 11 pm. `{"cmd":"heater","on":false}` ends it early;
> `"ms":0` disables the auto-off and keeps it on until told otherwise.

## Status LED

The XIAO RP2040's three-in-one user LED, active low, one die at a time. Same convention as the
pump board ([docs/pump.md](pump.md#status-led)) — and, as there, **every** LED on the package
including the unused NeoPixel is explicitly driven off at boot, because an uninitialised pin is
an input and these LEDs are tied to 3V3, so a floating cathode glows.

| Indication | Meaning |
|---|---|
| **green**, 60 ms blip every 3 s | last read was good — sensor answering, CRCs valid |
| **red**, 10 Hz | last read **failed** — wiring, address, or pull-ups |
| **blue**, 2 Hz | nothing has been read successfully yet |
| **blue**, solid | **heater on** — readings are deliberately biased |
| dark | the board is not running |

The firmware re-reads the sensor **every 5 s in the background** (silently — no line is emitted)
purely so this stays honest on a board nobody is talking to. That background poll is skipped
while streaming, so the sensor's duty cycle never doubles.

Fault outranks the heater: a solid blue "readings are biased" would be useless advice when the
real answer is that nothing is answering at all.

## Serial protocol

The same **NDJSON envelope** as `puit` and `pump` — one JSON object per line, `\n`-terminated,
in both directions, every response echoing the request `id`. The shared parts are documented in
[README.md § Serial protocol contract](../README.md#serial-protocol-contract); only what is
specific to this board is below.

- **Transport:** USB CDC serial, 9600 baud (nominal on native USB).
- **`proto` = 1.** Protocol versions are numbered **per firmware** in this repo, so this board's
  `proto 1` has nothing to do with `puit`'s or `pump`'s `proto 2`. Read `role` — `"sht31"` — to
  know what answered.
- **Boot banner:** `{"type":"ready","proto":1,"fw":"1.0.0","role":"sht31"}`.
- **This board can talk without being asked**, like the pump board and unlike the well sensor:
  `{"type":"event",…}` lines arrive while streaming, and one arrives whenever the heater
  auto-switches off. **Match responses on `type == "resp"` *and* the echoed `id`** — never on
  "next line in".

## Commands

| `cmd` | What it does |
|---|---|
| `scan` | walk the bus, list every address that acknowledges; `"sweep":true` repeats it across bus speeds |
| `bus` | set the I²C clock for the session (2 kHz – 400 kHz) |
| `lines` | measure the two bus lines electrically — **is a cable actually attached to this pin?** |
| `status` | identity, limits, and a **live** read of the sensor's status register |
| `read_env` | one measurement burst — the measurement command |
| `stream` | turn the unsolicited `ev:"env"` event stream on or off |
| `heater` | drive the sensor's internal heater (auto-off) |
| `selftest` | run the heater test and **judge it** — the definitive "is this probe real?" check |
| `reset` | soft-reset the sensor, report the reset-detected bit |

`addr` (`68`/`0x44` or `69`/`0x45`) is accepted by every command except `scan`. It is the one
parameter that is **not** clamped when out of range — those are the only two addresses an SHT31
can be at, so anything else is a mistake rather than an over-ambitious value, and clamping would
quietly talk to a different address than the one asked for and then report success.

### `read_env`

```json
{"id":3,"cmd":"read_env","n":3,"rep":"high","gap_ms":100,"addr":68}
```

| Field | Type | Default | Range | Notes |
|---|---|---|---|---|
| `n` | int | `3` | 1 – 10 | samples per burst |
| `rep` | string | `"high"` | `high`/`medium`/`low` | repeatability; sets the conversion wait (20/10/8 ms) |
| `gap_ms` | int | `100` | 2 – 2000 | spacing between samples |
| `addr` | int | `68` | `68` or `69` | I²C address |

Wrong **type** → `bad_param` with `field`. Out-of-**range** → silently clamped, which is safe
only because every reply echoes the effective value. `rep` is the exception: an unrecognised
string is an error rather than a fallback, since there is no nearest sensible repeatability to
pick and guessing would measure at one setting while reporting another.

Response:

```json
{"id":3,"type":"resp","proto":1,"status":"ok",
 "temp_c":22.41,"rh_pct":47.8,"unit_temp":"C","unit_rh":"%","t_raw":25236,"rh_raw":31326,
 "n":3,"n_valid":3,"n_crc":0,"n_nack":0,"n_bus":0,"n_short":0,
 "temp_min":22.40,"temp_max":22.42,"temp_spread":0.02,
 "rh_min":47.7,"rh_max":47.9,"rh_spread":0.2,
 "addr":68,"rep":"high","gap_ms":100,"heater":false,
 "temps_c":[22.40,22.41,22.42],"rh_pcts":[47.7,47.8,47.9],
 "t_raws":[25234,25236,25238],"rh_raws":[31300,31326,31352],
 "sample_status":"VVV"}
```

`temp_c` and `rh_pct` are the **medians over the valid samples only**. `t_raw` / `rh_raw` are the
raw 16-bit ticks behind them — stored for the same reason `puit` stores `pulse_us` next to its
cm: the ticks are what the sensor actually produced and the only part that cannot go stale, so a
conversion that later turns out to be wrong can be replayed over history. Both conversions are
monotonic, so the raw medians are the exact counterparts of the converted ones.

> **`temp_c` is deliberately the same field name `puit` already uses** for its assumed air
> temperature. When the probe moves to the well, the Pi keeps reading the same key and it simply
> stops being an assumption.

`heater` rides on **every** measurement reply. A reading taken with the heater on is several
degrees warm and several points dry; a consumer that cannot see that from the reply itself will
average a diagnostic into its history.

### `stream`

```json
{"id":4,"cmd":"stream","on":true,"period_ms":1000,"n":1,"ms":120000}
```

`period_ms` 250 – 60000 (default 1000), `n` 1 – 10 (default 1), `ms` 0 – 3600000 (default
**120000**), `fmt` `"json"` (default) or `"text"` — see below. Emits:

```json
{"type":"event","proto":1,"role":"sht31","ev":"env","seq":7,"uptime_ms":41233,
 "status":"ok","temp_c":22.41,"rh_pct":47.8,"t_raw":25236,"rh_raw":31326, …counts, params, detail…}
```

Event lines carry **no `id`**, carry `seq` instead, and a failed burst arrives as
`"status":"error"` with a `code` rather than not arriving at all.

#### Reading it by eye: `"fmt":"text"`

```json
{"id":4,"cmd":"stream","on":true,"fmt":"text"}
```

```
      temp     humidity
      22.41 C     47.83 %
      22.43 C     47.79 %
      22.44 C     47.81 %
      -- echo_timeout --
      -- stream stopped --
```

**This is the one place the firmware emits something that is not JSON**, and the NDJSON contract is
not weakened by it: nothing turns it on but a person typing `"fmt":"text"`, the Pi never sends it,
and a reset returns to `json`. The command's own *response* stays JSON either way — only the stream
lines change.

It exists because columns of numbers are legible in a way a 300-character object never is, and the
bench work this rig is for — watching a value settle in an ice bath, breathing on the probe, waiting
for a heater cycle to decay — is all eye work. `fmt` is `"json"` or `"text"`; anything else is
`bad_param`, because silently emitting the format the caller did not ask for is worse than refusing.

**It stops itself after `ms`** (2 minutes by default), announcing it rather than just going quiet —
a console that falls silent reads the same as a board that has crashed:

```json
{"type":"event","proto":1,"role":"sht31","ev":"stream","seq":88,"uptime_ms":132400,
 "stream":false,"reason":"auto_off"}
```

`{"cmd":"stream","on":false}` ends it early, and a reset always does — nothing is persisted.
`"ms":0` runs until told otherwise, for a deliberate long recording.

> The auto-stop exists because the first version of this command ran forever, and stopping it meant
> typing a JSON line into a serial monitor scrolling at one line per second. Same reasoning as the
> heater's: forgetting is the obvious mistake, so the board should not need to be told.

Streaming is **off at boot**, deliberately: a board that starts talking the moment it is powered
races the Pi's banner handshake, and a bench convenience is not worth making the wire less
predictable.

The 1000 ms default is not arbitrary — Sensirion specifies at most one high-repeatability
measurement per second to keep self-heating under 0.1 °C.

### `heater`

```json
{"id":5,"cmd":"heater","on":true,"ms":10000}
```

`ms` 0 – 30000 (default 10000); `0` means no auto-off. When it does auto-switch off, an
unsolicited line announces it — silence would leave a reader believing the readings are still
biased long after they stopped being:

```json
{"type":"event","proto":1,"role":"sht31","ev":"heater","seq":9,"uptime_ms":51240,
 "heater":false,"reason":"auto_off"}
```

If the *off* command is not acknowledged, `reason` is `"auto_off_failed"` and `heater` stays
`true`: the heater may well still be on and there is no way to be sure from the board, so the
readings keep being marked biased. A reading marked biased that is not is a nuisance; one marked
clean that is not is corrupt data.

### `selftest`

```json
{"id":5,"cmd":"selftest","ms":20000}
{"id":5,"type":"resp","proto":1,"status":"ok","verdict":"pass",
 "d_temp_c":2.04,"d_rh_pct":-7.3,"snr":68.1,
 "base_temp_c":22.41,"base_rh_pct":47.8,"base_spread_c":0.03,
 "hot_temp_c":24.45,"hot_rh_pct":40.5,
 "heat_ms":20000,"n":5,"addr":68,"heater_off":true}
```

Baseline → heat → re-measure → verdict, with a progress event every 2 s. `ms` is 1000 – 30000
(default 20000). It **blocks** for the duration and does not service serial meanwhile, which is why
it reports progress: a console that goes quiet for twenty seconds reads as a crash. The heater is
turned off on every exit path, failures included.

| `verdict` | Meaning |
|---|---|
| `pass` | temperature up ≥ 0.5 °C **and** humidity down ≥ 1 %RH — the probe is real |
| `weak` | right directions, small amplitude. Normal on a **potted probe**; check `snr` |
| `fail` | the directions are wrong or nothing moved |

**Why this is a command and not a procedure.** It used to be: turn the heater on, watch the stream,
satisfy yourself that temperature rises while humidity falls. That works on a bare breakout and
fails on a probe potted into a housing, where ~33 mW of heater goes into a hundred times the die's
thermal mass — the rise is real but small and slow, and *"difficult to be sure"* is the honest
verdict a person reaches watching a scrolling console.

The mistake was asking a person to judge a trend by eye. The SHT31's measurement noise is ~0.02 °C,
so even a **0.5 °C rise is twenty-five times the noise** — overwhelming evidence, and invisible in a
console. `snr` is the difference expressed in units of the baseline's own measured scatter, and it
is what makes a small number conclusive. Absolute degrees were never the criterion.

**Temperature up *and* humidity down together** is the part nothing can fake: warmer air holding the
same absolute water must read a lower relative humidity. Either one alone proves much less — which
is why `verdict` requires both directions before it will say `pass`.

### `scan`

```json
{"id":1,"cmd":"scan","i2c_hz":50000}
{"id":1,"type":"resp","proto":1,"status":"ok",
 "found":[68],"found_hex":["0x44"],"n_found":1,"i2c_hz":50000,"addr":68,"addr_present":true}
```

`i2c_hz` (10000 – 400000) is a **one-shot** override for that scan only; the session speed is
restored afterwards. `found_hex` is redundant with `found` and is there anyway: this reply is read
by a person in a serial monitor as often as by a program, and every SHT31 datasheet, silkscreen and
forum post says `0x44` while JSON says `68`.

`"sweep":true` instead probes the whole speed ladder — see
[Proving it is the edges](#proving-it-is-the-edges-and-not-the-wiring):

```json
{"id":1,"cmd":"scan","sweep":true}
{"id":1,"type":"resp","proto":1,"status":"ok","i2c_hz":100000,"any_found":true,
 "fastest_ok_hz":20000,"slowest_ok_hz":10000,
 "sweep":[{"i2c_hz":400000,"n_found":0,"found":[]},
          {"i2c_hz":200000,"n_found":0,"found":[]},
          {"i2c_hz":100000,"n_found":0,"found":[]},
          {"i2c_hz":50000,"n_found":0,"found":[]},
          {"i2c_hz":20000,"n_found":1,"found":[68]},
          {"i2c_hz":10000,"n_found":1,"found":[68]}]}
```

That example is the signature of pull-ups too weak for the cable: the sensor is fine, the edges are
too slow above 20 kHz.

### `bus`

```json
{"id":2,"cmd":"bus","i2c_hz":10000}
{"id":2,"type":"resp","proto":1,"status":"ok","i2c_hz":10000,"i2c_hz_default":100000}
```

2 kHz – 400 kHz, clamped. **Not persisted** — a reset returns to 100 kHz. A slow bus is a
diagnosis, and one that survived a power cycle would quietly become the configuration, hiding the
very fault it was set to reveal.

(2 kHz is the floor the RP2040 can actually produce: below about 1 kHz the SDK's 16-bit bit-period
counters overflow and the clock silently becomes something else.)

### `lines`

```json
{"id":3,"cmd":"lines"}
{"id":3,"type":"resp","proto":1,"status":"ok","n":1,"gap_ms":250,"sda_pin":6,"scl_pin":7,
 "sda_idle":1,"scl_idle":1,"sda_held_low":false,"scl_held_low":false,
 "sda_ext_pullup":false,"scl_ext_pullup":false,
 "sda_rise_us":20.75,"scl_rise_us":19.94,"sda_rise_free_us":null,"scl_rise_free_us":null,
 "sda_rc_ns":17291,"scl_rc_ns":16616,"sda_est_pf":265,"scl_est_pf":255,
 "int_pullup_ohms":65000,"rise_floor_us":0.2,"rise_timeout_us":5000,"free_timeout_us":30}
```

**The command that tells "nothing is connected to this pin" from "something is connected and not
answering."** It takes the pins back from the I²C block and probes each line **twice**:

1. **with the internal pull-up engaged** — the rise is `1.2 × (R_int ∥ R_ext) × C`
2. **with no pull at all** — a line with nothing on it stays down; a line that still snaps high has
   something else holding it up, and that is reported as `ext_pullup`

The second pass exists because the first cannot tell resistance from capacitance. Without it, a bus
with strong pull-up resistors looks identical to a pin with nothing attached — an ambiguity that
made a real field reading uninterpretable until the pass was added.

**Read `ext_pullup` first** — it decides how the rest of the numbers mean anything at all.

### If `ext_pullup: false` — the internal 65 kΩ is the only pull-up

`rise_us` divides straight into a capacitance, and `est_pf` is populated:

| `rise_us` | `est_pf` | Reading |
|---|---|---|
| **≲ 0.2 µs** (`rise_floor_us`) | ~0 pF | **nothing at all on that pin** — bare pad |
| 1 – 3 µs | 10 – 40 pF | a stub: a jumper, a track, a wire end not reaching the sensor |
| 5 – 15 µs | 60 – 200 pF | short leads, or a metre or so of cable |
| 15 – 40 µs | 200 – 500 pF | a few metres of cable — matches a 3 m shielded probe |
| > 100 µs | > 1 nF | far more than a sensor cable: something else is loaded onto the line |
| `null`, with `held_low: true` | `null` | **the line never rose** — clamped. Shorted to GND, or a slave holding the bus |

### If `ext_pullup: true` — something else holds the line up

A pull-up resistor is fitted, or a device is driving the line. `rise_free_us` (measured with **no**
internal pull) is then the honest figure, and `est_pf` is `null` unless you say what resistor you
fitted:

```json
{"id":4,"cmd":"lines","pullup_ohms":2200}
```

**This matters more than it looks.** With a 2.2 kΩ pull-up the same capacitance rises about **thirty
times faster** than on the internal 65 kΩ, so a heavily pulled-up bus carrying 3 m of cable reads
*exactly like a bare pin* if you compute against 65 kΩ. `sda_rc_ns` / `scl_rc_ns` are always
reported because the **R×C product is what was actually measured** — and it is also the number that
decides whether the bus works, since I²C wants the rise inside 1 µs at 100 kHz whatever mixture of
resistance and capacitance produces it.

### Both cases

`sda_idle` / `scl_idle` are the resting levels: both should be `1`. A `0` means that line is being
held low, which `scan` cannot distinguish from an empty bus.

The two lines should read **close to each other** — they run in the same jacket, so a large
asymmetry means one of them is not connected the way the other is. An asymmetry that *comes and
goes* is an intermittent contact; that is what the repeat mode below is for.

Each rise is timed by `micros()`, whose 1 µs resolution is coarser than a bare pad's actual rise, so
the figure is **averaged over 64 releases** to recover the fraction. Treat `est_pf` as an order of
magnitude rather than a value: the internal pull-up is specified only to ±25% (50–80 kΩ) and the
input is a Schmitt trigger.

> **`rise_floor_us` was wrong at first, and the field data is what corrected it.** It was set to
> 2.0 µs on the assumption that `pinMode`/`digitalRead` cost about that much between releasing a
> line and observing it — and the first readings, a rock-steady 2.00 µs on both lines, looked like
> confirmation. A later run on the same firmware read **0.0625–0.1875 µs**, which is impossible if
> the code path alone cost 2 µs. So the overhead is under a tenth of a microsecond, and that steady
> 2.00 µs was a real measurement of ~25 pF — a bare pad plus a stub of wire. A floor that is assumed
> rather than measured gets read back as confirmation of itself.

#### ⚠️ On a shielded cable, ground the shield before believing the number

Capacitance is always **to something**. A conductor in a shielded cable has a few hundred pF to the
shield — but if the shield is floating, and the other conductors are floating too, then the whole
cable is an isolated island and its only path back to the board's ground is its own stray
capacitance to the room: on the order of 10–20 pF.

So **an intact 3 m conductor in a fully floating cable reads about 25 pF — the same as a 10 cm
jumper.** Two completely different physical situations produce the same number, and the low reading
looks like "the cable is not connected" when the cable is connected perfectly well.

Before using `lines` to judge a cable, give it a ground reference: **connect the shield (and/or the
cable's GND conductor) to the board's `GND`**. Each remaining conductor then measures its real
capacitance to a real ground, and the reading separates cleanly:

| With the shield grounded | Reading |
|---|---|
| ≲ 0.2 µs | nothing on the pin at all |
| ~2 µs (≈25 pF) | a jumper or a stub — **you are not reaching the far conductor** |
| 15–40 µs (≈200–500 pF) | a few metres of conductor really is attached |

### `lines` as a cable tester

That last table is a **continuity test that needs no multimeter and no far-end access**, which is
exactly what a potted probe on a long lead denies you. Capacitance scales with length, so the
reading says whether the pin is touching 3 m of copper or 10 cm of jumper — and it works on a single
conductor with nothing connected at its far end, which an ohmmeter cannot do.

1. Both black conductors → `GND` (safe whichever is the shield and whichever is the real GND, and
   it is what gives the measurement its reference).
2. One remaining conductor at a time → `D5`, nothing else on the bus.
3. `{"cmd":"lines"}`, read `scl_rise_us`.

An intact 3 m conductor **must** read well over 10 µs. If it reads ~2 µs with the shield grounded,
the conductor is not electrically reaching the pin: a bad joint, a wire not gripping, or a break in
the cable.

> `held_low` is a separate field rather than a magic value of `rise_us`, because "never rose" and
> "rose too fast to time" are opposite findings — and a shared sentinel of `0` made them one. That
> was a real bug in the first version of this command, caught in the field: a bare SCL pin reported
> `0` and read as a short. `rise_us` is now `null` when the line is clamped, and `idle` corroborates
> it — a clamped line idles at `0`, a bare one at `1`.

#### Watch mode — finding an intermittent contact

`n` (1 – 200) and `gap_ms` (50 – 2000) repeat the probe while you move a wire. **Only the changes
are printed**, so a wire nobody is touching produces silence and the one line that appears when a
contact makes or breaks is unmissable:

```json
{"id":4,"cmd":"lines","n":40,"gap_ms":250}
{"type":"event","proto":1,"role":"sht31","ev":"lines","seq":3,"i":0,"n":40,
 "sda":"stub","scl":"stub","sda_us":2.06,"scl_us":1.94}
{"type":"event","proto":1,"role":"sht31","ev":"lines","seq":4,"i":7,"n":40,
 "sda":"stub","scl":"cable","sda_us":2.06,"scl_us":21.4}     ← contact made on SCL
{"type":"event","proto":1,"role":"sht31","ev":"lines","seq":5,"i":14,"n":40,
 "sda":"stub","scl":"stub","sda_us":2.06,"scl_us":2.13}      ← and lost again
```

Each line carries a **word per line** rather than the full field set, because this mode is watched
by a person, not parsed: `none` (bare pad), `stub` (a jumper or a wire end that stops short),
`cable` (metres of conductor really attached), `low` (clamped), `pulled` (an external pull-up is
present, so the rise measures R×C and the classification cannot be applied — read `rc_ns` instead).
The constants that do not change between rounds — pins, timeouts, the floor, the pull-up value —
appear once on the final response, where they can actually be read.

A round is printed when a state word changes, when the rise moves by more than 30%, or every 16th
round as a keepalive. `"all":true` prints every round instead, for a recording.

That is how a bad crimp or a breadboard hole that is not gripping is found, and it is the one fault
no single-shot measurement can catch.

#### The finger test

A human body is 100–200 pF to ground — far more than a jumper, comparable to metres of cable. So
**pinching a bare conductor between two fingers is a continuity test with no equipment at all**:

1. Touch the board's `GND` first, to discharge.
2. Start `{"cmd":"lines","n":40,"gap_ms":250}`.
3. Pinch each conductor in turn while it runs.

A conductor that is electrically connected to the pin makes the reading jump the moment you touch
it. One that does nothing is not connected to that pin, whatever it looks like. It tests the exact
link a potted probe denies you access to: the joint between the cable and the board.

### `status`

```json
{"id":2,"type":"resp","proto":1,"status":"ok","fw":"1.0.0","role":"sht31","uptime_ms":12345,
 "addr":68,"sda_pin":6,"scl_pin":7,"i2c_hz":100000,"i2c_hz_default":100000,
 "heater":false,"stream":false,"period_ms":1000,"health":"ok",
 "sensor_present":true,"reg":32768,"heater_reg":false,"reset_detected":true,
 "last_cmd_failed":false,"last_crc_failed":false,"alert_pending":true,
 "max_n":10,"n_default":3,"gap_default_ms":100,"heater_max_ms":30000,"line_max":192}
```

The status register is read **live**, not cached — "is the board up?" and "is the sensor there?"
are different questions and this answers both in one line.

`heater_reg` is the *sensor's own* account of the heater, reported next to the firmware's
`heater` on purpose: **the two disagreeing means a command did not land.** `reset_detected` set
means the sensor has been power-cycled or soft-reset since the bit was last cleared — expected
right after boot, and a surprise later means the probe lost power.

`{"cmd":"status","clear":true}` wipes the sensor's status register **after** reading it out, so the
reply still describes the state that was cleared. Those bits are sticky from power-up, which makes
them useless as they stand: `alert_pending` and `reset_detected` are both set on any fresh probe and
stay set forever. Clear them once and they become event flags — a `reset_detected` that comes back
true afterwards means the probe *actually* lost power, which is exactly what is worth knowing about
a sensor at the bottom of a well. `cleared` reports whether the clear command was acknowledged.

`alert_pending` refers to the SHT31's `ALERT` pin and its threshold registers, which this firmware
does not use. It is set on a fresh device and can be ignored.

### `reset`

Soft-reset, then read the status register back. The reset-detected bit coming back set is
confirmation the reset actually reached the device rather than being swallowed by a dead bus.
The firmware also clears its own idea of the heater, since the reset clears it in the sensor.

## What counts as a valid sample

Every SHT3x word ships with a **CRC-8 the sensor computed itself** (polynomial 0x31, init 0xFF),
and this firmware checks it on every word. That is what turns "the number looks plausible" into
"the sensor and I agree on every bit of this word" — the actual question being asked of a newly
bought module.

Each sample lands in exactly one of five buckets, and
`n == n_valid + n_crc + n_nack + n_bus + n_short` always holds:

| Field | Char | Meaning | Where to look |
|---|---|---|---|
| `n_valid` | `V` | 6 bytes, both CRCs good | — |
| `n_crc` | `C` | 6 bytes arrived, a CRC failed | the sensor **is there**; the *signal* is corrupt — leads too long, pull-ups missing, bus too fast |
| `n_nack` | `N` | nobody acknowledged the address | sensor absent, unpowered, on the other address, or SDA/SCL swapped |
| `n_bus` | `B` | the transaction timed out — a line is held low | a wire half in, or a slave stuck mid-byte |
| `n_short` | `S` | addressed fine, fewer than 6 bytes back | still busy, or the read was cut off |

`sample_status` gives one character per sample, in order. The *pattern* is diagnostic: scattered
`C`s read as a marginal signal, a solid run of `N`s as nothing there at all.

**There is no plausibility window here, and that is deliberate.** `puit` has `min_cm`/`max_cm`
because an ultrasonic module can return a physically plausible, internally consistent, completely
wrong number — an echo off the mounting bracket looks exactly like an echo off the water. That
failure has no counterpart here: the SHT31's conversion is defined over the whole 16-bit range,
so every possible raw word maps into −45…130 °C and 0…100 %RH, and a window over the *output*
would reject nothing a working sensor can produce. The failures a window would be reaching for —
a bus stuck low reading `0x0000`, stuck high reading `0xFFFF`, a bit flipped by a long lead — are
all caught upstream by the CRC, which checks the **word** rather than guessing about the value.
Adding a window would only add a way to reject good data.

In `temps_c` / `rh_pcts` / `t_raws` / `rh_raws`, `null` marks a sample that produced no word at
all. **A sample whose CRC failed keeps its numbers** — raw truth is never discarded, only
excluded from the statistics — so anything aggregating these **must filter on `sample_status`**,
exactly as `puit`'s `samples[]` must be filtered on `ping_status`. A corrupt word that is
*almost* right tells you the bus is marginal; a discarded one tells you nothing.

## Error codes

```json
{"id":3,"type":"resp","proto":1,"status":"error","code":"sensor_fault", …counts, params, detail…}
```

| `code` | Meaning | `id` |
|---|---|---|
| `bad_request` | the line is not parseable JSON | `null` |
| `line_too_long` | request exceeded `line_max` (192) bytes | `null` |
| `bad_id` | `id` missing or not an integer | `null` |
| `unknown_cmd` | parsed fine, but `cmd` is unrecognised or absent | echoed |
| `bad_param` | a parameter has the wrong type, or `addr`/`rep` is not one of the accepted values; carries `field` | echoed |
| `sensor_fault` | no valid samples, predominantly **address not acknowledged** — absent, unpowered, wrong address, lines swapped | echoed |
| `bus_error` | no valid samples, predominantly **transaction timeouts** — a line is being held low | echoed |
| `crc_error` | no valid samples, predominantly **CRC failures** — the sensor is there, the signal is not clean | echoed |
| `short_read` | no valid samples, predominantly **short reads** | echoed |

Ties break toward the more fundamental fault: a bus held low or an address that never answers
explains a CRC failure, but not the other way round.

Measurement errors carry the counts, the effective parameters and the per-sample detail, so a
single logged line explains itself.

`sensor_fault` and `bus_error` are physical: retrying cannot help. `crc_error` is worth one
retry and then a look at the leads and pull-ups.

## Build & flash

```
pio run -e sht31 -t upload
```

Same chip, core and bootloader story as the pump board, so if you have ever built `-e pump` the
platform and toolchain are already downloaded and this costs no extra install. See
[README.md § Build & flash](../README.md#build--flash-platformio-in-vscode) for making `pio`
work in a plain terminal.

> ⚠️ **With the pump board plugged in at the same time**, `tools/flash_uf2.py` sees two RP2040
> serial ports, cannot tell which to reset, and gives up — deliberately, since guessing would
> reboot the wrong board. Name the port:
> ```
> pio run -e sht31 -t upload --upload-port COM7
> ```
> `pio device list` says which is which: this firmware reports itself as **PiJardin SHT31**.
> With only one RP2040 attached, leave the flag off and it auto-detects as before.

The boot banner's `"role":"sht31"` is what the firmware on the board actually *is*; the USB
product string is only what it *claims* to be. They agree unless you crossed a flash.

## Testing after a flash

1. **Banner** → `{"type":"ready","proto":1,"fw":"1.0.0","role":"sht31"}`. On native USB you will
   probably miss it — the LED is the boot indicator that survives.
2. `{"id":1,"cmd":"scan"}` → `"found":[68]`, `"addr_present":true`. Then
   `{"id":1,"cmd":"scan","sweep":true}` → found at **every** rung of the ladder. Found only at the
   slow end means the bus works but has no margin: the pull-ups are too weak for the cable and it
   will start failing as the cable ages or the run gets longer.
3. `{"id":2,"cmd":"status"}` → `"sensor_present":true`, `"reset_detected":true` (fresh boot),
   `"health":"ok"`.
4. `{"id":3,"cmd":"read_env"}` → `"sample_status":"VVV"`, plausible room values,
   `temp_spread` a few hundredths of a degree. **Check `temp_c ≈ -45 + 175 × t_raw / 65535` by
   hand** — that is the regression guard for the conversion, and the reason `t_raw` is on the
   reply at all.
5. **Parameters:**
   - `{"id":4,"cmd":"read_env","n":999}` → clamped, echoes `n:10` with a 10-character
     `sample_status`.
   - `{"id":5,"cmd":"read_env","n":"three"}` → `bad_param`, `"field":"n"`.
   - `{"id":6,"cmd":"read_env","rep":"turbo"}` → `bad_param`, `"field":"rep"`.
   - `{"id":7,"cmd":"read_env","addr":80}` → `bad_param`, `"field":"addr"` — **not** clamped to
     68.
   - `{"id":8,"cmd":"read_env","addr":69}` → `sensor_fault` on a module strapped to 0x44. This
     is the check that the address is really being honoured rather than ignored.
6. **The world moves the numbers.** `{"id":9,"cmd":"stream","on":true,"fmt":"text"}`, then breathe
   on the probe: RH jumps 20–30 points in a second or two, then decays. The text mode is the one to
   use here — a trend is what you are looking at. It stops itself after 2 minutes, or
   `{"id":10,"cmd":"stream","on":false}`.
7. **`selftest`** — the proof the sensor is real, run and judged by the board.
   `{"id":11,"cmd":"selftest"}` → `"verdict":"pass"`, `d_temp_c` positive, `d_rh_pct` negative, and
   `snr` in the tens. Expect a modest `d_temp_c` on a potted probe and read `snr` instead of the
   absolute rise — see [`selftest`](#selftest). Then confirm `{"id":12,"cmd":"status"}` shows
   `"heater":false` **and** `"heater_reg":false`: the two disagreeing means the off command did not
   land.
8. **`lines`** — the electrical picture, and a check on the diagnostic itself.
   `{"id":13,"cmd":"lines"}` → `idle` 1 on both, `held_low` false on both, and the two lines within
   a few µs of each other. With this probe's own pull-ups present, expect
   `"ext_pullup":true` and `est_pf` `null` unless you pass `pullup_ohms`. A bare board with nothing
   attached reads near `rise_floor_us` on both lines — worth seeing once, so the connected figure
   means something by comparison.
9. **`bus`** — `{"id":14,"cmd":"bus","i2c_hz":10000}` then a `read_env` still works; reset and
   confirm `status` reports `i2c_hz` back at 100000, since the speed is deliberately not persisted.
10. **Fault detection** — the failures this rig exists to separate:
   - **Unplug SDA (D4)** → `sensor_fault` with `n_nack: n` and `sample_status` all `N`. LED goes
     red within 5 s without any command being sent.
   - **Unplug the sensor's 3V3** → the same, which is correct: an unpowered sensor and an absent
     one are the same fault.
   - **Short SDA to GND** → `bus_error` with `n_bus: n` and `sample_status` all `B`. Check that
     the board still answers `{"id":13,"cmd":"status"}` afterwards — that is the whole point of
     the bounded I²C timeout, and a firmware that stops talking here is worse than one that
     reads wrong.
   - Reconnect → the LED returns to green blipping on its own within 5 s, with no command sent.
11. **Errors:** send `hello` → `bad_request` with `"id":null`. Send `{"cmd":"status"}` → `bad_id`.
    Send `{"id":15,"cmd":"nope"}` → `unknown_cmd` with `"id":15`. Send
    `{"id":16,"cmd":"stream","fmt":"csv"}` → `bad_param`, `"field":"fmt"`. Paste a line longer than
    192 characters → `line_too_long`, then confirm the **next** valid request still answers — that
    proves the reader recovers by draining to the newline.
12. **Reader:** two requests back-to-back with no delay → two correlated responses, no lost line.
    An empty line → no reply at all.

> This checklist exercises the **firmware**. It does not check that the temperature is *right* —
> that is a question about the probe and its physical placement, and the procedure for it (settling
> time, the ice point, what accuracy to expect, drying after a wet test) is in
> [docs/sht31-hookup.md § Verifying the temperature is accurate](sht31-hookup.md#4-verifying-the-temperature-is-accurate).

## What moves into `puit` later

Only the driver. [README.md § Future: environment sensing](../README.md#future-environment-sensing)
sets the terms and none of them change:

- The well board reads the probe **inline, once per burst** (`n:1`) — not on a timer, and not
  cached. The temperature that matters is the one in the air column at ping time.
- It returns `value` **only** when it has a real reading for every input. A failed or missing
  environment read means omitting `value`, returning `pulse_us` plus whatever it does have, and
  letting the Pi derive the distance from its own last-known-good temperature. **The board must
  never fabricate a distance from a guessed temperature.**
- `pulse_us` is already in `puit`'s contract, so fitting the probe should be a firmware-only,
  **additive** change — no `proto 3`.

What does *not* move: the heater, `scan`, `stream`, the background health poll and the LED
scheme are bench affordances. `puit`'s LED means "a command is being processed" and its contract
is pure request/response; adding events and a second meaning to its LED to keep a diagnostic
convenience would be paying in the deployed firmware for something only ever used on a desk.

The per-sample CRC checking and the `V`/`C`/`N`/`B`/`S` split **do** move, and are the reason
this file exists rather than a call to `Adafruit_SHT31`: that library returns `NAN` for "nothing
on the bus", "the CRC is wrong" and "the read came up short" alike, and at the bottom of a well
those three send you to three different places.
