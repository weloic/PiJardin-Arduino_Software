# PiJardin — Arduino Software

Microcontroller firmware for the **PiJardin** garden monitoring project. This is the companion
repository to the main [PiJardin](https://github.com/wermeill/PiJardin) project: PiJardin
runs the Raspberry Pi side (Python, InfluxDB, Grafana, Telegram bot), and **this repo holds
the firmware** for the boards that do the actual sensing.

The two are kept separate on purpose — different toolchain (C++/Arduino vs Python) and a
different lifecycle (the firmware changes rarely). Firmware is flashed **physically over USB
from a laptop**, never remotely from the Pi.

## Firmwares in this repo

One repo, **one firmware per physical board**, each built from its own PlatformIO environment.
They share the serial framing, but **not the chip** — see the warning below the table.

| Board | Env | MCU | Source | Docs | What it does |
| --- | --- | --- | --- | --- | --- |
| Well level | `puit` | XIAO **SAMD21** | [src/puit/puit_sensor.cpp](src/puit/puit_sensor.cpp) | this file, below | Distance down to the water surface, HC-SR04 ultrasonic |
| Pump state | `pump` | XIAO **RP2040** | [src/pump/pump_sensor.cpp](src/pump/pump_sensor.cpp) | [docs/pump.md](docs/pump.md) | **When** the pump starts and stops, ZMPT101B AC voltage sense |

> ⚠️ **The two boards are different microcontrollers in an identical package.** A XIAO SAMD21 and a
> XIAO RP2040 are the same size, the same pin count and the same colour; only the silkscreen tells
> them apart. They need different platforms, cores and bootloaders, and **an image built for one
> will not run on the other**. Check the board before `-t upload`.

Both speak the **same NDJSON envelope** (`id` / `type` / `proto` / `status` / `code`) over USB
serial, documented once in [Serial protocol contract](#serial-protocol-contract) below. They do
**not** share a command set, and `proto` is numbered **per firmware** — each board's contract starts
at 1 and is versioned independently, so the two are both at `proto 2` by coincidence and mean
entirely different things by it. Always read `role` from the boot banner — **both** firmwares send
it — rather than inferring the board from a `proto` number. `role` is not a literal in either
sketch: it comes from `-DPIJARDIN_ROLE` in that environment's `build_flags`, so it cannot drift from
the environment that built the firmware.

> **Everything from [Serial protocol contract](#serial-protocol-contract) to the end of this file
> describes the well sensor.** The pump board's contract, wiring and — importantly — its required
> **calibration** live in [docs/pump.md](docs/pump.md).

### Where the pump board departs from the shared contract

The envelope is shared; the *interaction model* is not. Two differences a Pi-side reader must
handle, both introduced by pump `proto 2`:

- **The pump board talks without being asked.** It measures continuously and emits an unsolicited
  `{"type":"event", …}` line the moment it decides the pump changed state, plus a heartbeat every
  60 s. Such a line can arrive **at any moment, including between a request and its response** — so
  match responses on `type == "resp"` *and* the echoed `id`, never on "next line in". A Pi that only
  polls the pump board is not just slower, it silently misses every transition; that is exactly what
  the `proto 1 → 2` bump exists to make assertable.
- **The pump board is not stateless.** It latches, debounces and keeps the last 32 transitions in
  RAM, because it is the only side watching continuously. Nothing is persisted to flash and no
  threshold is settable over the wire — the rationale, and why the earlier stateless design was
  right for what it was, is in [docs/pump.md](docs/pump.md#the-principle-that-reversed).

The well sensor (`puit`) remains pure request/response and fully stateless.

### Adding a third board

1. `src/<name>/<name>_sensor.cpp`
2. `[env:<name>]` in [platformio.ini](platformio.ini) with `build_src_filter = -<*> +<<name>/>`
   **and its own `platform` / `board`** — those are deliberately not in the shared `[env]` block,
   because the boards are no longer all one chip and an inherited default would quietly build the
   wrong target.

Sources must be **`.cpp`, not `.ino`**. PlatformIO's sketch conversion only globs the *top level*
of `src/` and ignores `build_src_filter` while doing it, so a `.ino` here is either skipped (in a
subfolder) or concatenated with the others into one translation unit with several `setup()`/`loop()`
pairs. A `.cpp` needs `#include <Arduino.h>` and its own function prototypes — the only two things
the `.ino` step ever added.

## Hardware

Both boards are in the **Seeed Studio XIAO** form factor with native USB, but they are different
chips: the well sensor is a **XIAO SAMD21** (Atmel, Cortex-M0+) and the pump sensor is a **XIAO
RP2040** (Raspberry Pi, dual Cortex-M0+). On both, the status LED lights while a command is being
processed — note that the RP2040's user LEDs are **active low**, which the pump firmware accounts
for.

> ⚠️ **Both are 3.3 V parts and neither is 5 V tolerant.** The SAMD21's ADC reference is
> `VDDANA` = 3.3 V; the RP2040's is 3.3 V likewise. Any sensor module with an analog output must be
> powered from the **3V3** pad, not 5V — see [docs/pump.md](docs/pump.md), where getting this wrong
> destroys the analog input.

**Well sensor (`puit`)** — HC-SR04-style ultrasonic distance sensor, mounted above the well,
measuring the distance down to the water surface.

| Signal        | Pin           |
|---------------|---------------|
| Echo (input)  | `D7`          |
| Trigger (out) | `D8`          |
| Status LED    | `LED_BUILTIN` |

**Pump sensor (`pump`)** — ZMPT101B AC voltage module across the pump's switched feed. Wiring,
including the mistake that silently makes it useless, is in [docs/pump.md](docs/pump.md).

## Serial protocol contract

> ⚠️ **This is a shared contract with PiJardin.** The Pi-side consumer is
> [`sensors/read_puit.py`](https://github.com/wermeill/PiJardin/blob/main/sensors/read_puit.py).
> Changing any command, response field, the boot banner, or `proto` here **will break the Pi**
> unless `read_puit.py` is updated to match. Keep them in sync and bump `proto` on any
> incompatible change.

The link uses **newline-delimited JSON (NDJSON)**: exactly one JSON object per line, `\n`-terminated,
in both directions (UTF-8, no embedded newlines). Every response echoes the request's `id` so
the Pi can correlate replies and ignore stray lines; sensor failures are reported as an explicit
`code` rather than a silent `0`.

- **Transport:** USB CDC serial, **9600 baud** (nominal on native USB — data moves at USB
  speed regardless). Enumerates as `/dev/ttyACM0` on the Pi.
- **Protocol version:** `proto = 2`. Every line the board emits carries `proto`, so the Pi can
  identify what it is talking to from any reply, not just the banner.
- **Boot banner:** on reset the board prints one line and nothing else until commanded:
  ```json
  {"type":"ready","proto":2,"fw":"2.2.0","role":"puit"}
  ```
  The Pi resets the board (DTR toggle) and waits for a line that parses to JSON with
  `type == "ready"` before issuing commands. `role` says which board answered — check it
  alongside `type`, and the Pi never has to care which `/dev/ttyACM*` it opened.
- **Stateless:** the board keeps nothing between requests. Anything a request does not specify
  falls back to the documented default, and every response echoes the values actually used.
  Nothing to re-send after a reset, nothing to drift out of sync.

### Requests (Pi → board)

```json
{"id":<int>,"cmd":"<name>", <optional parameters…>}
```

- `id` — a request counter chosen by the Pi, **required and must be an integer**; echoed back
  verbatim. A missing or mistyped `id` is `bad_id`.
- `cmd` — one of `read_puit`, `sampling`, `status`.

#### Measurement parameters

Optional on `read_puit` and `sampling`; ignored by `status`. Absent — or explicitly `null` — means
"use the default".

| Field        | Type  | Default          | Accepted range | Notes |
|--------------|-------|------------------|----------------|-------|
| `n`          | int   | `10`             | 1 – 25         | pings per burst |
| `timeout_us` | int   | `45000`          | 2000 – 60000   | per-ping echo timeout; also caps reachable distance (~772 cm at the default). Deliberately above the sensor's ~38 ms "nothing found" pulse — see below |
| `ack_timeout_us` | int | `50000`        | 500 – 60000    | how long to wait for echo to **rise** after a trigger before calling the ping a no-response. The fitted module actually takes **~12.3 ms**, so the default is ~4× measured. Sweep this when diagnosing `sensor_fault` — see below |
| `temp_c`     | float | `20.0`           | −20 – 60       | assumed air temperature; drives the µs→cm divisor |
| `min_cm`     | float | `5.0`            | 0 – 1030       | lower edge of the plausibility window |
| `max_cm`     | float | `500.0`          | 0 – 1030       | upper edge; additionally capped to what `timeout_us` can reach |
| `min_valid`  | int   | ceil(50% of `n`) | 0 – `n`        | valid pings required for a reading; `0` disables the gate |

Two distinct behaviours, deliberately:

- **Wrong type** → `bad_param`, with `field` naming the offender. `{"n":"ten"}` is an error.
- **Out of range** → **silently clamped**. `{"n":999}` measures 25 pings. This is safe because
  every response echoes the *effective* value, so the clamp is visible rather than hidden.

`max_cm` is capped to `timeout_us / divisor` before use, so raising `max_cm` without also raising
`timeout_us` is a visible no-op rather than a silent one. If the window ends up empty —
`min_cm >= max_cm`, whether because the request inverted the bounds or because the timeout cannot
reach `min_cm` — that is `bad_param` with `field: "min_cm"`, since it would reject every ping.

### What counts as a valid ping

**This changed in `proto 2` and it changes what the Grafana data means.** In `proto 1`, `n_valid`
counted any ping that returned an echo, with no check on the value. That made it blind to the most
dangerous failure mode: a 200 µs echo (≈3.4 cm) is inside the HC-SR04's blind zone, and echoes off
the shaft wall or the mounting bracket are *consistent* — so a misaimed sensor reported
`n_valid: 10`, a tiny spread and `status: "ok"`, i.e. maximum apparent confidence on a completely
wrong number. A sensor that has gone **silent** is easy to notice; one that is confidently **wrong**
quietly poisons the time series.

In `proto 2` a ping is valid only if the sensor answered, an echo came back, **and** the distance it
implies is plausible. Every ping lands in exactly one of four buckets:

| Field           | Meaning |
|-----------------|---------|
| `n`             | pings fired |
| `n_no_response` | the sensor never reacted to the trigger — **hardware fault** |
| `n_timeout`     | the sensor reacted normally, but no echo returned in time |
| `n_rejected`    | an echo returned, but outside `[min_cm, max_cm]` |
| `n_valid`       | echo inside the window — the **only** pings feeding `value`/`min`/`max`/`spread` |

`n == n_valid + n_timeout + n_rejected + n_no_response` always holds, and all five are reported on
**every** measurement response, success or failure. This buys diagnoses that were impossible before:
`n_rejected: 10` means *the sensor is alive and aimed at the wrong thing*, while
`n_no_response: 10` means *the sensor is not talking to us at all*.

Two further fields refine that picture without disturbing the invariant above:

| Field        | Meaning |
|--------------|---------|
| `n_stuck`    | **subset of `n_no_response`** (never a fifth bucket): echo was still HIGH when the ping began, so the trigger was never fired at all. Points at the *line* — held high, miswired, damaged input — rather than a module ignoring a trigger it received |
| `ack_max_us` | the largest trigger→rise latency actually **measured** across the burst; `0` when nothing answered |

`ack_max_us` exists because `ack_timeout_us` is the one parameter with no physical ground truth
behind it, and getting it wrong is silent: a window set below the module's real reaction time turns
a perfectly healthy sensor into `sensor_fault` on every ping. Comparing a working burst's
`ack_max_us` against the window is what makes that falsifiable rather than a guess. See
[When `sensor_fault` might not be the sensor](#when-sensor_fault-might-not-be-the-sensor).

The default window (5–500 cm) rejects only the physically impossible — the blind zone below, the
sensor's reach above — without assuming anything about the well. Narrow it via `min_cm`/`max_cm`
once the real geometry is trusted.

#### Why `n_no_response` is separate from `n_timeout`

This is the distinction `proto 1` could not make, and it is the difference between "the sensor is
broken" and "the sensor sees nothing."

The board pulses **trigger** (D8); the sensor chirps and raises **echo** (D7), dropping it when the
chirp returns. The width of that HIGH period is the distance. Three outcomes:

```
normal, water at 123 cm            sensor fine, nothing in range        sensor dead / wire off
trigger  __|‾|_________             trigger  __|‾|_________              trigger  __|‾|_________
echo     ____|‾‾‾|_____             echo     ____|‾‾‾‾‾‾‾‾‾|_            echo     ______________
             └7.2ms┘                             └─ 38 ms ─┘                    (never rises)
              → 123 cm                            → 652 cm                       → no response
```

`pulseIn()` reports **0** for both of the right-hand cases, which is why `proto 1` conflated them.
`proto 2` handles it two ways at once:

- The firmware waits `ack_timeout_us` (default **50 ms**) for echo to **rise** after each trigger.
  The sensor raises it whether or not it finds anything, so no rise at all is a hardware fault →
  `n_no_response`. The default is far above the datasheet's "few hundred µs" because the fitted
  module measurably takes **12.3 ms** and because the cost of being wrong is asymmetric — see
  [When `sensor_fault` might not be the sensor](#when-sensor_fault-might-not-be-the-sensor).
- `timeout_us` defaults to **45 ms**, past the sensor's ~38 ms "nothing found" pulse. So a
  live-but-blind sensor produces a real ~652 cm reading (→ `n_rejected`, since it exceeds
  `max_cm`) rather than a silent zero.

Belt and braces: the first works on any module; the second means even a module with unusual timing
still looks different from a dead one.

#### When `sensor_fault` might not be the sensor

The rise detection above rests on one assumption — *the module raises echo quickly* — and that
assumption is the only part of the measurement path with no physical ground truth behind it. Every
other bound is derived from something real: `timeout_us` from the speed of sound, `max_cm` from the
well's geometry, `min_cm` from the published blind zone. The rise deadline comes from a datasheet
figure for a part number that clone modules print on the silkscreen without honouring.

That matters because the failure is **silent and total**. A deadline set below the module's real
reaction time does not degrade the reading — it converts a perfectly healthy sensor into
`n_no_response: n`, on every ping, forever, with a `code` that says *"unpowered, dead, or a wire
off"* and a decision table that says *do not retry, alert a human*. The firmware would be lying with
complete confidence, and the alert would send someone to the well to look at hardware that is fine.
This is the same class of bug as the `proto 1` blind-zone problem the plausibility window was added
to fix: not a wrong number, but **unearned certainty**.

`proto 1` never exposed it, because it used `pulseIn()` with the Arduino default timeout — **one
full second** to wait for the rise. That absorbed any latency any module could plausibly have, and
so never revealed what the real figure was.

**This is not hypothetical — it was measured, and the assumption was wrong.** On the module fitted
at the well (`n=25`, 20 °C, USB power):

| | µs |
|---|---|
| mean | 12286.4 |
| median | 12286 |
| min / max | 12283 / 12289 |
| std dev | 1.57 |

**~12.3 ms**, against a datasheet figure of a few hundred µs — and against the **2 ms** this
firmware used before anyone checked. At 2 ms, *every ping on this healthy sensor reports a hardware
fault.* The clone honours the pinout and the pulse encoding but not the timing, exactly as the
paragraph above feared.

Note the spread: 1.57 µs std dev, 0.013 %, and identical on the first ping of a burst. That is a
fixed internal schedule, not a variable reaction time — which is what makes a bound around it
meaningful rather than a coin flip.

Three things follow, and together they make the assumption falsifiable:

- **`ack_max_us` is measured and reported**, on every measurement reply. On a burst that works, it
  is the module's true reaction time; compare it against the echoed `ack_timeout_us` to see how much
  margin you actually have.
- **`ack_timeout_us` is a request parameter**, so the window can be swept from the Pi across a range
  without reflashing. If `n_no_response: n` at 2 ms becomes `n_valid: n` at 50 ms, the window was
  the fault and the sensor was never broken.
- **The default is 50 ms** — ~4× the measured maximum. The only thing traded away by erring long is
  burst duration, and that trade is **free here**: a ping that answers and then finds nothing already
  costs `12.3 + timeout_us + 3` = 60.3 ms, so an all-timeout `n=25` burst takes **1.51 s** while an
  all-dead one at this window takes 1.33 s. The window is not what bounds the worst case, so buying
  margin with it costs nothing. Erring short costs a false hardware alert.

⚠️ **Size the Pi's serial read timeout against 1.51 s, not against a good reading.** A healthy burst
returns in ~0.5 s, but a fully blind one takes three times that. Too short a timeout turns a
carefully diagnosed `sensor_fault` into a generic comms error at precisely the moment the diagnosis
matters.

Diagnostic recipe when a `sensor_fault` looks suspicious:

```json
{"id":1,"cmd":"sampling","n":5,"ack_timeout_us":60000}
```

| Result | Reading |
|--------|---------|
| `ok`, and `ack_max_us` well above the old window | the window was too tight — set the default from `ack_max_us`. This is what happened at 2 ms |
| `sensor_fault`, `ping_status` all `N` | triggers are going out and nothing answers — genuine fault, or the trigger line |
| `sensor_fault`, `ping_status` all `S` | echo never went idle, so **no trigger was ever fired** — the echo line is held high: miswiring, a level-shifter leg, or a damaged input |

The `N` vs `S` split is the reason `n_stuck` exists. They are both hardware faults, but they send
you to opposite ends of the wiring.

### Responses (board → Pi)

All responses carry `"type":"resp"`, `"proto":2`, the echoed `id`, and a `status` of `"ok"` or
`"error"`.

`read_puit` — median distance in cm over the valid pings:

```json
{"id":42,"type":"resp","proto":2,"status":"ok","value":123.4,"unit":"cm","pulse_us":7186,
 "min":122.8,"max":124.1,"spread":1.3,
 "n":10,"n_valid":9,"n_timeout":1,"n_rejected":0,"n_no_response":0,"n_stuck":0,"ack_max_us":12289,
 "temp_c":20,"min_cm":5,"max_cm":500,"timeout_us":45000,"ack_timeout_us":50000,"min_valid":5}
```

`sampling` — the same, plus per-ping detail for diagnostics:

```json
{"id":43,"type":"resp","proto":2,"status":"ok","value":123.4,"unit":"cm",
 "min":122.8,"max":124.1,"spread":1.3,
 "n":10,"n_valid":9,"n_timeout":1,"n_rejected":0,"n_no_response":0,"n_stuck":0,"ack_max_us":12289,
 "temp_c":20,"min_cm":5,"max_cm":500,"timeout_us":45000,"ack_timeout_us":50000,"min_valid":5,
 "samples":[123.4,null,124.0,…],"pulse_us":[7186,null,7221,…],
 "ack_us":[12286,12289,12285,…],"ping_status":"VTVVVVVVVV"}
```

In `samples` and `pulse_us`, `null` marks a ping that produced no pulse width at all — either no
echo or no response. A ping that echoed but fell outside the window keeps its cm and µs values: raw
truth is never discarded, only excluded from the statistics.

`ack_us` is index-aligned with them but follows a **different** rule: it holds the trigger→rise
latency for every ping the sensor *engaged with*, which includes the `T`imeout pings that produce no
width. So a `null` in `pulse_us` paired with a number in `ack_us` reads as "the module answered,
found nothing" — the clearest single indication that a sensor returning no data is nonetheless
alive. `ack_us` is `null` only for `N` and `S`.

`ping_status` gives one character per ping, in order — **V**alid, **R**ejected, **T**imeout,
**N**o response, **S**tuck. It disambiguates the `null` cases, and the pattern itself is
informative: scattered losses (`VVTVVVTVVV`) suggest surface noise, while clustered ones
(`VVVVVNNNNN`) suggest intermittent contact or a sensor dropping out mid-burst.

`N` and `S` are **both** counted by `n_no_response`, so `count('N') + count('S') == n_no_response`
and `count('S') == n_stuck`. The character split is a refinement of the count, not a fifth bucket.

`status` — identity plus limits, so the Pi can discover them instead of keeping a second copy of
these constants:

```json
{"id":44,"type":"resp","proto":2,"status":"ok","fw":"2.2.0","role":"puit","uptime_ms":12345,
 "max_n":25,"n_default":10,"timeout_default_us":45000,"ack_timeout_default_us":50000,
 "min_cm_default":5,"max_cm_default":500,"line_max":192}
```

#### Error codes

```json
{"id":42,"type":"resp","proto":2,"status":"error","code":"out_of_range", …}
```

| `code` | Meaning | `id` |
|--------|---------|------|
| `bad_request` | the line is not parseable JSON | `null` |
| `line_too_long` | request exceeded `line_max` (192) bytes | `null` |
| `bad_id` | `id` missing or not an integer | `null` |
| `unknown_cmd` | parsed fine, but `cmd` is unrecognised or absent | echoed |
| `bad_param` | a parameter has the wrong type, or the window is empty; carries `field` | echoed |
| `sensor_fault` | no valid pings, predominantly **no response to the trigger** — unpowered, dead, or a wire off | echoed |
| `echo_timeout` | no valid pings, predominantly **no echo** — sensor responds but finds nothing | echoed |
| `out_of_range` | no valid pings, predominantly **outside the window** — sensor alive, misaimed | echoed |
| `insufficient_samples` | `0 < n_valid < min_valid` — some echoes, too few to trust | echoed |

The three `"id":null` cases are told apart by their `code`, never by the `id`. The four measurement
errors all carry the counts and the effective parameters, so a single logged line explains itself.

### Telling outcomes apart — how to log this properly

Everything needed to classify a reply is in the reply itself. There are **three levels of detail**,
and which one you need depends on what you are logging.

**Level 1 — `status` + `code`: the verdict on the whole burst.** This answers "do I have a usable
reading, and if not, whose fault is it." Nine codes, listed above. Note what this level does *not*
tell you: a `status:"ok"` reply may still have had failed pings.

**Level 2 — the four counts: which physical outcome dominated.** Present on **every** measurement
reply, success *and* failure. This is the level to record in InfluxDB, because it distinguishes the
failure *modes* rather than just success/failure:

| Count | A healthy sensor | Non-zero means |
|-------|------------------|----------------|
| `n_valid` | `== n` | — |
| `n_timeout` | occasionally 1–2 | sensor answered, no echo: ripples, oblique surface, absorbent water |
| `n_rejected` | `0` | echo outside the window: misaim, bracket, blind zone, or the ~652 cm "nothing found" artefact |
| `n_no_response` | **always 0** | sensor ignored the trigger: **hardware** — power, wiring, dying module |
| `n_stuck` | **always 0** | subset of the above: echo line held HIGH, trigger never fired — wiring, not the module |
| `ack_max_us` | stable, a few hundred µs | not a fault signal; **trend it** — a reaction time creeping toward `ack_timeout_us` is a module going soft, and the point at which healthy pings start being reported as faults |

The asymmetry matters when choosing log levels: `n_timeout` has benign causes, **`n_no_response`
does not**. Any non-zero `n_no_response`, even on an otherwise perfect `ok` reading, is worth a
warning — it is the earliest signal of a failing connection, and no `min_valid` threshold will
surface it because the burst still succeeds.

Before treating a *fully* failed burst as a dead sensor, though, check `ack_max_us` against the
echoed `ack_timeout_us`: see
[When `sensor_fault` might not be the sensor](#when-sensor_fault-might-not-be-the-sensor).

**Level 3 — `ping_status`: which ping, in what pattern.** One character per ping, `sampling` only:

| Char | Outcome | `samples[i]` / `pulse_us[i]` | `ack_us[i]` |
|------|---------|------------------------------|-------------|
| `V` | valid | the values | latency |
| `R` | rejected — echoed, outside the window | the values (kept deliberately) | latency |
| `T` | timeout — answered, no echo | `null` | latency |
| `N` | no response — trigger fired, ignored | `null` | `null` |
| `S` | stuck — echo never idle, **trigger never fired** | `null` | `null` |

`V` vs `R` can also be derived by comparing `samples[i]` against the echoed `min_cm`/`max_cm`. **`T`
vs `N` cannot** from `samples`/`pulse_us` alone — both are `null` there — though `ack_us` now
separates them too, since a `T` ping did engage the sensor and an `N` ping did not. `ping_status`
remains the direct answer, and the only place `S` is visible per ping.

The *pattern* is itself diagnostic: `VVTVVVTVVV` (scattered) reads as surface noise, while
`VVVVVNNNNN` (clustered) reads as a sensor dropping out mid-burst — an intermittent connection.

#### Decision table

| Reply | Additional condition | Interpretation | Log level | Action |
|-------|---------------------|----------------|-----------|--------|
| `ok` | `n_valid == n` | clean measurement | debug | store |
| `ok` | `n_valid < n`, `n_no_response == 0` | usable; some pings lost to the environment | info | store, record counts |
| `ok` | `n_no_response > 0` | usable, but the sensor missed a trigger | **warning** | store; alert if it recurs |
| `error` | `insufficient_samples` | too few valid pings to trust | warning | **retry** |
| `error` | `echo_timeout` | sensor answers, finds nothing | warning | **retry** |
| `error` | `out_of_range` | sensor alive but misaimed / obstructed | **error** | **alert, do not retry** |
| `error` | `sensor_fault`, `n_stuck == 0` | triggers fired, sensor never answered — power, module, trigger line | **critical** | **alert, do not retry** |
| `error` | `sensor_fault`, `n_stuck > 0` | echo line held HIGH, triggers never fired — **echo-side** wiring | **critical** | **alert, do not retry**; points at different wiring than the row above |
| `error` | `bad_id`, `bad_param`, `bad_request`, `line_too_long`, `unknown_cmd` | bug in the Pi-side request | **error** | fix the code; retrying cannot help |

Retrying only ever helps the two transient rows. `out_of_range` and `sensor_fault` are physical
problems: something has to be looked at or moved. Include `n_stuck` and `ack_max_us` in the alert —
they are what turn "go look at the well" into "go look at the echo wire".

> **Known limitation affecting `n_no_response`.** The firmware currently waits only `delay(3)`
> between pings, while the HC-SR04 datasheet recommends a ≥60 ms measurement cycle. A trigger
> arriving while the module is still busy may be ignored, producing an `N` that is a *timing*
> artefact rather than a wiring fault. Until that gap is raised, treat an isolated `N` as worth
> investigating rather than as proof of a hardware failure; a burst that is mostly or entirely `N`
> is unambiguous either way.
>
> Note this interacts with `ack_timeout_us`: a module still finishing its previous cycle is exactly
> a module that answers *late*. If `ack_us` shows scattered high values, the ping spacing is the
> more likely culprit than the module — raise the gap before tightening the window.

#### Sketch of the Pi-side dispatch

```python
resp = read_response(ser, req_id)          # matching id, type == "resp"

if resp["status"] == "ok":
    if resp["n_no_response"]:              # no benign cause -- see above
        log.warning("puit: %d trigger(s) ignored (%s)",
                    resp["n_no_response"], resp.get("ping_status", "-"))
    elif resp["n_valid"] < resp["n"]:
        log.info("puit: %d/%d pings lost", resp["n_valid"], resp["n"])
    store(cm=resp["value"], pulse_us=resp["pulse_us"], temp_c=resp["temp_c"],
          n_valid=resp["n_valid"], n_timeout=resp["n_timeout"],
          n_rejected=resp["n_rejected"], n_no_response=resp["n_no_response"],
          n_stuck=resp["n_stuck"], ack_max_us=resp["ack_max_us"])
    return resp["value"]

code = resp["code"]
if code in ("insufficient_samples", "echo_timeout"):
    log.warning("puit: %s (%s)", code, counts(resp))
    raise Retryable(code)
if code in ("sensor_fault", "out_of_range"):
    # n_stuck says which end of the wiring to look at; ack_max_us == 0 confirms
    # nothing answered at all, rather than answering just outside the window.
    log.error("puit: %s (%s) n_stuck=%d ack_max_us=%d -- needs physical attention",
              code, counts(resp), resp["n_stuck"], resp["ack_max_us"])
    notify_telegram(code, resp)            # deliberately not retried
    raise Permanent(code)
log.error("puit: protocol bug -- %s %s", code, resp.get("field", ""))
raise Permanent(code)
```

Note `resp.get(...)` throughout: `value`, `min`, `max`, `spread` and `pulse_us` are **absent** on
every error reply, and `ping_status` exists only on a successful `sampling`. The counts and the
effective parameters are the only measurement fields guaranteed on both.

When something needs investigating, re-issue the same burst as `sampling` — identical parameters,
plus the per-ping arrays and `ping_status`.

- **Units / conversion:** this firmware reports **raw distance in cm only**. The
  distance→volume conversion (`volume_m3 = (220 - cm) * 0.04`) lives entirely on the Pi side
  and in the Grafana dashboards — it is *not* part of this contract.
- **Accuracy / conversion:** the pulse covers the round trip, so
  `divisor = 2 × 1e6 / (v × 100) = 20000 / v` with `v = 331.4 + 0.6·T` m/s. At 20 °C that is
  **58.24**, matching the `/58` constant used through `proto 1`. Temperature is worth ~0.17%/°C —
  about **4 cm across a 10→30 °C swing** at 220 cm — which is why `temp_c` is a request parameter
  even with no temperature sensor fitted: the Pi can supply it from any source it already has.
  ⚠️ The temperature-compensated formula previously sketched in the source comments as future work
  was **4× too small** (`1e6/(v*100)/2` → 14.56 at 20 °C). It was never used; the corrected form
  above is now `cmDivisorFor()` in the sketch.

### Why `pulse_us` is in every response

`pulse_us` is the raw echo width — the only quantity the board actually measures, and the one thing
that is temperature-independent. `value` is derived from it via the divisor above.

**Store both.** The 4×-wrong formula noted above is exactly the class of bug that silently corrupts
a time series: with only cm in InfluxDB, a formula fix cannot be applied to history. With `pulse_us`
and the assumed `temp_c` alongside it, any future correction can be replayed retroactively over
everything already recorded, for the cost of two extra fields per point.

For `read_puit`, `pulse_us` is the median pulse of the valid pings — the exact raw counterpart of
`value`, since `cm = pulse / divisor` is monotonic.

### Future: environment sensing

Not implemented — recorded here so the intent survives. Once a temperature/humidity sensor is fitted
at the well head (I²C on D4/D5; **not** a DHT22, whose ~2 s read would dominate a measurement burst):

- The board reads it **inline, once per burst** — not on a timer. The temperature that matters is the
  one in the air column at ping time, and a cached value reintroduces the very error the sensor is
  there to remove.
- The board returns `value` **only** when it has a real reading for every input. If the environment
  read is missing or fails, it omits `value`, returns `pulse_us` plus whatever it does have, and the
  Pi derives the distance from its own last-known-good temperature. The board must never fabricate a
  distance from a guessed temperature.
- This does not bite today: with no sensor fitted nothing can go *missing*, so 20 °C is a fixed,
  documented assumption and `value` is always present.

Because `pulse_us` is already in the contract, fitting the sensor should be a firmware-only,
additive change — no `proto 3`.

### Required Pi-side (`read_puit.py`) changes

`proto 2` is a **breaking change**. `read_puit.py` must be updated:

- In `open_arduino()`, wait for a line that `json.loads` to `type == "ready"` and assert
  `proto == 2`.
- Send `json.dumps({"id": n, "cmd": "read_puit"}) + "\n"` with an **incrementing integer** `n` —
  a missing or non-integer `id` is now rejected with `bad_id`.
- Read lines until one parses to JSON with a matching `id` and `type == "resp"` (bounded by the
  serial read timeout / an overall deadline). On `status == "ok"`, read `value`. The existing retry
  loop and file lock can stay unchanged.
- **Treat the error codes differently** — the whole point of having distinct codes. The decision
  table, log levels and a dispatch sketch are in
  [Telling outcomes apart](#telling-outcomes-apart--how-to-log-this-properly); in short, retry only
  `insufficient_samples` and `echo_timeout`, alert without retrying on `sensor_fault` and
  `out_of_range`, and treat every remaining code as a Pi-side bug.
- **Record more than cm.** Store `pulse_us`, `temp_c`, all four counts
  (`n_valid`/`n_timeout`/`n_rejected`/`n_no_response`) and `ack_max_us` as InfluxDB fields next to
  the distance, so formula fixes can be replayed over history and a degrading sensor becomes visible
  as a graph instead of a surprise. Graphing `n_valid / n` is the single most useful health signal:
  a sensor on its way out trends downward for weeks before it fails outright, which no single-burst
  threshold can tell you. `ack_max_us` is the second: it is the module's reaction time, and a slow
  upward trend in it is both an early warning and the thing that eventually trips `ack_timeout_us`
  into reporting a live sensor as dead.
- **A single lost ping is not an error.** Occasional timeouts over water are normal, and the median
  over the survivors is still sound. If you want to *log* imperfect bursts without losing the
  reading, check `n_valid < n` on a successful response — the counts are there precisely for that.
  Use `min_valid` only if you genuinely want to discard such readings.
- Optionally send `temp_c` from any temperature source the Pi already has.

## Build & flash (PlatformIO in VSCode)

You flash this by connecting the XIAO directly to your computer over USB.

> ⚠️ **Pick the environment before uploading.** The boards are physically identical, so nothing
> stops you aiming the pump firmware at the well sensor. `-e` is the only thing that decides which
> firmware gets written. Bare `pio run` defaults to `puit` (set by `default_envs` in
> [platformio.ini](platformio.ini)); anything to do with the pump board must say `-e pump`.
>
> They are also **different chips** — SAMD21 for `puit`, RP2040 for `pump` — so a mismatched upload
> now fails at the flashing step rather than producing a bricked-looking board. That is a safety net,
> not a reason to stop checking the silkscreen.

1. **Install the PlatformIO IDE extension** in VSCode. Opening this folder will prompt you to
   install it (see [`.vscode/extensions.json`](.vscode/extensions.json)).
2. **Open this folder** in VSCode. PlatformIO reads [`platformio.ini`](platformio.ini) and, on the
   first build, automatically downloads each environment's platform and toolchain — no manual
   "board core" install like the Arduino IDE requires. The two envs pull **different** toolchains
   (`atmelsam` for `puit`, an RP2040 one for `pump`), so expect the first `-e pump` build to take
   several minutes and need network.
3. **Connect the board** over USB — and be sure which one it is. `pio device list` names it:
   each firmware sets its own USB product string, so the boards show up as **PiJardin Puit**
   and **PiJardin Pompe** rather than two identical `Seeeduino XIAO` / `XIAO RP2040` entries.
   (Windows caches that name per VID+PID, so Device Manager may lag a firmware change until
   you uninstall the device and replug; `pio device list` reads the live descriptor.)
4. **Upload:**
   ```
   pio run -e puit -t upload      # well sensor
   pio run -e pump -t upload      # pump sensor
   ```
   In the VSCode status bar, the environment selector sits next to the **→ (Upload)** button —
   set it first, then upload. To build without uploading, drop `-t upload`:
   ```
   pio run -e puit -e pump        # build both, e.g. to check nothing broke
   ```
5. **Serial monitor** (to test): click the plug icon, or run `pio device monitor -b 9600`.
   The boot banner confirms which firmware is actually on the board: `"role":"puit"` or
   `"role":"pump"`. The USB product string in step 3 says what the board *claims* to be; the
   banner's `role` is what the firmware on it actually is. They agree unless you crossed a
   flash — which is exactly the mistake worth catching here.

### Making `pio` work in a terminal

The VSCode extension installs PlatformIO Core into its own Python venv (`~\.platformio\penv\`)
and **does not add it to your PATH**, so `pio` in a plain terminal fails with "command not
found" even though Core is installed. Either open a terminal that has it — Command Palette →
**PlatformIO: New Terminal** — or call it by full path:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

To get bare `pio` everywhere, add it to your user PATH once, then reopen the terminal:

```powershell
[Environment]::SetEnvironmentVariable("PATH", [Environment]::GetEnvironmentVariable("PATH","User") + ";$env:USERPROFILE\.platformio\penv\Scripts", "User")
```

Every `pio …` command below assumes one of these is in place.

### Troubleshooting upload

If the upload can't find the board / bootloader, **double-tap the reset pad** on the XIAO to
force it into bootloader mode, then upload again immediately.

## Standalone firmware image (flashing without PlatformIO)

PlatformIO is only needed to *compile*. The build produces a plain firmware image you can carry
to any machine and flash there — useful to hand the firmware to someone who has no toolchain
installed, or to keep a known-good image alongside a release.

### 1. Build the image

```
pio run -e puit          # or -e pump
```

(or the **✓ (Build)** button). Artifacts land in `.pio/build/<env>/` — so
`.pio/build/puit/` and `.pio/build/pump/`, one directory per firmware:

| File | What it is |
| --- | --- |
| `firmware.bin` | raw image, **linked for load address `0x2000`** |
| `firmware.elf` | same code with symbols, for debugging |

Both firmwares produce a file called `firmware.bin`, distinguished only by which directory it came
out of. **Rename them when you copy them out** (`puit-2.2.0.bin`, `pump-2.0.0.bin`) — past that
point nothing in the file says which board it belongs to, and flashing the wrong one gives a board
that boots, answers, and is silently wrong.

`0x2000` is the XIAO's application start: the bootloader owns `0x0000`–`0x1FFF`
(`offset_address` in the board definition). The offset matters when flashing — see below.

`.pio/` is gitignored, so **copy the file out** of the build directory if you want to archive or
ship it.

### 2a. Flash by drag & drop (no tooling required)

The XIAO has a UF2 bootloader, so the friendliest artifact is a `.uf2` rather than the `.bin`.
Convert it with [`tools/bin2uf2.py`](tools/bin2uf2.py) (pure Python, no dependencies):

```
python tools\bin2uf2.py .pio\build\puit\firmware.bin .pio\build\puit\firmware.uf2
```

Then:

1. **Double-tap the reset pad** on the XIAO → a USB drive named **Arduino** appears.
2. **Copy `firmware.uf2`** onto that drive.
3. The board reboots into the new firmware on its own — verify with the checks in
   [Testing after a flash](#testing-after-a-flash).

The `0x2000` offset and the SAMD21 family ID are baked into the `.uf2` by the converter, so
there is nothing to get wrong at flash time. This is the recommended route.

### 2b. Flash the `.bin` with `bossac`

`bossac` is the SAM-BA uploader PlatformIO itself calls; it ships with the Arduino IDE SAMD core
(e.g. `%LOCALAPPDATA%\Arduino15\packages\arduino\tools\bossac\1.7.0-arduino3\bossac.exe`).
Double-tap the reset pad first, note the COM port the bootloader exposes, then:

```
bossac.exe -i -d --port=COM5 -U -i -e -w -v --offset=0x2000 firmware.bin -R
```

> **`--offset=0x2000` is not optional.** Writing this image at `0x0000` overwrites the
> bootloader and destroys the drag-and-drop recovery path — re-flashing would then need an
> SWD probe (Atmel-ICE / J-Link).

### Building without PlatformIO at all

`arduino-cli` produces the same artifacts (`.bin`, `.hex`, and a ready-made `.uf2`):

```
arduino-cli core install Seeeduino:samd --additional-urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli lib install "ArduinoJson@7.4.3"
arduino-cli compile -b Seeeduino:samd:seeed_XIAO_m0 --output-dir dist src\puit\puit_sensor.cpp
```

Point it at the one `.cpp` you want; the per-firmware folders keep the sources from colliding the
way two sketches in one directory would.

Pin the same ArduinoJson version that [`platformio.ini`](platformio.ini) resolves (`^7` → 7.4.3
at the time of writing) if you want an image comparable to the PlatformIO build.

## Testing after a flash

> This checklist is for the **well sensor** (`puit`). The pump board has its own, including the
> calibration step it cannot work without — see
> [docs/pump.md](docs/pump.md#testing-after-a-flash).

Open the serial monitor at 9600 baud and reset the board.

1. **Banner** → `{"type":"ready","proto":2,"fw":"2.2.0","role":"puit"}`.
2. `{"id":1,"cmd":"status"}` → `ok` with `fw`/`role`/`uptime_ms` and the limits
   (`max_n:25`, `n_default:10`, `ack_timeout_default_us:50000`, `line_max:192`).
3. `{"id":2,"cmd":"read_puit"}` → `ok` with a numeric `value` in cm, the four counts summing to `n`,
   and `min`/`max`/`spread`. **Check `value ≈ pulse_us / 58.24` by hand** — that is the regression
   guard for the divisor. Against a tape-measured target the reading should be **unchanged from
   fw 1.1.0**; if it moved, the rewrite shifted the calibration.
4. **Parameters:** `{"id":3,"cmd":"sampling","n":25,"temp_c":8.5}` → 25-entry `samples` *and*
   `pulse_us`, `null`s aligned, echoing `n:25`/`temp_c:8.5`. Against a fixed target the cm values
   read *shorter* than at `temp_c:20` (colder air, slower sound) while `pulse_us` is **unchanged** —
   that is what makes retroactive recomputation from `pulse_us` valid.
   - `{"id":4,"cmd":"read_puit","n":999}` → clamped, echoes `n:25`.
   - `{"id":5,"cmd":"read_puit","n":"ten"}` → `bad_param`, `"field":"n"`.
   - `{"id":6,"cmd":"read_puit","min_cm":100,"max_cm":50}` → `bad_param`, `"field":"min_cm"`.
   - `{"id":7,"cmd":"read_puit","max_cm":900}` → echoes `max_cm` capped near 772 (timeout-limited).
5. **Plausibility window — the important one.** Hold a target ~3 cm from the sensor, inside the
   blind zone: expect `out_of_range` with `n_rejected ≈ n` and `n_timeout: 0`. *On fw 1.1.0 the same
   shot returns `status:"ok"` with a ~3 cm value* — worth reproducing on the old firmware first,
   since it is the whole reason this changed. Then `{"id":8,"cmd":"read_puit","min_cm":1}` on the
   same shot → `ok`, confirming it was the window that rejected it.
6. **Gate:** aim so only a few pings return (oblique or partly obstructed surface) →
   `insufficient_samples` with `0 < n_valid < min_valid`. Resend with `"min_valid":0` → the same shot
   returns `ok`.
7. **Sensor fault detection** — the two failures `proto 1` could not separate:
   - **Unplug the echo wire (D7)** → `sensor_fault` with `n_no_response: n`. Check it is *not*
     `echo_timeout`; the whole point is that a disconnected sensor no longer looks like a sensor
     seeing nothing.
   - **Unplug the trigger wire (D8)** → also `sensor_fault`. The sensor never fires, so it never
     raises echo.
   - **Aim at open air / the sky** with everything connected → the sensor answers but finds nothing.
     Expect `out_of_range` with `n_rejected: n` and `n_no_response: 0`, and `pulse_us` around
     `38000` (≈652 cm, beyond `max_cm`). If instead you get `n_timeout: n`, your module does not emit
     the 38 ms pulse — harmless, the rise detection still catches genuine faults, but note it.
   - Run `sampling` in each case and read `ping_status`: `NNNNNNNNNN` for a fault, `RRRRRRRRRR` for
     blind-but-alive.
   - **Tie echo (D7) high** (to 3V3 through a resistor) → `sensor_fault` with `ping_status` all `S`
     and `n_stuck: n`, *not* `N`. The trigger is never fired in this path, so this is the one fault
     that a scope on D8 shows as silence rather than as pulses going nowhere.
8. **Re-check `ack_timeout_us` — required whenever the sensor module is replaced.** The default is
   calibrated to the module currently fitted (~12.3 ms); a replacement clone may differ by an order
   of magnitude in either direction, and getting this wrong reports healthy hardware as dead.
   - `{"id":10,"cmd":"sampling","n":25}` → read `ack_max_us` and the `ack_us` array. Expect a tight
     spread; a *scattered* one points at the `delay(3)` ping spacing rather than the module.
   - If `ack_max_us` has moved, set `DEFAULT_ACK_TIMEOUT_US` to ~4× the new maximum, and record the
     measurement in the commit message the way the current one is recorded in the sketch. Keep it
     under ~50 ms or the all-dead burst starts to exceed the 1.51 s all-timeout worst case.
   - Confirm the failure direction is understood: `{"id":11,"cmd":"read_puit","ack_timeout_us":500}`
     on a *working* sensor must produce `sensor_fault` with `n_no_response: n`. **This is the
     firmware calling healthy hardware dead**, reproduced deliberately — the exact bug that shipped
     at 2 ms. Re-run without the override to confirm it recovers.
9. **Errors:** send `hello` → `bad_request` with `"id":null`. Send `{"cmd":"status"}` → `bad_id`.
   Send `{"id":12,"cmd":"nope"}` → `unknown_cmd` with `"id":12`. Paste a line longer than 192
   characters → `line_too_long`, then confirm the **next** valid request still answers — that proves
   the reader recovers by draining to the newline.
10. **Reader:** send two requests back-to-back with no delay → two correlated responses, no lost
    line. Send an empty line → no reply at all. A `read_puit` should return promptly, with no 200 ms
    idle penalty and no 1 s timeout stall.
11. Back on the Pi, once `read_puit.py` is updated for `proto 2`, run `/mesure` (Telegram) or wait
    for `sensors.service` — it should record a value, confirming end-to-end compatibility.
