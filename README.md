# PiJardin — Arduino Software

Firmware for the **PiJardin** well ("puit") water-level sensor. This is the companion
repository to the main [PiJardin](https://github.com/wermeill/PiJardin) project: PiJardin
runs the Raspberry Pi side (Python, InfluxDB, Grafana, Telegram bot), and **this repo holds
the microcontroller firmware** that actually reads the ultrasonic sensor.

The two are kept separate on purpose — different toolchain (C++/Arduino vs Python) and a
different lifecycle (the firmware changes rarely). Firmware is flashed **physically over USB
from a laptop**, never remotely from the Pi.

## Hardware

- **Board:** Seeed Studio XIAO SAMD21 (ARM Cortex-M0+, native USB).
- **Sensor:** HC-SR04-style ultrasonic distance sensor, mounted above the well, measuring the
  distance down to the water surface.

Wiring (from the sketch):

| Signal        | Pin           |
|---------------|---------------|
| Echo (input)  | `D7`          |
| Trigger (out) | `D8`          |
| Status LED    | `LED_BUILTIN` |

The LED lights while a command is being processed.

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
- **Protocol version:** `proto = 1`.
- **Boot banner:** on reset the board prints one line and nothing else until commanded:
  ```json
  {"type":"ready","proto":1,"fw":"1.1.0"}
  ```
  The Pi resets the board (DTR toggle) and waits for a line that parses to JSON with
  `type == "ready"` before issuing commands.

### Requests (Pi → board)

```json
{"id":<int>,"cmd":"<name>"}
```

- `id` — a request counter chosen by the Pi; echoed back verbatim in the response.
- `cmd` — one of `read_puit`, `sampling`, `status`.

### Responses (board → Pi)

All responses carry `"type":"resp"`, the echoed `id`, and a `status` of `"ok"` or `"error"`.

| Command / case | Example response |
|----------------|------------------|
| `read_puit` ok | `{"id":42,"type":"resp","status":"ok","value":123.4,"unit":"cm","n":10,"n_valid":9}` |
| `sampling` ok  | `{"id":42,"type":"resp","status":"ok","unit":"cm","samples":[123.4,124.0, …]}` |
| `status` ok    | `{"id":42,"type":"resp","status":"ok","fw":"1.1.0","proto":1,"uptime_ms":12345}` |
| sensor failure | `{"id":42,"type":"resp","status":"error","code":"echo_timeout"}` |
| unknown `cmd`  | `{"id":42,"type":"resp","status":"error","code":"unknown_cmd"}` |
| bad request    | `{"id":null,"type":"resp","status":"error","code":"bad_request"}` |

`read_puit` returns the **median distance in cm** over `n` pings, computed over the `n_valid`
pings that returned an echo. If *no* ping echoes back, it returns `echo_timeout` instead of a
bogus value. `bad_request` is emitted when the line is not valid JSON or has no known `cmd`
(the `id` is `null` because it could not be read).

- **Units / conversion:** this firmware reports **raw distance in cm only**. The
  distance→volume conversion (`volume_m3 = (220 - cm) * 0.04`) lives entirely on the Pi side
  and in the Grafana dashboards — it is *not* part of this contract.
- **Accuracy note:** the `/58` divisor in the sketch assumes ~20 °C (speed of sound ≈ 343 m/s).
  A more precise, temperature-compensated formula is sketched in the source comments for future
  work (would require a temperature sensor).

### Required Pi-side (`read_puit.py`) changes

This NDJSON protocol is a **breaking change** from the previous line-based text protocol.
`read_puit.py` must be updated to match:

- In `open_arduino()`, wait for a line that `json.loads` to `type == "ready"` (optionally
  assert `proto == 1`) instead of the literal string `Started serial com`.
- Send `json.dumps({"id": n, "cmd": "read_puit"}) + "\n"` with an incrementing `n`.
- Replace the fixed `sleep(0.5)` / 12 s waits with: read lines until one parses to JSON with a
  matching `id` and `type == "resp"` (bounded by the serial read timeout / an overall
  deadline). On `status == "error"`, surface the `code`; on `ok`, read `value`. The existing
  retry loop and file lock can stay unchanged.

## Build & flash (PlatformIO in VSCode)

You flash this by connecting the XIAO directly to your computer over USB.

1. **Install the PlatformIO IDE extension** in VSCode. Opening this folder will prompt you to
   install it (see [`.vscode/extensions.json`](.vscode/extensions.json)).
2. **Open this folder** in VSCode. PlatformIO reads [`platformio.ini`](platformio.ini) and, on
   the first build, automatically downloads the SAMD (`atmelsam`) platform and toolchain — no
   manual "board core" install like the Arduino IDE requires.
3. **Connect the XIAO** over USB.
4. **Upload:** click the **→ (Upload)** button in the PlatformIO status bar, or run:
   ```
   pio run -t upload
   ```
   To build without uploading, use **✓ (Build)** or `pio run`.
5. **Serial monitor** (to test): click the plug icon, or run `pio device monitor -b 9600`.

### Troubleshooting upload

If the upload can't find the board / bootloader, **double-tap the reset pad** on the XIAO to
force it into bootloader mode, then upload again immediately.

## Testing after a flash

1. Open the serial monitor at 9600 baud and reset the board — you should see the ready banner
   `{"type":"ready","proto":1,"fw":"1.1.0"}`.
2. Send `{"id":1,"cmd":"status"}` → expect an `ok` response with `fw`/`proto`/`uptime_ms` and `"id":1`.
3. Send `{"id":2,"cmd":"read_puit"}` → expect an `ok` response with a numeric `value` in cm.
4. Send `{"id":3,"cmd":"sampling"}` → expect an `ok` response with a `samples` array.
5. Send garbage (e.g. `hello`) → expect `{"id":null,...,"code":"bad_request"}`; aim the sensor
   at open air → expect `read_puit` to return `code":"echo_timeout"` rather than `0`.
6. Back on the Pi, once `read_puit.py` is updated to the NDJSON protocol, run `/mesure`
   (Telegram) or wait for `sensors.service` — it should record a value, confirming end-to-end
   compatibility.
