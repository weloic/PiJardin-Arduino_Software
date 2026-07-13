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
> Changing any command, response format, the boot banner, or the baud rate here **will break
> the Pi** unless `read_puit.py` is updated to match. Keep them in sync.

- **Transport:** USB CDC serial, **9600 baud**. Enumerates as `/dev/ttyACM0` on the Pi.
- **Boot banner:** on reset the board prints a single line `Started serial com`. PiJardin's
  `open_arduino()` toggles DTR to reset the board and waits for exactly this line before
  issuing commands.
- **Commands** (each terminated by a newline `\n`):

  | Command     | Response                                                                 |
  |-------------|--------------------------------------------------------------------------|
  | `READ_PUIT` | One float — the **median distance in cm** over 10 pings.                 |
  | `SAMPLING`  | A JSON-ish array of the 10 raw samples, e.g. `[61.00, 61.00, 158.00, …]` |
  | `STATUS`    | `System OK. Other functions: 'READ_PUIT'`                                |
  | *(other)*   | `Unknown command.`                                                       |

- **Units / conversion:** this firmware reports **raw distance in cm only**. The
  distance→volume conversion (`volume_m3 = (220 - cm) * 0.04`) lives entirely on the Pi side
  and in the Grafana dashboards — it is *not* part of this contract.
- **Accuracy note:** the `/58` divisor in the sketch assumes ~20 °C (speed of sound ≈ 343 m/s).
  A more precise, temperature-compensated formula is sketched in the source comments for future
  work (would require a temperature sensor).

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

1. Open the serial monitor at 9600 baud and reset the board — you should see `Started serial com`.
2. Send `STATUS` → expect `System OK...`.
3. Send `READ_PUIT` → expect a numeric distance in cm.
4. Back on the Pi, run `/mesure` (Telegram) or wait for `sensors.service` — it should record a
   value as before, confirming the firmware is still compatible with `read_puit.py`.
