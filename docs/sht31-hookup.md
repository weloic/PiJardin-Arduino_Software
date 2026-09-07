# SHT31 probe — hookup reference

Everything needed to wire the PiJardin **SHT31 temperature/humidity probe** to any board and confirm
it works. Board-agnostic: the firmware in [src/sht31/](../src/sht31/) is one consumer of this probe,
not a requirement of it.

For the bench firmware's own commands and diagnostics, see [docs/sht31.md](sht31.md).

## The probe

A sealed SHT31 on a **3 m five-conductor cable**, terminated in flying leads with no silkscreen.

| | |
|---|---|
| Sensor | Sensirion SHT31, I²C |
| I²C address | **`0x44`** (`0b1000100`) |
| Supply | **3.3 V**, ~1 mA |
| Cable | 3 m, 5 conductors |
| Bus pull-ups | **already on the probe** — none to add for runs up to 3 m |

## Conductor map

**Verified by measurement on this probe** — not read off a datasheet, because there is none:

| Colour | Function | Connect to |
|---|---|---|
| **red** | VCC | 3.3 V |
| **yellow** | **SDA** | the board's SDA |
| **green** | **SCL** | the board's SCL |
| **black** ×2 | GND / cable shield | **both** to GND |

> ⚠️ **Yellow is SDA and green is SCL — the opposite of the convention every tutorial quotes**
> (yellow = SCL, green = SDA). Do not trust the colours on a different probe either. SDA and SCL are
> open-drain lines at both ends, so **swapping them damages nothing** — if a bus scan comes back
> empty, swap those two wires and scan again before suspecting anything else.

The two blacks are **not connected to each other** inside the probe: one is the ground return, the
other the cable shield. Which is which was never isolated, and does not need to be — grounding both
is correct either way. If the second conductor is instead the SHT31's `ADDR` pin, grounding it
selects `0x44`, which is the address the probe answers on regardless.

On a long run, ground the shield **at the controller end only**; grounding both ends makes a ground
loop.

## Electrical requirements

> ⚠️ **Power the probe from 3.3 V. Never 5 V.** The bare SHT31 die tolerates 2.4–5.5 V, so "it is a
> 5 V-tolerant sensor" is a half-truth that destroys microcontrollers: the probe pulls SDA and SCL up
> to **its own supply rail**. Powered from 5 V it drives 5 V into the SDA/SCL pins, which most 3.3 V
> parts (RP2040, SAMD21, ESP32) are not tolerant of.

**On a 5 V board (Uno, Nano, Mega), this probe cannot be wired directly.** Powering it from the
board's 3.3 V output while connecting SDA/SCL straight to 5 V logic is out of spec in the other
direction — the probe's `V_IH` is referenced to 3.3 V, and the board's pull-ups would sit at 5 V. Use
a proper I²C level shifter (BSS138-based), or prefer a 3.3 V board.

### Measured characteristics

Useful for sanity-checking an installation, and for deciding what a longer cable needs:

| Quantity | Measured |
|---|---|
| Highest working bus speed | **200 kHz** (400 kHz fails) |
| Capacitance per data line | ~100–150 pF |
| Probe's own pull-ups | ~20–50 kΩ (weak, but sufficient at 3 m) |
| Rise time on the bus | ~3 µs |
| Measurement noise, temperature | 0.04 °C peak-to-peak over 5 samples |
| Heater response, 20 s | **+2.7 °C**, **−6.2 %RH** |

**No external pull-up resistors are needed** for a 3 m run at 100 kHz. If the cable is extended:
fit 2.2 kΩ from SDA to 3.3 V and 2.2 kΩ from SCL to 3.3 V at the controller end. Beyond about 4 m,
I²C's 400 pF bus budget is spent and the right answer is a bus extender (P82B715 at each end) rather
than ever-stronger pull-ups.

## Wiring, per board

The rule everywhere: red → 3.3 V, both blacks → GND, yellow → SDA, green → SCL.

| Board | 3.3 V | GND | SDA (yellow) | SCL (green) |
|---|---|---|---|---|
| **XIAO RP2040** | `3V3` pad | `GND` | `D4` (GPIO6) | `D5` (GPIO7) |
| **XIAO SAMD21** | `3V3` pad | `GND` | `D4` (PA08) | `D5` (PA09) |
| **ESP32 (devkit)** | `3V3` | `GND` | `GPIO21` | `GPIO22` |
| **Raspberry Pi** (any) | pin 1 | pin 6 | pin 3 (GPIO2) | pin 5 (GPIO5) |
| **Arduino Uno / Nano** | — | — | `A4` | `A5` — **needs a level shifter, see above** |

> On the **XIAO SAMD21** — the well sensor's board — D4/D5 are already the pins the README reserves
> for environment sensing, and they do not collide with the HC-SR04 on D7/D8.

### Mechanical

Fine stranded 3 m conductors **do not hold in breadboard clips**. They splay, oxidise, and seat
partially while looking connected. Solder a short length of solid 22 AWG wire to each conductor and
plug *that* in, or use a 2.54 mm screw-terminal block. Tinning the strands works for a short bench
session but the solder creeps under the clip pressure and the contact degrades.

## Confirming it works

### 1. Bus scan

Any I²C scanner. Expect **one device at `0x44`**.

- Nothing found → swap yellow and green, scan again.
- Still nothing → check 3.3 V reaches the probe, and that a black is really on GND.
- `0x45` found instead → the probe is strapped to the alternate address; use that.

On a Raspberry Pi: `i2cdetect -y 1`.

### 2. One measurement

Single-shot, high repeatability, clock stretching disabled:

1. Write **`0x2400`** (two bytes, MSB first) to `0x44`.
2. Wait **20 ms** (datasheet maximum is 15 ms).
3. Read **6 bytes**: `T_msb, T_lsb, T_crc, RH_msb, RH_lsb, RH_crc`.

Conversions, defined over the full 16-bit range:

```
temperature_C = -45 + 175 * raw_T  / 65535
humidity_%    =       100 * raw_RH / 65535
```

**Check the CRCs.** Each word carries a CRC-8 the sensor computed itself — polynomial `0x31`
(x⁸+x⁵+x⁴+1), init `0xFF`, no reflection, no final XOR. It is what turns "the number looks
plausible" into "the sensor and I agree on every bit", and it is the difference between a working
probe and a marginal one that will start lying at the end of a long cable.

```cpp
uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}
```

A minimal, board-agnostic read:

```cpp
#include <Wire.h>

bool readSHT31(float *t_c, float *rh) {
  Wire.beginTransmission(0x44);
  Wire.write(0x24); Wire.write(0x00);        // single shot, high repeatability
  if (Wire.endTransmission() != 0) return false;   // nobody acknowledged

  delay(20);                                  // conversion time
  if (Wire.requestFrom((uint8_t)0x44, (size_t)6) != 6) return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  if (crc8(b, 2) != b[2] || crc8(b + 3, 2) != b[5]) return false;  // corrupt

  *t_c = -45.0f + 175.0f * ((b[0] << 8 | b[1]) / 65535.0f);
  *rh  =          100.0f * ((b[3] << 8 | b[4]) / 65535.0f);
  return true;
}
```

Set the bus to **100 kHz** (`Wire.setClock(100000)`); 400 kHz does not work on this cable.

### 3. Prove the probe is real

A faulty, counterfeit or miswired module can still return a plausible constant. The SHT31's
**internal heater** is what no such module can fake:

1. Take a baseline reading.
2. Write **`0x306D`** (heater on) to `0x44`.
3. Wait 20 s, reading as you go.
4. Write **`0x3066`** (heater off).

**Temperature must rise and humidity must fall, together** — warmer air holding the same absolute
water reads a lower relative humidity. On this probe, 20 s gives **+2.7 °C and −6.2 %RH**.

Expect a modest rise: the die is potted into a housing with far more thermal mass than a bare
breakout, so the change is real but small and slow. Judge it against the sensor's noise (0.04 °C
here), not against an absolute threshold — a 0.5 °C rise is already twenty-five times the noise.

> **Always turn the heater back off.** It biases humidity and warms the die for a minute afterwards,
> and Sensirion specifies it for intermittent use only.

### 4. Verifying the temperature is accurate

Steps 1–3 prove the probe *works*. They do not prove the number is *right*. Three things to do, in
this order — the first two are usually the whole answer.

#### a. Let it settle, and know what disturbs it

A reading is only ambient once the probe body is. Before trusting any absolute value:

- **Wait 10 minutes after any heater use.** The die cools in seconds; the potting and housing take
  minutes. A baseline taken right after a heater test reads several degrees high, and looks exactly
  like a miscalibrated sensor.
- **Wait a few minutes after handling it.** A hand is at 37 °C and the housing stores it.
- Keep it away from a laptop, a USB power brick, a wall in the sun, and any airflow from a vent.
  These move a real reading by degrees — far more than the sensor's own error.
- **Self-heating is not the culprit here.** Sensirion specifies at most one high-repeatability
  measurement per second to keep self-heating under 0.1 °C. To rule it out anyway: take a reading
  after five minutes of silence, then stream at 1 Hz for five minutes and compare. A rise of more
  than ~0.1 °C means the duty cycle is too high for the installation.

#### b. The ice point — an absolute reference, no instrument needed

A well-made ice bath is **0.0 °C by definition**, to within about 0.01 °C. It is a primary standard
sitting in your freezer, and it needs no calibrated thermometer to trust.

1. Fill a cup with **crushed ice**, then add just enough cold water to fill the gaps. Mostly ice,
   not mostly water — a few floating cubes is not an ice bath and will read high.
2. **Put the probe in a sealed plastic bag** and immerse the bagged tip. The bag costs a few tenths
   of a degree of response time and removes any question of water reaching the electronics through
   a cable gland or a housing seam. Do not immerse an unbagged probe unless you are certain it is
   sealed for it.
3. Stir, and wait for the reading to stop falling — several minutes on a potted probe.

**Expect 0.0 °C ± 0.3 °C.** Outside ±0.5 °C, something is wrong with the probe rather than with the
bath. Ignore the humidity reading during this test; it is meaningless in a bag.

Do not use boiling water as the other point: it is altitude-dependent (about 98.3 °C at 500 m), it
saturates the humidity sensor, and it tests a part of the range this probe will never see.

#### c. Compare against a reference

Any thermometer you already trust, **in the same air, both left to equilibrate 15 minutes**, out of
sunlight and airflow. Two instruments 30 cm apart in a room with a radiator can legitimately differ
by a degree, so put them side by side and give them time.

#### What accuracy to expect

An SHT31 is specified at roughly **±0.3 °C** for temperature and **±2 %RH** for humidity in ordinary
room conditions — check the datasheet for the exact part, since the SHT35 in the same package is
three times tighter. So:

| Difference from a trusted reference | Verdict |
|---|---|
| < 0.5 °C | normal — within spec plus the reference's own error |
| 0.5 – 1 °C | suspect the *setup* first: equilibration, airflow, a nearby heat source |
| > 1 °C, after a proper ice-point test | the probe itself |

Precision and accuracy are different questions, and this probe already answered the first: five
consecutive samples spread by **0.04 °C**. A sensor that repeatable is not noisy — if it is wrong,
it is wrong by a stable offset, which is exactly the kind of error an ice-point test exposes and a
glance at the value never will.

#### d. After a wet or cold test: drying and re-settling

A probe taken out of an ice bath is below the dew point of the room air, so **water condenses on and
inside the filter cap**. Until it dries, humidity sits pinned near 95–100 %RH and temperature reads
*low*, from evaporative cooling. Neither number means anything in that state.

Clearing it is the heater's other documented purpose:

1. Watch the humidity. **Near 100 % means there is condensation**, not that the room is saturated.
2. Heater on for 30 s. Temperature rises, humidity falls.
3. Let it cool. If humidity **stays** low, it is dry. If it climbs back toward 100 %, there is water
   left — wait two minutes and repeat.
4. Once dry, **turn everything off and wait 10 minutes** before trusting an absolute value again.

> ⚠️ **The heater does not bring the probe back to ambient faster — it does the opposite.** It
> drives the housing *above* room temperature, so every heater burst adds minutes of cooling
> afterwards. Use it to dry the probe, never to hurry it. To simply return to ambient, do nothing
> and wait.

Thirty seconds is the ceiling on purpose: Sensirion specifies the heater for **intermittent** use.
Repeat bursts with gaps rather than looking for a way to leave it on.

Humidity that refuses to fall below ~90 % after several cycles is a different problem: **water is
inside the housing**, not on it. That is what an unbagged immersion of a probe whose seal is not
rated for it produces, and drying it out takes hours in warm still air, if it recovers at all.

### Other useful commands

| Command | Bytes | Notes |
|---|---|---|
| Soft reset | `0x30A2` | ready again after 1.5 ms |
| Read status register | `0xF32D` | returns 2 bytes + CRC |
| Clear status register | `0x3041` | makes the sticky bits meaningful as event flags |
| Heater on / off | `0x306D` / `0x3066` | |
| Measure, medium / low repeatability | `0x240B` / `0x2416` | 10 ms / 8 ms conversion |

Status register bits worth reading: **15** alert pending, **13** heater on, **4** system reset
detected, **1** last command failed, **0** last write checksum failed. Bits 15 and 4 are set on a
fresh device and stay set until cleared — clear them once and a `reset detected` that comes back
means the probe *actually* lost power.

## Gotchas, in one list

1. **3.3 V only.** 5 V destroys the controller through the probe's own pull-ups.
2. **The colours lie.** Yellow is SDA, green is SCL. If a scan is empty, swap them first.
3. **Both blacks to GND.** They are separate nets; grounding both is correct whatever they are.
4. **Sensirion rate limit.** At most one high-repeatability measurement per second, or self-heating
   exceeds 0.1 °C.
5. **Not 400 kHz.** 100 kHz is the right default; 200 kHz works, 400 kHz does not.
6. **Not in a breadboard bare.** Solder a solid-wire pigtail or use a screw terminal.
7. **Check the CRCs**, always. They are the only thing separating a good reading from a plausible
   corrupt one.
