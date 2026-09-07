// PiJardin SHT31 bench rig -- XIAO RP2040 + Sensirion SHT31 temperature/humidity.
//
// WHAT THIS IS, AND WHAT IT IS NOT
// --------------------------------
// This is a BENCH firmware, not a deployed one. Its job is to answer one
// question about a newly bought SHT31 module before it goes anywhere near the
// well: is it wired right, does it answer, and are the numbers it returns real?
// It runs on a spare XIAO RP2040 because the well board (`puit`, a SAMD21) is
// not to hand -- the sensor is I2C and the protocol is the repo's usual NDJSON,
// so nothing here depends on which of the two chips is underneath.
//
// The eventual home is the well sensor. README.md's "Future: environment
// sensing" section already specifies what that looks like: the board reads the
// probe INLINE, once per measurement burst, and returns `value` only when it has
// a real reading for every input -- never a distance derived from a guessed
// temperature. Nothing in this file contradicts that; readEnv() is written to be
// lifted into src/puit/ as-is, and the response field is deliberately called
// `temp_c` because that is the name puit's contract ALREADY uses for the assumed
// air temperature. When the probe moves to the well, the Pi keeps reading the
// same key and it simply stops being an assumption.
//
//   Pi  -> {"id":42,"cmd":"read_env"}\n
//   MCU -> {"id":42,"type":"resp","proto":1,"status":"ok","temp_c":22.4,...}\n
//   MCU -> {"type":"event","proto":1,"role":"sht31","ev":"env","seq":7,...}\n   (only while streaming)
//
// WIRING (XIAO RP2040, and the one way to destroy the module)
// ----------------------------------------------------------
//   SHT31 VIN -> 3V3 pad          NOT 5V, and not because of the RP2040 alone:
//                                 the bare SHT31 die is a 2.4-5.5 V part, but
//                                 most breakouts regulate to 3.3 V and pull SDA
//                                 and SCL up to THEIR rail. A 5 V-powered
//                                 breakout therefore drives 5 V into GPIO6/7,
//                                 which the RP2040 is not tolerant of. 3V3
//                                 everywhere is the only arrangement that is
//                                 safe regardless of what the breakout does.
//   SHT31 GND -> GND
//   SHT31 SDA -> D4  (GPIO6)      i2c1 SDA on this variant
//   SHT31 SCL -> D5  (GPIO7)      i2c1 SCL
//   SHT31 ADR -> GND or unwired   -> address 0x44 (the default here)
//                tied to VIN      -> address 0x45 (pass "addr":69 per request)
//
// Most breakouts carry their own pull-ups (10k). If yours does not, the bus will
// look dead in exactly the way a missing sensor does -- `scan` returning nothing
// is the symptom, and 4.7k to 3V3 on each line is the fix.
//
// WHY THIS DRIVES THE SENSOR DIRECTLY INSTEAD OF USING A LIBRARY
// -------------------------------------------------------------
// The obvious move is Adafruit_SHT31. It was not taken, for the same reason
// src/puit/ hand-rolls ping() instead of calling pulseIn(): the convenient API
// collapses distinguishable failures into one indistinguishable value. That
// library returns NAN for "nothing on the bus", "the sensor answered but the
// CRC is wrong" and "the read came up short" alike -- and on a bench rig those
// three send you to three different places. Nothing there means check the
// wiring and the pull-ups; a bad CRC means the wiring is fine and the SIGNAL is
// not (too long a lead, no pull-ups, the wrong bus speed); a short read means
// the sensor was still busy. Roughly 80 lines below buy that split, and the
// counts and per-sample status characters put it on every reply.
//
// The other reason is CRC. Every SHT3x word ships with a CRC-8 the sensor
// computed itself, which makes "is this number real?" -- the actual question
// being asked of a newly bought module -- answerable rather than a matter of
// squinting at the value. It is checked here on every word and reported.
//
// WHY THERE IS NO PLAUSIBILITY WINDOW
// -----------------------------------
// puit has min_cm/max_cm because an ultrasonic module CAN return a physically
// plausible, internally consistent, completely wrong number -- an echo off the
// bracket looks exactly like an echo off the water. That failure has no
// counterpart here. The SHT31's conversion is defined over the full 16-bit
// range, so every possible raw word maps into -45..130 C and 0..100 %RH: a
// window over the OUTPUT would reject nothing a working sensor can produce. The
// failures that a window would be reaching for -- a bus stuck low reading
// 0x0000, stuck high reading 0xFFFF, a bit flipped by a long lead -- are all
// caught upstream by the CRC, which is a check on the WORD rather than a guess
// about the value. Adding a window would only add a way to reject good data.
//
// This is a .cpp rather than a .ino for the reason given in platformio.ini: the
// repo builds several firmwares from one src/ tree, and PlatformIO's sketch
// conversion ignores build_src_filter.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

// --- Protocol / firmware identity -------------------------------------------
// proto is numbered PER FIRMWARE in this repo, not per repo: this board shares
// the NDJSON envelope (id/type/proto/status/code) with puit and pump but not
// their command sets, so its contract starts at 1 and is versioned on its own.
// puit and pump both being at 2 is a coincidence and means different things.
#define FW_VERSION "1.0.0"
#define PROTO_VERSION 1

// Board role, injected per environment by platformio.ini (-DPIJARDIN_ROLE). It rides on
// the boot banner, every event line and the status response so the Pi can tell the boards
// apart from the data itself rather than guessing from a USB VID or a proto number.
// Deliberately not a literal here: a copy in the source could disagree with the
// environment that built it, and a firmware that lies about its role is worse than one
// that will not compile.
#ifndef PIJARDIN_ROLE
#error "PIJARDIN_ROLE is not defined -- set it in this environment's build_flags in platformio.ini"
#endif

// --- I2C ---------------------------------------------------------------------
// D4/D5 on this variant, which is i2c1's default pin pair -- Wire.begin() alone
// would land here. setSDA/setSCL are called anyway so the pins are stated in the
// firmware rather than inherited from a variant header nobody opens.
#define I2C_SDA_PIN SDA  // D4 = GPIO6
#define I2C_SCL_PIN SCL  // D5 = GPIO7

// 100 kHz, not the 400 kHz the SHT31 supports: the sensor's conversion (~15 ms)
// dominates the transaction anyway, so a faster bus would buy roughly nothing
// and cost edge margin.
//
// SETTABLE AT RUNTIME (`bus` command), which is not a convenience -- it is the
// single most useful diagnostic on this rig, because the probe is on a THREE
// METRE cable and I2C was never specified for that.
//
// The spec budgets 400 pF for the whole bus. Shielded cable runs ~100 pF/m per
// conductor to the shield, so 3 m is ~300 pF before the pin capacitance at
// either end -- at the edge of the budget with a proper pull-up, and far past it
// without one. What that does to the rising edge (the falling edge is driven, so
// only the rise is at risk):
//
//   pull-up    tau = R*C    rise to V_IH (~1.2 tau)    fits in a 100 kHz high
//   65k (internal)  19.5 us     ~23 us                  NO -- high is ~4 us
//   4.7k            1.4 us      ~1.7 us                 marginal (spec: 1 us)
//   2.2k            0.66 us     ~0.8 us                 yes
//
// So a 3 m probe on the RP2040's internal pull-ups alone CANNOT work at
// 100 kHz, and the symptom is every address NACKing -- indistinguishable from an
// empty bus. Dropping to 10 kHz stretches the clock high time to ~50 us, which a
// 23 us rise fits, so a slow bus is what makes a rise-time problem TESTABLE
// without soldering: if `scan` finds the sensor at 10 kHz and not at 100 kHz,
// the pull-ups are the fault and the wiring is fine. `scan` with "sweep":true
// runs exactly that ladder in one command.
//
// The fix is 2.2k from each line to 3V3, not a permanently slow bus. Slow is the
// diagnosis; the resistors are the cure.
#define DEFAULT_I2C_HZ 100000
// 2 kHz is the floor the RP2040 can actually produce: the SDK splits the bit
// period between two 16-bit counters, and below ~1 kHz the low half overflows
// and the baudrate silently becomes something else. It is set this low so that
// "nothing found at any speed" is a statement about the WIRING rather than about
// how far the ladder happened to reach.
#define MIN_I2C_HZ 2000
#define MAX_I2C_HZ 400000

// --- Line probing ------------------------------------------------------------
// How long to wait for a released line to rise before calling it held low.
// 5 ms corresponds to ~64 nF, which is not a cable, it is a fault.
#define RISE_TIMEOUT_US 5000
// RP2040 pad pull-up, datasheet 50-80 kOhm. Only used to turn a measured rise
// time into an estimated capacitance, which is why a mid-range figure is honest
// enough -- the rise time itself is the measurement, the picofarads are a
// convenience with +/-25% of slop before anything else is counted.
#define PULLUP_OHMS 65000.0f

// One rise is timed by micros(), whose resolution is 1 us -- and a bare pad
// rises in well under that, so a single reading of a disconnected pin lands on 0
// or 1 essentially at random. Averaging over many releases recovers the
// fraction, which is what makes "bare pad" and "3 m of cable" separable rather
// than merely different.
#define RISE_REPS 64
#define DISCHARGE_US 100

// MEASURED FLOOR, and a correction worth recording.
//
// This constant was first set to 2.0 us on the assumption that the
// pinMode/digitalRead path between releasing a line and first observing it cost
// about that much, and the first field readings -- a steady 2.00 us on both
// lines -- appeared to confirm it. They did not: a later run on the same
// firmware read 0.0625-0.1875 us, which is impossible if the code path alone
// costs 2 us. So the overhead is under a tenth of a microsecond, and that
// steady 2.00 us was a REAL measurement of roughly 25 pF -- a bare pad plus a
// stub of wire, not an instrumentation artefact.
//
// The lesson is the one this whole file is built around: a "floor" that is
// assumed rather than measured will be read back as confirmation of itself. The
// number below is now the largest overhead the field data can support, and the
// interpretation table in docs/sht31.md was corrected with it.
#define RISE_FLOOR_US 0.2f

// --- Line state, for the watch mode ------------------------------------------
// A word per line instead of a number, because the repeat mode is watched by a
// PERSON while they move a wire, and nobody reads four hundred characters of
// JSON a second. Thresholds on the internal-pull-up rise:
//
//   none   nothing on the pin at all -- bare pad
//   stub   a jumper, a track, a wire end that stops short (~10-40 pF)
//   cable  metres of conductor really are attached (>100 pF)
//   low    the line never rose: clamped
//   pulled an external pull-up is present, so the rise measures R*C and the
//          classification above cannot be applied -- read rc_ns instead
#define LINE_NONE_US 0.5f
#define LINE_STUB_US 8.0f

// How long to wait for a line released with NO pull at all. Anything that pulls
// it high in this window is external -- a real pull-up resistor, or a device
// driving the line. Deliberately short: a genuinely floating line also creeps
// upward from pad leakage (nA into pF is volts per millisecond), so a long
// window would report leakage as a pull-up. A 10k pull-up on 300 pF rises in
// ~3.6 us, so 30 us is generous for anything anyone would actually fit.
#define FREE_TIMEOUT_US 30

// Hard ceiling on any single blocking transaction. Without it a half-seated SDA
// wire -- the single most likely bench fault -- holds the line low and the
// firmware stops answering the Pi at all, which is the one outcome worse than a
// wrong reading. The `true` asks the core to attempt bus recovery (nine clock
// pulses, then a STOP) on the way out, since a slave mid-byte when the master
// gave up is exactly what that sequence exists to unstick.
#define I2C_TIMEOUT_MS 25

#define ADDR_LOW 0x44   // ADR pin low / unconnected -- the usual module default
#define ADDR_HIGH 0x45  // ADR pin tied high
#define DEFAULT_ADDR ADDR_LOW

// SHT3x commands. All 16-bit, MSB first.
#define CMD_MEAS_HIGH 0x2400   // single shot, high repeatability, clock stretching OFF
#define CMD_MEAS_MED 0x240B
#define CMD_MEAS_LOW 0x2416
#define CMD_SOFT_RESET 0x30A2
#define CMD_HEATER_ON 0x306D
#define CMD_HEATER_OFF 0x3066
#define CMD_READ_STATUS 0xF32D
#define CMD_CLEAR_STATUS 0x3041

// Conversion time by repeatability, datasheet maximum plus margin. Clock
// stretching is deliberately NOT used (0x24xx, not 0x2Cxx): with it the sensor
// holds SCL until it is ready, which is convenient but makes "the sensor is
// busy" and "the sensor is holding the bus because something is wrong"
// indistinguishable from up here -- both are just a long blocking call. Waiting
// a known time and then reading turns a sensor that is not ready into an
// ordinary short read, which is reportable.
#define WAIT_HIGH_MS 20  // datasheet max 15
#define WAIT_MED_MS 10   // datasheet max 6
#define WAIT_LOW_MS 8    // datasheet max 4

// --- Burst parameters --------------------------------------------------------
#define MAX_N 10
#define DEFAULT_N 3  // see the note on gap_ms; puit will use 1 when this merges
#define DEFAULT_GAP_MS 100
#define MIN_GAP_MS 2
#define MAX_GAP_MS 2000

// Sensirion specifies at most one high-repeatability measurement per second to
// keep self-heating under 0.1 C. A 3-sample burst at 100 ms breaks that on
// purpose and briefly: the spread across three samples is the cheapest evidence
// that a module is genuinely measuring rather than returning a latched word, and
// that is the entire point of a bench rig. It is bounded -- three samples, then
// silence -- and `stream` defaults to a 1000 ms period precisely so the
// continuous case stays inside the datasheet's duty cycle. The version that ends
// up in puit reads once per burst and never runs into this at all.

#define DEFAULT_STREAM_MS 1000
#define MIN_STREAM_MS 250
#define MAX_STREAM_MS 60000

// The stream stops itself, for the same reason the heater does: forgetting is
// the obvious mistake, and the cost of it here is that the console fills with a
// line a second. Typing `{"cmd":"stream","on":false}` into a serial monitor that
// is scrolling is exactly the situation an auto-stop exists to avoid -- the
// first version of this command ran forever and that is how it was found.
//
// Two minutes is long enough to breathe on the probe, watch the humidity climb
// and fall back, and run a heater cycle without touching the keyboard. `ms:0`
// still means "until told otherwise", for a deliberate long recording.
#define DEFAULT_STREAM_TOTAL_MS 120000
#define MAX_STREAM_TOTAL_MS 3600000

// The heater is a diagnostic, not a feature, and it BIASES the humidity reading
// and warms the die by several degrees. Leaving it on silently corrupts every
// subsequent measurement, and forgetting to turn it off is the obvious mistake
// to make on a bench at 11pm -- so it turns itself off. Every measurement
// response also carries `heater`, so a reading taken while it was on can never
// be mistaken for an ambient one after the fact.
#define DEFAULT_HEATER_MS 10000
#define MAX_HEATER_MS 30000

// --- Self-test ---------------------------------------------------------------
// The heater test, done by the board instead of by eye.
//
// It was originally a human procedure: turn the heater on, watch the stream,
// satisfy yourself that temperature rises while humidity falls. That works on a
// bare breakout and fails on a POTTED PROBE, where ~33 mW goes into a housing
// with a hundred times the thermal mass of the die: the rise is real but small
// and slow, and "difficult to be sure" is the honest verdict a person reaches.
//
// The mistake was asking a person to judge a trend in a scrolling console. The
// SHT31's measurement noise is ~0.02 C, so a 0.5 C rise is twenty-five times the
// noise -- overwhelming evidence, and invisible to the eye. So the board takes a
// baseline, heats, re-measures, and reports the DIFFERENCE against the spread of
// its own baseline. Signal against noise is what makes a small number
// conclusive; absolute degrees never were the criterion.
#define SELFTEST_DEFAULT_MS 20000
#define SELFTEST_N 5             // samples per baseline / hot point
#define SELFTEST_MIN_D_TEMP 0.5f // C, for an unqualified pass
#define SELFTEST_MIN_D_RH 1.0f   // %RH of FALL, for an unqualified pass
#define SELFTEST_MIN_SNR 10.0f   // d_temp / baseline spread, for a pass on noise alone
#define SELFTEST_PROGRESS_MS 2000

// Health is re-checked in the background at this interval so the status LED
// means something on a board nobody is talking to. Silent -- no line is emitted;
// only `stream` puts events on the wire.
#define HEALTH_POLL_MS 5000

#define LINE_MAX 192  // *request* line cap; responses are unbounded

// --- Status LED --------------------------------------------------------------
// THE XIAO RP2040 HAS FOUR USER-VISIBLE LEDS and every one has to be accounted
// for -- an uninitialised pin is an INPUT, these LEDs are tied to 3V3, and a
// floating cathode glows. Four LEDs competing is how an indicator stops being
// one. Same reasoning, and the same fix, as src/pump/pump_sensor.cpp.
//
//   GPIO17 / GPIO16 / GPIO25   red / green / blue dice of the 3-in-1 user LED
//   GPIO12 + GPIO11            WS2812 "NeoPixel", data + power enable
//
// ACTIVE LOW -- the pin is pulled low to light the die.
#define LED_R_PIN PIN_LED_R  // GPIO17
#define LED_G_PIN PIN_LED_G  // GPIO16
#define LED_B_PIN PIN_LED_B  // GPIO25
#define LED_ON LOW
#define LED_OFF HIGH

// What the colours mean:
//
//   green, 60 ms blip every 3 s   last read was good, board alive
//   red,   10 Hz                  last read FAILED -- wiring, address, pull-ups
//   blue,  2 Hz                   nothing read successfully yet
//   blue,  solid                  heater on: readings are deliberately biased
//   dark                          THE BOARD IS NOT RUNNING
//
// This is the whole reason for the background health poll. On a bench rig the
// LED answers "did I wire it right?" before the serial monitor is even open --
// which matters on native USB, where anything printed at boot is gone before the
// host attaches. Green blipping means the sensor is answering with valid CRCs;
// red means go look at the wires. `off` is a blip rather than dark for the same
// reason the pump board's is: dark has to mean one thing, and it means the board
// is not running.
#define LED_UNKNOWN_MS 250   // half-period -> 2 Hz
#define LED_FAULT_MS 50      // half-period -> 10 Hz
#define LED_ALIVE_PERIOD_MS 3000
#define LED_ALIVE_FLASH_MS 60

enum LedColour { LED_NONE = 0, LED_RED, LED_GREEN, LED_BLUE };

// --- Measurement types -------------------------------------------------------

// How one sample turned out. The four failures are kept apart because they send
// you to four different places -- see the header comment.
//
// SAMPLE_NACK      nobody acknowledged the address. Sensor absent, unpowered,
//                  on the other address, or SDA/SCL swapped.
// SAMPLE_BUS       the transaction timed out: a line is being held low. A wire
//                  is half in, or a slave is stuck mid-byte.
// SAMPLE_SHORT     addressed fine, returned fewer than 6 bytes -- still busy,
//                  or the read was cut off.
// SAMPLE_CRC       6 bytes arrived and at least one CRC failed. The sensor is
//                  THERE and the signal is corrupt: leads, pull-ups, speed.
enum SampleStatus : uint8_t { SAMPLE_OK, SAMPLE_CRC, SAMPLE_NACK, SAMPLE_BUS, SAMPLE_SHORT };

struct BurstParams {
  int n;
  uint8_t addr;
  uint16_t cmd;             // one of CMD_MEAS_*
  unsigned long wait_ms;    // conversion wait for that repeatability
  unsigned long gap_ms;     // spacing between samples
  const char *rep;          // echoed back: "high" | "medium" | "low"
};

// One measurement point of the self-test: the medians, plus the spread of the
// samples behind them. The spread is what makes the verdict defensible -- a
// temperature difference only means something relative to the noise it had to
// climb out of.
struct EnvPoint {
  float temp_c, rh_pct;
  float temp_spread;
  int n_valid;
};

// What one bus line looks like electrically. `held_low` is a separate field
// rather than a magic value of rise_us, because "never rose" and "rose too fast
// to time" are opposite findings and a shared sentinel makes them one.
struct LineProbe {
  int idle;             // resting level under the internal pull-up: 1 healthy, 0 held low
  bool held_low;        // never rose even with the internal pull-up engaged
  bool ext_pullup;      // rose with NO pull at all -- something else holds this line up
  float rise_us;        // internal pull-up engaged; meaningless when held_low
  float rise_free_us;   // no pull at all; meaningless unless ext_pullup
};

struct EnvBurst {
  int n;
  uint16_t t_raw[MAX_N], rh_raw[MAX_N];
  float temp_c[MAX_N], rh_pct[MAX_N];
  SampleStatus status[MAX_N];
  // always: n == n_valid + n_crc + n_nack + n_bus + n_short
  int n_valid, n_crc, n_nack, n_bus, n_short;
  float temp_median, rh_median;
  float temp_min, temp_max, rh_min, rh_max;
  uint16_t t_raw_median, rh_raw_median;
};

// --- State -------------------------------------------------------------------
// A bench rig is allowed the state a deployed board would not be, but only the
// state a HUMAN AT THE BENCH needs: what the LED is showing, whether the heater
// is counting down, and whether streaming is on. Nothing is persisted, nothing
// survives a reset, and nothing here changes what a measurement returns.
enum Health { HEALTH_UNKNOWN, HEALTH_OK, HEALTH_FAULT };
static Health g_health = HEALTH_UNKNOWN;

static uint8_t g_addr = DEFAULT_ADDR;  // last address a command used; status reports it
static uint32_t g_i2c_hz = DEFAULT_I2C_HZ;
static bool g_heater_on = false;
static unsigned long g_heater_off_at = 0;
static bool g_stream_on = false;
static unsigned long g_stream_ms = DEFAULT_STREAM_MS;
static int g_stream_n = 1;
static unsigned long g_next_stream_at = 0;
static unsigned long g_stream_off_at = 0;  // 0 = runs until told otherwise

// THE ONE PLACE THIS FIRMWARE EMITS SOMETHING THAT IS NOT JSON.
//
// The NDJSON contract is what the Pi reads and it is not weakened here: nothing
// turns this on but a person typing `"fmt":"text"`, the Pi never sends it, and a
// reset returns to JSON. What it buys is the thing JSON is worst at -- being
// read by eye, once a second, while you hold a probe in an ice bath. Columns of
// numbers are legible in a way that a 300-character object never will be, and
// pretending otherwise just means the bench work happens in a text editor
// instead.
static bool g_stream_text = false;
static unsigned long g_last_read_at = 0;
static uint32_t g_seq = 0;

static LedColour g_led_shown = LED_NONE;

// --- Request line buffer -----------------------------------------------------
// Fixed size, filled incrementally in loop(). No String, so a garbage stream
// cannot grow the heap, and no reliance on the serial read timeout.
static char lineBuf[LINE_MAX];
static size_t lineLen = 0;
static bool lineOverflow = false;

// --- Prototypes --------------------------------------------------------------
bool hasContent();
void handleLine(const char *line);
void handleReadEnv(JsonVariantConst id, JsonVariantConst req);
void handleStream(JsonVariantConst id, JsonVariantConst req);
void handleHeater(JsonVariantConst id, JsonVariantConst req);
void handleScan(JsonVariantConst id, JsonVariantConst req);
void handleSelftest(JsonVariantConst id, JsonVariantConst req);
bool samplePoint(uint8_t addr, int n, EnvPoint *out);
void handleBus(JsonVariantConst id, JsonVariantConst req);
void handleLines(JsonVariantConst id, JsonVariantConst req);
void probeLine(pin_size_t pin, LineProbe *r);
float estPf(const LineProbe *p, float r_ext_known);
float rcNs(const LineProbe *p);
void addLines(JsonDocument &doc, const LineProbe *sda, const LineProbe *scl,
              float r_ext_known);
void emitLinesEvent(int i, int n, const LineProbe *sda, const LineProbe *scl);
const char *lineState(const LineProbe *p);
bool bigChange(float now, float before);
void handleReset(JsonVariantConst id, JsonVariantConst req);
int scanBus(uint8_t *out, int max);
void handleStatus(JsonVariantConst id, JsonVariantConst req);
bool optInt(JsonVariantConst req, const char *key, int *out, const char **bad_field);
bool optULong(JsonVariantConst req, const char *key, unsigned long *out, const char **bad_field);
bool optBool(JsonVariantConst req, const char *key, bool *out, const char **bad_field);
bool parseAddr(JsonVariantConst req, uint8_t *out, const char **bad_field);
bool parseParams(JsonVariantConst req, BurstParams *p, const char **bad_field);
void beginResponse(JsonDocument &doc, JsonVariantConst id);
void addBurst(JsonDocument &doc, const EnvBurst *b, const BurstParams *p);
const char *burstErrorCode(const EnvBurst *b);
void sendError(const JsonVariantConst *id, const char *code, const char *field);
void sendResponse(const JsonDocument &doc);
void emitEnvEvent(const EnvBurst *b, const BurstParams *p);
void emitEnvText(const EnvBurst *b);
void printFixed(float v, int decimals, int width);
void emitHeaterEvent(bool on, const char *reason);
uint8_t writeCommand(uint8_t addr, uint16_t cmd);
bool readWords(uint8_t addr, uint16_t *out, int words, SampleStatus *why);
bool readStatusRegister(uint8_t addr, uint16_t *reg, SampleStatus *why);
void measure(EnvBurst *b, const BurstParams *p);
void summarize(EnvBurst *b);
void noteHealth(const EnvBurst *b);
void serviceHeater();
void serviceStream();
void serviceHealth();
void ledBegin();
void ledWrite(LedColour c);
void updateLed();
uint8_t crc8(const uint8_t *data, size_t len);
float tempCFromRaw(uint16_t raw);
float rhPctFromRaw(uint16_t raw);
float findMedian(float arr[], int n);
int compareFloat(const void *a, const void *b);
uint16_t findMedianU16(uint16_t arr[], int n);
int compareU16(const void *a, const void *b);

void setup() {
  Serial.begin(9600);

  ledBegin();

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(g_i2c_hz);
  Wire.setTimeout(I2C_TIMEOUT_MS, true);  // see I2C_TIMEOUT_MS: bounded, self-recovering

  // Boot banner: structured "ready" line, same shape as the other two boards.
  JsonDocument doc;
  doc["type"] = "ready";
  doc["proto"] = PROTO_VERSION;
  doc["fw"] = FW_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  serializeJson(doc, Serial);
  Serial.println();

  // Self-test, silently. On native USB the banner above is very likely lost --
  // the host has not attached yet -- so the first thing anyone actually sees is
  // the LED, and it has to mean something by the time they look. Nothing is
  // printed: the wire stays clean for the Pi, and `status` reports the same
  // facts on demand.
  BurstParams p;
  const char *unused = nullptr;
  parseParams(JsonVariantConst(), &p, &unused);
  EnvBurst b;
  measure(&b, &p);
  noteHealth(&b);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      if (lineOverflow) {
        sendError(nullptr, "line_too_long", nullptr);
      } else if (hasContent()) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      }
      lineLen = 0;
      lineOverflow = false;
      continue;
    }

    if (c == '\r') {
      continue;  // tolerate CRLF senders
    }
    if (lineLen >= LINE_MAX - 1) {
      lineOverflow = true;  // keep draining until the newline, then report once
      continue;
    }
    lineBuf[lineLen++] = c;
  }

  serviceHeater();
  serviceStream();
  serviceHealth();
  updateLed();
}

// True if the buffer holds anything but whitespace. Blank lines (and bare
// newlines used as keepalives) are ignored rather than answered with an error.
bool hasContent() {
  for (size_t i = 0; i < lineLen; i++) {
    if (!isspace((unsigned char)lineBuf[i])) {
      return true;
    }
  }
  return false;
}

// Parse one request line and dispatch to the matching handler.
void handleLine(const char *line) {
  JsonDocument req;
  DeserializationError err = deserializeJson(req, line);
  if (err) {
    sendError(nullptr, "bad_request", nullptr);
    return;
  }

  // The id is echoed back verbatim, so it must be an integer: a missing or
  // mistyped id is reported as bad_id rather than silently answered with
  // "id":null, which the Pi could not tell apart from bad_request.
  JsonVariantConst id = req["id"];
  if (!id.is<long>()) {
    sendError(nullptr, "bad_id", nullptr);
    return;
  }

  const char *cmd = req["cmd"] | "";
  if (strcmp(cmd, "read_env") == 0) {
    handleReadEnv(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "stream") == 0) {
    handleStream(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "heater") == 0) {
    handleHeater(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "selftest") == 0) {
    handleSelftest(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "scan") == 0) {
    handleScan(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "bus") == 0) {
    handleBus(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "lines") == 0) {
    handleLines(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "reset") == 0) {
    handleReset(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "status") == 0) {
    handleStatus(id, req.as<JsonVariantConst>());
  } else {
    sendError(&id, "unknown_cmd", nullptr);
  }
}

// --- Command handlers --------------------------------------------------------

// The measurement command. Everything it can be asked to do differently is a
// request parameter (n, addr, rep, gap_ms); there is no second command and no
// mode flag, and the per-sample detail is on every reply rather than behind one.
// That is the lesson puit learned the expensive way when `sampling` and
// `read_puit` had to be merged: a split like that keeps the raw samples out of
// the measurement path, and then the consumer aggregates aggregates.
void handleReadEnv(JsonVariantConst id, JsonVariantConst req) {
  BurstParams p;
  const char *bad_field = nullptr;
  if (!parseParams(req, &p, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  EnvBurst b;
  measure(&b, &p);
  noteHealth(&b);

  JsonDocument doc;
  beginResponse(doc, id);
  if (b.n_valid == 0) {
    doc["status"] = "error";
    doc["code"] = burstErrorCode(&b);
    addBurst(doc, &b, &p);
    sendResponse(doc);
    return;
  }

  doc["status"] = "ok";
  doc["temp_c"] = b.temp_median;
  doc["rh_pct"] = b.rh_median;
  doc["unit_temp"] = "C";
  doc["unit_rh"] = "%";
  // Raw counterparts of the two values above, for the same reason puit stores
  // pulse_us next to its cm: the ticks are what the sensor actually produced and
  // the only part that cannot go stale. Both conversions are monotonic, so these
  // are the exact raw counterparts of the medians -- no second sort policy to
  // keep in step. If a conversion ever turns out to be wrong, history recorded
  // with these can be replayed; history recorded without them cannot.
  doc["t_raw"] = b.t_raw_median;
  doc["rh_raw"] = b.rh_raw_median;
  addBurst(doc, &b, &p);
  sendResponse(doc);
}

// Turn the unsolicited env event stream on or off. This is the command that
// makes the rig useful by hand: breathe on the probe and watch RH move.
//
// Streaming is OFF at boot, deliberately. A board that starts talking the moment
// it is powered would race the Pi's banner handshake, and a bench convenience is
// not worth making the wire less predictable.
void handleStream(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  bool on = true;
  unsigned long period = g_stream_ms;
  int n = g_stream_n;
  unsigned long ms = DEFAULT_STREAM_TOTAL_MS;

  if (!optBool(req, "on", &on, &bad_field) ||
      !optULong(req, "period_ms", &period, &bad_field) ||
      !optInt(req, "n", &n, &bad_field) ||
      !optULong(req, "ms", &ms, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  ms = constrain(ms, 0UL, (unsigned long)MAX_STREAM_TOTAL_MS);

  // "json" (default) or "text". An unrecognised value is an error rather than a
  // fallback: silently emitting the format the caller did not ask for is worse
  // than refusing, since one of the two is unparseable to the other reader.
  JsonVariantConst fmt = req["fmt"];
  if (!fmt.isNull()) {
    const char *s = fmt.as<const char *>();
    if (s == nullptr || (strcmp(s, "json") != 0 && strcmp(s, "text") != 0)) {
      sendError(&id, "bad_param", "fmt");
      return;
    }
    g_stream_text = (strcmp(s, "text") == 0);
  }

  g_stream_ms = constrain(period, MIN_STREAM_MS, MAX_STREAM_MS);
  g_stream_n = constrain(n, 1, MAX_N);
  g_stream_on = on;
  g_next_stream_at = millis();  // first event immediately, so `on` is visibly on
  g_stream_off_at = (on && ms > 0) ? millis() + ms : 0;

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["stream"] = g_stream_on;
  doc["period_ms"] = g_stream_ms;
  doc["n"] = g_stream_n;
  doc["fmt"] = g_stream_text ? "text" : "json";
  doc["off_in_ms"] = g_stream_off_at ? ms : 0;  // 0 = until told otherwise
  sendResponse(doc);

  // Column headings, once, after the response so the reply above stays the last
  // parseable line for anyone who did send this from a program.
  if (on && g_stream_text) {
    Serial.println();
    Serial.println("      temp     humidity");
  }
}

// Drive the SHT31's internal heater.
//
// THIS IS THE DEFINITIVE "is the sensor real?" TEST, and it is why the command
// exists on a rig whose whole purpose is qualifying a module. A sensor that is
// merely echoing a plausible constant -- a counterfeit, a latched word, a
// crossed wire reading some other device -- cannot respond to it. Turn the
// heater on and the temperature must climb a few degrees over several seconds
// and the relative humidity must fall as it does (warmer air, same absolute
// water). Both moving, together, in the right directions, is proof no static
// reading can give you.
//
// It turns itself off after ms (default 10 s, max 30 s) -- see DEFAULT_HEATER_MS
// for why that is not optional.
void handleHeater(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  bool on = true;
  unsigned long ms = DEFAULT_HEATER_MS;
  uint8_t addr = DEFAULT_ADDR;

  if (!optBool(req, "on", &on, &bad_field) ||
      !optULong(req, "ms", &ms, &bad_field) ||
      !parseAddr(req, &addr, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  ms = constrain(ms, 0UL, (unsigned long)MAX_HEATER_MS);
  g_addr = addr;

  uint8_t err = writeCommand(addr, on ? CMD_HEATER_ON : CMD_HEATER_OFF);
  if (err != 0) {
    // Reported rather than assumed: with no ack the heater state is unknown, and
    // pretending the request took effect would leave the LED and every later
    // `heater` field lying about the bias on the readings.
    sendError(&id, err == 5 ? "bus_error" : "sensor_fault", nullptr);
    g_health = HEALTH_FAULT;
    return;
  }

  g_heater_on = on;
  g_heater_off_at = (on && ms > 0) ? millis() + ms : 0;

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["heater"] = g_heater_on;
  doc["off_in_ms"] = g_heater_off_at ? ms : 0;  // 0 = stays on until told otherwise
  doc["addr"] = addr;
  sendResponse(doc);
}

// Walk the bus and report every address that acknowledges.
//
// The first command to run on a new module, and the one that separates the two
// bringup failures people conflate: nothing at all in `found` means the sensor
// is absent, unpowered, or the data lines are swapped; 0x45 in `found` when you
// expected 0x44 means it is wired fine and the ADR pin is tied the other way.
// Neither is visible from a failed read_env, which reports both as sensor_fault.
// Probe every unreserved 7-bit address once at the current bus speed. Returns
// how many acknowledged and fills `out` with them.
//
// A zero-length write is the standard probe: the core special-cases it and only
// the address phase goes out, so nothing is written to whatever answers.
int scanBus(uint8_t *out, int max) {
  int n = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0 && n < max) {
      out[n++] = a;
    }
  }
  return n;
}

void handleScan(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  bool sweep = false;
  unsigned long hz = g_i2c_hz;
  if (!optBool(req, "sweep", &sweep, &bad_field) ||
      !optULong(req, "i2c_hz", &hz, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  hz = constrain(hz, (unsigned long)MIN_I2C_HZ, (unsigned long)MAX_I2C_HZ);

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";

  uint8_t found[112];  // every unreserved address, so a bus that ACKs
                       // everything (SDA stuck low) is reported rather than
                       // silently truncated -- that pattern IS the diagnosis
  if (sweep) {
    // The rise-time ladder. A sensor that appears only at the bottom of it is a
    // sensor on a bus whose edges are too slow, which on this rig means the
    // 3 m cable against pull-ups too weak for it -- see DEFAULT_I2C_HZ. A
    // sensor that appears at every speed, or at none, is not an edge problem
    // and the wiring is where to look instead.
    static const uint32_t ladder[] = {400000, 200000, 100000, 50000, 20000, 10000, 5000, 2000};
    JsonArray rungs = doc["sweep"].to<JsonArray>();
    uint32_t slowest_ok = 0, fastest_ok = 0;

    for (size_t i = 0; i < sizeof(ladder) / sizeof(ladder[0]); i++) {
      Wire.setClock(ladder[i]);
      int n = scanBus(found, (int)sizeof(found));
      JsonObject rung = rungs.add<JsonObject>();
      rung["i2c_hz"] = ladder[i];
      rung["n_found"] = n;
      JsonArray f = rung["found"].to<JsonArray>();
      for (int k = 0; k < n; k++) {
        f.add(found[k]);
      }
      if (n > 0) {
        if (fastest_ok == 0) fastest_ok = ladder[i];  // ladder runs fast -> slow
        slowest_ok = ladder[i];
      }
    }

    Wire.setClock(g_i2c_hz);  // the sweep is a probe, not a setting
    doc["i2c_hz"] = g_i2c_hz;
    doc["any_found"] = fastest_ok != 0;
    if (fastest_ok != 0) {
      doc["fastest_ok_hz"] = fastest_ok;
      doc["slowest_ok_hz"] = slowest_ok;
    }
    sendResponse(doc);
    return;
  }

  Wire.setClock(hz);
  int n = scanBus(found, (int)sizeof(found));
  Wire.setClock(g_i2c_hz);  // a one-shot i2c_hz does not become the setting

  JsonArray f = doc["found"].to<JsonArray>();
  JsonArray f_hex = doc["found_hex"].to<JsonArray>();
  bool expected_present = false;
  for (int k = 0; k < n; k++) {
    f.add(found[k]);
    char hex[6];
    snprintf(hex, sizeof(hex), "0x%02X", found[k]);
    f_hex.add(hex);  // copied into the document, not referenced
    if (found[k] == g_addr) {
      expected_present = true;
    }
  }

  // found_hex is redundant with found and is here anyway: this reply is read by
  // a person in a serial monitor as often as by the Pi, and every SHT31
  // datasheet, silkscreen and forum post says 0x44 while JSON says 68.
  doc["n_found"] = n;
  doc["i2c_hz"] = hz;
  doc["addr"] = g_addr;
  doc["addr_present"] = expected_present;
  sendResponse(doc);
}

// One burst, reduced to medians plus the spread behind them.
bool samplePoint(uint8_t addr, int n, EnvPoint *out) {
  BurstParams p;
  const char *unused = nullptr;
  parseParams(JsonVariantConst(), &p, &unused);
  p.n = constrain(n, 1, MAX_N);
  p.addr = addr;
  p.gap_ms = 200;

  EnvBurst b;
  measure(&b, &p);
  noteHealth(&b);
  if (b.n_valid == 0) {
    return false;
  }
  out->temp_c = b.temp_median;
  out->rh_pct = b.rh_median;
  out->temp_spread = b.temp_max - b.temp_min;
  out->n_valid = b.n_valid;
  return true;
}

// The heater test, run and judged by the board. See SELFTEST_DEFAULT_MS.
//
// This BLOCKS for the duration (20 s by default) and does not service serial
// while it runs, which is acceptable on a bench rig and is why it emits progress
// events -- a console that goes silent for twenty seconds reads as a crash.
//
// The heater is turned off on every exit path, including the failures. Leaving
// it on after a test that could not complete would bias every later reading with
// nothing on the wire to say so.
void handleSelftest(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  uint8_t addr = DEFAULT_ADDR;
  unsigned long ms = SELFTEST_DEFAULT_MS;
  if (!parseAddr(req, &addr, &bad_field) ||
      !optULong(req, "ms", &ms, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  ms = constrain(ms, 1000UL, (unsigned long)MAX_HEATER_MS);
  g_addr = addr;

  EnvPoint base;
  if (!samplePoint(addr, SELFTEST_N, &base)) {
    sendError(&id, "sensor_fault", nullptr);
    return;
  }

  if (writeCommand(addr, CMD_HEATER_ON) != 0) {
    sendError(&id, "sensor_fault", nullptr);
    g_health = HEALTH_FAULT;
    return;
  }
  g_heater_on = true;
  g_heater_off_at = 0;  // this function owns the heater for the duration

  unsigned long t0 = millis();
  unsigned long next = t0;
  while ((long)(millis() - (t0 + ms)) < 0) {
    if ((long)(millis() - next) >= 0) {
      next = millis() + SELFTEST_PROGRESS_MS;
      EnvPoint now;
      if (samplePoint(addr, 1, &now)) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["proto"] = PROTO_VERSION;
        doc["role"] = PIJARDIN_ROLE;
        doc["ev"] = "selftest";
        doc["seq"] = ++g_seq;
        doc["t_ms"] = millis() - t0;
        doc["temp_c"] = now.temp_c;
        doc["rh_pct"] = now.rh_pct;
        doc["d_temp_c"] = now.temp_c - base.temp_c;
        doc["d_rh_pct"] = now.rh_pct - base.rh_pct;
        sendResponse(doc);
      }
    }
  }

  EnvPoint hot;
  bool hot_ok = samplePoint(addr, SELFTEST_N, &hot);

  bool off_ok = writeCommand(addr, CMD_HEATER_OFF) == 0;
  g_heater_on = !off_ok;  // if the off did not land, keep saying the readings are biased

  JsonDocument doc;
  beginResponse(doc, id);
  if (!hot_ok) {
    doc["status"] = "error";
    doc["code"] = "sensor_fault";
    doc["heater_off"] = off_ok;
    sendResponse(doc);
    return;
  }

  float d_temp = hot.temp_c - base.temp_c;
  float d_rh = hot.rh_pct - base.rh_pct;
  // Signal against noise. The baseline spread is the only estimate of the
  // sensor's own scatter that this test has, and it is measured rather than
  // assumed -- floored only to keep the division finite.
  float noise = base.temp_spread > 0.01f ? base.temp_spread : 0.01f;
  float snr = d_temp / noise;

  // Temperature UP and humidity DOWN together is the part no broken, counterfeit
  // or miswired module can fake: warmer air holding the same absolute water must
  // read a lower relative humidity. Either one alone proves much less.
  bool direction = (d_temp > 0.0f && d_rh < 0.0f);
  const char *verdict;
  if (direction && d_temp >= SELFTEST_MIN_D_TEMP && -d_rh >= SELFTEST_MIN_D_RH) {
    verdict = "pass";
  } else if (direction && (d_temp >= 0.2f || snr >= SELFTEST_MIN_SNR)) {
    verdict = "weak";  // right direction, small amplitude -- a potted probe
  } else {
    verdict = "fail";
  }

  doc["status"] = "ok";
  doc["verdict"] = verdict;
  doc["d_temp_c"] = d_temp;
  doc["d_rh_pct"] = d_rh;
  doc["snr"] = snr;  // d_temp in units of the baseline's own scatter
  doc["base_temp_c"] = base.temp_c;
  doc["base_rh_pct"] = base.rh_pct;
  doc["base_spread_c"] = base.temp_spread;
  doc["hot_temp_c"] = hot.temp_c;
  doc["hot_rh_pct"] = hot.rh_pct;
  doc["heat_ms"] = ms;
  doc["n"] = SELFTEST_N;
  doc["addr"] = addr;
  doc["heater_off"] = off_ok;
  sendResponse(doc);
}

// Set the bus speed for the session. See DEFAULT_I2C_HZ for why this is on the
// wire rather than a constant: on a long cable the speed is the difference
// between a sensor that answers and one that looks absent, and being able to
// change it without a reflash is what turns that from a theory into a test.
//
// Not persisted -- a reset returns to DEFAULT_I2C_HZ. A slow bus is a diagnosis,
// and one that survived a power cycle would quietly become the configuration,
// hiding the very fault it was set to reveal.
void handleBus(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  unsigned long hz = g_i2c_hz;
  if (!optULong(req, "i2c_hz", &hz, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  g_i2c_hz = (uint32_t)constrain(hz, (unsigned long)MIN_I2C_HZ, (unsigned long)MAX_I2C_HZ);
  Wire.setClock(g_i2c_hz);

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["i2c_hz"] = g_i2c_hz;
  doc["i2c_hz_default"] = DEFAULT_I2C_HZ;
  sendResponse(doc);
}

// Pull one line low, release it, and time how long it takes to rise under the
// pad's internal pull-up. Also reports the level the line idles at.
//
// The rise time is R*C with R known, so this MEASURES the bus capacitance --
// which is to say it measures whether anything is actually attached to the pin.
// A bare pin carries a few pF and snaps up in a microsecond or two; three metres
// of shielded cable carries a few hundred pF and takes tens of microseconds.
// Those are an order of magnitude apart, so the answer is never ambiguous.
//
// This exists because "the wire is not making contact" and "the sensor is not
// answering" are indistinguishable from every other command on this board -- and
// on a potted probe on a 3 m lead, a meter cannot easily tell them apart either.
// Same instinct as puit measuring the trigger->echo latency instead of inferring
// a dead sensor from a zero: the assumption that has no ground truth behind it is
// the one worth measuring.
// TWO passes, and the second one exists because the first cannot tell R from C.
//
// Pass 1 releases the line with the internal pull-up engaged, so the rise is
// 1.2 * (R_int || R_ext) * C. With no external pull-up that is 1.2 * 65k * C and
// the capacitance falls straight out. But fit a 2.2k resistor and the same
// capacitance now rises THIRTY TIMES faster -- so a heavily pulled-up bus with a
// long cable reads exactly like a bare pin, and `est_pf` computed against 65k
// reports ~1 pF for a line carrying 300.
//
// Pass 2 releases the line with NO pull at all. A line with nothing on it stays
// down; a line that still snaps high has something else holding it up -- a real
// pull-up, or a device driving it. That single bit is what stops the pass 1
// number from being read as capacitance when it is really resistance, and it is
// exactly the ambiguity that made a first field reading uninterpretable.
void probeLine(pin_size_t pin, LineProbe *r) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(500);  // let the line settle before reading its resting state
  r->idle = digitalRead(pin);
  r->held_low = false;
  r->ext_pullup = false;
  r->rise_us = 0.0f;
  r->rise_free_us = 0.0f;

  // --- Pass 1: internal pull-up engaged -------------------------------------
  unsigned long total = 0;
  for (int i = 0; i < RISE_REPS; i++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(DISCHARGE_US);  // fully discharge whatever hangs off the pin
    pinMode(pin, INPUT_PULLUP);

    unsigned long t0 = micros();
    while (digitalRead(pin) == LOW) {
      if (micros() - t0 > RISE_TIMEOUT_US) {
        r->held_low = true;  // never rose: something is clamping the line
        pinMode(pin, INPUT_PULLUP);
        return;
      }
    }
    total += micros() - t0;
  }
  r->rise_us = (float)total / RISE_REPS;

  // --- Pass 2: no pull at all -----------------------------------------------
  total = 0;
  int risen = 0;
  for (int i = 0; i < RISE_REPS; i++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(DISCHARGE_US);
    pinMode(pin, INPUT);  // gpio_disable_pulls -- verified in the core

    unsigned long t0 = micros();
    bool up = false;
    while (micros() - t0 <= FREE_TIMEOUT_US) {
      if (digitalRead(pin) != LOW) {
        up = true;
        break;
      }
    }
    if (!up) {
      break;  // one line that stays down is conclusive; no need for 64 of them
    }
    total += micros() - t0;
    risen++;
  }
  if (risen == RISE_REPS) {
    r->ext_pullup = true;
    r->rise_free_us = (float)total / RISE_REPS;
  }

  pinMode(pin, INPUT_PULLUP);  // park the line high rather than floating
}

// Capacitance, when it is derivable at all. Returns < 0 for "cannot be known
// from this measurement", which is a real outcome rather than a failure: with an
// external pull-up of unknown value, the rise time fixes the PRODUCT R*C and
// nothing more. Pass the fitted resistor as `pullup_ohms` and it resolves.
float estPf(const LineProbe *p, float r_ext_known) {
  if (p->held_low) {
    return -1.0f;
  }
  if (p->ext_pullup) {
    if (r_ext_known <= 0.0f) {
      return -1.0f;
    }
    return p->rise_free_us * 1e-6f / (1.2f * r_ext_known) * 1e12f;
  }
  return p->rise_us * 1e-6f / (1.2f * PULLUP_OHMS) * 1e12f;
}

// The R*C product in nanoseconds -- always reportable, since it is what was
// actually measured. This is also the number that decides whether the bus works:
// I2C wants the rise inside 1 us at 100 kHz, whatever combination of resistance
// and capacitance produces it.
float rcNs(const LineProbe *p) {
  if (p->held_low) {
    return -1.0f;
  }
  float t_us = p->ext_pullup ? p->rise_free_us : p->rise_us;
  return t_us * 1000.0f / 1.2f;
}

// Characterise the two bus lines electrically, with the I2C peripheral out of
// the way. The one command that can tell "nothing is connected to this pin" from
// "something is connected and not answering".
//
// `n` > 1 repeats the probe and emits each round as an event BEFORE the final
// response, which turns it into a live continuity tester: start it, wiggle a
// wire, and watch rise_us jump between the bare-pad floor and the cable's real
// figure. That is how an intermittent contact is found, and it is the one fault
// no single-shot measurement can catch.
void handleLines(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  int n = 1;
  unsigned long gap_ms = 250;
  // The value of any pull-up resistor fitted on the bus. Asked for rather than
  // measured because it cannot be measured from here: with an external pull-up
  // the rise time fixes only the product R*C. Tell the board what was soldered
  // and the capacitance falls out; say nothing and est_pf is honestly null.
  unsigned long pullup_ohms = 0;
  bool all = false;  // emit every round rather than only the changes
  if (!optInt(req, "n", &n, &bad_field) ||
      !optULong(req, "gap_ms", &gap_ms, &bad_field) ||
      !optULong(req, "pullup_ohms", &pullup_ohms, &bad_field) ||
      !optBool(req, "all", &all, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  n = constrain(n, 1, 200);
  gap_ms = constrain(gap_ms, 50UL, 2000UL);

  Wire.end();  // take the pins back from the I2C block for the duration

  // Only the CHANGES get a line, plus the first round and a keepalive every 16.
  // A wire that is not being touched produces silence, so the one line that
  // appears when a contact makes or breaks is unmissable -- which is the entire
  // point of watching. `"all":true` restores every round for a recording.
  LineProbe sda, scl;
  const char *last_sda = nullptr;
  const char *last_scl = nullptr;
  float last_sda_us = -1.0f, last_scl_us = -1.0f;

  for (int i = 0; i < n; i++) {
    if (i > 0) {
      delay(gap_ms);
    }
    probeLine(I2C_SDA_PIN, &sda);
    probeLine(I2C_SCL_PIN, &scl);

    if (n > 1) {
      const char *s_sda = lineState(&sda);
      const char *s_scl = lineState(&scl);
      bool changed = last_sda == nullptr ||
                     strcmp(s_sda, last_sda) != 0 || strcmp(s_scl, last_scl) != 0 ||
                     bigChange(sda.rise_us, last_sda_us) ||
                     bigChange(scl.rise_us, last_scl_us);
      if (all || changed || (i % 16) == 0) {
        emitLinesEvent(i, n, &sda, &scl);
        last_sda = s_sda;
        last_scl = s_scl;
        last_sda_us = sda.rise_us;
        last_scl_us = scl.rise_us;
      }
    }
  }

  // Hand the pins back exactly as setup() configured them.
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(g_i2c_hz);
  Wire.setTimeout(I2C_TIMEOUT_MS, true);

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["n"] = n;
  doc["gap_ms"] = gap_ms;
  doc["sda"] = lineState(&sda);
  doc["scl"] = lineState(&scl);
  addLines(doc, &sda, &scl, (float)pullup_ohms);  // LAST round, in full, once
  sendResponse(doc);
}

// The line figures, shared by the response and the per-round events.
//
// Written out per line rather than generated from a prefix on purpose:
// ArduinoJson stores a `const char *` key by pointer rather than copying it, so
// keys built into a reused buffer would all end up naming the last one written.
void addLines(JsonDocument &doc, const LineProbe *sda, const LineProbe *scl,
              float r_ext_known) {
  doc["sda_pin"] = (int)I2C_SDA_PIN;
  doc["scl_pin"] = (int)I2C_SCL_PIN;
  doc["sda_idle"] = sda->idle;
  doc["scl_idle"] = scl->idle;
  doc["sda_held_low"] = sda->held_low;
  doc["scl_held_low"] = scl->held_low;
  doc["sda_ext_pullup"] = sda->ext_pullup;
  doc["scl_ext_pullup"] = scl->ext_pullup;

  // null rather than 0 wherever a number was not measured: 0 would read as
  // "rose instantly", which is the opposite finding to "never rose" and to
  // "there is nothing pulling this line up at all".
  if (sda->held_low) {
    doc["sda_rise_us"] = nullptr;
  } else {
    doc["sda_rise_us"] = sda->rise_us;
  }
  if (scl->held_low) {
    doc["scl_rise_us"] = nullptr;
  } else {
    doc["scl_rise_us"] = scl->rise_us;
  }
  if (sda->ext_pullup) {
    doc["sda_rise_free_us"] = sda->rise_free_us;
  } else {
    doc["sda_rise_free_us"] = nullptr;
  }
  if (scl->ext_pullup) {
    doc["scl_rise_free_us"] = scl->rise_free_us;
  } else {
    doc["scl_rise_free_us"] = nullptr;
  }

  // R*C is what was measured; the capacitance is only derivable when R is known.
  float sda_rc = rcNs(sda), scl_rc = rcNs(scl);
  if (sda_rc >= 0.0f) doc["sda_rc_ns"] = sda_rc; else doc["sda_rc_ns"] = nullptr;
  if (scl_rc >= 0.0f) doc["scl_rc_ns"] = scl_rc; else doc["scl_rc_ns"] = nullptr;

  // t = 1.2*R*C to the input threshold, so C ~= t / (1.2*R). A convenience
  // beside the raw microseconds, never instead of them: R is known only to
  // +/-25%, the threshold is a Schmitt trigger, and RISE_FLOOR_US of the time is
  // the measurement path rather than the line. Order of magnitude, not value.
  float sda_pf = estPf(sda, r_ext_known), scl_pf = estPf(scl, r_ext_known);
  if (sda_pf >= 0.0f) doc["sda_est_pf"] = (int)sda_pf; else doc["sda_est_pf"] = nullptr;
  if (scl_pf >= 0.0f) doc["scl_est_pf"] = (int)scl_pf; else doc["scl_est_pf"] = nullptr;

  doc["int_pullup_ohms"] = (int)PULLUP_OHMS;
  if (r_ext_known > 0.0f) {
    doc["ext_pullup_ohms"] = (int)r_ext_known;  // as told by the request, not measured
  }
  doc["rise_floor_us"] = RISE_FLOOR_US;  // at or near this = nothing attached
  doc["rise_timeout_us"] = RISE_TIMEOUT_US;
  doc["free_timeout_us"] = FREE_TIMEOUT_US;
}

// Which of the five words describes this line. See LINE_NONE_US.
const char *lineState(const LineProbe *p) {
  if (p->held_low) return "low";
  if (p->ext_pullup) return "pulled";
  if (p->rise_us < LINE_NONE_US) return "none";
  if (p->rise_us < LINE_STUB_US) return "stub";
  return "cable";
}

// Whether two rise times differ enough to be worth a line on the console.
// Relative rather than absolute, so it works whatever the pull-up: 2.06 vs 2.25
// is the same contact measured twice, 2.1 vs 21 is a contact that just closed.
bool bigChange(float now, float before) {
  if (before < 0.0f) return true;  // nothing to compare against yet
  float hi = (now > before) ? now : before;
  float lo = (now > before) ? before : now;
  if (hi < LINE_NONE_US) return false;  // both sitting at nothing
  return (hi - lo) > 0.3f * hi;
}

// The compact per-round line, for a human watching while they move a wire.
//
// Deliberately NOT the full field set: the constants (pins, timeouts, the floor,
// the pull-up value) do not change between rounds, and repeating them forty
// times is what made the watch mode unreadable in practice. They are still on
// the final response, once, where they can actually be read.
void emitLinesEvent(int i, int n, const LineProbe *sda, const LineProbe *scl) {
  JsonDocument doc;
  doc["type"] = "event";
  doc["proto"] = PROTO_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["ev"] = "lines";
  doc["seq"] = ++g_seq;
  doc["i"] = i;
  doc["n"] = n;
  doc["sda"] = lineState(sda);
  doc["scl"] = lineState(scl);
  doc["sda_us"] = sda->held_low ? -1.0f : sda->rise_us;
  doc["scl_us"] = scl->held_low ? -1.0f : scl->rise_us;
  sendResponse(doc);
}

// Soft-reset the sensor and report what the status register says afterwards.
// The reset-detected bit coming back set is confirmation the reset actually
// reached the device rather than being swallowed by a dead bus.
void handleReset(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  uint8_t addr = DEFAULT_ADDR;
  if (!parseAddr(req, &addr, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }
  g_addr = addr;

  uint8_t err = writeCommand(addr, CMD_SOFT_RESET);
  if (err != 0) {
    sendError(&id, err == 5 ? "bus_error" : "sensor_fault", nullptr);
    g_health = HEALTH_FAULT;
    return;
  }
  delay(2);  // datasheet: 1.5 ms to be ready again

  // The reset clears the heater bit in the sensor, so the firmware's idea of it
  // has to be cleared too or `heater` would keep claiming a bias that is gone.
  g_heater_on = false;
  g_heater_off_at = 0;

  uint16_t reg = 0;
  SampleStatus why = SAMPLE_OK;
  bool ok = readStatusRegister(addr, &reg, &why);

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["addr"] = addr;
  doc["reg_read"] = ok;
  if (ok) {
    doc["reg"] = reg;
    doc["reset_detected"] = (reg & 0x0010) != 0;
  }
  sendResponse(doc);
}

// Identity, limits, and a LIVE look at the sensor -- deliberately not a cached
// one. "Is the board up?" and "is the sensor there?" are different questions and
// this answers both in one line, which is what makes it the right thing to send
// first when something looks wrong.
void handleStatus(JsonVariantConst id, JsonVariantConst req) {
  const char *bad_field = nullptr;
  uint8_t addr = g_addr;
  // `clear` wipes the sensor's status register AFTER reading it. Those bits are
  // sticky from power-up, which makes them useless as they stand: alert_pending
  // and reset_detected are both set on any fresh probe and stay set forever.
  // Clear them once and they become event flags -- a reset_detected that comes
  // back true afterwards means the probe actually lost power, which is exactly
  // what you want to know about a sensor at the bottom of a well.
  bool clear = false;
  if (!parseAddr(req, &addr, &bad_field) ||
      !optBool(req, "clear", &clear, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["fw"] = FW_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["uptime_ms"] = millis();

  doc["addr"] = addr;
  doc["sda_pin"] = (int)I2C_SDA_PIN;
  doc["scl_pin"] = (int)I2C_SCL_PIN;
  doc["i2c_hz"] = g_i2c_hz;
  doc["i2c_hz_default"] = DEFAULT_I2C_HZ;
  doc["heater"] = g_heater_on;
  doc["stream"] = g_stream_on;
  doc["period_ms"] = g_stream_ms;
  doc["health"] = (g_health == HEALTH_OK) ? "ok"
                  : (g_health == HEALTH_FAULT) ? "fault"
                                               : "unknown";

  uint16_t reg = 0;
  SampleStatus why = SAMPLE_OK;
  bool present = readStatusRegister(addr, &reg, &why);
  doc["sensor_present"] = present;
  if (present) {
    doc["reg"] = reg;
    // The bits worth having by name. `heater_reg` is the sensor's own account of
    // the heater and is reported next to this firmware's `heater` on purpose:
    // the two disagreeing means a command did not land, which is exactly the
    // kind of drift a status command exists to expose.
    doc["heater_reg"] = (reg & 0x2000) != 0;
    doc["reset_detected"] = (reg & 0x0010) != 0;
    doc["last_cmd_failed"] = (reg & 0x0002) != 0;
    doc["last_crc_failed"] = (reg & 0x0001) != 0;
    doc["alert_pending"] = (reg & 0x8000) != 0;
    // Reported only when asked for, and only after the values above have been
    // read out -- so the reply always describes the state that was cleared,
    // never an empty register.
    if (clear) {
      doc["cleared"] = writeCommand(addr, CMD_CLEAR_STATUS) == 0;
    }
  } else {
    doc["reg_error"] = (why == SAMPLE_NACK)  ? "nack"
                       : (why == SAMPLE_BUS) ? "bus"
                       : (why == SAMPLE_CRC) ? "crc"
                                             : "short";
  }

  // Limits and defaults, so the Pi can discover them instead of hardcoding a
  // second copy of these constants.
  doc["max_n"] = MAX_N;
  doc["n_default"] = DEFAULT_N;
  doc["gap_default_ms"] = DEFAULT_GAP_MS;
  doc["heater_max_ms"] = MAX_HEATER_MS;
  doc["line_max"] = LINE_MAX;
  sendResponse(doc);
}

// --- Request parameters ------------------------------------------------------

// Read one optional field. Absent (or explicitly null) leaves *out at its
// default; present but the wrong type is a bad_param, reported by name. Same
// two-behaviour rule as puit: a wrong TYPE is an error, an out-of-range VALUE is
// clamped -- which is safe only because every response echoes what it actually
// used, so the clamp is visible rather than hidden.
bool optInt(JsonVariantConst req, const char *key, int *out, const char **bad_field) {
  JsonVariantConst v = req[key];
  if (v.isNull()) return true;
  if (!v.is<int>()) { *bad_field = key; return false; }
  *out = v.as<int>();
  return true;
}

bool optULong(JsonVariantConst req, const char *key, unsigned long *out, const char **bad_field) {
  JsonVariantConst v = req[key];
  if (v.isNull()) return true;
  if (!v.is<unsigned long>()) { *bad_field = key; return false; }
  *out = v.as<unsigned long>();
  return true;
}

bool optBool(JsonVariantConst req, const char *key, bool *out, const char **bad_field) {
  JsonVariantConst v = req[key];
  if (v.isNull()) return true;
  if (!v.is<bool>()) { *bad_field = key; return false; }
  *out = v.as<bool>();
  return true;
}

// The address is the one parameter that is NOT clamped. 0x44 and 0x45 are the
// only two an SHT31 can be at, so a request naming anything else is a mistake
// rather than an over-ambitious value -- and clamping it would quietly talk to a
// different address than the one asked for, then report success.
bool parseAddr(JsonVariantConst req, uint8_t *out, const char **bad_field) {
  int addr = DEFAULT_ADDR;
  if (!optInt(req, "addr", &addr, bad_field)) return false;
  if (addr != ADDR_LOW && addr != ADDR_HIGH) {
    *bad_field = "addr";
    return false;
  }
  *out = (uint8_t)addr;
  return true;
}

bool parseParams(JsonVariantConst req, BurstParams *p, const char **bad_field) {
  p->n = DEFAULT_N;
  p->addr = DEFAULT_ADDR;
  p->cmd = CMD_MEAS_HIGH;
  p->wait_ms = WAIT_HIGH_MS;
  p->gap_ms = DEFAULT_GAP_MS;
  p->rep = "high";

  if (!optInt(req, "n", &p->n, bad_field)) return false;
  if (!optULong(req, "gap_ms", &p->gap_ms, bad_field)) return false;
  if (!parseAddr(req, &p->addr, bad_field)) return false;

  JsonVariantConst rep = req["rep"];
  if (!rep.isNull()) {
    const char *s = rep.as<const char *>();
    if (s == nullptr) {
      *bad_field = "rep";
      return false;
    }
    if (strcmp(s, "high") == 0) {
      p->cmd = CMD_MEAS_HIGH; p->wait_ms = WAIT_HIGH_MS; p->rep = "high";
    } else if (strcmp(s, "medium") == 0) {
      p->cmd = CMD_MEAS_MED; p->wait_ms = WAIT_MED_MS; p->rep = "medium";
    } else if (strcmp(s, "low") == 0) {
      p->cmd = CMD_MEAS_LOW; p->wait_ms = WAIT_LOW_MS; p->rep = "low";
    } else {
      // Unlike a numeric range, there is no nearest sensible value to fall back
      // to, and picking one would silently measure at a different repeatability
      // than the one asked for.
      *bad_field = "rep";
      return false;
    }
  }

  p->n = constrain(p->n, 1, MAX_N);
  p->gap_ms = constrain(p->gap_ms, (unsigned long)MIN_GAP_MS, (unsigned long)MAX_GAP_MS);
  return true;
}

// --- Response helpers --------------------------------------------------------

// Seed a response document with the echoed id, type and protocol version. proto
// rides on every line so the Pi can tell what it is talking to from any reply,
// not just the boot banner or a status call.
void beginResponse(JsonDocument &doc, JsonVariantConst id) {
  doc["id"] = id;  // copies the int as sent
  doc["type"] = "resp";
  doc["proto"] = PROTO_VERSION;
}

// The counts, the effective parameters, and the per-sample detail. On EVERY
// measurement reply, success or failure, so a single logged line explains itself
// without the reader having to remember what was asked for.
void addBurst(JsonDocument &doc, const EnvBurst *b, const BurstParams *p) {
  doc["n"] = b->n;
  doc["n_valid"] = b->n_valid;
  doc["n_crc"] = b->n_crc;
  doc["n_nack"] = b->n_nack;
  doc["n_bus"] = b->n_bus;
  doc["n_short"] = b->n_short;

  if (b->n_valid > 0) {
    doc["temp_min"] = b->temp_min;
    doc["temp_max"] = b->temp_max;
    doc["temp_spread"] = b->temp_max - b->temp_min;
    doc["rh_min"] = b->rh_min;
    doc["rh_max"] = b->rh_max;
    doc["rh_spread"] = b->rh_max - b->rh_min;
  }

  doc["addr"] = p->addr;
  doc["rep"] = p->rep;
  doc["gap_ms"] = p->gap_ms;
  // On every measurement reply, never inferred: a reading taken with the heater
  // on is warm and dry by several degrees and points, and a consumer that cannot
  // see that from the reply itself will average a diagnostic into its history.
  doc["heater"] = g_heater_on;

  // Per-sample detail. null marks a sample that produced no word at all; a
  // sample whose CRC failed keeps its numbers, because raw truth is never
  // discarded, only excluded from the statistics -- which means anything
  // aggregating these MUST filter on sample_status, exactly as puit's samples[]
  // must be filtered on ping_status.
  JsonArray temps = doc["temps_c"].to<JsonArray>();
  JsonArray rhs = doc["rh_pcts"].to<JsonArray>();
  JsonArray traws = doc["t_raws"].to<JsonArray>();
  JsonArray rhraws = doc["rh_raws"].to<JsonArray>();
  for (int i = 0; i < b->n; i++) {
    if (b->status[i] == SAMPLE_OK || b->status[i] == SAMPLE_CRC) {
      temps.add(b->temp_c[i]);
      rhs.add(b->rh_pct[i]);
      traws.add(b->t_raw[i]);
      rhraws.add(b->rh_raw[i]);
    } else {
      temps.add(nullptr);
      rhs.add(nullptr);
      traws.add(nullptr);
      rhraws.add(nullptr);
    }
  }

  // One character per sample, in order: V)alid, C)rc failed, N)ack, B)us
  // timeout, S)hort read. The null cases above are indistinguishable in the
  // arrays, and the pattern itself is informative -- scattered C's read as a
  // marginal signal (leads, pull-ups), a solid run of N's as nothing there.
  char pattern[MAX_N + 1];
  for (int i = 0; i < b->n; i++) {
    switch (b->status[i]) {
      case SAMPLE_OK:    pattern[i] = 'V'; break;
      case SAMPLE_CRC:   pattern[i] = 'C'; break;
      case SAMPLE_NACK:  pattern[i] = 'N'; break;
      case SAMPLE_BUS:   pattern[i] = 'B'; break;
      default:           pattern[i] = 'S'; break;
    }
  }
  pattern[b->n] = '\0';
  doc["sample_status"] = pattern;  // copied into the document, not referenced
}

// Which failure dominated a burst that produced nothing usable. The codes are
// distinct because they send you to different places -- see the SampleStatus
// comment. Ties break toward the more fundamental fault, since a bus that is
// held low or an address that never answers explains a CRC failure but not the
// other way round.
const char *burstErrorCode(const EnvBurst *b) {
  if (b->n_bus >= b->n_nack && b->n_bus >= b->n_crc && b->n_bus >= b->n_short) {
    return "bus_error";
  }
  if (b->n_nack >= b->n_crc && b->n_nack >= b->n_short) {
    return "sensor_fault";
  }
  if (b->n_crc >= b->n_short) {
    return "crc_error";
  }
  return "short_read";
}

// Emit an error response. `id` may be null (pass nullptr) when it could not be
// read; the codes are distinct so "id":null is never ambiguous. `field` names
// the offending request field and is only used by bad_param.
void sendError(const JsonVariantConst *id, const char *code, const char *field) {
  JsonDocument doc;
  if (id != nullptr) {
    doc["id"] = *id;
  } else {
    doc["id"] = nullptr;
  }
  doc["type"] = "resp";
  doc["proto"] = PROTO_VERSION;
  doc["status"] = "error";
  doc["code"] = code;
  if (field != nullptr) {
    doc["field"] = field;
  }
  sendResponse(doc);
}

void sendResponse(const JsonDocument &doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

// An unsolicited measurement line, emitted only while streaming.
//
// It carries no `id` and `type` is "event", not "resp" -- so a reader must match
// responses on type AND the echoed id, never on "next line in". That is the same
// rule the pump board's events impose, and it is stated here rather than assumed
// because a bench rig streaming at 1 Hz into a Pi that polls is the fastest way
// to discover a reader that got it wrong.
void emitEnvEvent(const EnvBurst *b, const BurstParams *p) {
  JsonDocument doc;
  doc["type"] = "event";
  doc["proto"] = PROTO_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["ev"] = "env";
  doc["seq"] = ++g_seq;
  doc["uptime_ms"] = millis();
  if (b->n_valid > 0) {
    doc["status"] = "ok";
    doc["temp_c"] = b->temp_median;
    doc["rh_pct"] = b->rh_median;
    doc["t_raw"] = b->t_raw_median;
    doc["rh_raw"] = b->rh_raw_median;
  } else {
    doc["status"] = "error";
    doc["code"] = burstErrorCode(b);
  }
  addBurst(doc, b, p);
  sendResponse(doc);
}

// The heater turning itself off is a state change nobody asked for, so it is
// announced. Silence would leave a reader believing the readings are still
// biased long after they stopped being.
void emitHeaterEvent(bool on, const char *reason) {
  JsonDocument doc;
  doc["type"] = "event";
  doc["proto"] = PROTO_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["ev"] = "heater";
  doc["seq"] = ++g_seq;
  doc["uptime_ms"] = millis();
  doc["heater"] = on;
  doc["reason"] = reason;
  sendResponse(doc);
}

// --- Sensor driver -----------------------------------------------------------

// Sensirion's CRC-8: polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0xFF, no
// reflection, no final XOR. One byte per 16-bit word, computed by the sensor
// over the data it is about to send.
//
// This is the single most valuable thing in the file for the question being
// asked. It turns "the number looks plausible" into "the sensor and I agree on
// every bit of this word", which is what actually distinguishes a working probe
// on a good bus from a marginal one that will start lying once it is at the end
// of a cable in a well.
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Datasheet conversions. Defined over the whole 16-bit range, which is why there
// is no plausibility window -- see the header comment.
float tempCFromRaw(uint16_t raw) {
  return -45.0f + 175.0f * ((float)raw / 65535.0f);
}

float rhPctFromRaw(uint16_t raw) {
  return 100.0f * ((float)raw / 65535.0f);
}

// Send a 16-bit command. Returns the Wire error code: 0 ok, 2 address NACK,
// 4 data NACK or not running, 5 timeout (a line held low).
uint8_t writeCommand(uint8_t addr, uint16_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  return Wire.endTransmission();
}

// Read `words` CRC-protected 16-bit words. Each is 3 bytes on the wire: MSB,
// LSB, CRC. Returns false with *why set to the specific failure.
//
// requestFrom() returns 0 for both "nobody answered" and "the bus timed out",
// which is precisely the conflation this rig exists to avoid -- so the core's
// timeout flag is read to tell them apart, the same way puit measures the
// trigger->echo latency rather than inferring a dead sensor from a zero.
bool readWords(uint8_t addr, uint16_t *out, int words, SampleStatus *why) {
  const size_t want = (size_t)words * 3;
  uint8_t buf[9];  // 3 words is the most anything here asks for
  if (want > sizeof(buf)) {
    *why = SAMPLE_SHORT;
    return false;
  }

  Wire.clearTimeoutFlag();
  size_t got = Wire.requestFrom((uint8_t)addr, want);
  if (got != want) {
    *why = Wire.getTimeoutFlag() ? SAMPLE_BUS : (got == 0 ? SAMPLE_NACK : SAMPLE_SHORT);
    while (Wire.available()) {
      Wire.read();  // leave nothing behind for the next read to mistake for its own
    }
    return false;
  }
  for (size_t i = 0; i < want; i++) {
    buf[i] = (uint8_t)Wire.read();
  }

  for (int w = 0; w < words; w++) {
    const uint8_t *p = buf + w * 3;
    // The value is decoded even when the CRC fails: the caller reports it as a
    // C sample with its numbers intact, because a corrupt word that is ALMOST
    // right tells you the bus is marginal, while a discarded one tells you
    // nothing at all.
    out[w] = (uint16_t)((uint16_t)p[0] << 8 | p[1]);
    if (crc8(p, 2) != p[2]) {
      *why = SAMPLE_CRC;
      return false;
    }
  }
  *why = SAMPLE_OK;
  return true;
}

// Read the 16-bit status register (CRC-protected like any other word).
bool readStatusRegister(uint8_t addr, uint16_t *reg, SampleStatus *why) {
  uint8_t err = writeCommand(addr, CMD_READ_STATUS);
  if (err != 0) {
    *why = (err == 5) ? SAMPLE_BUS : SAMPLE_NACK;
    return false;
  }
  return readWords(addr, reg, 1, why);
}

// Fire p->n measurements and classify each one.
//
// Note the shape: command, wait, read -- with the wait a known constant rather
// than the sensor holding the clock. See the CMD_MEAS_* comment for why that
// trade is deliberate.
void measure(EnvBurst *b, const BurstParams *p) {
  b->n = p->n;
  b->n_valid = 0;
  b->n_crc = 0;
  b->n_nack = 0;
  b->n_bus = 0;
  b->n_short = 0;

  for (int i = 0; i < p->n; i++) {
    if (i > 0) {
      delay(p->gap_ms);
    }

    b->t_raw[i] = 0;
    b->rh_raw[i] = 0;
    b->temp_c[i] = 0.0f;
    b->rh_pct[i] = 0.0f;

    uint8_t err = writeCommand(p->addr, p->cmd);
    if (err != 0) {
      b->status[i] = (err == 5) ? SAMPLE_BUS : SAMPLE_NACK;
      if (b->status[i] == SAMPLE_BUS) b->n_bus++; else b->n_nack++;
      continue;
    }

    delay(p->wait_ms);

    uint16_t words[2];
    SampleStatus why = SAMPLE_OK;
    bool ok = readWords(p->addr, words, 2, &why);

    // Both the good and the CRC-failed path keep the numbers; only the paths
    // that produced no word at all leave them zeroed. addBurst() emits null for
    // exactly those, which is what makes a null in the arrays unambiguous.
    if (ok || why == SAMPLE_CRC) {
      b->t_raw[i] = words[0];
      b->rh_raw[i] = words[1];
      b->temp_c[i] = tempCFromRaw(words[0]);
      b->rh_pct[i] = rhPctFromRaw(words[1]);
    }

    if (ok) {
      b->status[i] = SAMPLE_OK;
      b->n_valid++;
    } else {
      b->status[i] = why;
      switch (why) {
        case SAMPLE_CRC:  b->n_crc++; break;
        case SAMPLE_BUS:  b->n_bus++; break;
        case SAMPLE_NACK: b->n_nack++; break;
        default:          b->n_short++; break;
      }
    }
  }

  g_addr = p->addr;
  g_last_read_at = millis();
  summarize(b);
}

// Median, min and max over the valid samples only, plus the raw ticks behind the
// two medians. Leaves everything zeroed when nothing is valid; callers check
// n_valid before reporting a value.
void summarize(EnvBurst *b) {
  b->temp_median = 0.0f;
  b->rh_median = 0.0f;
  b->temp_min = 0.0f;
  b->temp_max = 0.0f;
  b->rh_min = 0.0f;
  b->rh_max = 0.0f;
  b->t_raw_median = 0;
  b->rh_raw_median = 0;
  if (b->n_valid == 0) {
    return;
  }

  float t[MAX_N], h[MAX_N];
  uint16_t tr[MAX_N], hr[MAX_N];
  int k = 0;
  for (int i = 0; i < b->n; i++) {
    if (b->status[i] == SAMPLE_OK) {
      t[k] = b->temp_c[i];
      h[k] = b->rh_pct[i];
      tr[k] = b->t_raw[i];
      hr[k] = b->rh_raw[i];
      k++;
    }
  }

  b->temp_min = t[0];
  b->temp_max = t[0];
  b->rh_min = h[0];
  b->rh_max = h[0];
  for (int i = 1; i < k; i++) {
    if (t[i] < b->temp_min) b->temp_min = t[i];
    if (t[i] > b->temp_max) b->temp_max = t[i];
    if (h[i] < b->rh_min) b->rh_min = h[i];
    if (h[i] > b->rh_max) b->rh_max = h[i];
  }

  // Both conversions are monotonic in the raw word, so each raw median is the
  // exact counterpart of its converted median -- no second sort policy to keep
  // in step. (findMedian sorts its argument.)
  b->temp_median = findMedian(t, k);
  b->rh_median = findMedian(h, k);
  b->t_raw_median = findMedianU16(tr, k);
  b->rh_raw_median = findMedianU16(hr, k);
}

// --- Background services -----------------------------------------------------

// One valid sample is enough to call the sensor healthy: the LED answers "is it
// wired right", not "is every reading perfect", and the counts on the reply are
// where the nuance lives.
void noteHealth(const EnvBurst *b) {
  g_health = (b->n_valid > 0) ? HEALTH_OK : HEALTH_FAULT;
}

void serviceHeater() {
  if (!g_heater_on || g_heater_off_at == 0) {
    return;
  }
  // Subtraction, not `millis() > deadline`: the difference is unsigned and stays
  // correct across the 49-day wrap, which the comparison does not.
  if ((long)(millis() - g_heater_off_at) < 0) {
    return;
  }
  uint8_t err = writeCommand(g_addr, CMD_HEATER_OFF);
  g_heater_off_at = 0;
  if (err == 0) {
    g_heater_on = false;
    emitHeaterEvent(false, "auto_off");
  } else {
    // The heater may well still be on and there is no way to be sure from here,
    // so g_heater_on is left set: a reading marked biased that is not is a
    // nuisance, one marked clean that is not is corrupt data.
    g_health = HEALTH_FAULT;
    emitHeaterEvent(true, "auto_off_failed");
  }
}

void serviceStream() {
  if (!g_stream_on) {
    return;
  }

  // Auto-stop, announced rather than silent: a console that simply goes quiet
  // reads the same as a board that has crashed. Subtraction rather than a
  // comparison, so it stays correct across the 49-day millis() wrap.
  if (g_stream_off_at != 0 && (long)(millis() - g_stream_off_at) >= 0) {
    g_stream_on = false;
    g_stream_off_at = 0;
    if (g_stream_text) {
      Serial.println("      -- stream stopped --");
      return;
    }
    JsonDocument doc;
    doc["type"] = "event";
    doc["proto"] = PROTO_VERSION;
    doc["role"] = PIJARDIN_ROLE;
    doc["ev"] = "stream";
    doc["seq"] = ++g_seq;
    doc["uptime_ms"] = millis();
    doc["stream"] = false;
    doc["reason"] = "auto_off";
    sendResponse(doc);
    return;
  }

  if ((long)(millis() - g_next_stream_at) < 0) {
    return;
  }
  g_next_stream_at = millis() + g_stream_ms;

  BurstParams p;
  const char *unused = nullptr;
  parseParams(JsonVariantConst(), &p, &unused);
  p.n = g_stream_n;

  EnvBurst b;
  measure(&b, &p);
  noteHealth(&b);
  if (g_stream_text) {
    emitEnvText(&b);
  } else {
    emitEnvEvent(&b, &p);
  }
}

// Right-align a float in `width` columns, without printf's float support --
// which newlib-nano omits by default, and which would print garbage rather than
// fail to build if it were missing.
void printFixed(float v, int decimals, int width) {
  int whole = (int)fabsf(v);
  int digits = 1;
  while (whole >= 10) {
    whole /= 10;
    digits++;
  }
  int len = digits + 1 + decimals + (v < 0.0f ? 1 : 0);
  for (int i = len; i < width; i++) {
    Serial.print(' ');
  }
  Serial.print(v, decimals);
}

// One line of plain text per reading. Columns, so the eye can follow a trend
// down the screen -- which is the whole reason this mode exists.
//
//       temp     humidity
//      22.41 C     47.83 %
//      22.43 C     47.79 %
//      -- echo_timeout --
void emitEnvText(const EnvBurst *b) {
  if (b->n_valid == 0) {
    Serial.print("      -- ");
    Serial.print(burstErrorCode(b));
    Serial.println(" --");
    return;
  }
  printFixed(b->temp_median, 2, 11);
  Serial.print(" C");
  printFixed(b->rh_median, 2, 10);
  Serial.println(" %");
}

// Keep the LED honest on a board nobody is talking to. Skipped whenever a read
// has happened recently for any other reason, so this never doubles the sensor's
// duty cycle while streaming.
void serviceHealth() {
  if (g_stream_on) {
    return;
  }
  if (g_last_read_at != 0 && (long)(millis() - g_last_read_at) < (long)HEALTH_POLL_MS) {
    return;
  }

  BurstParams p;
  const char *unused = nullptr;
  parseParams(JsonVariantConst(), &p, &unused);
  p.n = 1;

  EnvBurst b;
  measure(&b, &p);
  noteHealth(&b);
}

// --- Status LED --------------------------------------------------------------

// Claim every LED on the board, including the ones this firmware will never use.
// Leaving a pin uninitialised is not "leaving it alone" -- it leaves an input
// floating against an LED tied to 3V3, which glows.
void ledBegin() {
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  ledWrite(LED_NONE);
  g_led_shown = LED_NONE;

  // The WS2812 is not used and is parked dark rather than left to chance: at
  // cold boot it comes up with whatever happens to be in its shift register.
  // Both pins, deliberately -- cutting its power is what actually turns it off,
  // and holding the data line low means that if it is somehow powered anyway it
  // sees a permanent reset rather than noise it might latch as a colour.
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
  pinMode(PIN_NEOPIXEL, OUTPUT);
  digitalWrite(PIN_NEOPIXEL, LOW);
}

// Light exactly one die, or none.
void ledWrite(LedColour c) {
  digitalWrite(LED_R_PIN, (c == LED_RED) ? LED_ON : LED_OFF);
  digitalWrite(LED_G_PIN, (c == LED_GREEN) ? LED_ON : LED_OFF);
  digitalWrite(LED_B_PIN, (c == LED_BLUE) ? LED_ON : LED_OFF);
}

// Drive the LED from the health state. See the colour table by LED_R_PIN.
//
// Fault outranks the heater: a solid blue "readings are biased" is useless
// advice when the real answer is that nothing is answering at all.
void updateLed() {
  unsigned long now = millis();
  LedColour want;

  if (g_health == HEALTH_FAULT) {
    want = ((now / LED_FAULT_MS) % 2) ? LED_RED : LED_NONE;
  } else if (g_heater_on) {
    want = LED_BLUE;
  } else if (g_health == HEALTH_UNKNOWN) {
    want = ((now / LED_UNKNOWN_MS) % 2) ? LED_BLUE : LED_NONE;
  } else {
    want = ((now % LED_ALIVE_PERIOD_MS) < LED_ALIVE_FLASH_MS) ? LED_GREEN : LED_NONE;
  }

  if (want != g_led_shown) {
    ledWrite(want);
    g_led_shown = want;
  }
}

// --- Median helpers ----------------------------------------------------------

float findMedian(float arr[], int n) {
  qsort(arr, n, sizeof(float), compareFloat);
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0f;
  }
  return arr[n / 2];
}

int compareFloat(const void *a, const void *b) {
  float x = *(const float *)a;
  float y = *(const float *)b;
  return (x > y) - (x < y);
}

uint16_t findMedianU16(uint16_t arr[], int n) {
  qsort(arr, n, sizeof(uint16_t), compareU16);
  if (n % 2 == 0) {
    return (uint16_t)(((uint32_t)arr[n / 2 - 1] + arr[n / 2]) / 2);
  }
  return arr[n / 2];
}

int compareU16(const void *a, const void *b) {
  uint16_t x = *(const uint16_t *)a;
  uint16_t y = *(const uint16_t *)b;
  return (x > y) - (x < y);
}
