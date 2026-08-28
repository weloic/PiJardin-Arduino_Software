// PiJardin pump sensor firmware -- XIAO RP2040 + ZMPT101B AC voltage module.
//
// NOTE: this board is an RP2040, NOT the SAMD21 the well sensor uses. The two
// XIAOs are the same footprint and look identical, so check the silkscreen
// before flashing -- `pio run -e pump` and `-e puit` build for different chips
// and neither image will run on the other board.
//
// Answers one question: WHEN did the pump start and stop? It does that by
// looking for mains AC on the pump's own feed, which is a far more honest signal
// than a current clamp threshold or a flow switch -- either the contactor is
// closed and there is 230 V on the motor, or there is not.
//
// Talks to the Raspberry Pi over USB serial with the same newline-delimited JSON
// (NDJSON) envelope as the well sensor: one JSON object per line, in both
// directions, every reply echoing the request "id". See README.md. Unlike the
// well sensor, this board also speaks WITHOUT being asked:
//
//   Pi  -> {"id":7,"cmd":"read_pump"}\n
//   MCU -> {"id":7,"type":"resp","proto":2,"status":"ok","state":"on",...}\n
//   MCU -> {"type":"event","proto":2,"role":"pump","ev":"pump","seq":412,...}\n
//   MCU -> {"type":"event","proto":2,"role":"pump","ev":"hb","seq":413,...}\n
//
// WIRING, AND THE ONE WAY TO GET THIS WRONG
// -----------------------------------------
// The ZMPT101B must sit across the pump's SWITCHED feed -- downstream of the
// contactor, relay or switch that starts the motor. Wired across the incoming
// house mains instead it reads a healthy 230 V forever and this firmware will
// cheerfully report "on" whether the pump is running or stopped. There is no way
// for the board to detect that mistake: a correct reading and a useless one look
// identical from here.
//
// The module is powered from the XIAO's 3V3 pad, NOT 5V, and its OUT goes
// straight to A0 with no divider. See VPIN below -- that arrangement is out of
// spec on purpose and has a cost worth understanding.
//
// STATE: A PRINCIPLE THAT WAS RECONSIDERED, NOT FORGOTTEN
// -------------------------------------------------------
// Up to fw 1.x this file argued the opposite of what it now does, and the
// argument is preserved here because the next reader needs to know it was
// reconsidered rather than overlooked. It said: the board keeps no state between
// requests, there is no on/off latching and no hysteresis, the board reports
// what it measured plus an explicit "uncertain" verdict and the Pi (which has
// history) decides -- and, plainly, "a pump that switches faster than the Pi
// polls will be missed; that is a property of polling, not something the board
// can paper over."
//
// Every sentence of that was true FOR A POLLED BOARD. What changed is the
// question, not the reasoning: the job is now to RECORD when the pump starts and
// stops, and polling cannot do that. The Pi would have to ask every few seconds
// forever, would still miss any cycle shorter than its interval, and -- the part
// that actually breaks it -- would have no way to know what it missed while it
// was rebooting or being redeployed. The board is the only thing watching
// continuously, so the board must be the thing that decides, remembers and
// reports.
//
// So latching, hysteresis and debounce move HERE (see stepDetector). The band
// between off_counts and on_counts stops being a reported verdict and becomes
// the thing hysteresis is made of: a reading inside it HOLDS the current state
// rather than announcing indecision. `read_pump` still returns "uncertain",
// because it is still an instantaneous, unlatched measurement and lying about
// that would break tools/pump_tune.py's calibration.
//
// Statelessness survives everywhere it still applies, and is deliberately NOT
// weakened:
//   - thresholds and parameters are compile-time constants. They are settable
//     per request for a measurement, never for the detector: nothing the Pi
//     sends can change how this board decides.
//   - nothing is persisted to flash. A reboot starts from "unknown", never from
//     a remembered guess about a pump nobody was watching. uptime_ms in `status`
//     is what tells the Pi a reboot happened, so since_ms is never extrapolated
//     across one.
//   - every response still echoes the effective values it used.
// The only history the board keeps is what it has WATCHED since boot: a state,
// how long it has held, and a 32-entry ring of the transitions it saw.
//
// This file duplicates the NDJSON transport from src/puit/puit_sensor.cpp on
// purpose, and is deliberately structured the same way so the shared part can be
// lifted into lib/ later with both callers visible.

#include <Arduino.h>
#include <ArduinoJson.h>

// ZMPT101B analog out. On the XIAO RP2040 A0 is GPIO26 = ADC0.
//
// THE MODULE RUNS FROM THE XIAO'S 3V3 PAD AND ITS `OUT` GOES DIRECTLY TO THIS
// PIN. No divider, no 5 V anywhere. That is what is installed, every threshold
// below was measured through it, and it is out of specification on purpose.
//
// What that costs, honestly:
//
//   - The ZMPT101B is specified for +5 V to +30 V, so 3V3 is out of spec. It
//     works here only because the gain pot is set low enough that the signal
//     never reaches the region where the module breaks down. That is a working
//     point, not a guarantee -- which is why the check below exists.
//
//   - The module amplifies through an LM358, which cannot drive its output
//     closer than ~1.3 V to its positive rail. On 3V3 that puts its ceiling near
//     2.0 V, about 2480 counts -- measured on this exact hardware at high gain,
//     where the positive peaks topped out at 2476 and stopped moving. With the
//     bias at 2052 that leaves roughly 430 counts of usable headroom ABOVE the
//     bias against roughly 2050 below it. The swing is asymmetric and the
//     positive peak is the binding constraint; the ADC's 4095 rail is not the
//     limit and never comes into it.
//
//   - At the calibrated pot setting the running signal is rms 199.2 counts, so
//     the peak is ~283 counts (rms/peak 0.703 -- essentially a clean sine).
//     That is about 66% of the available headroom. It is a real margin rather
//     than a comfortable one, and it is exactly why the headroom check below is
//     needed instead of trusting n_clipped.
//
// DO NOT "fix" this by moving to 5 V or adding a 2:1 divider. Both would put the
// LM358 on its proper rail and make the waveform symmetric, and both would also
// invalidate every count in this file: DEFAULT_ON_COUNTS, DEFAULT_OFF_COUNTS and
// FREQ_MIN_RMS were all measured through the front end described above. Changing
// the front end means re-running the calibration, not editing this comment.
// (5 V straight to A0 would additionally destroy the pin -- the RP2040 is not
// 5 V tolerant.)
#define VPIN A0

// GPIO17, the red user LED. THE XIAO RP2040'S USER LEDS ARE ACTIVE LOW -- the
// pin is pulled low to light them. Driving this the intuitive way round would
// leave the LED on whenever the board is idle and dark while it is working,
// which is backwards for a busy indicator and reads as a hung board.
#define LEDPIN LED_BUILTIN
#define LED_ON LOW
#define LED_OFF HIGH

// --- Protocol / firmware identity -------------------------------------------
// proto is numbered per firmware, not per repo: this board shares the well
// sensor's envelope (id/type/proto/status/code) but not its command set, so the
// pump contract started at 1 and is versioned independently.
//
// 1 -> 2 is the move from polled-only to event-pushing. Every proto-1 command
// still works unchanged, so the break is not in the request path -- it is that a
// Pi which only polls now silently misses every transition the board records.
// That is precisely the kind of change a version number has to make assertable,
// so the Pi can refuse to run against a board it would misread. Nothing on the
// Pi consumes pump proto 1 yet, so there is no flag day to manage.
#define FW_VERSION "2.0.0"
#define PROTO_VERSION 2

// Board role, injected per environment by platformio.ini (-DPIJARDIN_ROLE). It rides on
// the boot banner, every event line and the status response so the Pi can tell the two
// boards apart from the data itself, rather than guessing from a USB VID or a proto
// number. Deliberately not a literal here: a copy in the source could disagree with the
// environment that built it, and a firmware that lies about its role is worse than one
// that will not compile.
#ifndef PIJARDIN_ROLE
#error "PIJARDIN_ROLE is not defined -- set it in this environment's build_flags in platformio.ini"
#endif

// --- ADC ---------------------------------------------------------------------
#define ADC_BITS 12
#define ADC_MAX 4095
// A sample this close to either rail is clipped: the true peak was cut off, so
// the RMS below it is an underestimate. Counted, not fatal -- for on/off
// purposes a clipped waveform still unambiguously means "on".
//
// On THIS front end this check never fires, and that is the point of the
// headroom check below rather than a reason to delete it: the LM358 flattens the
// wave at ~2480 counts, nowhere near 0 or 4095, so n_clipped stays 0 through the
// entire failure. It still earns its place because a different module, or this
// one on a proper 5 V rail, can genuinely reach the rails.
#define CLIP_MARGIN 4
// The first conversions after power-up or a mux change are unreliable, on the
// RP2040's SAR ADC as on the SAMD21's. Throw a few away before the timed window
// rather than letting them skew bias.
#define DISCARD_READS 8
// Below this RMS (counts) no frequency is derived at all -- see analyse(). A
// real signal at any usable gain sits in the hundreds, so this only ever gates
// out traces that are pure noise. It also gates the headroom check below, for
// the same reason: both are ratios that mean nothing on a noise floor.
//
// Raised from the original 10.0 on 2026-08-28, and this one is measured too. The
// RP2040's SAR ADC has a documented differential-nonlinearity problem -- codes it
// will never return -- which shows up as extra apparent noise on a quiet input,
// and this installation's pump-off floor came in at 8.4-14.2 counts. Above the
// old 10.0 guard, so the board was deriving a frequency from that noise and
// reporting it with a straight face: 274, 290, 307, 341, 425 Hz across
// consecutive windows. Numbers that wander like that are not a frequency.
//
// 40.0 is ~3x the observed floor. It leaves the running signal (199 counts)
// untouched and returns an honest freq_hz of 0 when the pump is stopped.
// Re-measure it if the installation changes; tools/pump_tune.py prints the
// figure and now recommends the value directly.
#define FREQ_MIN_RMS 40.0f

// --- Headroom / soft clipping ------------------------------------------------
// The failure mode the 3V3 supply creates, and the one n_clipped structurally
// cannot see. The LM358 runs out of headroom and flattens the top of the wave at
// ~2480 counts while the bottom half swings freely; both ADC rails stay
// untouched, so every number in the reply looks healthy while the RMS quietly
// understates the signal.
//
// Detected by ASYMMETRY about the measured bias rather than against a hardcoded
// ceiling:
//
//   asym = (max_counts - bias) / (bias - min_counts)
//
// A clean sine gives ~1.0. The measured over-gain case gives
// (2476-2032)/(2032-1480) = 0.80. The ratio is self-calibrating -- it stays
// valid if the supply sags, the pot moves, or the module is replaced, none of
// which a fixed 2480 would survive.
//
// Evaluated ONLY above FREQ_MIN_RMS. On a pump-off noise floor min and max are
// a handful of DNL-inflated counts either side of the bias and the ratio is
// meaningless.
//
// It is a WARNING and never a fault. A soft-clipped waveform still unambiguously
// means the pump is on, and refusing to report a running pump because its
// waveform is ugly would be a worse failure than the one being reported. What it
// means is that the pot wants turning down -- and it is early warning that a
// mains overvoltage will start clipping: 230 V +10% = 253 V pushes the peak from
// ~283 to ~311 counts, against ~430 of headroom.
//
// Soft clipping does NOT affect freq_hz. The Schmitt band sits at 0.25 x rms
// around the bias, far below the flattened peak, so the crossings it counts are
// nowhere near the compressed region.
#define ASYM_MIN 0.90f

// --- Sampling window ---------------------------------------------------------
// Samples are stored rather than accumulated on the fly, so the buffer is what
// bounds the window. 1200 x uint16 = 2.4 kB, which the RP2040's 264 kB leaves
// room to grow -- but the size is part of the published contract (`status`
// returns max_samples, and the Pi sizes its expectations from it), so raising it
// is a protocol change, not a free win.
#define MAX_SAMPLES 1200
#define DEFAULT_CYCLES 10            // 200 ms at 50 Hz
#define MIN_CYCLES 1
#define MAX_CYCLES 60
#define DEFAULT_MAINS_HZ 50.0f       // Switzerland; 60 works via the parameter
#define MIN_MAINS_HZ 40.0f
#define MAX_MAINS_HZ 70.0f
// Deliberately paced rather than "as fast as analogRead goes". A free-running
// loop on this chip overruns the buffer long before the window closes, which
// truncates it to a non-integer number of mains cycles -- and a partial cycle
// biases the RMS by a few percent, in a direction that depends on the phase the
// window happened to start at. A known uniform rate makes the window an exact
// whole number of cycles by construction, and makes the zero-crossing timing
// below mean something. 2 kHz is 40 samples per 50 Hz cycle.
#define DEFAULT_RATE_HZ 2000UL
#define MIN_RATE_HZ 500UL
#define MAX_RATE_HZ 20000UL

// --- Decision thresholds -----------------------------------------------------
// RMS in ADC counts, not volts, because counts are what the board can actually
// observe: the ZMPT101B's output amplitude is set by the multi-turn pot on the
// module, so there is no factory relationship between counts and volts to
// assume. These two are MEASURED, not guessed -- calibrated 2026-08-28 against
// the real installation with tools/pump_tune.py:
//
//   floor  12.93 counts (worst of 5, pump stopped)   -> off_counts 2.7x clear
//   signal 199.20 counts (worst of 5, pump running)  -> on_counts  2.8x clear
//   separation 15.4x; bias 2052; waveform rms/peak 0.703 (a sine is 0.707)
//
// Placed geometrically between the two states rather than by the 5x / div-3
// rule, which needs about 20x separation before it leaves a usable gap -- at
// 15.4x it put the two thresholds 1.7 counts apart. The band between them does
// real work: a 200 ms window that straddles the contactor lands in it, which is
// exactly the case the board must not act on. The detector HOLDS its state
// through that band; `read_pump` reports it as "uncertain".
//
// The detector uses these same constants. It does not take them from a request:
// a per-request override changes one measurement, never how the board decides.
//
// Re-run the calibration if the pot moves, the module is replaced, or anything
// changes on that circuit -- see docs/pump.md.
#define DEFAULT_ON_COUNTS 71.1f
#define DEFAULT_OFF_COUNTS 35.5f

// --- Sensor plausibility -----------------------------------------------------
// A live module idles near mid-scale whatever the pump is doing, because the
// bias is generated from its own supply. A bias far from mid-scale means the
// board is not looking at a working module at all: unpowered, output shorted, or
// -- most likely in the field -- the signal wire off, leaving the ADC input
// floating. That is a hardware fault, and it is the one thing that must never be
// reported as "pump off": both produce a low RMS and they are otherwise
// indistinguishable. Same reasoning as the well sensor separating a dead
// HC-SR04 from one that simply found no echo.
//
// With the divider gone the module really does idle at mid-scale, so the
// measured 2052 sits centrally in this window and the check is doing exactly
// what its name says.
#define DEFAULT_BIAS_MIN 1024        // 25% of full scale
#define DEFAULT_BIAS_MAX 3072        // 75% of full scale

// --- Detector ----------------------------------------------------------------
// A window is measured continuously in loop(), not only on request, and the
// state machine below runs on every one.
//
// DETECT_CYCLES is DEFAULT_CYCLES on purpose and should not be shortened to
// improve latency. The calibrated floor (12.93 counts) and signal (199.20
// counts) were both measured at a 10-cycle window; a narrower one integrates
// less noise away and moves the floor, which is the figure the thresholds were
// placed against. Latency is bought with DEBOUNCE_WINDOWS, not by making the
// measurement worse.
#define DETECT_CYCLES DEFAULT_CYCLES

// A candidate state must survive this many consecutive windows before it is
// declared. 3 x 200 ms = 600 ms, and it absorbs the three things that otherwise
// manufacture a phantom transition: contactor bounce, motor inrush, and the
// window that straddles the switching instant (which lands in the uncertain band
// or at a nonsense frequency).
//
// This is what sets the resolution floor of the WHOLE system: a pump cycle
// shorter than roughly 600-800 ms is not resolved, here or on the Pi. Say so
// plainly rather than implying millisecond timestamps mean millisecond accuracy.
#define DEBOUNCE_WINDOWS 3

// To declare ON it is not enough for the RMS to clear on_counts: the frequency
// must also be mains. A high RMS at the wrong frequency is induced hum, pickup
// or a wiring fault, not a running pump. A rejected window HOLDS the current
// state and is counted in n_freq_reject -- deliberately not a fifth state, which
// would push a diagnostic into the contract the Pi has to switch on, and would
// still not tell it anything the counter does not.
#define FREQ_MATCH_HZ 5.0f

// Emitted regardless of state. Without it the Pi cannot distinguish "pump off
// for six hours" from "board dead for six hours", and that distinction is the
// entire point of recording this.
#define HEARTBEAT_MS 60000UL

// Transitions kept in RAM so a Pi that was down can ask for what it missed. 32
// entries is ~256 bytes and covers a couple of days of ordinary irrigation; a Pi
// outage longer than 32 transitions loses the oldest, which `history` reports as
// truncated rather than hiding. Published in `status` as history_max so the Pi
// sizes its expectations from the board.
#define HISTORY_MAX 32
// One reply carries at most this many events, so it stays bounded: ~480 bytes
// worst case against LINE_MAX's 192-byte cap on the *request*, which bounds
// nothing on the way back. The Pi calls again with a higher after_seq when
// `more` is true.
//
// That is larger than the 256-byte CDC TX buffer, and deliberately so: this is a
// SOLICITED reply, sent to a host that just asked for it and is therefore
// reading, so it may take two buffer fills the way a `sampling` dump already
// takes ten. The never-block rule (see emitUnsolicited) applies to lines nobody
// asked for -- those are the ones that can pile up against a host that has
// stopped listening.
#define HISTORY_REPLY_MAX 8

// The board now runs unattended and continuously, and nothing can restart it
// remotely: the Pi cannot reset it, and recovery from a hang would mean a UF2
// reflash or a walk to the well. A hung detector is silent data loss, which is
// the failure this whole firmware exists to prevent.
//
// 8 s is comfortably longer than the slowest legitimate blocking stretch -- a
// 60-cycle window at 40 Hz mains is 1.5 s, and a `sampling` reply with 400 raw
// counts is a couple of kB to a host that is actively reading. It is deliberately
// NOT fed from inside a stalled Serial.write: a solicited reply that cannot
// drain in 8 s means the host stopped reading mid-reply, and resetting is the
// correct response to that -- the alternative is a board parked in write()
// forever, no longer watching the pump.
#define WATCHDOG_MS 8000

// --- Waveform dump -----------------------------------------------------------
// `sampling` can return raw counts so the pot can be set by eye. The head of the
// window is returned rather than a decimation across it: sampling every k-th
// point of a 50 Hz sine aliases into a slower-looking sine, which is exactly the
// kind of plausible-but-wrong picture you would then calibrate against.
#define DEFAULT_DUMP_N 200
#define MAX_DUMP_N 400

#define LINE_MAX 192                 // *request* line cap; responses are unbounded

// Parameters for one measurement window, after defaults, clamping and validation.
struct SampleParams {
  int cycles;                  // whole mains cycles to cover
  float mains_hz;              // nominal mains frequency, sizes the window
  unsigned long rate_hz;       // requested sampling rate
  float on_counts, off_counts; // RMS decision thresholds, on >= off
  int bias_min, bias_max;      // plausible idle window for the module
  float counts_per_volt;       // 0 = not supplied, so no voltage is reported
  int dump_n;                  // raw samples to return (`sampling` only)
};

// Result of one measurement window: the raw statistics, the derived frequency,
// and enough about the sampling itself to tell whether it can be trusted.
struct Reading {
  int n;                       // samples actually taken
  unsigned long interval_us;   // scheduled spacing between samples
  unsigned long window_us;     // measured wall time of the whole window
  // Worst arrival lateness against the schedule, over all samples. Measured
  // rather than assumed for the same reason the well sensor measures its ack
  // latency: it is the only evidence that the requested rate was actually
  // achieved. A max_late_us approaching interval_us means analogRead could not
  // keep up, the spacing was not uniform, and freq_hz is the first casualty.
  unsigned long max_late_us;
  bool truncated;              // window did not fit MAX_SAMPLES; cycles cut short
  float cycles_eff;            // whole-cycle equivalent actually covered
  float bias;                  // mean level, measured -- not assumed Vcc/2
  float rms;                   // RMS of the bias-removed signal, in counts
  uint16_t min_counts, max_counts;
  int n_clipped;
  float asym;                  // (max-bias)/(bias-min); ~1.0 for a clean sine
  bool headroom_warn;          // asym below ASYM_MIN, with enough signal to mean it
  int n_rise;                  // rising crossings of the bias level
  float freq_hz;               // n_rise / window, 0 when nothing crossed
};

// The four states the detector can be in. `unknown` is the boot state and is
// never a guess: the board cannot know what the pump was doing before it was
// watching. `fault` is its own state rather than a flavour of off, for the same
// reason sensor_fault is its own error code.
enum DetState { ST_UNKNOWN = 0, ST_OFF, ST_ON, ST_FAULT };
static const char *const STATE_NAME[] = {"unknown", "off", "on", "fault"};

// One recorded transition. Only `pump` events land here -- heartbeats consume a
// seq but carry no information a replay could need.
struct HistEntry {
  uint32_t seq;
  uint32_t ms;
  uint8_t state;
};

// --- Sample buffer -----------------------------------------------------------
// Static, so a long window cannot fail to allocate mid-request.
static uint16_t sampleBuf[MAX_SAMPLES];

// --- Request line buffer -----------------------------------------------------
// Fixed size, filled incrementally in loop(). No String, so a garbage stream
// cannot grow the heap, and no reliance on the serial read timeout.
static char lineBuf[LINE_MAX];
static size_t lineLen = 0;
static bool lineOverflow = false;

// --- Detector state ----------------------------------------------------------
// All of it RAM only. Nothing here is written to flash, so a reboot starts from
// ST_UNKNOWN with an empty ring -- see the header.
static SampleParams detParams;          // the compile-time defaults, fixed at boot

static DetState detState = ST_UNKNOWN;
static uint32_t detSinceMs = 0;         // millis() when detState was declared
static DetState detPending = ST_UNKNOWN;
static uint8_t detPendingN = 0;         // consecutive windows agreeing on detPending

static uint32_t evSeq = 0;              // monotonic per boot, every event line
static uint32_t nDropped = 0;           // event lines the TX path had no room for
static uint32_t nFreqReject = 0;        // windows loud enough for ON at a wrong freq
static uint32_t nHeadroom = 0;          // windows flagged by the asym check
static uint32_t lastHbMs = 0;

static HistEntry hist[HISTORY_MAX];
static uint8_t histCount = 0;           // entries currently held
static uint8_t histHead = 0;            // next write slot (and the oldest, once full)
static uint32_t histEvictedSeq = 0;     // seq of the newest entry ever overwritten

// --- Prototypes --------------------------------------------------------------
bool hasContent();
void handleLine(const char *line);
void handleReadPump(JsonVariantConst id, JsonVariantConst req);
void handleSampling(JsonVariantConst id, JsonVariantConst req);
void handleStatus(JsonVariantConst id);
void handleHistory(JsonVariantConst id, JsonVariantConst req);
bool optInt(JsonVariantConst req, const char *key, int *out, const char **bad_field);
bool optULong(JsonVariantConst req, const char *key, unsigned long *out, const char **bad_field);
bool optFloat(JsonVariantConst req, const char *key, float *out, const char **bad_field);
void setDefaults(SampleParams *p);
bool parseParams(JsonVariantConst req, SampleParams *p, const char **bad_field);
void beginResponse(JsonDocument &doc, JsonVariantConst id);
void addContext(JsonDocument &doc, const Reading *r, const SampleParams *p);
void addMeasurement(JsonDocument &doc, const Reading *r, const SampleParams *p);
bool gateReading(JsonVariantConst id, const Reading *r, const SampleParams *p);
const char *classify(const Reading *r, const SampleParams *p);
void sendError(const JsonVariantConst *id, const char *code, const char *field);
void sendResponse(const JsonDocument &doc);
void sampleWindow(Reading *r, const SampleParams *p);
void analyse(Reading *r);
void serviceSerial();
void stepDetector();
void resetDebounce();
void declareState(DetState s, const Reading *r);
void emitHeartbeat(const Reading *r);
void beginEvent(JsonDocument &doc, const char *ev);
void addFixed2(JsonDocument &doc, const char *key, float v, char *buf);
bool emitUnsolicited(const JsonDocument &doc, bool count_drop);
void historyPush(uint32_t seq, DetState s, uint32_t ms);

void setup() {
  // 12-bit conversions: the signal of interest at the "off" end is a handful of
  // counts of noise, and 10-bit would quantise a quarter of the useful range
  // away for no saving.
  analogReadResolution(ADC_BITS);
  pinMode(VPIN, INPUT);

  Serial.begin(9600);

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LED_OFF);

  setDefaults(&detParams);
  detParams.cycles = DETECT_CYCLES;
  lastHbMs = millis();

  // Armed before the first line goes out, deliberately. The banner is the only
  // thing between reset and the detector running, and a board wedged in
  // Serial.write() printing a greeting is a board that never starts watching.
  rp2040.wdt_begin(WATCHDOG_MS);

  // Boot banner: structured "ready" line the Pi waits for after reset. Sent
  // through the same non-blocking guard as every other unsolicited line, and for
  // the same reason -- it is best-effort by nature anyway, because the XIAO's
  // native USB does not reset on port open and the banner has usually already
  // gone out before a host is listening. `status` is the real handshake.
  JsonDocument doc;
  doc["type"] = "ready";
  doc["proto"] = PROTO_VERSION;
  doc["fw"] = FW_VERSION;
  doc["role"] = PIJARDIN_ROLE;  // two boards speak this envelope; say which one this is
  emitUnsolicited(doc, false);  // not counted in `dropped`: that counts lost events
}

// One pass: drain whatever the host has sent, then measure one window and run
// the state machine on it.
//
// Requests are parsed BETWEEN windows, never during one, so a reply can be up to
// one window (~200 ms) late. That is the right trade: a late reply costs
// nothing, and a missed window costs a transition.
void loop() {
  rp2040.wdt_reset();
  serviceSerial();
  stepDetector();
}

void serviceSerial() {
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
  digitalWrite(LEDPIN, LED_ON);  // lit while the request is being served

  JsonDocument req;
  DeserializationError err = deserializeJson(req, line);
  if (err) {
    sendError(nullptr, "bad_request", nullptr);
    digitalWrite(LEDPIN, LED_OFF);
    return;
  }

  // The id is echoed back verbatim, so it must be an integer: a missing or
  // mistyped id is reported as bad_id rather than silently answered with
  // "id":null, which the Pi could not tell apart from bad_request.
  JsonVariantConst id = req["id"];
  if (!id.is<long>()) {
    sendError(nullptr, "bad_id", nullptr);
    digitalWrite(LEDPIN, LED_OFF);
    return;
  }

  const char *cmd = req["cmd"] | "";
  if (strcmp(cmd, "read_pump") == 0) {
    handleReadPump(id, req.as<JsonVariantConst>());
    resetDebounce();
  } else if (strcmp(cmd, "sampling") == 0) {
    handleSampling(id, req.as<JsonVariantConst>());
    resetDebounce();
  } else if (strcmp(cmd, "status") == 0) {
    handleStatus(id);
  } else if (strcmp(cmd, "history") == 0) {
    handleHistory(id, req.as<JsonVariantConst>());
  } else {
    sendError(&id, "unknown_cmd", nullptr);
  }

  digitalWrite(LEDPIN, LED_OFF);
}

// --- Command handlers --------------------------------------------------------

void handleReadPump(JsonVariantConst id, JsonVariantConst req) {
  SampleParams p;
  const char *bad_field = nullptr;
  if (!parseParams(req, &p, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  Reading r;
  sampleWindow(&r, &p);
  if (!gateReading(id, &r, &p)) {
    return;  // an explanatory error has already been sent
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  // Still three-valued, and still unlatched. This is an INSTANTANEOUS
  // measurement with whatever thresholds the request asked for -- it is what
  // tools/pump_tune.py calibrates against, so it must not quietly start
  // returning the detector's latched verdict instead. "uncertain" is a real
  // outcome for one window and collapsing it into true or false would invent a
  // decision this reading is not in a position to make. The detector's answer,
  // which does have history behind it, is deliberately NOT mixed in here -- it
  // lives on the event lines and in `status`, where it cannot be confused with
  // this one.
  doc["state"] = classify(&r, &p);
  addMeasurement(doc, &r, &p);
  addContext(doc, &r, &p);
  sendResponse(doc);
}

void handleSampling(JsonVariantConst id, JsonVariantConst req) {
  SampleParams p;
  const char *bad_field = nullptr;
  if (!parseParams(req, &p, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  Reading r;
  sampleWindow(&r, &p);

  // Unlike read_pump this does NOT gate on plausibility. Diagnosing a sensor
  // fault is the main reason to call it, so refusing to show the samples that
  // prove the fault would defeat the purpose. The bias check is reported as a
  // flag instead of an error.
  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["state"] = classify(&r, &p);
  doc["bias_ok"] = (r.bias >= (float)p.bias_min && r.bias <= (float)p.bias_max);
  addMeasurement(doc, &r, &p);
  addContext(doc, &r, &p);

  // Raw counts, oldest first, contiguous from the start of the window. Plot
  // these to set the module's gain pot: a clean sine well inside 0..ADC_MAX is
  // the target, and flat tops mean the gain is too high. On this front end the
  // flat top appears at ~2480 counts, not at the rail -- see asym.
  int dump = (p.dump_n < r.n) ? p.dump_n : r.n;
  JsonArray samples = doc["samples"].to<JsonArray>();
  for (int i = 0; i < dump; i++) {
    samples.add(sampleBuf[i]);
  }
  doc["dump_n"] = dump;

  sendResponse(doc);
}

// Identity, limits, defaults -- and the detector's live state.
//
// This is the Pi's handshake AND its resync primitive. On every connect it reads
// `state` and `since_ms` and back-dates a change that happened while it was
// away; `uptime_ms` is what tells it the board rebooted, so since_ms is never
// extrapolated across a reboot it did not see. `seq` tells it whether it has
// missed event lines, and `history` gets them back.
void handleStatus(JsonVariantConst id) {
  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["fw"] = FW_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["uptime_ms"] = millis();

  // Detector state. Unsigned subtraction, so the millis() wrap is handled here
  // and never by the Pi.
  doc["state"] = STATE_NAME[detState];
  doc["since_ms"] = (uint32_t)(millis() - detSinceMs);
  doc["seq"] = evSeq;
  doc["dropped"] = nDropped;
  doc["n_freq_reject"] = nFreqReject;
  doc["n_headroom"] = nHeadroom;

  // Limits and defaults, so the Pi can discover them instead of hardcoding a
  // second copy of these constants.
  doc["adc_bits"] = ADC_BITS;
  doc["adc_max"] = ADC_MAX;
  doc["max_samples"] = MAX_SAMPLES;
  doc["cycles_default"] = DEFAULT_CYCLES;
  doc["max_cycles"] = MAX_CYCLES;
  doc["mains_hz_default"] = DEFAULT_MAINS_HZ;
  doc["rate_hz_default"] = DEFAULT_RATE_HZ;
  doc["max_rate_hz"] = MAX_RATE_HZ;
  doc["on_counts_default"] = DEFAULT_ON_COUNTS;
  doc["off_counts_default"] = DEFAULT_OFF_COUNTS;
  doc["bias_min_default"] = DEFAULT_BIAS_MIN;
  doc["bias_max_default"] = DEFAULT_BIAS_MAX;
  doc["max_dump_n"] = MAX_DUMP_N;
  doc["line_max"] = LINE_MAX;

  // The detector's own constants. Not settable -- published so nothing
  // downstream has to keep a second copy that can drift.
  doc["detect_cycles"] = DETECT_CYCLES;
  doc["debounce"] = DEBOUNCE_WINDOWS;
  doc["hb_ms"] = HEARTBEAT_MS;
  doc["history_max"] = HISTORY_MAX;
  doc["freq_min_rms"] = FREQ_MIN_RMS;
  doc["freq_match_hz"] = FREQ_MATCH_HZ;
  doc["asym_min"] = ASYM_MIN;
  sendResponse(doc);
}

// Replay the transitions the Pi missed while it was away.
//
// This is what makes a Pi reboot lossless. Without it a pump cycle that both
// starts AND ends while the Pi is down disappears with no trace at all -- the
// daily runtime is quietly short and nothing indicates it.
void handleHistory(JsonVariantConst id, JsonVariantConst req) {
  unsigned long after_seq = 0;  // absent means "everything still buffered"
  const char *bad_field = nullptr;
  if (!optULong(req, "after_seq", &after_seq, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";

  JsonArray events = doc["events"].to<JsonArray>();
  bool more = false;
  int sent = 0;
  for (uint8_t i = 0; i < histCount; i++) {
    const HistEntry &e = hist[(uint8_t)((histHead + HISTORY_MAX - histCount + i) % HISTORY_MAX)];
    if (e.seq <= (uint32_t)after_seq) continue;
    if (sent >= HISTORY_REPLY_MAX) {
      // Capped, not exhausted. The Pi calls again with the last seq it got.
      more = true;
      break;
    }
    JsonObject o = events.add<JsonObject>();
    o["seq"] = e.seq;
    o["state"] = STATE_NAME[e.state];
    o["ms"] = e.ms;
    sent++;
  }

  // True when entries newer than after_seq have already been overwritten, so
  // some transitions are permanently gone. The Pi has to know that rather than
  // assuming an empty or short reply means it got everything.
  doc["truncated"] = (histEvictedSeq > (uint32_t)after_seq);
  doc["more"] = more;
  sendResponse(doc);
}

// --- Request parameters ------------------------------------------------------

// Read one optional numeric field. Absent (or explicitly null) leaves *out at
// its default; present but not a number is a bad_param, reported by name.
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

bool optFloat(JsonVariantConst req, const char *key, float *out, const char **bad_field) {
  JsonVariantConst v = req[key];
  if (v.isNull()) return true;
  if (!v.is<float>()) { *bad_field = key; return false; }
  *out = v.as<float>();
  return true;
}

// The documented defaults, in one place, because the detector needs exactly the
// same set with nothing from the wire in it.
void setDefaults(SampleParams *p) {
  p->cycles = DEFAULT_CYCLES;
  p->mains_hz = DEFAULT_MAINS_HZ;
  p->rate_hz = DEFAULT_RATE_HZ;
  p->on_counts = DEFAULT_ON_COUNTS;
  p->off_counts = DEFAULT_OFF_COUNTS;
  p->bias_min = DEFAULT_BIAS_MIN;
  p->bias_max = DEFAULT_BIAS_MAX;
  p->counts_per_volt = 0.0f;  // absent means absent; see addMeasurement
  p->dump_n = DEFAULT_DUMP_N;
}

// Fill p from the request. A wrong *type* is an error; an out-of-range value is
// clamped, because every response echoes the effective parameters and so makes
// the clamp visible to the Pi.
bool parseParams(JsonVariantConst req, SampleParams *p, const char **bad_field) {
  setDefaults(p);

  if (!optInt(req, "cycles", &p->cycles, bad_field)) return false;
  if (!optFloat(req, "mains_hz", &p->mains_hz, bad_field)) return false;
  if (!optULong(req, "rate_hz", &p->rate_hz, bad_field)) return false;
  if (!optFloat(req, "on_counts", &p->on_counts, bad_field)) return false;
  if (!optFloat(req, "off_counts", &p->off_counts, bad_field)) return false;
  if (!optInt(req, "bias_min", &p->bias_min, bad_field)) return false;
  if (!optInt(req, "bias_max", &p->bias_max, bad_field)) return false;
  if (!optFloat(req, "counts_per_volt", &p->counts_per_volt, bad_field)) return false;
  if (!optInt(req, "dump_n", &p->dump_n, bad_field)) return false;

  p->cycles = constrain(p->cycles, MIN_CYCLES, MAX_CYCLES);
  p->mains_hz = constrain(p->mains_hz, MIN_MAINS_HZ, MAX_MAINS_HZ);
  p->rate_hz = constrain(p->rate_hz, MIN_RATE_HZ, MAX_RATE_HZ);
  p->on_counts = constrain(p->on_counts, 0.0f, (float)ADC_MAX);
  p->off_counts = constrain(p->off_counts, 0.0f, (float)ADC_MAX);
  p->bias_min = constrain(p->bias_min, 0, ADC_MAX);
  p->bias_max = constrain(p->bias_max, 0, ADC_MAX);
  p->dump_n = constrain(p->dump_n, 0, MAX_DUMP_N);

  // Inverted thresholds are never intentional and would make every reading
  // "uncertain" while looking like a working configuration.
  if (p->on_counts < p->off_counts) {
    *bad_field = "on_counts";
    return false;
  }
  // An empty plausibility window rejects every reading as a sensor fault.
  if (p->bias_min >= p->bias_max) {
    *bad_field = "bias_min";
    return false;
  }
  // A negative or zero scale factor cannot produce a voltage. Silently ignoring
  // it would drop `vrms` from the reply with no explanation.
  if (p->counts_per_volt < 0.0f ||
      (req["counts_per_volt"].is<float>() && p->counts_per_volt == 0.0f)) {
    *bad_field = "counts_per_volt";
    return false;
  }
  return true;
}

// --- Response helpers --------------------------------------------------------

// Seed a response document with the echoed id, type and protocol version.
// proto rides on every line so the Pi can tell what it is talking to from any
// reply, not just the boot banner or a status call.
void beginResponse(JsonDocument &doc, JsonVariantConst id) {
  doc["id"] = id;  // copies the int as sent
  doc["type"] = "resp";
  doc["proto"] = PROTO_VERSION;
}

// What was measured. Added to every reading reply, success or failure, so a
// single logged line explains itself.
void addMeasurement(JsonDocument &doc, const Reading *r, const SampleParams *p) {
  doc["rms_counts"] = r->rms;
  doc["bias_counts"] = r->bias;
  doc["min_counts"] = r->min_counts;
  doc["max_counts"] = r->max_counts;
  // Non-zero means the peaks were cut off at an ADC RAIL, so rms_counts (and any
  // voltage below it) understates the real signal. On this front end it stays 0
  // even while the wave is being flattened -- see asym.
  doc["n_clipped"] = r->n_clipped;
  // Swing above the bias over swing below it. ~1.0 is a clean sine; below
  // ASYM_MIN the LM358 is running out of headroom and flattening the positive
  // peaks hundreds of counts below the rail, where n_clipped cannot see it. A
  // warning, never a fault: the pump is still unambiguously on. Meaningless
  // below FREQ_MIN_RMS, where min and max are just noise.
  doc["asym"] = r->asym;
  // Frequency from rising crossings of the measured bias. Its job is to catch
  // the case a bare RMS threshold cannot: a floating or badly routed input picks
  // up enough hum to clear on_counts without the pump running. Mains reads
  // within a fraction of a hertz of mains_hz; induced rubbish does not. 0 means
  // no frequency could be established -- either nothing crossed, or the trace
  // was too quiet to derive one from (the ordinary pump-off result). It is
  // never a guess: see FREQ_MIN_RMS.
  doc["freq_hz"] = r->freq_hz;
  doc["n_rise"] = r->n_rise;

  // Volts only when the caller supplied the scale factor. The counts-to-volts
  // relationship lives in the module's gain pot, so the board has no way to know
  // it -- and reporting a plausible voltage derived from a guessed constant is
  // worse than reporting none, because nothing downstream can tell it was made
  // up. Same rule the well sensor applies to temperature.
  if (p->counts_per_volt > 0.0f) {
    doc["vrms"] = r->rms / p->counts_per_volt;
    doc["unit"] = "V";
    doc["counts_per_volt"] = p->counts_per_volt;
  }
}

// The sampling conditions plus the effective parameters, so the numbers above
// can be judged without the Pi having to remember what it asked for.
void addContext(JsonDocument &doc, const Reading *r, const SampleParams *p) {
  doc["n_samples"] = r->n;
  doc["window_us"] = r->window_us;
  doc["interval_us"] = r->interval_us;
  doc["max_late_us"] = r->max_late_us;
  // True when the requested cycles did not fit MAX_SAMPLES, so the window covers
  // a fractional number of mains cycles and the RMS carries a phase-dependent
  // error. Lower rate_hz or cycles to clear it.
  doc["truncated"] = r->truncated;
  doc["cycles_eff"] = r->cycles_eff;
  doc["adc_bits"] = ADC_BITS;
  doc["cycles"] = p->cycles;
  doc["mains_hz"] = p->mains_hz;
  doc["rate_hz"] = p->rate_hz;
  doc["on_counts"] = p->on_counts;
  doc["off_counts"] = p->off_counts;
  doc["bias_min"] = p->bias_min;
  doc["bias_max"] = p->bias_max;
}

// Decide whether the reading describes a working sensor at all. When it does
// not, the matching error is sent here and the caller simply returns.
bool gateReading(JsonVariantConst id, const Reading *r, const SampleParams *p) {
  if (r->bias >= (float)p->bias_min && r->bias <= (float)p->bias_max) {
    return true;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "error";
  // Deliberately its own code rather than a "off" verdict: the module is not
  // reporting, so the pump's state is unknown. Retrying will not help.
  doc["code"] = "sensor_fault";
  addMeasurement(doc, r, p);
  addContext(doc, r, p);
  sendResponse(doc);
  return false;
}

// RMS against the two thresholds, for ONE window with no memory.
//
// The band between them is reported as "uncertain" here and resolved by the
// detector, which holds its current state through it -- that band is what the
// hysteresis is made of. Both callers matter: this is the unlatched view that
// tools/pump_tune.py calibrates against, and it is also the raw input the state
// machine consumes.
const char *classify(const Reading *r, const SampleParams *p) {
  if (r->rms >= p->on_counts) return "on";
  if (r->rms <= p->off_counts) return "off";
  return "uncertain";
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

// Solicited replies only, and this one MAY block: somebody asked, so somebody is
// reading, and a `sampling` dump is several kB no TX buffer would ever hold at
// once. The watchdog is the backstop if that assumption turns out to be false.
// Unsolicited lines take emitUnsolicited() instead and never block at all.
void sendResponse(const JsonDocument &doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

// --- Unsolicited output ------------------------------------------------------

// Seed an event line. Note the seq is consumed here, before the line is known to
// be sendable: seq counts events the board GENERATED, and a gap is exactly how
// the Pi learns one was lost. Bumping it only on success would hide the loss.
void beginEvent(JsonDocument &doc, const char *ev) {
  doc["type"] = "event";
  doc["proto"] = PROTO_VERSION;
  doc["role"] = PIJARDIN_ROLE;
  doc["ev"] = ev;
  doc["seq"] = ++evSeq;
}

// Put a float on an event line as a JSON number with exactly two decimals,
// rather than letting ArduinoJson pick a length.
//
// This is a size constraint, not a cosmetic one. The CDC TX buffer is 256 bytes
// and emitUnsolicited() below refuses to write a line that will not fit in it,
// so a line's WORST-CASE length has to be known: an event that could grow past
// the buffer would not be dropped occasionally, it would be dropped every time,
// forever, with only the `dropped` counter to show for it. ArduinoJson's own
// float formatting can run to nine significant digits, which is enough to push a
// heartbeat over. Bounding the text bounds the line -- worst case 251 bytes for
// a heartbeat, 203 for a transition, both with every counter at 2^32.
//
// Two decimals is also all any of these figures means: rms is counts, freq is
// hertz to a hundredth, asym is a ratio near 1.
//
// `buf` must be at least 16 bytes and stay alive until the document is
// serialized -- the callers keep it on their own stack frame for that reason.
void addFixed2(JsonDocument &doc, const char *key, float v, char *buf) {
  dtostrf(v, 0, 2, buf);
  doc[key] = serialized(buf);
}

// Write an unsolicited line, or drop it. NEVER blocks.
//
// If the host is enumerated but not reading, the CDC TX buffer fills and
// Serial.write() stalls -- and a board stalled in Serial.write() has stopped
// watching the pump. So the line is only written when it is already known to
// fit, and otherwise thrown away and counted. Dropping costs nothing but
// timeliness: the transition is in the ring buffer either way, and `history`
// hands it back. Detection must never be hostage to the link.
bool emitUnsolicited(const JsonDocument &doc, bool count_drop) {
  size_t need = measureJson(doc) + 1;  // + the newline
  if (!Serial || (size_t)Serial.availableForWrite() < need) {
    if (count_drop) nDropped++;
    return false;
  }
  serializeJson(doc, Serial);
  Serial.write('\n');
  return true;
}

// --- Detector ----------------------------------------------------------------

// Forget any part-accumulated candidate and start counting again from the
// current state.
//
// Called after read_pump and sampling. Those take the ADC for up to ~1.2 s, and
// the detector is blind for all of it: the windows either side of the gap can
// straddle a switching instant, or simply be separated by long enough that three
// "consecutive" windows no longer mean 600 ms of continuous evidence. A blind
// spot must not be able to manufacture a phantom transition, so the debounce
// starts over rather than resuming mid-count.
void resetDebounce() {
  detPending = detState;
  detPendingN = 0;
}

// One measurement window, then the state machine.
void stepDetector() {
  Reading r;
  sampleWindow(&r, &detParams);

  if (r.headroom_warn) nHeadroom++;

  // Candidate state for this window. Anything inconclusive falls through as
  // "hold what we have", which is the hysteresis.
  DetState cand = detState;
  bool bias_ok = (r.bias >= (float)detParams.bias_min && r.bias <= (float)detParams.bias_max);
  if (!bias_ok) {
    // A real transition with a real event, never reported as `off`. Both a dead
    // module and a stopped pump read a low RMS; the bias is what separates them.
    cand = ST_FAULT;
  } else {
    const char *verdict = classify(&r, &detParams);
    if (strcmp(verdict, "on") == 0) {
      // Loud enough, but is it mains? A high RMS at the wrong frequency is hum
      // or a wiring fault. Hold and count it, so it is visible rather than
      // silent.
      if (r.freq_hz > 0.0f && fabsf(r.freq_hz - detParams.mains_hz) <= FREQ_MATCH_HZ) {
        cand = ST_ON;
      } else {
        nFreqReject++;
      }
    } else if (strcmp(verdict, "off") == 0) {
      cand = ST_OFF;
    }
    // "uncertain" holds. That band is the hysteresis, not a verdict.
  }

  if (cand == detPending) {
    if (detPendingN < 255) detPendingN++;
  } else {
    detPending = cand;
    detPendingN = 1;
  }

  if (detPendingN >= DEBOUNCE_WINDOWS && detPending != detState) {
    declareState(detPending, &r);
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastHbMs) >= HEARTBEAT_MS) {
    // Reset from `now`, not by adding HEARTBEAT_MS, so a long blocking command
    // cannot leave a backlog of heartbeats to fire off back to back.
    lastHbMs = now;
    emitHeartbeat(&r);
  }
}

// Commit a debounced state change: record it, then try to announce it.
void declareState(DetState s, const Reading *r) {
  DetState prev = detState;
  uint32_t prev_ms = detSinceMs;

  detState = s;
  detSinceMs = millis();

  char b_rms[16], b_freq[16], b_asym[16];
  JsonDocument doc;
  beginEvent(doc, "pump");
  doc["state"] = STATE_NAME[s];
  doc["ms"] = detSinceMs;
  addFixed2(doc, "rms_counts", r->rms, b_rms);
  addFixed2(doc, "freq_hz", r->freq_hz, b_freq);
  addFixed2(doc, "asym", r->asym, b_asym);
  doc["prev_state"] = STATE_NAME[prev];
  doc["prev_ms"] = prev_ms;

  // Ring first, wire second. The buffer is the record; the line is only the
  // notification, and it is allowed to fail.
  historyPush(evSeq, s, detSinceMs);
  emitUnsolicited(doc, true);
}

// "I am alive, and this is what I still think." Every HEARTBEAT_MS regardless of
// state, because silence has to mean something and it cannot mean two things.
void emitHeartbeat(const Reading *r) {
  uint32_t now = millis();

  char b_rms[16], b_freq[16], b_asym[16];
  JsonDocument doc;
  beginEvent(doc, "hb");
  doc["state"] = STATE_NAME[detState];
  doc["ms"] = now;
  addFixed2(doc, "rms_counts", r->rms, b_rms);
  addFixed2(doc, "freq_hz", r->freq_hz, b_freq);
  addFixed2(doc, "asym", r->asym, b_asym);
  // Unsigned subtraction: the millis() wrap at ~49.7 days is handled here, once,
  // and never by the Pi.
  doc["since_ms"] = (uint32_t)(now - detSinceMs);
  doc["dropped"] = nDropped;
  doc["n_freq_reject"] = nFreqReject;
  doc["n_headroom"] = nHeadroom;
  emitUnsolicited(doc, true);
}

// Append one transition, overwriting the oldest when full.
void historyPush(uint32_t seq, DetState s, uint32_t ms) {
  if (histCount == HISTORY_MAX) {
    // histHead points at the oldest entry once the ring is full, and it is about
    // to be overwritten. Remembering its seq is what lets `history` answer
    // "some of what you asked for is permanently gone" instead of returning a
    // short list that reads like completeness.
    histEvictedSeq = hist[histHead].seq;
  }
  hist[histHead].seq = seq;
  hist[histHead].ms = ms;
  hist[histHead].state = (uint8_t)s;
  histHead = (uint8_t)((histHead + 1) % HISTORY_MAX);
  if (histCount < HISTORY_MAX) histCount++;
}

// --- Sensing -----------------------------------------------------------------

// Fill sampleBuf with one paced window, then reduce it.
//
// The sample count is chosen so the window spans p->cycles whole mains cycles at
// the requested rate. When that does not fit the buffer the window is cut to
// MAX_SAMPLES and flagged truncated -- cutting it is better than overrunning
// into unowned memory, and flagging it is better than quietly returning an RMS
// with a phase-dependent error in it.
void sampleWindow(Reading *r, const SampleParams *p) {
  r->interval_us = (unsigned long)(1000000.0f / (float)p->rate_hz + 0.5f);
  if (r->interval_us == 0) r->interval_us = 1;

  long want = (long)((float)p->cycles * (float)p->rate_hz / p->mains_hz + 0.5f);
  if (want < 1) want = 1;
  r->truncated = (want > MAX_SAMPLES);
  r->n = r->truncated ? MAX_SAMPLES : (int)want;

  // Discard the ADC's unreliable first conversions before timing anything.
  for (int i = 0; i < DISCARD_READS; i++) {
    (void)analogRead(VPIN);
  }

  r->max_late_us = 0;
  unsigned long t0 = micros();
  for (int i = 0; i < r->n; i++) {
    // Absolute schedule, not "sleep interval_us each time": a per-iteration
    // delay accumulates the conversion time into the spacing and the real rate
    // drifts below the requested one.
    unsigned long due = t0 + (unsigned long)i * r->interval_us;
    // Signed difference so the comparison survives the micros() wrap at ~71
    // minutes; the whole window is at most a couple of seconds wide.
    while ((long)(micros() - due) < 0) {
      // spin
    }
    unsigned long late = (unsigned long)(long)(micros() - due);
    if (late > r->max_late_us) r->max_late_us = late;

    sampleBuf[i] = (uint16_t)analogRead(VPIN);
  }
  r->window_us = micros() - t0;

  // What the window really covered, in cycles -- the honest counterpart of
  // p->cycles once truncation and any pacing slip are accounted for.
  r->cycles_eff = ((float)r->window_us / 1000000.0f) * p->mains_hz;

  analyse(r);
}

// Reduce sampleBuf to bias, RMS, extremes, headroom and frequency.
void analyse(Reading *r) {
  r->bias = 0.0f;
  r->rms = 0.0f;
  r->min_counts = 0;
  r->max_counts = 0;
  r->n_clipped = 0;
  r->asym = 0.0f;
  r->headroom_warn = false;
  r->n_rise = 0;
  r->freq_hz = 0.0f;
  if (r->n <= 0) {
    return;
  }

  // Pass 1: mean, extremes, clipping. The mean IS the bias -- it is measured
  // rather than assumed to be ADC_MAX/2 because the module derives its bias from
  // its own supply and sets it with a pot, so the real idle level drifts with
  // both. Assuming mid-scale would leak that offset straight into the RMS as a
  // fake signal, which is worst precisely where it matters: near "off".
  uint32_t sum = 0;
  r->min_counts = sampleBuf[0];
  r->max_counts = sampleBuf[0];
  for (int i = 0; i < r->n; i++) {
    uint16_t v = sampleBuf[i];
    sum += v;
    if (v < r->min_counts) r->min_counts = v;
    if (v > r->max_counts) r->max_counts = v;
    if (v <= CLIP_MARGIN || v >= ADC_MAX - CLIP_MARGIN) r->n_clipped++;
  }
  r->bias = (float)sum / (float)r->n;

  // Pass 2: RMS about the measured bias.
  //
  // Two passes, not the one-pass E[x^2] - E[x]^2 identity, and the samples are
  // buffered specifically to allow it. With a bias near 2048 counts and an "off"
  // signal of a few counts, those two terms agree to five significant figures
  // and subtracting them cancels away the entire answer. The failure is silent
  // and it lands exactly on the reading that decides "off" versus "on".
  //
  // The accumulator is double because the sum of squares reaches ~5e9, past
  // where a 32-bit float still resolves single units. There is no FPU here, so
  // this is software arithmetic -- a thousand-odd operations once per window,
  // which is nothing next to the time it just spent sampling.
  double sumsq = 0.0;
  for (int i = 0; i < r->n; i++) {
    double d = (double)sampleBuf[i] - (double)r->bias;
    sumsq += d * d;
  }
  r->rms = (float)sqrt(sumsq / (double)r->n);

  // Headroom: the swing above the bias against the swing below it. See ASYM_MIN
  // for why this is a ratio about the MEASURED bias rather than a comparison
  // against a hardcoded ceiling.
  float pos = (float)r->max_counts - r->bias;
  float neg = r->bias - (float)r->min_counts;
  if (neg > 0.0f) {
    r->asym = pos / neg;
    // Only above FREQ_MIN_RMS. On a pump-off floor pos and neg are two or three
    // counts of DNL-inflated noise and their ratio is a coin toss.
    r->headroom_warn = (r->rms >= FREQ_MIN_RMS && r->asym < ASYM_MIN);
  }

  // Pass 3: rising crossings of the bias, with a Schmitt band so noise around
  // the crossing is counted once rather than as a burst. The band scales with
  // the signal -- a quarter of RMS sits well inside a sine's 1.41x RMS peak, and
  // therefore far below any flattening at the top of the wave, which is why soft
  // clipping does not disturb freq_hz.
  //
  // Skipped entirely below FREQ_MIN_RMS. A flat, pump-off trace is a few counts
  // of ADC noise, and noise crosses any band this small over and over: measured,
  // 3 counts of noise against a 2-count band yields 49 "crossings" and a
  // confident 245 Hz. That is not a frequency, and reporting it as one is worse
  // than reporting nothing, because it looks like evidence.
  if (r->rms >= FREQ_MIN_RMS) {
    const float band = r->rms * 0.25f;
    const float hi = r->bias + band;
    const float lo = r->bias - band;

    int side = 0;         // -1 below the band, +1 above it, 0 = not established
    int first = -1, last = -1;  // sample index of the first and last rise
    for (int i = 0; i < r->n; i++) {
      float v = (float)sampleBuf[i];
      if (v > hi) {
        if (side < 0) {  // came up through the whole band: one period boundary
          if (first < 0) first = i;
          last = i;
          r->n_rise++;
        }
        side = 1;
      } else if (v < lo) {
        side = -1;
      }
    }

    // Frequency from the span between the first and last rise, not from
    // crossings-per-window. The window's edges fall at an arbitrary phase, and
    // the first rise is never counted at all -- `side` has no established value
    // until the signal has been below the band once. Dividing by the full window
    // therefore loses most of a cycle: measured, a clean 10-cycle 50 Hz trace
    // counts 9 rises and reports 45 Hz. Those 9 rises do bracket exactly 8 whole
    // periods, so the span between them is exact and needs no edge correction.
    if (r->n_rise >= 2 && r->window_us > 0) {
      // Derived from the measured window rather than the requested rate, so any
      // pacing slip (see max_late_us) is already folded in.
      float interval_s = ((float)r->window_us / 1000000.0f) / (float)r->n;
      float span_s = (float)(last - first) * interval_s;
      if (span_s > 0.0f) {
        r->freq_hz = (float)(r->n_rise - 1) / span_s;
      }
    }
  }
}
