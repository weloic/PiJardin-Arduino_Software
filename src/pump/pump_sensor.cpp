// PiJardin pump sensor firmware -- XIAO SAMD21 + ZMPT101B AC voltage module.
//
// Answers one question: is the pump running right now? It does that by looking
// for mains AC on the pump's own feed, which is a far more honest signal than a
// current clamp threshold or a flow switch -- either the contactor is closed and
// there is 230 V on the motor, or there is not.
//
// Talks to the Raspberry Pi over USB serial with the same newline-delimited JSON
// (NDJSON) request/response protocol as the well sensor: one JSON object per
// line, in both directions, every reply echoing the request "id". See README.md.
//
//   Pi  -> {"id":7,"cmd":"read_pump"}\n
//   MCU -> {"id":7,"type":"resp","proto":1,"status":"ok","state":"on",...}\n
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
// The module is powered from the XIAO's 3V3 pad, NOT 5V. See VPIN below.
//
// STATELESSNESS
// -------------
// Like the well sensor, the board keeps no state between requests. Every reading
// is a fresh measurement window; whatever a request does not specify falls back
// to the documented default, and every response echoes the values actually used.
// In particular there is no on/off latching and no hysteresis here -- the board
// reports what it measured plus an explicit "uncertain" verdict when the reading
// falls between the thresholds, and the Pi (which has history) decides. A pump
// that switches faster than the Pi polls will be missed; that is a property of
// polling, not something the board can paper over.
//
// This file duplicates the NDJSON transport from src/puit/puit_sensor.cpp on
// purpose, and is deliberately structured the same way so the shared part can be
// lifted into lib/ later with both callers visible.

#include <Arduino.h>
#include <ArduinoJson.h>

// ZMPT101B analog out. A0 is an ADC input on the XIAO SAMD21.
//
// POWER THE MODULE FROM 3V3, NOT 5V. The ZMPT101B biases its output at Vcc/2 and
// swings around it, so on 5 V it idles at 2.5 V and peaks near 5 V. The SAMD21's
// analog inputs are not 5 V tolerant and its ADC reference is VDDANA = 3.3 V, so
// that arrangement clips every reading and damages the pin. On 3V3 the module
// idles near mid-scale (~1.65 V, ~2048 counts) and cannot overshoot the pin.
#define VPIN A0
#define LEDPIN LED_BUILTIN

// --- Protocol / firmware identity -------------------------------------------
// proto is numbered per firmware, not per repo: this board shares the well
// sensor's envelope (id/type/proto/status/code) but not its command set, so
// there is nothing for it to be "version 2" of. The pump contract starts at 1.
#define FW_VERSION "1.0.0"
#define PROTO_VERSION 1

// --- ADC ---------------------------------------------------------------------
#define ADC_BITS 12
#define ADC_MAX 4095
// A sample this close to either rail is clipped: the true peak was cut off, so
// the RMS below it is an underestimate. Counted, not fatal -- for on/off
// purposes a clipped waveform still unambiguously means "on".
#define CLIP_MARGIN 4
// The SAMD21's first conversion after power-up or a mux change is unreliable.
// Throw a few away before the timed window rather than letting them skew bias.
#define DISCARD_READS 8
// Below this RMS (counts) no frequency is derived at all -- see analyse(). A
// real signal at any usable gain sits in the hundreds, so this only ever gates
// out traces that are pure noise.
#define FREQ_MIN_RMS 10.0f

// --- Sampling window ---------------------------------------------------------
// Samples are stored rather than accumulated on the fly, so the buffer is what
// bounds the window. 1200 x uint16 = 2.4 kB of the 32 kB available.
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
// assume. THESE TWO DEFAULTS ARE PLACEHOLDERS. Run `sampling` against the real
// installation with the pump off and then on, and set them from the two RMS
// figures you get back -- see the calibration section in README.md. Left
// uncalibrated they will misreport.
#define DEFAULT_ON_COUNTS 200.0f
#define DEFAULT_OFF_COUNTS 80.0f

// --- Sensor plausibility -----------------------------------------------------
// A live module idles near mid-scale whatever the pump is doing, because the
// bias is generated from its own supply. A bias far from mid-scale means the
// board is not looking at a working module at all: unpowered, output shorted, or
// -- most likely in the field -- the signal wire off, leaving the ADC input
// floating. That is a hardware fault, and it is the one thing that must never be
// reported as "pump off": both produce a low RMS and they are otherwise
// indistinguishable. Same reasoning as the well sensor separating a dead
// HC-SR04 from one that simply found no echo.
#define DEFAULT_BIAS_MIN 1024        // 25% of full scale
#define DEFAULT_BIAS_MAX 3072        // 75% of full scale

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
  int n_rise;                  // rising crossings of the bias level
  float freq_hz;               // n_rise / window, 0 when nothing crossed
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

// --- Prototypes --------------------------------------------------------------
bool hasContent();
void handleLine(const char *line);
void handleReadPump(JsonVariantConst id, JsonVariantConst req);
void handleSampling(JsonVariantConst id, JsonVariantConst req);
void handleStatus(JsonVariantConst id);
bool optInt(JsonVariantConst req, const char *key, int *out, const char **bad_field);
bool optULong(JsonVariantConst req, const char *key, unsigned long *out, const char **bad_field);
bool optFloat(JsonVariantConst req, const char *key, float *out, const char **bad_field);
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

void setup() {
  // 12-bit conversions: the signal of interest at the "off" end is a handful of
  // counts of noise, and 10-bit would quantise a quarter of the useful range
  // away for no saving.
  analogReadResolution(ADC_BITS);
  pinMode(VPIN, INPUT);

  Serial.begin(9600);

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  // Boot banner: structured "ready" line the Pi waits for after reset.
  JsonDocument doc;
  doc["type"] = "ready";
  doc["proto"] = PROTO_VERSION;
  doc["fw"] = FW_VERSION;
  doc["role"] = "pump";  // two boards speak this envelope; say which one this is
  serializeJson(doc, Serial);
  Serial.println();
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
  digitalWrite(LEDPIN, HIGH);  // lit while the request is being served

  JsonDocument req;
  DeserializationError err = deserializeJson(req, line);
  if (err) {
    sendError(nullptr, "bad_request", nullptr);
    digitalWrite(LEDPIN, LOW);
    return;
  }

  // The id is echoed back verbatim, so it must be an integer: a missing or
  // mistyped id is reported as bad_id rather than silently answered with
  // "id":null, which the Pi could not tell apart from bad_request.
  JsonVariantConst id = req["id"];
  if (!id.is<long>()) {
    sendError(nullptr, "bad_id", nullptr);
    digitalWrite(LEDPIN, LOW);
    return;
  }

  const char *cmd = req["cmd"] | "";
  if (strcmp(cmd, "read_pump") == 0) {
    handleReadPump(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "sampling") == 0) {
    handleSampling(id, req.as<JsonVariantConst>());
  } else if (strcmp(cmd, "status") == 0) {
    handleStatus(id);
  } else {
    sendError(&id, "unknown_cmd", nullptr);
  }

  digitalWrite(LEDPIN, LOW);
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
  // The answer is three-valued, so it cannot be a boolean: "uncertain" is a real
  // outcome (RMS between the thresholds) and collapsing it into either true or
  // false would invent a decision the board is not in a position to make.
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
  // the target, and flat tops mean the gain is too high.
  int dump = (p.dump_n < r.n) ? p.dump_n : r.n;
  JsonArray samples = doc["samples"].to<JsonArray>();
  for (int i = 0; i < dump; i++) {
    samples.add(sampleBuf[i]);
  }
  doc["dump_n"] = dump;

  sendResponse(doc);
}

void handleStatus(JsonVariantConst id) {
  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["fw"] = FW_VERSION;
  doc["role"] = "pump";
  doc["uptime_ms"] = millis();

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

// Fill p from the request. A wrong *type* is an error; an out-of-range value is
// clamped, because every response echoes the effective parameters and so makes
// the clamp visible to the Pi.
bool parseParams(JsonVariantConst req, SampleParams *p, const char **bad_field) {
  p->cycles = DEFAULT_CYCLES;
  p->mains_hz = DEFAULT_MAINS_HZ;
  p->rate_hz = DEFAULT_RATE_HZ;
  p->on_counts = DEFAULT_ON_COUNTS;
  p->off_counts = DEFAULT_OFF_COUNTS;
  p->bias_min = DEFAULT_BIAS_MIN;
  p->bias_max = DEFAULT_BIAS_MAX;
  p->counts_per_volt = 0.0f;  // absent means absent; see addMeasurement
  p->dump_n = DEFAULT_DUMP_N;

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
  // Non-zero means the peaks were cut off, so rms_counts (and any voltage below
  // it) understates the real signal. Harmless for an on/off verdict, but it is
  // the sign that the module's gain pot wants turning down.
  doc["n_clipped"] = r->n_clipped;
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

// RMS against the two thresholds. The band between them is reported rather than
// resolved, because resolving it needs history and the board has none.
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

void sendResponse(const JsonDocument &doc) {
  serializeJson(doc, Serial);
  Serial.println();
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

  // Discard the SAMD21's unreliable first conversions before timing anything.
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

// Reduce sampleBuf to bias, RMS, extremes and frequency.
void analyse(Reading *r) {
  r->bias = 0.0f;
  r->rms = 0.0f;
  r->min_counts = 0;
  r->max_counts = 0;
  r->n_clipped = 0;
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
  // this is software arithmetic -- a thousand-odd operations once per request,
  // which is nothing next to the window it just spent sampling.
  double sumsq = 0.0;
  for (int i = 0; i < r->n; i++) {
    double d = (double)sampleBuf[i] - (double)r->bias;
    sumsq += d * d;
  }
  r->rms = (float)sqrt(sumsq / (double)r->n);

  // Pass 3: rising crossings of the bias, with a Schmitt band so noise around
  // the crossing is counted once rather than as a burst. The band scales with
  // the signal -- a quarter of RMS sits well inside a sine's 1.41x RMS peak.
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
