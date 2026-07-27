// PiJardin well ("puit") sensor firmware -- XIAO SAMD21 + HC-SR04 ultrasonic.
//
// Talks to the Raspberry Pi (PiJardin/sensors/read_puit.py) over USB serial
// using a newline-delimited JSON (NDJSON) request/response protocol. One JSON
// object per line, in both directions. See README.md for the full contract.
//
//   Pi  -> {"id":42,"cmd":"read_puit"}\n
//   MCU -> {"id":42,"type":"resp","proto":2,"status":"ok","value":123.4,...}\n
//
// Every reply echoes the request "id" so the Pi can correlate it and ignore
// stray lines. Sensor failures are reported as an explicit error code instead
// of a silent 0 cm reading.
//
// Measurement parameters (sample count, echo timeout, assumed air temperature,
// plausibility window) are per-request fields rather than compile-time
// constants, so the Pi can tune the sensor without a reflash. The board keeps
// no state between requests: whatever a request does not specify falls back to
// the documented default below, and every response echoes the values actually
// used.

#include <ArduinoJson.h>

#define ECHOPIN 7  // Pin to receive echo pulse
#define TRIGPIN 8  // Pin to send trigger pulse
#define LEDPIN LED_BUILTIN

// --- Protocol / firmware identity -------------------------------------------
#define FW_VERSION "2.0.0"
#define PROTO_VERSION 2

// --- Defaults and safe bounds for the per-request parameters -----------------
#define DEFAULT_N 10
#define MAX_N 25                     // sizes the per-ping arrays below
// Deliberately longer than the ~38 ms "nothing found" pulse the HC-SR04 emits:
// waiting past it means a live-but-blind sensor reports a real (if implausible)
// ~652 cm reading instead of looking identical to a dead one. ~772 cm ceiling.
#define DEFAULT_TIMEOUT_US 45000UL
#define MIN_TIMEOUT_US 2000UL
#define MAX_TIMEOUT_US 60000UL       // ~10 m
// The sensor raises Echo a few hundred us after a valid trigger, whether or not
// it will find anything. Nothing within this window means it never answered.
#define ACK_TIMEOUT_US 2000UL
// A ping abandoned early (small timeout_us) can leave Echo still high; wait this
// long for it to settle before triggering again.
#define SETTLE_TIMEOUT_US 40000UL
#define DEFAULT_TEMP_C 20.0f         // equivalent to the historical /58 divisor
#define MIN_TEMP_C -20.0f
#define MAX_TEMP_C 60.0f
#define DEFAULT_MIN_CM 5.0f          // HC-SR04 blind zone: below this is an artefact
// Well of interest is ~220 cm deep, so 500 cm is generous while still rejecting
// the ~652 cm artefact of a "nothing found" pulse (see DEFAULT_TIMEOUT_US).
#define DEFAULT_MAX_CM 500.0f
#define ABS_MAX_CM 1030.0f           // what MAX_TIMEOUT_US physically allows
#define DEFAULT_MIN_VALID_PCT 50     // gate: n_valid must reach 50% of n
#define LINE_MAX 192                 // *request* line cap; responses are unbounded

// How one ping turned out.
//
// PING_NO_RESPONSE and PING_TIMEOUT are the two cases a plain pulseIn() cannot
// tell apart, because both make it return 0: the sensor never reacting to the
// trigger at all (dead, unpowered, broken wire) versus the sensor reacting
// normally but finding nothing to reflect off. The first is a hardware fault,
// the second is an ordinary measurement outcome -- so they are counted apart.
//
// PING_REJECTED exists because a ping is only "valid" if an echo came back AND
// the distance it implies is physically plausible: an echo off the mounting
// bracket or from inside the sensor's blind zone is consistent and would
// otherwise pass as a high-confidence wrong answer.
enum PingStatus : uint8_t { PING_OK, PING_TIMEOUT, PING_REJECTED, PING_NO_RESPONSE };

// Parameters for one measurement burst, after defaults, clamping and validation.
struct MeasureParams {
  int n;                   // pings to fire
  unsigned long timeout_us;
  float temp_c;            // assumed air temperature, drives the divisor
  float min_cm, max_cm;    // plausibility window
  int min_valid;           // absolute count required, 0 disables the gate
};

// Result of one measurement burst: raw pulses, derived distances, per-ping
// outcome, and the statistics over the valid pings only.
struct Measurement {
  unsigned long pulses[MAX_N];  // raw echo widths; 0 only when PING_TIMEOUT
  float samples[MAX_N];         // derived cm, parallel to pulses
  PingStatus status[MAX_N];
  // always: n == n_valid + n_timeout + n_rejected + n_no_response
  int n, n_valid, n_timeout, n_rejected, n_no_response;
  unsigned long median_pulse;
  float median, min, max;       // over PING_OK samples only
};

// --- Request line buffer -----------------------------------------------------
// Fixed size, filled incrementally in loop(). No String, so a garbage stream
// cannot grow the heap, and no reliance on the serial read timeout.
static char lineBuf[LINE_MAX];
static size_t lineLen = 0;
static bool lineOverflow = false;

void setup() {
  // Setup pins for ultrasound puit sensor
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  // Start serial communication
  Serial.begin(9600);

  // Setup LED pin
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  // Boot banner: structured "ready" line the Pi waits for after reset.
  JsonDocument doc;
  doc["type"] = "ready";
  doc["proto"] = PROTO_VERSION;
  doc["fw"] = FW_VERSION;
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
  if (strcmp(cmd, "read_puit") == 0) {
    handleReadPuit(id, req.as<JsonVariantConst>());
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

void handleReadPuit(JsonVariantConst id, JsonVariantConst req) {
  MeasureParams p;
  const char *bad_field = nullptr;
  if (!parseParams(req, &p, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  Measurement m;
  measure(&m, &p);
  if (!gateMeasurement(id, &m, &p)) {
    return;  // an explanatory error has already been sent
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["value"] = m.median;
  doc["unit"] = "cm";
  doc["pulse_us"] = m.median_pulse;  // raw counterpart of value; see README
  addStats(doc, &m);
  addContext(doc, &m, &p);
  sendResponse(doc);
}

void handleSampling(JsonVariantConst id, JsonVariantConst req) {
  MeasureParams p;
  const char *bad_field = nullptr;
  if (!parseParams(req, &p, &bad_field)) {
    sendError(&id, "bad_param", bad_field);
    return;
  }

  Measurement m;
  measure(&m, &p);
  if (!gateMeasurement(id, &m, &p)) {
    return;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["value"] = m.median;
  doc["unit"] = "cm";
  addStats(doc, &m);
  addContext(doc, &m, &p);

  // Per-ping detail for diagnostics. null marks a ping that produced no width
  // at all (no echo, or no response) -- a ping that echoed but fell outside the
  // window keeps its numbers, because raw truth is never discarded, only
  // excluded from the statistics.
  JsonArray samples = doc["samples"].to<JsonArray>();
  JsonArray pulses = doc["pulse_us"].to<JsonArray>();
  for (int i = 0; i < m.n; i++) {
    if (m.status[i] == PING_OK || m.status[i] == PING_REJECTED) {
      samples.add(m.samples[i]);
      pulses.add(m.pulses[i]);
    } else {
      samples.add(nullptr);
      pulses.add(nullptr);
    }
  }

  // One character per ping, in order: V)alid, R)ejected, T)imeout, N)o response.
  // The two null cases above are indistinguishable in the arrays, and reading
  // the pattern beats re-deriving it from min_cm/max_cm -- a glance shows
  // whether losses are scattered (noise) or clustered (intermittent contact).
  char pattern[MAX_N + 1];
  for (int i = 0; i < m.n; i++) {
    switch (m.status[i]) {
      case PING_OK:       pattern[i] = 'V'; break;
      case PING_REJECTED: pattern[i] = 'R'; break;
      case PING_TIMEOUT:  pattern[i] = 'T'; break;
      default:            pattern[i] = 'N'; break;
    }
  }
  pattern[m.n] = '\0';
  doc["ping_status"] = pattern;  // copied into the document, not referenced

  sendResponse(doc);
}

void handleStatus(JsonVariantConst id) {
  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["fw"] = FW_VERSION;
  doc["uptime_ms"] = millis();

  // Limits and defaults, so the Pi can discover them instead of hardcoding a
  // second copy of these constants.
  doc["max_n"] = MAX_N;
  doc["n_default"] = DEFAULT_N;
  doc["timeout_default_us"] = DEFAULT_TIMEOUT_US;
  doc["min_cm_default"] = DEFAULT_MIN_CM;
  doc["max_cm_default"] = DEFAULT_MAX_CM;
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
bool parseParams(JsonVariantConst req, MeasureParams *p, const char **bad_field) {
  p->n = DEFAULT_N;
  p->timeout_us = DEFAULT_TIMEOUT_US;
  p->temp_c = DEFAULT_TEMP_C;
  p->min_cm = DEFAULT_MIN_CM;
  p->max_cm = DEFAULT_MAX_CM;
  int min_valid = -1;  // -1 = not given, derive from n below

  if (!optInt(req, "n", &p->n, bad_field)) return false;
  if (!optULong(req, "timeout_us", &p->timeout_us, bad_field)) return false;
  if (!optFloat(req, "temp_c", &p->temp_c, bad_field)) return false;
  if (!optFloat(req, "min_cm", &p->min_cm, bad_field)) return false;
  if (!optFloat(req, "max_cm", &p->max_cm, bad_field)) return false;
  if (!optInt(req, "min_valid", &min_valid, bad_field)) return false;

  p->n = constrain(p->n, 1, MAX_N);
  p->timeout_us = constrain(p->timeout_us, MIN_TIMEOUT_US, MAX_TIMEOUT_US);
  p->temp_c = constrain(p->temp_c, MIN_TEMP_C, MAX_TEMP_C);
  p->min_cm = constrain(p->min_cm, 0.0f, ABS_MAX_CM);
  p->max_cm = constrain(p->max_cm, 0.0f, ABS_MAX_CM);

  // The echo timeout caps the reachable distance, so a max_cm beyond it is a
  // no-op. Fold the cap in first, so the echoed max_cm is the one truly applied.
  float reach = (float)p->timeout_us / cmDivisorFor(p->temp_c);
  if (p->max_cm > reach) {
    p->max_cm = reach;
  }

  // An empty window rejects every ping and is never intentional -- whether the
  // request inverted the bounds or the timeout is too short to reach min_cm.
  if (p->min_cm >= p->max_cm) {
    *bad_field = "min_cm";
    return false;
  }

  p->min_valid = (min_valid < 0) ? (p->n * DEFAULT_MIN_VALID_PCT + 99) / 100  // ceil
                                 : constrain(min_valid, 0, p->n);
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

// Statistics over the valid pings. Only meaningful when n_valid > 0.
void addStats(JsonDocument &doc, const Measurement *m) {
  doc["min"] = m->min;
  doc["max"] = m->max;
  doc["spread"] = m->max - m->min;
}

// The three per-ping counts plus the effective parameters. Added to every
// measurement reply, success or failure, so a single logged line explains
// itself without the Pi having to remember what it asked for.
void addContext(JsonDocument &doc, const Measurement *m, const MeasureParams *p) {
  doc["n"] = m->n;
  doc["n_valid"] = m->n_valid;
  doc["n_timeout"] = m->n_timeout;
  doc["n_rejected"] = m->n_rejected;
  doc["n_no_response"] = m->n_no_response;
  doc["temp_c"] = p->temp_c;
  doc["min_cm"] = p->min_cm;
  doc["max_cm"] = p->max_cm;
  doc["timeout_us"] = p->timeout_us;
  doc["min_valid"] = p->min_valid;
}

// Decide whether the burst is good enough to report a distance. When it is not,
// the matching error is sent here and the caller simply returns.
bool gateMeasurement(JsonVariantConst id, const Measurement *m, const MeasureParams *p) {
  if (m->n_valid > 0 && m->n_valid >= p->min_valid) {
    return true;
  }

  const char *code;
  if (m->n_valid > 0) {
    code = "insufficient_samples";  // some echoes, too few to trust
  } else if (m->n_no_response > 0 && m->n_no_response >= m->n_timeout &&
             m->n_no_response >= m->n_rejected) {
    // The sensor never even acknowledged the trigger: not a measurement
    // failure but a hardware one -- unpowered, dead, or a wire off. Retrying
    // will not help, so this is deliberately its own code.
    code = "sensor_fault";
  } else if (m->n_rejected > m->n_timeout) {
    // Sensor is alive and answering, but everything it sees is implausible:
    // typically aimed at the shaft wall or a bracket rather than the water.
    code = "out_of_range";
  } else {
    // The sensor answered the trigger but no echo returned in time: aimed at
    // open air, or a surface too absorbent or oblique to reflect.
    code = "echo_timeout";
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "error";
  doc["code"] = code;
  addContext(doc, m, p);
  sendResponse(doc);
  return false;
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

// Pulse width (us) -> distance (cm). The pulse covers the round trip, hence the
// factor 2 folded into the constant: divisor = 2 * 1e6 / (v * 100), v in m/s.
// Sanity: T=20 C -> v=343.4 m/s -> divisor 58.24, matching the historical /58.
// NOTE: the formula previously sketched here as future work was 4x too small.
float cmDivisorFor(float temp_c) {
  float v_m_s = 331.4f + 0.6f * temp_c;
  return 20000.0f / v_m_s;
}

// Fire one ping and measure the echo width in microseconds.
//
// This replaces pulseIn() in order to separate two failures it reports
// identically as 0: the sensor never answering the trigger, and the sensor
// answering but finding nothing in range. *answered distinguishes them.
//   *answered == false, returns 0  -> never reacted (hardware fault)
//   *answered == true,  returns 0  -> reacted, no echo within timeout_us
//   *answered == true,  returns >0 -> echo width in us
//
// Timing is a digitalRead poll rather than pulseIn's tuned assembly, so the
// resolution is a couple of microseconds (~0.03 cm) instead of one -- far below
// the sensor's own real-world accuracy.
unsigned long ping(unsigned long timeout_us, bool *answered) {
  *answered = false;

  // Echo must be idle before triggering: a ping abandoned early (small
  // timeout_us) can still be holding the line high, and measuring the tail of
  // the previous pulse would look like a plausible reading. A line that never
  // settles is itself a fault, reported as "never answered".
  unsigned long t0 = micros();
  while (digitalRead(ECHOPIN) == HIGH) {
    if (micros() - t0 > SETTLE_TIMEOUT_US) return 0;
  }

  digitalWrite(TRIGPIN, LOW);  // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);  // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN, LOW);   // Send pin low again

  // The sensor raises Echo shortly after a valid trigger even when it will find
  // nothing, so no rise at all means it is not responding.
  t0 = micros();
  while (digitalRead(ECHOPIN) == LOW) {
    if (micros() - t0 > ACK_TIMEOUT_US) return 0;
  }
  *answered = true;

  // Echo is high: time how long it stays high.
  unsigned long rise = micros();
  while (digitalRead(ECHOPIN) == HIGH) {
    if (micros() - rise > timeout_us) return 0;
  }
  unsigned long width = micros() - rise;
  return (width == 0) ? 1 : width;  // never return 0 for a real echo
}

// Fire p->n pings and classify each one into the four PingStatus buckets, so
// the Pi can tell a dead sensor from a blind one from a misaimed one. Echoes
// outside [min_cm, max_cm] keep their value but are excluded from the statistics.
void measure(Measurement *m, const MeasureParams *p) {
  const float divisor = cmDivisorFor(p->temp_c);
  m->n = p->n;
  m->n_valid = 0;
  m->n_timeout = 0;
  m->n_rejected = 0;
  m->n_no_response = 0;

  for (int i = 0; i < p->n; i++) {
    bool answered = false;
    unsigned long pulse = ping(p->timeout_us, &answered);
    m->pulses[i] = pulse;

    if (!answered) {
      m->samples[i] = 0.0f;
      m->status[i] = PING_NO_RESPONSE;
      m->n_no_response++;
    } else if (pulse == 0) {
      m->samples[i] = 0.0f;
      m->status[i] = PING_TIMEOUT;
      m->n_timeout++;
    } else {
      float cm = pulse / divisor;
      m->samples[i] = cm;
      if (cm < p->min_cm || cm > p->max_cm) {
        m->status[i] = PING_REJECTED;
        m->n_rejected++;
      } else {
        m->status[i] = PING_OK;
        m->n_valid++;
      }
    }

    delay(3);
  }

  summarize(m);
}

// Median, min and max over the valid pings, plus the raw pulse width that
// corresponds to the median distance. Leaves the statistics zeroed when nothing
// is valid; gateMeasurement() rejects that case before they are ever reported.
void summarize(Measurement *m) {
  m->median = 0.0f;
  m->min = 0.0f;
  m->max = 0.0f;
  m->median_pulse = 0;
  if (m->n_valid == 0) {
    return;
  }

  float cm[MAX_N];
  unsigned long us[MAX_N];
  int k = 0;
  for (int i = 0; i < m->n; i++) {
    if (m->status[i] == PING_OK) {
      cm[k] = m->samples[i];
      us[k] = m->pulses[i];
      k++;
    }
  }

  m->min = cm[0];
  m->max = cm[0];
  for (int i = 1; i < k; i++) {
    if (cm[i] < m->min) m->min = cm[i];
    if (cm[i] > m->max) m->max = cm[i];
  }

  // Both medians come from the same set of pings. cm = pulse / divisor is
  // monotonic, so median_pulse is the exact raw counterpart of median -- no
  // second sort policy to keep in step. (findMedian sorts its argument.)
  m->median = findMedian(cm, k);
  m->median_pulse = findMedianPulse(us, k);
}

float findMedian(float arr[], int n) {
    qsort(arr, n, sizeof(float), compare);

  	// If even, median is the average of the two middle elements
    if (n % 2 == 0) {
        return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
    }
  	// If odd, median is the middle element
  	else {
        return arr[n / 2];
    }
}

int compare(const void *a, const void *b) {
    return (*(float*)a > *(float*)b) - (*(float*)a < *(float*)b);
}

unsigned long findMedianPulse(unsigned long arr[], int n) {
    qsort(arr, n, sizeof(unsigned long), compareUlong);

    if (n % 2 == 0) {
        return (arr[n / 2 - 1] + arr[n / 2]) / 2;
    }
    else {
        return arr[n / 2];
    }
}

int compareUlong(const void *a, const void *b) {
    unsigned long x = *(const unsigned long *)a;
    unsigned long y = *(const unsigned long *)b;
    return (x > y) - (x < y);
}
