// PiJardin well ("puit") sensor firmware -- XIAO SAMD21 + HC-SR04 ultrasonic.
//
// Talks to the Raspberry Pi (PiJardin/sensors/read_puit.py) over USB serial
// using a newline-delimited JSON (NDJSON) request/response protocol. One JSON
// object per line, in both directions. See README.md for the full contract.
//
//   Pi  -> {"id":42,"cmd":"read_puit"}\n
//   MCU -> {"id":42,"type":"resp","status":"ok","value":123.4,"unit":"cm","n":10,"n_valid":9}\n
//
// Every reply echoes the request "id" so the Pi can correlate it and ignore
// stray lines. Sensor failures are reported as an explicit error code instead
// of a silent 0 cm reading.

#include <ArduinoJson.h>

#define ECHOPIN 7  // Pin to receive echo pulse
#define TRIGPIN 8  // Pin to send trigger pulse
#define LEDPIN LED_BUILTIN

// --- Protocol / firmware identity -------------------------------------------
#define FW_VERSION "1.1.0"
#define PROTO_VERSION 1

// --- Sensing parameters ------------------------------------------------------
#define SAMPLE_COUNT 10          // pings taken per measurement
#define ECHO_TIMEOUT_US 30000UL  // ~5 m max range; missing echo returns 0 fast
#define CM_DIVISOR 58.0f         // pulse-us -> cm (assumes ~20 C / 343 m/s)

// JSON document capacity. The largest payload is the "sampling" response
// (SAMPLE_COUNT floats + a few scalar fields); 512 bytes is comfortable.
#define JSON_CAPACITY 512

// Result of one measurement burst: raw samples plus how many were valid.
struct Measurement {
  float samples[SAMPLE_COUNT];
  int valid_count;
};

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
  if (Serial.available() > 0) {
    digitalWrite(LEDPIN, HIGH);
    String line = Serial.readStringUntil('\n');  // Read one JSON line
    line.trim();                                  // Remove whitespace / CR
    if (line.length() > 0) {
      handleLine(line);
    }
  } else {
    delay(200);
    digitalWrite(LEDPIN, LOW);
  }
}

// Parse one request line and dispatch to the matching handler.
void handleLine(const String &line) {
  JsonDocument req;
  DeserializationError err = deserializeJson(req, line);
  if (err) {
    sendError(nullptr, "bad_request");
    return;
  }

  // "id" is echoed back verbatim; JsonVariant preserves int/null as sent.
  JsonVariantConst id = req["id"];
  const char *cmd = req["cmd"] | "";

  if (strcmp(cmd, "read_puit") == 0) {
    handleReadPuit(id);
  } else if (strcmp(cmd, "sampling") == 0) {
    handleSampling(id);
  } else if (strcmp(cmd, "status") == 0) {
    handleStatus(id);
  } else {
    sendError(&id, "unknown_cmd");
  }
}

// --- Command handlers --------------------------------------------------------

void handleReadPuit(JsonVariantConst id) {
  Measurement m;
  measure(&m);

  if (m.valid_count == 0) {
    sendError(&id, "echo_timeout");
    return;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["value"] = medianOfValid(&m);
  doc["unit"] = "cm";
  doc["n"] = SAMPLE_COUNT;
  doc["n_valid"] = m.valid_count;
  sendResponse(doc);
}

void handleSampling(JsonVariantConst id) {
  Measurement m;
  measure(&m);

  if (m.valid_count == 0) {
    sendError(&id, "echo_timeout");  // whole sensor not responding
    return;
  }

  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["unit"] = "cm";
  doc["n"] = SAMPLE_COUNT;
  doc["n_valid"] = m.valid_count;
  JsonArray samples = doc["samples"].to<JsonArray>();
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (m.samples[i] == 0.0f) {
      samples.add(nullptr);  // timed-out ping -> null, not an ambiguous 0.0
    } else {
      samples.add(m.samples[i]);
    }
  }
  sendResponse(doc);
}

void handleStatus(JsonVariantConst id) {
  JsonDocument doc;
  beginResponse(doc, id);
  doc["status"] = "ok";
  doc["fw"] = FW_VERSION;
  doc["proto"] = PROTO_VERSION;
  doc["uptime_ms"] = millis();
  sendResponse(doc);
}

// --- Response helpers --------------------------------------------------------

// Seed a response document with the echoed id and type.
void beginResponse(JsonDocument &doc, JsonVariantConst id) {
  doc["id"] = id;  // copies null or the int as sent
  doc["type"] = "resp";
}

// Emit an error response. `id` may be null (pass nullptr) for bad_request.
void sendError(const JsonVariantConst *id, const char *code) {
  JsonDocument doc;
  if (id != nullptr) {
    doc["id"] = *id;
  } else {
    doc["id"] = nullptr;
  }
  doc["type"] = "resp";
  doc["status"] = "error";
  doc["code"] = code;
  sendResponse(doc);
}

void sendResponse(const JsonDocument &doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

// --- Sensing -----------------------------------------------------------------

// Fire SAMPLE_COUNT pings. Echo timeouts return 0 from pulseIn and are stored
// as 0 but excluded from valid_count (and thus from the median).
void measure(Measurement *m) {
  m->valid_count = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    digitalWrite(TRIGPIN, LOW);  // Set the trigger pin to low for 2uS
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);  // Send a 10uS high to trigger ranging
    delayMicroseconds(20);
    digitalWrite(TRIGPIN, LOW);   // Send pin low again

    // Read echo pulse width (us) with a bounded timeout, convert to cm.
    unsigned long pulse = pulseIn(ECHOPIN, HIGH, ECHO_TIMEOUT_US);
    if (pulse == 0) {
      m->samples[i] = 0.0f;  // timeout / no echo -> invalid
    } else {
      m->samples[i] = pulse / CM_DIVISOR;
      m->valid_count++;
    }

    delay(3);
  }
}

// Median over the valid (non-zero) samples only. Assumes valid_count > 0.
float medianOfValid(const Measurement *m) {
  float valid[SAMPLE_COUNT];
  int n = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (m->samples[i] != 0.0f) {
      valid[n++] = m->samples[i];
    }
  }
  return findMedian(valid, n);
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

// // Conversion to cm
// The divisor /58 assumes ~20°C (speed ≈ 343 m/s).
// more precise formula: v=331.4+0.6×T(in m/s)

// float temperature = 25;  // Ideally read from a temp sensor
// float speedOfSound = 331.4 + 0.6 * temperature;  // m/s
// float divisor = 1e6 / (speedOfSound * 100) / 2;  // Convert pulse time to cm

// float pulseDuration = pulseIn(ECHOPIN, HIGH);
// float distance = pulseDuration / divisor;
