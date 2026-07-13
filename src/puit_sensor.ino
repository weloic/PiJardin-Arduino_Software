#define ECHOPIN 7  // Pin to receive echo pulse
#define TRIGPIN 8  // Pin to send trigger pulse
#define LEDPIN LED_BUILTIN

void setup() {
  // Setup pins for ultrasound puit sensor
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
  Serial.println("Started serial com");

  // Setup LED pin
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    digitalWrite(LEDPIN, HIGH);
    String command = Serial.readStringUntil('\n'); // Read command until newline
    command.trim(); // Remove whitespace or newlines

    if (command == "READ_PUIT") {
      // return the median of 5 samples
      get_puit_height(0, 10);
    } else if (command == "SAMPLING") {
      // return 10 samples
      get_puit_height(1, 10);
    } else if (command == "STATUS") {
      Serial.println("System OK. Other functions: 'READ_PUIT'");
    } else {
      Serial.println("Unknown command.");
    }
  } else {
    delay(200);
    digitalWrite(LEDPIN, LOW);
  }
}

void get_puit_height(int mode, int number_samples) {
  float distances[number_samples];

  for (int i = 0; i < number_samples; i++) {
    digitalWrite(TRIGPIN, LOW);  // Set the trigger pin to low for 2uS
    delayMicroseconds(2);
    digitalWrite(TRIGPIN, HIGH);  // Send a 10uS high to trigger ranging
    delayMicroseconds(20);
    digitalWrite(TRIGPIN, LOW);   // Send pin low again

    // Convert pulse to centimeters
    distances[i] = pulseIn(ECHOPIN, HIGH) / 58;  // Read in times pulse

    delay(3);
  }

  if (mode == 0) {
    // Return the median
    Serial.println(findMedian(distances, number_samples));
  } else if (mode == 1) {
    // Return every samples in JSON format
    Serial.print("[");
    for (int i = 0; i < number_samples; i++) {
      Serial.print(distances[i], 2);
      if (i < number_samples - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("]");
  }
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

// OLD below:
// float compare(const void *a, const void *b) {
//     return (*(float *)a - *(float *)b);
// }

// // Conversion to cm
// The divisor /58 assumes ~20°C (speed ≈ 343 m/s).
// more precise formula: v=331.4+0.6×T(in m/s)

// float temperature = 25;  // Ideally read from a temp sensor
// float speedOfSound = 331.4 + 0.6 * temperature;  // m/s
// float divisor = 1e6 / (speedOfSound * 100) / 2;  // Convert pulse time to cm

// float pulseDuration = pulseIn(ECHOPIN, HIGH);
// float distance = pulseDuration / divisor;
