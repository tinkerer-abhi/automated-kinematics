/*
 * ======================================================================================
 * Automated Kinematics & Damped Harmonic Motion Tracking System
 * High-Precision HC-SR04 Ultrasonic Sensor Firmware for Arduino Uno R3
 *
 * Capabilities:
 *  - Microsecond-accurate timestamping (micros())
 *  - High-rate acquisition (30-50 Hz) without blocking or lockup
 *  - Timeout-guarded pulseIn() to reject missed echoes and prevent freezing
 *  - Temperature-compensated speed of sound (v_sound = 331.3 + 0.606 * T)
 *  - Multi-sensor support (Sensor 1: Primary Axis, Sensor 2: Optional Orthogonal Axis)
 *  - Fast 115200 Baud Serial Streaming in standard CSV format
 *  - Interactive serial commands for Tare (Zeroing) and Temperature Calibration
 * ======================================================================================
 */

// --- PIN DEFINITIONS ---
// Sensor 1 (Primary Tracking Axis, e.g. Vertical Spring / Pendulum Radial)
const uint8_t TRIG_PIN_1 = 9;
const uint8_t ECHO_PIN_1 = 10;

// Sensor 2 (Optional Secondary Axis, e.g. Horizontal Pendulum / 2D motion)
const bool ENABLE_DUAL_SENSOR = false; // Set to true if 2nd HC-SR04 is wired
const uint8_t TRIG_PIN_2 = 7;
const uint8_t ECHO_PIN_2 = 8;

const uint8_t LED_PIN = 13; // Heartbeat LED

// --- TIMING & ACQUISITION PARAMETERS ---
// Sampling interval in milliseconds.
// 25ms to 30ms gives ~33-40 Hz, ideal for damping motion without echo reverberation.
const unsigned long SAMPLE_INTERVAL_MS = 25; 

// Pulse timeout in microseconds. 25000 us corresponds to ~4.3 meters max range.
// Keeping this bounded prevents pulseIn() from locking up for 1 second on missed echoes!
const unsigned long PULSE_TIMEOUT_US = 25000; 

// --- CALIBRATION & PHYSICS CONSTANTS ---
float ambientTemperatureC = 20.0; // Default ambient room temp in Celsius
float speedOfSound_m_per_s = 343.2; // Calculated dynamically from temperature
float tareOffset_m = 0.0;          // Tare distance offset (zero reference)

// State tracking
unsigned long lastSampleTimeMs = 0;
bool streamActive = true;

/**
 * Calculates the speed of sound in air (dry air approx) based on temperature in Celsius:
 * v = 331.3 * sqrt(1 + T / 273.15) ≈ 331.3 + 0.606 * T (m/s)
 */
void updateSpeedOfSound() {
  speedOfSound_m_per_s = 331.3f + (0.606f * ambientTemperatureC);
}

/**
 * Measures raw echo pulse duration (microseconds) for a given HC-SR04 sensor.
 * Returns 0 if echo timed out or sensor disconnected.
 */
unsigned long measureEchoMicroseconds(uint8_t trigPin, uint8_t echoPin) {
  // Ensure trigger pin is LOW before pulsing
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a clean 10us HIGH pulse to trigger the 40 kHz burst
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure round-trip high pulse duration with strict timeout
  unsigned long durationUs = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  return durationUs;
}

/**
 * Converts microsecond round-trip time into distance in meters.
 * distance = (time_in_seconds * speed_of_sound) / 2
 */
float microsecondsToMeters(unsigned long durationUs) {
  if (durationUs == 0) {
    return -1.0f; // Invalid / Out of Range
  }
  // durationUs in seconds: durationUs * 1e-6
  // Round trip distance divided by 2
  float distance = ((float)durationUs * 1e-6f * speedOfSound_m_per_s) / 2.0f;
  return distance;
}

void setup() {
  // Configure sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  digitalWrite(TRIG_PIN_1, LOW);

  if (ENABLE_DUAL_SENSOR) {
    pinMode(TRIG_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
    digitalWrite(TRIG_PIN_2, LOW);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize high-speed serial
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for serial port to connect (native USB boards like Leonardo/Micro)
  }

  updateSpeedOfSound();

  // Print system header
  Serial.println(F("# Automated Kinematics Tracking System v1.0"));
  Serial.println(F("# Hardware: Arduino Uno R3 + HC-SR04"));
  Serial.print(F("# Ambient Temp (C): ")); Serial.println(ambientTemperatureC, 2);
  Serial.print(F("# Speed of Sound (m/s): ")); Serial.println(speedOfSound_m_per_s, 2);
  Serial.println(F("# Commands: 'START', 'STOP', 'TARE', 'TEMP:<degC>'"));
  Serial.println(F("# CSV Header:"));
  Serial.println(F("timestamp_us,sensor_id,distance_m,raw_pulse_us"));
  
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // --- HANDLE SERIAL INBOUND COMMANDS ---
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "STOP") {
      streamActive = false;
      Serial.println(F("# [STATUS] Streaming PAUSED"));
    } else if (cmd == "START") {
      streamActive = true;
      Serial.println(F("# [STATUS] Streaming RESUMED"));
    } else if (cmd == "TARE" || cmd == "ZERO") {
      // Average 5 readings to set tare baseline
      float sum = 0.0;
      int count = 0;
      for (int i = 0; i < 5; i++) {
        unsigned long echo = measureEchoMicroseconds(TRIG_PIN_1, ECHO_PIN_1);
        float d = microsecondsToMeters(echo);
        if (d > 0) {
          sum += d;
          count++;
        }
        delay(15);
      }
      if (count > 0) {
        tareOffset_m = sum / count;
        Serial.print(F("# [CALIBRATION] Tare equilibrium offset set to: "));
        Serial.print(tareOffset_m, 4);
        Serial.println(F(" m"));
      } else {
        Serial.println(F("# [ERROR] Tare failed: sensor read timeout"));
      }
    } else if (cmd.startsWith("TEMP:")) {
      float newTemp = cmd.substring(5).toFloat();
      if (newTemp > -20.0 && newTemp < 60.0) {
        ambientTemperatureC = newTemp;
        updateSpeedOfSound();
        Serial.print(F("# [CALIBRATION] Temperature updated to: "));
        Serial.print(ambientTemperatureC, 1);
        Serial.print(F(" C -> Speed of sound: "));
        Serial.print(speedOfSound_m_per_s, 2);
        Serial.println(F(" m/s"));
      }
    }
  }

  // --- NON-BLOCKING DATA ACQUISITION LOOP ---
  unsigned long currentMillis = millis();
  if (streamActive && (currentMillis - lastSampleTimeMs >= SAMPLE_INTERVAL_MS)) {
    lastSampleTimeMs = currentMillis;

    // Sample Sensor 1
    unsigned long sampleTimestampUs = micros();
    unsigned long echoUs1 = measureEchoMicroseconds(TRIG_PIN_1, ECHO_PIN_1);
    float distM1 = microsecondsToMeters(echoUs1);

    // Apply tare offset if defined (preserves absolute if tareOffset_m == 0.0)
    float reportedDistM1 = (distM1 >= 0.0) ? (distM1 - tareOffset_m) : -1.0;

    // Output CSV packet: timestamp_us,sensor_id,distance_m,raw_pulse_us
    Serial.print(sampleTimestampUs);
    Serial.print(',');
    Serial.print(1);
    Serial.print(',');
    Serial.print(reportedDistM1, 5);
    Serial.print(',');
    Serial.println(echoUs1);

    // Optional Sensor 2 reading
    if (ENABLE_DUAL_SENSOR) {
      delayMicroseconds(2000); // Guard interval between ultrasonic bursts
      unsigned long echoUs2 = measureEchoMicroseconds(TRIG_PIN_2, ECHO_PIN_2);
      float distM2 = microsecondsToMeters(echoUs2);
      unsigned long sampleTimestampUs2 = micros();

      Serial.print(sampleTimestampUs2);
      Serial.print(',');
      Serial.print(2);
      Serial.print(',');
      Serial.print(distM2, 5);
      Serial.print(',');
      Serial.println(echoUs2);
    }
  }
}
