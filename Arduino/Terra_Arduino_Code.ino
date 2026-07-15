#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <math.h>

// ===============================================================
// WIFI / DASHBOARD
// ===============================================================
char ssid[] = "MiraCosta-WiFi";

const char* laptopIP = "10.8.35.240";
const int PORT = 12345;

WiFiUDP udp;

bool udpReady = false;
unsigned long lastWiFiAttempt = 0;
const unsigned long WIFI_RETRY_MS = 10000;


// ===============================================================
// SENSORS
// ===============================================================
const int SOIL_PIN = A0;
const int THERM_PIN = A1;
const int LIGHT_PIN = A2;

// Change this to true after connecting the photoresistor.
const bool PHOTORESISTOR_CONNECTED = false;

const float beta = 3950.0;
const float resistance = 10.0;

const int WET = 70;
const int DRY = 440;

const int NUMBER_OF_READINGS = 20;
const unsigned long READING_INTERVAL_MS = 50;


// ===============================================================
// L298N MOTOR DRIVER
//
// ENA -> D5
// IN1 -> D2
// IN2 -> D3
// IN3 -> D4
// IN4 -> D7
// ENB -> D6
// ===============================================================
const int ENA = 5;
const int IN1 = 2;
const int IN2 = 3;

const int IN3 = 4;
const int IN4 = 7;
const int ENB = 6;

const int MOTOR_SPEED = 200;


// ===============================================================
// SAMPLING STATE MACHINE
// ===============================================================
enum SamplingState {
  SAMPLE_IDLE,
  SAMPLE_SETTLING,
  SAMPLE_LOWERING,
  SAMPLE_STABILIZING,
  SAMPLE_READING,
  SAMPLE_RAISING
};

SamplingState samplingState = SAMPLE_IDLE;

unsigned long stateStartTime = 0;
unsigned long lastReadingTime = 0;

const unsigned long SETTLE_TIME_MS = 500;
const unsigned long SIMULATED_LOWER_TIME_MS = 2000;
const unsigned long SENSOR_STABILIZE_TIME_MS = 2000;
const unsigned long SIMULATED_RAISE_TIME_MS = 2000;

long soilTotal = 0;
long lightTotal = 0;
float temperatureTotal = 0;

int readingCount = 0;
int validTemperatureCount = 0;
unsigned long sampleID = 0;


// ===============================================================
// SETUP
// ===============================================================
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  stopMotors();

  // Start Wi-Fi without trapping the rover in a blocking loop.
  WiFi.begin(ssid);
  lastWiFiAttempt = millis();

  Serial.println("Arduino ready");
  Serial.println("Send P to begin a sampling cycle");
}


// ===============================================================
// MAIN LOOP
// ===============================================================
void loop() {
  readPiCommands();
  maintainWiFi();
  updateSamplingCycle();
}


// ===============================================================
// RASPBERRY PI SERIAL COMMANDS
// ===============================================================
void readPiCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    // P begins a sampling cycle.
    if (command == 'P' || command == 'p') {
      if (samplingState == SAMPLE_IDLE) {
        startSamplingCycle();
      }

      continue;
    }

    // X aborts the sampling cycle.
    if (command == 'X' || command == 'x') {
      abortSamplingCycle();
      continue;
    }

    // Ignore movement commands while sampling.
    if (samplingState != SAMPLE_IDLE) {
      stopMotors();
      continue;
    }

    switch (command) {
      case 'F':
      case 'f':
        moveForward();
        break;

      case 'B':
      case 'b':
        moveBackward();
        break;

      case 'L':
      case 'l':
        turnLeft();
        break;

      case 'R':
      case 'r':
        turnRight();
        break;

      case 'S':
      case 's':
        stopMotors();
        break;

      default:
        // Ignore newlines and unknown characters.
        break;
    }
  }
}


// ===============================================================
// SAMPLING CYCLE
// ===============================================================
void startSamplingCycle() {
  stopMotors();

  samplingState = SAMPLE_SETTLING;
  stateStartTime = millis();

  Serial.println("SAMPLING: STOPPING");
  Serial.println("SAMPLING: SETTLING");
}


void updateSamplingCycle() {
  unsigned long currentTime = millis();

  switch (samplingState) {
    case SAMPLE_IDLE:
      // Nothing to do.
      break;

    case SAMPLE_SETTLING:
      if (currentTime - stateStartTime >= SETTLE_TIME_MS) {
        samplingState = SAMPLE_LOWERING;
        stateStartTime = currentTime;

        Serial.println("SAMPLING: LOWERING");
      }
      break;

    case SAMPLE_LOWERING:
      if (currentTime - stateStartTime >= SIMULATED_LOWER_TIME_MS) {
        samplingState = SAMPLE_STABILIZING;
        stateStartTime = currentTime;

        Serial.println("SAMPLING: RACK LOWERED");
        Serial.println("SAMPLING: STABILIZING");
      }
      break;

    case SAMPLE_STABILIZING:
      if (currentTime - stateStartTime >= SENSOR_STABILIZE_TIME_MS) {
        beginSensorReadings();

        samplingState = SAMPLE_READING;
        lastReadingTime = 0;

        Serial.println("SAMPLING: READING");
      }
      break;

    case SAMPLE_READING:
      collectSensorReading(currentTime);
      break;

    case SAMPLE_RAISING:
      if (currentTime - stateStartTime >= SIMULATED_RAISE_TIME_MS) {
        samplingState = SAMPLE_IDLE;

        Serial.println("SAMPLING: RACK RAISED");
        Serial.println("SAMPLING: DONE");
        Serial.println("DONE");
      }
      break;
  }
}


void beginSensorReadings() {
  soilTotal = 0;
  lightTotal = 0;
  temperatureTotal = 0;

  readingCount = 0;
  validTemperatureCount = 0;
}


void collectSensorReading(unsigned long currentTime) {
  if (readingCount > 0 &&
      currentTime - lastReadingTime < READING_INTERVAL_MS) {
    return;
  }

  lastReadingTime = currentTime;

  int soilRaw = analogRead(SOIL_PIN);
  int thermistorRaw = analogRead(THERM_PIN);

  soilTotal += soilRaw;

  if (PHOTORESISTOR_CONNECTED) {
    lightTotal += analogRead(LIGHT_PIN);
  }

  float temperature = calculateTemperature(thermistorRaw);

  if (!isnan(temperature)) {
    temperatureTotal += temperature;
    validTemperatureCount++;
  }

  readingCount++;

  if (readingCount >= NUMBER_OF_READINGS) {
    finishSensorReadings();

    samplingState = SAMPLE_RAISING;
    stateStartTime = currentTime;

    Serial.println("SAMPLING: RAISING");
  }
}


void finishSensorReadings() {
  float averageSoilRaw =
      (float)soilTotal / NUMBER_OF_READINGS;

  int moisturePercent =
      map((int)averageSoilRaw, DRY, WET, 0, 100);

  moisturePercent =
      constrain(moisturePercent, 0, 100);

  float averageTemperature = NAN;

  if (validTemperatureCount > 0) {
    averageTemperature =
        temperatureTotal / validTemperatureCount;
  }

  int averageLight = 0;

  if (PHOTORESISTOR_CONNECTED) {
    averageLight =
        lightTotal / NUMBER_OF_READINGS;
  }

  sampleID++;

  sendCompletedSample(
      moisturePercent,
      averageTemperature,
      averageLight
  );
}


void abortSamplingCycle() {
  stopMotors();

  samplingState = SAMPLE_IDLE;

  Serial.println("SAMPLING: ABORTED");
  Serial.println("ABORTED");
}


// ===============================================================
// COMPLETED DASHBOARD SAMPLE
// ===============================================================
void sendCompletedSample(
    int moisturePercent,
    float temperature,
    int lightRaw
) {
  // Print locally for testing.
  Serial.print("SAMPLE: {\"type\":\"soil_sample\",");
  Serial.print("\"sample_id\":");
  Serial.print(sampleID);
  Serial.print(",\"moisture_percent\":");
  Serial.print(moisturePercent);
  Serial.print(",\"temperature_c\":");

  if (isnan(temperature)) {
    Serial.print("null");
  } else {
    Serial.print(temperature, 1);
  }

  Serial.print(",\"light_raw\":");
  Serial.print(lightRaw);
  Serial.println("}");

  if (!udpReady) {
    Serial.println("SAMPLE: UDP not connected");
    return;
  }

  udp.beginPacket(laptopIP, PORT);

  udp.print("{\"type\":\"soil_sample\",");
  udp.print("\"sample_id\":");
  udp.print(sampleID);
  udp.print(",\"moisture_percent\":");
  udp.print(moisturePercent);
  udp.print(",\"temperature_c\":");

  if (isnan(temperature)) {
    udp.print("null");
  } else {
    udp.print(temperature, 1);
  }

  udp.print(",\"light_raw\":");
  udp.print(lightRaw);
  udp.print("}");

  udp.endPacket();
}


// ===============================================================
// MOTOR FUNCTIONS
// ===============================================================
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}


void moveBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}


void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}


void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}


void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// ===============================================================
// WIFI CONNECTION
// ===============================================================
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);

    if (!udpReady) {
      udp.begin(PORT);
      udpReady = true;
    }

    return;
  }

  digitalWrite(LED_BUILTIN, LOW);

  if (udpReady) {
    udp.stop();
    udpReady = false;
  }

  unsigned long currentTime = millis();

  if (currentTime - lastWiFiAttempt >= WIFI_RETRY_MS) {
    lastWiFiAttempt = currentTime;
    WiFi.begin(ssid);
  }
}


// ===============================================================
// THERMISTOR CALCULATION
// ===============================================================
float calculateTemperature(int analogValue) {
  if (analogValue <= 0) {
    return NAN;
  }

  float ratio =
      (1025.0 * resistance / analogValue - resistance)
      / resistance;

  if (ratio <= 0) {
    return NAN;
  }

  return beta /
         (log(ratio) + beta / 298.0) -
         273.0;
}