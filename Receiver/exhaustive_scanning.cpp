#include <WiFi.h>
#include <ESP32Servo.h>
#include "SPIFFS.h"
#include <Preferences.h>
Preferences prefs;

void setupSPIFFS()
{
  if (!SPIFFS.begin(true))
  { // format if failed
    Serial.println("Failed to mount SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted");
}

void storeScanPoint(int pan, int tilt, int rssi)
{
  File f = SPIFFS.open("/scan_data.txt", FILE_APPEND);
  if (!f)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  f.printf("%d,%d,%d\n", pan, tilt, rssi); // CSV format: pan,tilt,rssi
  f.close();
}

// -----------------------------------
// WiFi Configuration
// -----------------------------------
const char *ssid = "ESP32_TX_AP";
const char *password = "12345678";

// -----------------------------------
// Pins
// -----------------------------------
#define STEP_PIN 6
#define DIR_PIN 7
#define SERVO_PIN 21

// -----------------------------------
// Scan Parameters
// -----------------------------------
int panMin = 0;
int panMax = 180;
int panStep = 10;

int tiltMin = 25;
int tiltMax = 75;
int tiltStep = 5;

int fineStep = 2;

int samplesPerPoint = 10;
int settleTimeMs = 100;

// -----------------------------------
// Servo
// -----------------------------------
Servo tiltServo;

// -----------------------------------
// Stepper Control
// -----------------------------------
#define STEP_DELAY_US 3000
#define STEPS_PER_DEGREE 5 // Adjust to your mechanics

void stepperMoveDegrees(int degrees)
{
  int steps = abs(degrees) * STEPS_PER_DEGREE;
  digitalWrite(DIR_PIN, degrees >= 0 ? HIGH : LOW);
  delay(5);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// -----------------------------------
// RSSI Measurement
// -----------------------------------
int measureRSSI()
{
  long total = 0;
  for (int i = 0; i < samplesPerPoint; i++)
  {
    total += WiFi.RSSI();
    delay(10);
  }
  return total / samplesPerPoint;
}

// -----------------------------------
// Global Tracking
// -----------------------------------
int currentPan = 0;

// -----------------------------------
// Set Orientation
// -----------------------------------
void setOrientation(int pan, int tilt)
{
  int delta = pan - currentPan;
  stepperMoveDegrees(delta);
  currentPan = pan;

  tiltServo.write(tilt);

  delay(settleTimeMs);
}

// -----------------------------------
// Fine Scan
// -----------------------------------
void runFineScan(int coarsePan, int coarseTilt,
                 int &bestPan, int &bestTilt, int &bestRSSI)
{
  for (int pan = coarsePan - 10; pan <= coarsePan + 10; pan += fineStep)
  {
    for (int tilt = coarseTilt - 10; tilt <= coarseTilt + 10; tilt += fineStep)
    {
      int clippedPan = constrain(pan, panMin, panMax);
      int clippedTilt = constrain(tilt, tiltMin, tiltMax);

      setOrientation(clippedPan, clippedTilt);
      int rssi = measureRSSI();

      // store this point
      storeScanPoint(clippedPan, clippedTilt, rssi);

      if (rssi > bestRSSI)
      {
        bestRSSI = rssi;
        bestPan = clippedPan;
        bestTilt = clippedTilt;
      }

      Serial.printf("Fine Scan -> Pan: %d Tilt: %d RSSI: %d\n",
                    clippedPan, clippedTilt, rssi);
    }
  }
}

// -----------------------------------
// Coarse Scan
// -----------------------------------
void runScan()
{
  int bestRSSI = -1000;
  int bestPan = 0;
  int bestTilt = 0;

  for (int pan = panMin; pan <= panMax; pan += panStep)
  {
    for (int tilt = tiltMin; tilt <= tiltMax; tilt += tiltStep)
    {
      setOrientation(pan, tilt);
      int rssi = measureRSSI();

      // store this point
      storeScanPoint(pan, tilt, rssi);

      Serial.printf("Coarse Scan -> Pan: %d Tilt: %d RSSI: %d\n",
                    pan, tilt, rssi);

      if (rssi > bestRSSI)
      {
        bestRSSI = rssi;
        bestPan = pan;
        bestTilt = tilt;
      }
    }
  }

  Serial.println("Starting Fine Scan...");
  runFineScan(bestPan, bestTilt, bestPan, bestTilt, bestRSSI);

  // move to best rssi point
  delay(settleTimeMs);
  setOrientation(bestPan, bestTilt);

  Serial.println("----- FINAL BEST -----");
  Serial.printf("Best Pan: %d\n", bestPan);
  Serial.printf("Best Tilt: %d\n", bestTilt);
  Serial.printf("Best RSSI: %d\n", bestRSSI);

  prefs.begin("scan_data", false);
  prefs.putInt("best_pan", bestPan);
  prefs.putInt("best_tilt", bestTilt);
  prefs.putInt("best_rssi", bestRSSI);
  prefs.end();
}

// -----------------------------------
// Setup
// -----------------------------------
void setup()
{
  Serial.begin(115200);
  setupSPIFFS(); // mount SPIFFS
  SPIFFS.remove("/scan_data.txt");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  tiltServo.attach(SERVO_PIN);
  tiltServo.write(45);
  delay(1000);
  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());

  runScan(); // run once
}

// -----------------------------------
// Loop
// -----------------------------------
void loop()
{
  // Do nothing after scan
}