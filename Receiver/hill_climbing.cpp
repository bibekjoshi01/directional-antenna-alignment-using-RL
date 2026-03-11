#include <WiFi.h>
#include <ESP32Servo.h>
#include "SPIFFS.h"
#include <Preferences.h>

Preferences prefs;

// ---------------- SPIFFS ----------------
void setupSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS mount failed");
    return;
  }
  Serial.println("SPIFFS mounted");
}

void storeScanPoint(int pan, int tilt, int rssi)
{
  File f = SPIFFS.open("/scan_data.txt", FILE_APPEND);

  if (!f)
  {
    Serial.println("File open failed");
    return;
  }

  f.printf("%d,%d,%d\n", pan, tilt, rssi);
  f.close();
}

// ---------------- WiFi ----------------
const char *ssid = "ESP32_TX_AP";
const char *password = "12345678";

// ---------------- Pins ----------------
#define STEP_PIN 6
#define DIR_PIN 7
#define SERVO_PIN 21

// ---------------- Scan Limits ----------------
int panMin = 0;
int panMax = 180;

int tiltMin = 25;
int tiltMax = 75;

int panStep = 10;
int tiltStep = 5;

int finePanStep = 2;
int fineTiltStep = 2;

int samplesPerPoint = 10;
int settleTimeMs = 100;

Servo tiltServo;

// ---------------- Stepper ----------------

#define STEP_DELAY_US 3000
#define STEPS_PER_DEGREE 5

int currentPan = 0;

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

// ---------------- Orientation ----------------
void setOrientation(int pan, int tilt)
{
  int delta = pan - currentPan;

  stepperMoveDegrees(delta);

  currentPan = pan;

  tiltServo.write(tilt);

  delay(settleTimeMs);
}

// ---------------- RSSI ----------------
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

// ---------------- Hill Climb Core ----------------
void hillClimb(int panStepSize, int tiltStepSize,
               int &pan, int &tilt, int &bestRSSI)
{
  bool improved = true;

  while (improved)
  {
    improved = false;

    int candidatePan[8] =
        {
            pan + panStepSize,
            pan - panStepSize,
            pan,
            pan,
            pan + panStepSize,
            pan + panStepSize,
            pan - panStepSize,
            pan - panStepSize};

    int candidateTilt[8] =
        {
            tilt,
            tilt,
            tilt + tiltStepSize,
            tilt - tiltStepSize,
            tilt + tiltStepSize,
            tilt - tiltStepSize,
            tilt + tiltStepSize,
            tilt - tiltStepSize};

    for (int i = 0; i < 8; i++)
    {
      int p = constrain(candidatePan[i], panMin, panMax);
      int t = constrain(candidateTilt[i], tiltMin, tiltMax);

      setOrientation(p, t);

      int rssi = measureRSSI();

      storeScanPoint(p, t, rssi);

      Serial.printf("Check -> Pan:%d Tilt:%d RSSI:%d\n", p, t, rssi);

      if (rssi > bestRSSI)
      {
        bestRSSI = rssi;
        pan = p;
        tilt = t;
        improved = true;
      }
    }

    if (improved)
    {
      Serial.printf("Move -> Pan:%d Tilt:%d RSSI:%d\n", pan, tilt, bestRSSI);
    }
  }
}

// ---------------- Multi Resolution Hill Climb ----------------
void runHillClimb()
{
  int pan = 90;
  int tilt = 45;

  setOrientation(pan, tilt);

  int bestRSSI = measureRSSI();

  storeScanPoint(pan, tilt, bestRSSI);

  Serial.println("Start Hill Climb");
  Serial.printf("Start RSSI: %d\n", bestRSSI);

  // Stage 1: coarse search
  hillClimb(20, 10, pan, tilt, bestRSSI);

  // Stage 2: medium search
  hillClimb(10, 5, pan, tilt, bestRSSI);

  // Stage 3: fine search
  hillClimb(2, 2, pan, tilt, bestRSSI);

  setOrientation(pan, tilt);

  Serial.println("---- FINAL RESULT ----");

  Serial.printf("Best Pan: %d\n", pan);
  Serial.printf("Best Tilt: %d\n", tilt);
  Serial.printf("Best RSSI: %d\n", bestRSSI);

  prefs.begin("scan_data", false);

  prefs.putInt("best_pan", pan);
  prefs.putInt("best_tilt", tilt);
  prefs.putInt("best_rssi", bestRSSI);

  prefs.end();
}

// ---------------- Setup ----------------
void setup()
{
  Serial.begin(115200);
  setupSPIFFS();
  SPIFFS.remove("/scan_data.txt");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  tiltServo.attach(SERVO_PIN);
  tiltServo.write(45);
  delay(1000);

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Initial RSSI: ");
  Serial.println(WiFi.RSSI());

  runHillClimb();
}

// ---------------- Loop ----------------
void loop()
{
}
