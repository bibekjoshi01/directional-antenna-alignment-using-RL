#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ------------------- WiFi -------------------
const char *ssid = "ESP32_TX_AP";
const char *password = "12345678";

// ------------------- Stepper (PAN) -------------------
#define DIR_PIN 7
#define STEP_PIN 6
#define STEPS_PER_DEGREE 2.222
#define STEP_DELAY_US 3000

Adafruit_NeoPixel LED_RGB(1, 48, NEO_GRBW + NEO_KHZ800);

// ------------------- Servo (TILT) -------------------
#define SERVO_PIN 21
Servo tiltServo;

// ------------------- State -------------------
const int SETTLE_TIME_MS = 300;
const int SAMPLES = 20;

// ------------------- WiFi -------------------
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - start > 15000)
    {
      Serial.println("\nWiFi failed");
      return;
    }
    Serial.println("wifi wifi");
  }
  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 255, 0)));
  LED_RGB.show();
  Serial.println("\nConnected");
}

// ------------------- RSSI -------------------
int measureRSSI()
{
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    sum += WiFi.RSSI();
    delay(15);
  }
  return sum / SAMPLES;
}

// ------------------- Movement -------------------
long currentPanStep = 0; // absolute step position
float currentPanDeg = 0; // 0-360 degrees

void movePanTo(float targetDeg)
{
  // --- normalize target ---
  targetDeg = fmod(targetDeg, 360.0);
  if (targetDeg < 0)
    targetDeg += 360.0;

  // --- shortest path ---
  float delta = targetDeg - currentPanDeg;
  if (delta > 180)
    delta -= 360;
  if (delta < -180)
    delta += 360;

  // --- convert to steps (rounded) ---
  long moveSteps = (long)(abs(delta) * STEPS_PER_DEGREE + 0.5);

  // --- set direction ---
  bool dir = (delta >= 0);
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(5);

  // --- step pulses ---
  for (long i = 0; i < moveSteps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  // --- update state ---
  currentPanStep += dir ? moveSteps : -moveSteps;
  currentPanDeg = targetDeg;
}

int currentTiltDeg = 45;

void moveTiltTo(int target)
{
  int TILT_MIN = 25;
  int TILT_MAX = 75;
  target = constrain(target, TILT_MIN, TILT_MAX);
  tiltServo.write(target);
  currentTiltDeg = target;
}

void setOrientation(int pan, int tilt)
{
  movePanTo(pan);
  moveTiltTo(tilt);
  delay(SETTLE_TIME_MS);
}

// ------------------- Hill Climb (2D) -------------------
void hillClimb()
{
  Serial.println("Starting 2D hill climb...");

  int panStep = 10;
  int tiltStep = 5;
  bool improved = true;
  int iter = 0;

  while (improved && iter < 25) // guard against endless loops
  {
    iter++;
    improved = false;
    int bestPan = currentPanDeg;
    int bestTilt = currentTiltDeg;
    int bestRSSI = measureRSSI();

    // 8-direction neighborhood (includes diagonals)
    int neighbors[8][2] = {
        {bestPan + panStep, bestTilt},
        {bestPan - panStep, bestTilt},
        {bestPan, bestTilt + tiltStep},
        {bestPan, bestTilt - tiltStep},
        {bestPan + panStep, bestTilt + tiltStep},
        {bestPan + panStep, bestTilt - tiltStep},
        {bestPan - panStep, bestTilt + tiltStep},
        {bestPan - panStep, bestTilt - tiltStep}};

    for (int i = 0; i < 8; i++)
    {
      int p = neighbors[i][0];
      int t = neighbors[i][1];

      // skip duplicate evaluations
      if (p == currentPanDeg && t == currentTiltDeg)
        continue;

      setOrientation(p, t);
      int rssi = measureRSSI();

      if (rssi > bestRSSI)
      {
        bestRSSI = rssi;
        bestPan = p;
        bestTilt = t;
        improved = true;
      }
    }

    if (improved)
    {
      setOrientation(bestPan, bestTilt);
    }
  }
}

void testMove()
{
  // 8-direction neighborhood (includes diagonals)
  int neighbors[8] = {10, -10, -20, -40, 40, 180, -60, 0};

  for (int i = 0; i < 8; i++)
  {
    setOrientation(neighbors[i], 45);
    delay(3000);
    Serial.println("current angle: ---> " + String(currentPanDeg));
  }
}

// ------------------- Setup -------------------
void setup()
{
  Serial.begin(115200);
  delay(2000); // give time for monitor to connect
  Serial.println("Serial ready");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  LED_RGB.begin();
  LED_RGB.setBrightness(50); // Set brightness to about 60%

  tiltServo.attach(SERVO_PIN, 500, 2500);

  connectWiFi();

  // Initial assumption (manual alignment)
  setOrientation(0, 45);

  hillClimb();
}

// ------------------- Loop -------------------
void loop()
{
  Serial.println("Loop running...");
  delay(1000);
}