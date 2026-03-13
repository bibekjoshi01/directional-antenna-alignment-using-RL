#include <WiFi.h>
#include <ESP32Servo.h>
#include "SPIFFS.h"
#include <Preferences.h>
#include "q_table.h"

Preferences prefs;

// ---------------- WIFI ----------------
const char *ssid = "ESP32_TX_AP";
const char *password = "12345678";

// ---------------- PINS ----------------
#define STEP_PIN 18
#define DIR_PIN 19
#define SERVO_PIN 21

// ---------------- MECHANICS ----------------
#define STEP_DELAY_US 2500
#define STEPS_PER_DEGREE 5

Servo tiltServo;

int currentPan = 0;

// ---------------- RL CONFIG ----------------
#define PAN_MIN 0
#define PAN_MAX 180
#define PAN_STEP 10

#define TILT_MIN 25
#define TILT_MAX 75
#define TILT_STEP 5

#define ACTIONS 5

int samplesPerPoint = 10;
int settleTimeMs = 100;

// ---------------- SPIFFS ----------------
void setupSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS failed");
  }
}

void storeScanPoint(int pan,int tilt,int rssi)
{
  File f = SPIFFS.open("/rl_path.txt", FILE_APPEND);

  if(!f) return;

  f.printf("%d,%d,%d\n",pan,tilt,rssi);

  f.close();
}

// ---------------- STEPPER ----------------
void stepperMoveDegrees(int degrees)
{
  int steps = abs(degrees) * STEPS_PER_DEGREE;

  digitalWrite(DIR_PIN, degrees >= 0 ? HIGH : LOW);

  for(int i=0;i<steps;i++)
  {
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(STEP_DELAY_US);

    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ---------------- ORIENTATION ----------------
void setOrientation(int pan,int tilt)
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

  for(int i=0;i<samplesPerPoint;i++)
  {
    total += WiFi.RSSI();
    delay(10);
  }

  return total / samplesPerPoint;
}

// ---------------- STATE ENCODING ----------------
int encodeState(int pan,int tilt,int deltaSign)
{
  int panIndex = (pan - PAN_MIN) / PAN_STEP;
  int tiltIndex = (tilt - TILT_MIN) / TILT_STEP;

  int deltaIndex = deltaSign + 1;

  int panSteps = (PAN_MAX - PAN_MIN) / PAN_STEP + 1;
  int tiltSteps = (TILT_MAX - TILT_MIN) / TILT_STEP + 1;

  return (panIndex * tiltSteps * 3) +
         (tiltIndex * 3) +
         deltaIndex;
}

// ---------------- ACTION SELECT ----------------
int selectAction(int state)
{
  float best = -1e9;
  int bestAction = 0;

  for(int a=0;a<ACTIONS;a++)
  {
    float val = Q_TABLE[state][a];

    if(val > best)
    {
      best = val;
      bestAction = a;
    }
  }

  return bestAction;
}

// ---------------- APPLY ACTION ----------------
void applyAction(int action,int &pan,int &tilt)
{
  switch(action)
  {
    case 0: pan += PAN_STEP; break;
    case 1: pan -= PAN_STEP; break;
    case 2: tilt += TILT_STEP; break;
    case 3: tilt -= TILT_STEP; break;
    case 4: break;
  }

  pan = constrain(pan,PAN_MIN,PAN_MAX);
  tilt = constrain(tilt,TILT_MIN,TILT_MAX);
}

// ---------------- RL LOOP ----------------
void runRL()
{
  int pan = 90;
  int tilt = 45;

  setOrientation(pan,tilt);

  int prevRSSI = measureRSSI();

  int deltaSign = 0;

  Serial.println("Starting RL inference");

  for(int step=0; step<300; step++)
  {
    int state = encodeState(pan,tilt,deltaSign);

    int action = selectAction(state);

    applyAction(action,pan,tilt);

    setOrientation(pan,tilt);

    int rssi = measureRSSI();

    int delta = rssi - prevRSSI;

    if(delta > 1) deltaSign = 1;
    else if(delta < -1) deltaSign = -1;
    else deltaSign = 0;

    prevRSSI = rssi;

    storeScanPoint(pan,tilt,rssi);

    Serial.printf("Step %d  Pan:%d Tilt:%d RSSI:%d Action:%d\n",
                  step,pan,tilt,rssi,action);
  }

  Serial.println("RL finished");
}

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(115200);

  pinMode(STEP_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);

  tiltServo.attach(SERVO_PIN);

  setupSPIFFS();

  WiFi.begin(ssid,password);

  Serial.print("Connecting");

  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected");

  runRL();
}

// ---------------- LOOP ----------------

void loop()
{
}