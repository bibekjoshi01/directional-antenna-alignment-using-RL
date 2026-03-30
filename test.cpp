#include <WiFi.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// ------------------- WiFi -------------------
const char *ssid = "ESP32_TX_AP";
const char *password = "12345678";

// ------------------- Stepper (PAN) -------------------
#define DIR_PIN 7
#define STEP_PIN 6
#define STEPS_PER_DEGREE 5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ------------------- Servo (TILT) -------------------
#define SERVO_PIN 21
Servo tiltServo;

// ------------------- Parameters -------------------
#define BUTTON_PIN 0

// multi-resolution search: start coarse then refine
const int PAN_STEPS[] = {10, 5, 2};
const int TILT_STEPS[] = {6, 3, 1};
const int NUM_RES = 3;

const int PAN_MIN = 0;
const int PAN_MAX = 180;

const int TILT_MIN = 25;
const int TILT_MAX = 75;

const int SAMPLES = 20;
const int SETTLE_TIME_MS = 300;

int RSSI_THRESHOLD = -72;

// ------------------- State -------------------
int currentPan = 0;
int currentTilt = 45;

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
    }

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
void movePanTo(int target)
{
    target = constrain(target, PAN_MIN, PAN_MAX);

    long targetSteps = target * STEPS_PER_DEGREE;
    stepper.moveTo(targetSteps);

    while (stepper.distanceToGo() != 0)
    {
        stepper.run();
    }

    currentPan = target;
}

void moveTiltTo(int target)
{
    target = constrain(target, TILT_MIN, TILT_MAX);
    tiltServo.write(target);
    currentTilt = target;
}

void setOrientation(int pan, int tilt)
{
    movePanTo(pan);
    delay(SETTLE_TIME_MS);
    moveTiltTo(tilt);
    delay(SETTLE_TIME_MS);
}

void setup()
{
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // stepper setup
    stepper.setMaxSpeed(600); // slower & smoother for accuracy
    stepper.setAcceleration(300);

    // servo setup
    tiltServo.attach(SERVO_PIN, 500, 2500);

    connectWiFi();

    // Initial assumption (manual alignment)
    setOrientation(0, 45);

    // hill climb chalyenw
}

void loop()
{
}