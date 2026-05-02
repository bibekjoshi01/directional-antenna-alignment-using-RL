#include <Arduino.h>
#include <Servo.h>

/* ==============================
   CONFIGURATION
================================ */

#define PAN_MIN 30
#define PAN_MAX 150
#define TILT_MIN 60
#define TILT_MAX 120
#define STEP_DEG 5

#define PAN_STATES 24  // (150-30)/5
#define TILT_STATES 12 // (120-60)/5
#define DELTA_STATES 3
#define ACTIONS 5

#define CONTROL_DELAY 150  // ms
#define RSSI_THRESHOLD -90 // fail-safe threshold

/* ==============================
   ACTION ENUM
================================ */

enum ActionType
{
    ACT_PAN_POS = 0,
    ACT_PAN_NEG = 1,
    ACT_TILT_POS = 2,
    ACT_TILT_NEG = 3,
    ACT_STAY = 4
};

/* ==============================
   Q-TABLE
   (Generated from Python, pasted here)
================================ */

/*
Total size:
24 × 12 × 3 × 5 = 4320 floats
*/

const float Q_TABLE[4320] PROGMEM = {
    // ---- PASTE GENERATED VALUES HERE ----
    // Example placeholder values:
    0.0,
    0.1,
    0.0,
    0.0,
    0.0,
    0.0,
    0.2,
    0.0,
    0.0,
    0.0,
    // ...
};

/* ==============================
   HARDWARE OBJECTS
================================ */

Servo panServo;
Servo tiltServo;

#define PAN_SERVO_PIN 18
#define TILT_SERVO_PIN 19

/* ==============================
   GLOBAL STATE
================================ */

float prev_rssi = -100.0;
int pan = 90;
int tilt = 90;

/* ==============================
   HARDWARE INTERFACE
================================ */

void init_servos()
{
    panServo.attach(PAN_SERVO_PIN);
    tiltServo.attach(TILT_SERVO_PIN);
}

void move_pan_servo(int angle)
{
    panServo.write(angle);
}

void move_tilt_servo(int angle)
{
    tiltServo.write(angle);
}

void init_radio()
{
    // init LoRa / WiFi / RF module here
}

float read_rssi()
{
    // Replace with real RSSI reading
    // Example placeholder:
    return analogRead(34) * (-0.1);
}

/* ==============================
   RL CORE
================================ */

int discretize_angle(int angle, int min_a)
{
    return (angle - min_a) / STEP_DEG;
}

int delta_rssi_sign(float delta)
{
    if (delta > 0.5)
        return 2; // increase
    if (delta < -0.5)
        return 0; // decrease
    return 1;     // stable
}

int encode_state(int pan, int tilt, int delta_sign)
{
    int pan_i = discretize_angle(pan, PAN_MIN);
    int tilt_i = discretize_angle(tilt, TILT_MIN);

    return (((pan_i * TILT_STATES + tilt_i) * DELTA_STATES) + delta_sign);
}

float get_q(int state, int action)
{
    return Q_TABLE[state * ACTIONS + action];
}

int select_best_action(int state)
{
    float best_q = -1e9;
    int best_action = 0;

    for (int a = 0; a < ACTIONS; a++)
    {
        float q = get_q(state, a);
        if (q > best_q)
        {
            best_q = q;
            best_action = a;
        }
    }
    return best_action;
}

void apply_action(int action)
{
    switch (action)
    {
    case ACT_PAN_POS:
        pan += STEP_DEG;
        break;
    case ACT_PAN_NEG:
        pan -= STEP_DEG;
        break;
    case ACT_TILT_POS:
        tilt += STEP_DEG;
        break;
    case ACT_TILT_NEG:
        tilt -= STEP_DEG;
        break;
    case ACT_STAY:
        break;
    }

    pan = constrain(pan, PAN_MIN, PAN_MAX);
    tilt = constrain(tilt, TILT_MIN, TILT_MAX);

    move_pan_servo(pan);
    move_tilt_servo(tilt);
}

/* ==============================
   FAILSAFE
================================ */

void fallback_scan()
{
    // simple sweep recovery
    for (int p = PAN_MIN; p <= PAN_MAX; p += STEP_DEG)
    {
        move_pan_servo(p);
        delay(30);
    }
}

/* ==============================
   SETUP
================================ */

void setup()
{
    Serial.begin(115200);

    init_servos();
    init_radio();

    pan = 90;
    tilt = 90;

    move_pan_servo(pan);
    move_tilt_servo(tilt);

    delay(1000);
}

/* ==============================
   MAIN LOOP
================================ */

void loop()
{

    // 1. Measure RSSI
    float rssi = read_rssi();

    // Fail-safe
    if (rssi < RSSI_THRESHOLD)
    {
        fallback_scan();
        return;
    }

    // 2. Compute delta
    float delta = rssi - prev_rssi;
    int delta_sign = delta_rssi_sign(delta);

    // 3. Encode state
    int state = encode_state(pan, tilt, delta_sign);

    // 4. Select action
    int action = select_best_action(state);

    // 5. Apply action
    apply_action(action);

    // 6. Update RSSI memory
    prev_rssi = rssi;

    // Debug
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" | PAN: ");
    Serial.print(pan);
    Serial.print(" | TILT: ");
    Serial.print(tilt);
    Serial.print(" | ACTION: ");
    Serial.println(action);

    // 7. Control timing
    delay(CONTROL_DELAY);
}
