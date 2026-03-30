// Online Q-learning firmware for ESP32-S3 (OOP style)
// - Trains on-device from live RSSI
// - Persists Q-table as NumPy .npy in SPIFFS for later reuse

#include <Arduino.h>
#include <SPIFFS.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AccelStepper.h>

/* ==============================
   CONFIG
================================ */

struct RLConfig
{
  // PAN driven by stepper
  static constexpr int PAN_MIN = 0;
  static constexpr int PAN_MAX = 180;
  static constexpr int TILT_MIN = 60;
  static constexpr int TILT_MAX = 120;
  // RL action step size (deg). Increase to move the stepper in larger jumps.
  static constexpr int STEP_DEG = 10;
  static constexpr int STEPS_PER_DEGREE = 5; // hardware resolution stays the same

  static constexpr int PAN_STATES = ((PAN_MAX - PAN_MIN) / STEP_DEG) + 1;
  static constexpr int TILT_STATES = ((TILT_MAX - TILT_MIN) / STEP_DEG) + 1;
  static constexpr int DELTA_STATES = 3; // {-1,0,+1} -> {0,1,2}
  static constexpr int ACTIONS = 5;      // pan+/pan-/tilt+/tilt-/stay

  static constexpr uint32_t CONTROL_DELAY_MS = 300;   // slow the control loop
  static constexpr uint32_t SAVE_INTERVAL_MS = 60000;
  static constexpr float RSSI_THRESHOLD = -90.0f;
  static constexpr int RSSI_SAMPLES = 20;
  static constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
  static constexpr int SETTLE_TIME_MS = 300;          // servo / stepper settle
  static constexpr int STEPPER_MAX_SPEED = 300;       // steps/sec
  static constexpr int STEPPER_ACCEL = 200;           // steps/sec^2
};

constexpr char WIFI_SSID[] = "ESP32_TX_AP";
constexpr char WIFI_PASSWORD[] = "12345678";

enum ActionType : uint8_t
{
  ACT_PAN_POS = 0,
  ACT_PAN_NEG = 1,
  ACT_TILT_POS = 2,
  ACT_TILT_NEG = 3,
  ACT_STAY = 4,
};

inline float rand01()
{
  return static_cast<float>(esp_random()) / static_cast<float>(UINT32_MAX);
}

inline float clip(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

inline int delta_rssi_sign(float delta, float eps = 0.5f)
{
  if (delta > eps)
    return 2;
  if (delta < -eps)
    return 0;
  return 1;
}

/* ==============================
   WIFI RSSI HELPER
================================ */

class WifiRssi
{
public:
  void begin(const char *ssid, const char *password, uint32_t timeout_ms)
  {
    ssid_ = ssid;
    pass_ = password;
    timeout_ms_ = timeout_ms;
    connect();
  }

  bool ensure_connected()
  {
    if (WiFi.status() == WL_CONNECTED)
      return true;
    connect();
    return WiFi.status() == WL_CONNECTED;
  }

  float measure(int samples)
  {
    if (!ensure_connected())
      return -200.0f;

    long sum = 0;
    for (int i = 0; i < samples; i++)
    {
      sum += WiFi.RSSI();
      delay(20);
    }
    return static_cast<float>(sum) / samples;
  }

private:
  void connect()
  {
    Serial.print("Connecting WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_, pass_);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms_)
    {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.print("\nWiFi OK, IP: ");
      Serial.println(WiFi.localIP());
    }
    else
    {
      Serial.println("\nWiFi failed, will retry lazily.");
    }
  }

  const char *ssid_ = nullptr;
  const char *pass_ = nullptr;
  uint32_t timeout_ms_ = 15000;
};

/* ==============================
   Q-TABLE
================================ */

class QTable
{
public:
  void optimistic_init(float v)
  {
    for (int p = 0; p < RLConfig::PAN_STATES; p++)
      for (int t = 0; t < RLConfig::TILT_STATES; t++)
        for (int d = 0; d < RLConfig::DELTA_STATES; d++)
          for (int a = 0; a < RLConfig::ACTIONS; a++)
            data[p][t][d][a] = v;
  }

  float get(int state, int action) const
  {
    int idx = state * RLConfig::ACTIONS + action;
    const float *base = &data[0][0][0][0];
    return base[idx];
  }

  void set(int state, int action, float v)
  {
    int idx = state * RLConfig::ACTIONS + action;
    float *base = &data[0][0][0][0];
    base[idx] = v;
  }

  uint8_t *raw() { return reinterpret_cast<uint8_t *>(&data[0][0][0][0]); }
  const uint8_t *raw() const { return reinterpret_cast<const uint8_t *>(&data[0][0][0][0]); }
  size_t bytes() const { return sizeof(data); }

private:
  float data[RLConfig::PAN_STATES][RLConfig::TILT_STATES][RLConfig::DELTA_STATES][RLConfig::ACTIONS];
};

/* ==============================
   PERSISTENCE (NPY v1.0)
================================ */

class NpyStore
{
public:
  bool save(const QTable &q, const char *path)
  {
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f)
    {
      Serial.println("[ERR] Unable to open npy for write");
      return false;
    }

    const char magic[] = "\x93NUMPY";
    uint8_t ver[2] = {1, 0};

    char header[200];
    snprintf(header, sizeof(header),
             "{'descr': '<f4', 'fortran_order': False, 'shape': (%d, %d, %d, %d), }",
             RLConfig::PAN_STATES, RLConfig::TILT_STATES, RLConfig::DELTA_STATES, RLConfig::ACTIONS);

    size_t header_len = strlen(header);
    size_t pad = 16 - ((10 + header_len + 1) % 16);
    for (size_t i = 0; i < pad; i++)
      header[header_len++] = ' ';
    header[header_len++] = '\n';
    header[header_len] = '\0';
    uint16_t hlen_u16 = static_cast<uint16_t>(header_len);

    f.write((const uint8_t *)magic, 6);
    f.write(ver, 2);
    f.write((uint8_t *)&hlen_u16, 2);
    f.write((const uint8_t *)header, header_len);

    size_t wrote = f.write(q.raw(), q.bytes());
    f.close();

    if (wrote != q.bytes())
    {
      Serial.println("[ERR] npy data write truncated");
      return false;
    }
    Serial.printf("[OK] Saved Q-table (%u bytes)\n", (unsigned)q.bytes());
    return true;
  }

  bool load(QTable &q, const char *path)
  {
    File f = SPIFFS.open(path, FILE_READ);
    if (!f)
      return false;

    uint8_t magic[6];
    if (f.read(magic, 6) != 6 || memcmp(magic, "\x93NUMPY", 6) != 0)
    {
      Serial.println("[WARN] npy magic mismatch");
      f.close();
      return false;
    }

    uint8_t ver[2];
    if (f.read(ver, 2) != 2)
    {
      f.close();
      return false;
    }
    uint16_t hlen = 0;
    if (f.read((uint8_t *)&hlen, 2) != 2)
    {
      f.close();
      return false;
    }

    if (!f.seek(10 + hlen))
    {
      f.close();
      return false;
    }

    size_t read_bytes = f.read(q.raw(), q.bytes());
    f.close();

    if (read_bytes != q.bytes())
    {
      Serial.println("[WARN] npy size mismatch; starting fresh");
      return false;
    }
    Serial.println("[OK] Loaded Q-table from SPIFFS");
    return true;
  }
};

/* ==============================
   HARDWARE ENVIRONMENT
================================ */

class ServoEnv
{
public:
  void begin()
  {
    panStepper.setMaxSpeed(RLConfig::STEPPER_MAX_SPEED);
    panStepper.setAcceleration(RLConfig::STEPPER_ACCEL);

    tiltServo.attach(TILT_SERVO_PIN, 500, 2500); // explicit pulse bounds for ESP32
    pan = (RLConfig::PAN_MIN + RLConfig::PAN_MAX) / 2;
    tilt = (RLConfig::TILT_MIN + RLConfig::TILT_MAX) / 2;
    panStepper.setCurrentPosition(static_cast<long>(pan) * RLConfig::STEPS_PER_DEGREE);
    move_pan_to(pan);
    move_tilt_to(tilt);
  }

  void apply(ActionType action)
  {
    switch (action)
    {
    case ACT_PAN_POS:
      pan += RLConfig::STEP_DEG;
      break;
    case ACT_PAN_NEG:
      pan -= RLConfig::STEP_DEG;
      break;
    case ACT_TILT_POS:
      tilt += RLConfig::STEP_DEG;
      break;
    case ACT_TILT_NEG:
      tilt -= RLConfig::STEP_DEG;
      break;
    case ACT_STAY:
    default:
      break;
    }
    pan = constrain(pan, RLConfig::PAN_MIN, RLConfig::PAN_MAX);
    tilt = constrain(tilt, RLConfig::TILT_MIN, RLConfig::TILT_MAX);
    int current_pan_deg = static_cast<int>(panStepper.currentPosition() / RLConfig::STEPS_PER_DEGREE);
    if (pan != current_pan_deg)
      move_pan_to(pan);
    if (tilt != tiltServo.read())
      move_tilt_to(tilt);
  }

  int encode_state(int delta_sign) const
  {
    int pan_i = (pan - RLConfig::PAN_MIN) / RLConfig::STEP_DEG;
    int tilt_i = (tilt - RLConfig::TILT_MIN) / RLConfig::STEP_DEG;
    return ((pan_i * RLConfig::TILT_STATES + tilt_i) * RLConfig::DELTA_STATES) + delta_sign;
  }

  int current_pan() const { return pan; }
  int current_tilt() const { return tilt; }
  void service_stepper() { panStepper.run(); }

private:
  void move_pan_to(int target_deg)
  {
    target_deg = constrain(target_deg, RLConfig::PAN_MIN, RLConfig::PAN_MAX);
    long target_steps = static_cast<long>(target_deg) * RLConfig::STEPS_PER_DEGREE;
    panStepper.moveTo(target_steps);
    pan = target_deg;
  }

  void move_tilt_to(int target_deg)
  {
    target_deg = constrain(target_deg, RLConfig::TILT_MIN, RLConfig::TILT_MAX);
    tiltServo.write(target_deg);
  }

  int current_pan_deg() const { return pan; }

  AccelStepper panStepper = AccelStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
  Servo tiltServo;
  int pan = 90;
  int tilt = 90;

  // Hardware pins
  static constexpr int PAN_STEP_PIN = 6;
  static constexpr int PAN_DIR_PIN = 7;
  static constexpr int TILT_SERVO_PIN = 21;
};

/* ==============================
   Q-LEARNING AGENT
================================ */

class QAgent
{
public:
  void configure(float alpha_, float gamma_, float eps, float eps_min, float eps_decay)
  {
    alpha = alpha_;
    gamma = gamma_;
    epsilon = eps;
    epsilon_min = eps_min;
    epsilon_decay = eps_decay;
  }

  int select_action(const QTable &q, int state)
  {
    if (rand01() < epsilon)
      return esp_random() % RLConfig::ACTIONS;
    float best_q = -1e9f;
    int best_a = 0;
    for (int a = 0; a < RLConfig::ACTIONS; a++)
    {
      float v = q.get(state, a);
      if (v > best_q)
      {
        best_q = v;
        best_a = a;
      }
    }
    return best_a;
  }

  void update(QTable &q, int state, int action, float reward, int next_state)
  {
    float best_next = -1e9f;
    for (int a = 0; a < RLConfig::ACTIONS; a++)
    {
      best_next = max(best_next, q.get(next_state, a));
    }
    float q_sa = q.get(state, action);
    float td_target = reward + gamma * best_next;
    float td_error = td_target - q_sa;
    q_sa = clip(q_sa + alpha * td_error, -10.0f, 10.0f);
    q.set(state, action, q_sa);
  }

  void decay_epsilon()
  {
    if (epsilon > epsilon_min)
    {
      epsilon = max(epsilon_min, epsilon * epsilon_decay);
    }
  }

  float eps() const { return epsilon; }

private:
  float alpha = 0.15f;
  float gamma = 0.05f;
  float epsilon = 1.0f;
  float epsilon_min = 0.05f;
  float epsilon_decay = 0.995f;
};

/* ==============================
   RL CONTROLLER
================================ */

class RLController
{
public:
  void begin()
  {
    Serial.begin(115200);
    delay(200);

    if (!SPIFFS.begin(true))
    {
      Serial.println("[ERR] SPIFFS mount failed");
    }

    wifi.begin(WIFI_SSID, WIFI_PASSWORD, RLConfig::WIFI_TIMEOUT_MS);
    env.begin();
    agent.configure(0.15f, 0.05f, 0.0f, 0.05f, 0.995f); // start greedy for safety
    qtable.optimistic_init(1.0f);
    store.load(qtable, QTABLE_PATH);

    prev_rssi = wifi.measure(RLConfig::RSSI_SAMPLES);
    last_save_ms = millis();
    Serial.println("[INFO] Online Q-learning (OOP) started");
  }

  void tick()
  {
    // non-blocking stepper progress
    env.service_stepper();

    float rssi = wifi.measure(RLConfig::RSSI_SAMPLES);
    if (rssi < RLConfig::RSSI_THRESHOLD)
    {
      Serial.println("[WARN] RSSI below threshold, holding position");
      delay(RLConfig::CONTROL_DELAY_MS);
      return;
    }

    float delta = rssi - prev_rssi;
    int delta_sign = delta_rssi_sign(delta);
    int state = env.encode_state(delta_sign);

    if (have_prev)
    {
      float reward = clip(delta, -5.0f, 5.0f);
      agent.update(qtable, prev_state, prev_action, reward, state);
    }

    int action = agent.select_action(qtable, state);
    env.apply(static_cast<ActionType>(action));

    prev_state = state;
    prev_action = action;
    prev_rssi = rssi;
    have_prev = true;

    step_count++;
    if (step_count % 20 == 0)
      agent.decay_epsilon();

    if (millis() - last_save_ms > RLConfig::SAVE_INTERVAL_MS)
    {
      store.save(qtable, QTABLE_PATH);
      last_save_ms = millis();
    }

    Serial.printf("RSSI: %.1f PAN:%d TILT:%d action:%d eps:%.3f\n",
                  rssi, env.current_pan(), env.current_tilt(), action, agent.eps());
    delay(RLConfig::CONTROL_DELAY_MS);
  }

private:
  static const char QTABLE_PATH[]; // definition follows after class

  ServoEnv env;
  WifiRssi wifi;
  QAgent agent;
  QTable qtable;
  NpyStore store;

  float prev_rssi = -100.0f;
  int prev_state = -1;
  int prev_action = -1;
  bool have_prev = false;
  uint32_t last_save_ms = 0;
  uint32_t step_count = 0;
};

// Definition of static data member
const char RLController::QTABLE_PATH[] = "/q_table.npy";

/* ==============================
   ARDUINO HOOKS
================================ */

static RLController rl;

void setup() { rl.begin(); }
void loop() { rl.tick(); }
