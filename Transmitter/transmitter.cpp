#include <WiFi.h>
#include <WiFiUdp.h>

// ----------------- CONFIG -----------------
static const char *AP_SSID = "ESP32_TX_AP";
static const char *AP_PASSWORD = "12345678";
static const uint16_t UDP_PORT = 4210;
static const uint32_t SEND_INTERVAL_MS = 500;
static const uint32_t HEALTH_CHECK_INTERVAL_MS = 3000;
static const uint32_t NO_CLIENT_LOG_INTERVAL_MS = 5000;
static const uint8_t WIFI_CHANNEL = 6;
static const uint8_t MAX_CLIENTS = 4;

// ----------------- GLOBALS -----------------
WiFiUDP udp;
IPAddress apIP;
IPAddress apSubnet;
IPAddress broadcastIP;

uint32_t lastSendTime = 0;
uint32_t lastHealthCheck = 0;
uint32_t lastNoClientLog = 0;
uint32_t packetCounter = 0;
int lastClientCount = -1;
bool udpReady = false;

// ----------------- UTILITIES -----------------
static IPAddress calculateBroadcast(const IPAddress &ip, const IPAddress &mask)
{
  return IPAddress(
      (uint8_t)(ip[0] | (uint8_t)(~mask[0])),
      (uint8_t)(ip[1] | (uint8_t)(~mask[1])),
      (uint8_t)(ip[2] | (uint8_t)(~mask[2])),
      (uint8_t)(ip[3] | (uint8_t)(~mask[3])));
}

static bool startSoftAP()
{
  Serial.println("[WiFi] Starting SoftAP...");

  // Full reset keeps AP stable through receiver power cycles/reconnects.
  udp.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(120);

  if (!WiFi.mode(WIFI_AP))
  {
    Serial.println("[WiFi] Failed to set AP mode.");
    udpReady = false;
    return false;
  }

  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL, false, MAX_CLIENTS))
  {
    Serial.println("[WiFi] softAP start failed.");
    udpReady = false;
    return false;
  }

  delay(200);

  apIP = WiFi.softAPIP();
  apSubnet = WiFi.softAPSubnetMask();
  broadcastIP = calculateBroadcast(apIP, apSubnet);

  if (apIP == IPAddress(0, 0, 0, 0))
  {
    Serial.println("[WiFi] AP has invalid IP.");
    udpReady = false;
    return false;
  }

  udpReady = udp.begin(UDP_PORT);
  if (!udpReady)
  {
    Serial.println("[UDP] udp.begin failed.");
    return false;
  }

  Serial.print("[WiFi] AP IP: ");
  Serial.println(apIP);
  Serial.print("[WiFi] Broadcast IP: ");
  Serial.println(broadcastIP);
  Serial.print("[WiFi] Channel: ");
  Serial.println(WIFI_CHANNEL);
  return true;
}

static bool isAPHealthy()
{
  if (WiFi.getMode() != WIFI_AP)
  {
    return false;
  }
  if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0))
  {
    return false;
  }
  return udpReady;
}

static void ensureAPHealthy()
{
  if (isAPHealthy())
  {
    return;
  }

  Serial.println("[WiFi] AP unhealthy, restarting...");
  startSoftAP();
}

static void logClientCountIfChanged()
{
  int clientCount = WiFi.softAPgetStationNum();
  if (clientCount != lastClientCount)
  {
    Serial.printf("[WiFi] Connected clients: %d\n", clientCount);
    lastClientCount = clientCount;
  }
}

static void sendBroadcastPacket(uint32_t now)
{
  if (!udpReady)
  {
    return;
  }

  int clientCount = WiFi.softAPgetStationNum();
  if (clientCount <= 0)
  {
    if (now - lastNoClientLog >= NO_CLIENT_LOG_INTERVAL_MS)
    {
      lastNoClientLog = now;
      Serial.println("[UDP] No clients connected.");
    }
    return;
  }

  char msg[64];
  int len = snprintf(
      msg,
      sizeof(msg),
      "PING:%lu;UP_MS:%lu",
      (unsigned long)packetCounter,
      (unsigned long)now);
  packetCounter++;

  if (len <= 0)
  {
    Serial.println("[UDP] Message build failed.");
    return;
  }

  if (!udp.beginPacket(broadcastIP, UDP_PORT))
  {
    Serial.println("[UDP] beginPacket failed.");
    udpReady = false;
    return;
  }

  udp.write((const uint8_t *)msg, (size_t)len);
  if (!udp.endPacket())
  {
    Serial.println("[UDP] endPacket failed.");
    udpReady = false;
    return;
  }

  Serial.print("[UDP] Sent: ");
  Serial.println(msg);
}

// ----------------- SETUP -----------------
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========== ESP32 UDP TRANSMITTER ==========");
  startSoftAP();
}

// ----------------- MAIN LOOP -----------------
void loop()
{
  uint32_t now = millis();

  if (now - lastHealthCheck >= HEALTH_CHECK_INTERVAL_MS)
  {
    lastHealthCheck = now;
    ensureAPHealthy();
    logClientCountIfChanged();
  }

  if (now - lastSendTime >= SEND_INTERVAL_MS)
  {
    lastSendTime = now;
    sendBroadcastPacket(now);
  }

  delay(2);
}
