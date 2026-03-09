#include <WiFi.h>
#include "SPIFFS.h"
#include <Preferences.h>

// Optional: read NVS for final best
Preferences prefs;

// ----------------- SPIFFS Setup -----------------
void setupSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("Failed to mount SPIFFS");
        return;
    }
    Serial.println("SPIFFS mounted");
}

// ----------------- Read Full Scan File -----------------
void readScanFile()
{
    if (!SPIFFS.exists("/scan_data.txt"))
    {
        Serial.println("No scan data found");
        return;
    }

    File f = SPIFFS.open("/scan_data.txt", FILE_READ);
    Serial.println("------ FULL SCAN DATA ------");
    while (f.available())
    {
        String line = f.readStringUntil('\n'); // format: pan,tilt,rssi
        Serial.println(line);
    }
    f.close();
}

// ----------------- Read Final Best from NVS -----------------
void readFinalBest()
{
    prefs.begin("scan_data", false);
    int bestPan = prefs.getInt("best_pan", -1);
    int bestTilt = prefs.getInt("best_tilt", -1);
    int bestRSSI = prefs.getInt("best_rssi", -100);
    prefs.end();

    if (bestPan == -1)
    {
        Serial.println("No final best stored in NVS.");
    }
    else
    {
        Serial.println("------ FINAL BEST VALUES ------");
        Serial.printf("Pan: %d, Tilt: %d, RSSI: %d\n", bestPan, bestTilt, bestRSSI);
    }
}

// ----------------- Setup -----------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    setupSPIFFS();   // mount filesystem
    readScanFile();  // read and print full scan data
    readFinalBest(); // read and print final best if stored
}

// ----------------- Loop -----------------
void loop()
{
    // Nothing to do
}