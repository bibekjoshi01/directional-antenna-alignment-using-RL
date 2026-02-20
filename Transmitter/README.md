# ESP32 UDP Transmitter Firmware

This project runs an ESP32 as a Wi-Fi Access Point (AP) and periodically broadcasts UDP packets to connected clients.

It is designed to be simple and robust:
- Sends packets at a fixed interval.
- Handles receiver disconnect/reconnect automatically.
- Recovers AP/UDP stack if health checks detect a problem.

## Default Configuration

From `transmitter.cpp`:
- SSID: `ESP32_TX_AP`
- Password: `12345678`
- UDP Port: `4210`
- Send Interval: `500 ms`
- Wi-Fi Channel: `6`
- Max Clients: `4`

## How Firmware Works

1. Boot (`setup`)
- Starts serial monitor at `115200`.
- Starts SoftAP.
- Computes AP broadcast address.
- Starts UDP on port `4210`.

2. Main loop (`loop`)
- Every `500 ms`: sends one UDP broadcast packet.
- Every `3 s`: runs AP/UDP health check and restarts AP if unhealthy.
- Logs client count only when it changes.

3. Health Recovery
- If AP mode is lost, AP IP becomes invalid, or UDP send path fails:
  - Firmware marks AP/UDP unhealthy.
  - Next health check restarts AP + UDP cleanly.

This allows normal operation when receiver powers off/on or disconnects and reconnects.

## Packet Format

Each packet payload is plain text:

`PING:<counter>;UP_MS:<millis>`

Example:

`PING:42;UP_MS:21567`

## Serial Logs (Expected)

- AP startup:
  - `[WiFi] Starting SoftAP...`
  - `[WiFi] AP IP: ...`
  - `[WiFi] Broadcast IP: ...`
- Client events:
  - `[WiFi] Connected clients: N`
- UDP transmit:
  - `[UDP] Sent: PING:...`
- No client connected (throttled):
  - `[UDP] No clients connected.`
- Recovery:
  - `[WiFi] AP unhealthy, restarting...`

## Receiver Side Expectation

Receiver should:
- Join `ESP32_TX_AP`
- Listen on UDP port `4210`
- Decode payload as UTF-8 text

## Notes

- This firmware does not require internet access.
- Broadcast is used so any connected receiver on the AP can read packets.
