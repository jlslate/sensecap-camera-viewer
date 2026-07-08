# SenseCAP Camera Viewer

Firmware that turns a [Seeed Studio SenseCAP Indicator D1](https://www.seeedstudio.com/SenseCAP-Indicator-D1-p-5643.html) (ESP32-S3, 480x480 touchscreen) into a motion-triggered snapshot viewer for a Synology Surveillance Station NAS. When a camera detects motion, Surveillance Station calls the device's web server, which fetches a snapshot over the Synology API and displays it on screen with the camera name and timestamp overlaid.

Built with [PlatformIO](https://platformio.org/) + Arduino framework.

## Features

- Fetches JPEG snapshots from Synology Surveillance Station's `SYNO.SurveillanceStation.Camera` API (not RTSP)
- Auto-discovers every camera configured on the NAS — no need to hardcode camera IDs
- On-screen overlay: camera name and date/time, independently toggleable, with 12/24-hour and Y-M-D/M-D-Y/D-M-Y format options
- On-device photo history (5-frame ring buffer) with left/right touchscreen arrows to browse recent captures, instant redisplay with no re-fetch or re-decode
- Web UI for live viewing, NAS/display settings, and camera list
- WiFi setup via captive portal (WiFiManager) on first boot or if the saved network can't be reached
- Screensaver with tap-to-wake

## Hardware

- SenseCAP Indicator D1 (ESP32-S3 + RP2040, though this firmware only uses the ESP32-S3 side — the SD card slot is wired to the RP2040 and isn't accessible from here)
- A Synology NAS running Surveillance Station, reachable from the device's network

## Building and flashing

```
pio run                                    # build
pio run -t upload --upload-port <port>     # flash
pio device monitor --port <port>           # serial console
```

## First-time setup

1. On first boot (or if saved WiFi can't be reached), the device opens a `SenseCAP-Setup` WiFi access point with a captive portal. Connect to it and fill in your WiFi credentials plus the NAS host/port/username/password.
2. Once connected, open `http://<device-ip>/` for the live viewer, or `http://<device-ip>/settings` for NAS connection and display overlay settings.

## Setting up motion triggers in Surveillance Station

Create an **Action Rule** per camera with a **Webhook** action:

- Method: `GET`
- URL: `http://<device-ip>/motion?cam=%DEVICE_NAME%`

Synology's "Add ingredient" picker provides `%DEVICE_NAME%` (the triggering camera's name) — using it means every camera's rule can share the same URL pattern instead of hardcoding a different numeric camera ID per rule. The device resolves the name to Synology's internal camera ID automatically (falling back to treating the value as a literal numeric ID for old-style rules).

**Camera names must not contain spaces.** Synology's ingredient substitution doesn't URL-encode the value, and a raw space in an HTTP request line truncates the request before it reaches the device. Use single-word names (`KitchenDoor`, not `Kitchen Door`) or `-`/`_` separators. If you'd rather not rename cameras, a hardcoded numeric ID (`cam=14`) always works regardless of spaces — check the current id/name mapping at `http://<device-ip>/cameras`.

## Web endpoints

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Live viewer page |
| `/status` | GET | JSON: current state + last camera shown |
| `/cameras` | GET | JSON camera id/name list (`?refresh=1` to force a re-fetch from the NAS) |
| `/snapshot` | GET | Last fetched JPEG (raw bytes) |
| `/trigger`, `/motion` | GET | `?cam=<id-or-name>` — fetch and display a snapshot |
| `/settings` | GET/POST | NAS connection + display overlay settings |
| `/reconfigure` | GET | Clears WiFi + NAS settings and reboots into setup mode |

## Known hardware quirks

- This specific display has a rotated color channel mapping and a rotated touch axis (both corrected in software — see `remap_pixel()` and `readTouch()` in `main.cpp`). If you're porting this to a different unit, don't assume these hold without re-verifying.
- There's a long-running, not-fully-resolved intermittent display corruption issue under certain timing conditions (see comments throughout `main.cpp`). Touch polling interval was tuned as a partial mitigation; if you see "split screen" artifacts, that's a known open issue, not a new bug.
