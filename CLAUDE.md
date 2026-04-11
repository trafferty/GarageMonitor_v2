# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP8266-based garage door monitor (NodeMCU board) built with PlatformIO and the Arduino framework. Monitors a magnetic door sensor and controls garage/kitchen lights and a piezo alarm based on door state.

## Build & Flash Commands

```bash
# Build the project
pio run

# Build and upload to connected NodeMCU
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor
```

## Hardware

- **Board**: NodeMCU (ESP8266)
- **GPIO assignments** (from `src/main.cpp`):
  - `D5` — garage door sensor input (LOW = open, HIGH = closed)
  - `D1` — garage light output
  - `D3` — kitchen light output
  - `D2` — piezo alarm output
  - `D0` — heartbeat LED output

## Architecture

The active firmware is `src/main.cpp`. It runs a polling loop (50ms delay) that:
1. Toggles a heartbeat LED every 1 second via `millis()` tracking
2. Reads the door sensor and transitions between `state_unknown`, `state_open`, `state_closed`
3. On door open: turns on kitchen and garage lights, beeps 2×
4. On door close: turns off kitchen light, keeps garage light on, beeps 3×
5. Auto-shuts off garage light after `GarageLightOnPeriod_ms` (10 minutes) using `millis()`

`src/main_dependsOnTime.c_src` is an archived earlier version (v0.5) that included WiFi, NTP time sync, EEPROM-backed scheduling, and an HTTP server. It was intentionally disabled because the monitor should work when the network is down. The `.c_src` extension prevents PlatformIO from compiling it.

## External Dependencies

- `lib/Time/` — local copy of the Arduino Time library (used only in the archived version)
- `sierra_wifi_defs.h` — external header with WiFi credentials/network config, pulled in via `${sysenv.inc_extras}/` build flag in `platformio.ini`. This file lives outside the repo and must be present in the directory pointed to by the `inc_extras` environment variable.

## Key Design Decision

WiFi and NTP were removed in v0.6 so the monitor operates standalone without network dependency. If re-adding network features, refer to `main_dependsOnTime.c_src` for the WiFi init, NTP sync, and EEPROM scheduling patterns.
