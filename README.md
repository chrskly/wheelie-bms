```
/opt/homebrew/bin/python3 -m venv venv
```

## Web interface

A read-only status page, served by the ESP32 itself and aimed at a phone held
next to the car. It shows what the BMS currently believes: state, state of
charge, every cell voltage and temperature, per-pack contactor and balancing
status, and the decoded fault bytes.

It cannot change anything. There are no endpoints that close a contactor, edit
a threshold or upload firmware — the only write interface to this BMS is the
one on the bench.

### Flashing it

The page lives in `data/` and goes into a LittleFS partition, which is a
**separate upload** from the firmware:

```
pio run -t upload      # firmware
pio run -t uploadfs    # data/index.html, style.css, app.js
```

Flashing the firmware alone leaves the old page in flash; if the filesystem has
never been uploaded the server says so on a built-in page rather than 404ing.

### Connecting

By default the board runs its own access point, so it works in a car park with
no WiFi:

| | |
|---|---|
| SSID | `wheelie-bms` |
| Password | `wheeliebms` |
| Address | http://192.168.4.1/ or http://wheelie-bms.local/ |

To join an existing network instead, set `WEB_WIFI_STATION_MODE` to 1 and fill
in `WEB_WIFI_STA_SSID` / `WEB_WIFI_STA_PASSWORD`. Everything is configured in
the "Web interface" block in `src/settings.h`, including
`WEB_INTERFACE_ENABLED 0` to leave it out entirely — which saves about 36 KB of
RAM and 500 KB of flash, nearly all of it the WiFi stack.

### How it gets its data

The BMS worker task owns all battery state and is the only thing allowed to
enter the state machine. The web server runs on a lower-priority task, and on a
dual-core part it genuinely runs at the same instant on the other core, so it
never touches a `Bms`, `Battery`, `Shunt` or `Io`.

Instead the worker publishes a consistent copy of everything the page shows —
`WebSnapshot`, 744 bytes — once every `WEB_SNAPSHOT_INTERVAL_MS`, and the web
task serialises that. The handoff is a seqlock rather than a mutex so that the
publishing side never blocks: it runs inside a worker tick that a 5 second
hardware watchdog is watching. See the comment at the top of
`include/webstatus.h`.

```
BMS worker task (prio 3) ──publish──> WebSnapshot ──read──> web task (prio 1)
                                       (seqlock)             /api/status
```

`src/webstatus.cpp` holds the snapshot store and the JSON serialiser, has no
Arduino or WiFi dependencies, and is covered by the native test suite.
`src/webserver.cpp` is the WiFi and HTTP transport and is ESP32-only.

### The API

`GET /api/status` returns the whole snapshot as JSON, about 3 KB. Thresholds
travel with it under `cfg`, so the page colours readings against the limits the
running firmware was actually built with rather than a second copy of the
numbers. `503` means the BMS has not published its first snapshot yet.
