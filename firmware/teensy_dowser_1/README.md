# teensy_dowser_1 (Teensy 4.1, first adaptation)

Sketch: [`teensy_dowser_1.ino`](teensy_dowser_1.ino). It is a single file; the web pages are embedded in it.

## Target

| Item | Value |
|------|-------|
| MCU | Teensy 4.1 (i.MX RT1062) |
| Board package | Teensyduino / `teensy:avr`, 1.62.0 on the dev machine |
| Arduino IDE board | Teensy 4.1 (compiles with `-b teensy:avr:teensy41`) |
| Serial | USB serial; the baud setting is ignored |

## Pinout

| Function | Pin | Notes |
|----------|-----|-------|
| Servo PWM | 3 | `SERVO_PIN` |
| Button | 2 | `BUTTON_PIN` |
| Status LED | 4 | `LED_PIN` |
| IR receiver | 5 | `IR_RECEIVE_PIN` |
| Ethernet | Dedicated PHY header | See below; no GPIO or SPI pins used |

`ETHERNET_CS_PIN` is defined but unused on this board. Pins 10-13 (SPI) are free.

## Network stack

The Teensy 4.1 has a native Ethernet MAC with an on-board DP83825 PHY. Do **not** use the W5500 `Ethernet.h`
library: it reports "Ethernet hardware not found".

- Library: **QNEthernet** (0.37.0), selected with `#if defined(ARDUINO_TEENSY41)`. The classes are
  `EthernetUDP`, `EthernetServer`, and `EthernetClient`, so the rest of the code matches the Nano builds.
- Hardware: solder the Ethernet header pins on the underside of the board and connect the Teensy 4.1
  Ethernet kit (RJ45 with magnetics) with its ribbon cable.
- Startup: DHCP with an 8 s wait, falling back to the stored static IP. With `static=1` it uses the static IP.
- Services: OSC/UDP (default 8001), Telnet (23), Web UI (80).

## Libraries

`QNEthernet`, `OSC` (CNMAT), `IRremote` (4.x), `Servo`, `EEPROM`, `SPI`. Install QNEthernet from the Library
Manager.

## Features beyond the Nano builds

- `bootmode=0|1`: boot to the default angle or the last settled position (stored in EEPROM; layout `0x57`).
  Settings are upgraded automatically from older layouts.
- `reboot`: restart without saving (serial, Telnet, and the web Config page).
- Two-page web UI: Controls at `/` and Config at `/config`. There is no authentication; use trusted networks.

| Web endpoint | Description |
|--------------|-------------|
| `GET /` | Controls page |
| `GET /config` | Config page |
| `GET /status` | JSON: position, target, preset, and all settings |
| `GET /cmd?c=<console command>` | Runs any serial-console command, e.g. `c=up`, `c=fade=0.4`, `c=reboot` |
| `GET /up`, `GET /down` | Legacy links; act and redirect to `/` |

## OSC routes

`/dowser/{up,down,angle,query,reset}` and `/dowser/config/{up,down,dnbound,upbound,default,fade,localport,
targetport0-2,static,telemetry,heartbeat,queryAll,save}`, identical to the Nano R4 sketch; see the
[root README](../../README.md). There is no OSC route for `bootmode`; set it by console or web.
