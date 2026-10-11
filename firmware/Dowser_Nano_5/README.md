# Dowser_Nano_5 (Arduino Nano R4, primary)

Primary production firmware. Sketch: [`Dowser_Nano_5.ino`](Dowser_Nano_5.ino), with the web UI in
[`WebServer.ino`](WebServer.ino) and [`web_page.h`](web_page.h).

## Target

| Item | Value |
|------|-------|
| MCU | Arduino Nano R4 (Renesas RA4M1) |
| Board package | Arduino Renesas UNO R4 Boards (`arduino:renesas_uno`), 1.6.0 on the dev machine |
| Network | External W5500 SPI Ethernet module (`Ethernet.h`) |
| Serial | USB, 1,000,000 baud |

## Pinout

| Function | Pin | Notes |
|----------|-----|-------|
| Servo PWM | D3 | `SERVO_PIN` |
| Button | D2 | `BUTTON_PIN`, toggles up/down |
| Status LED | D4 | `LED_PIN` |
| IR receiver | D5 | `IR_RECEIVE_PIN` |
| W5500 CS | D10 | `ETHERNET_CS_PIN` |
| W5500 SCK / MOSI / MISO | D13 / D11 / D12 | Hardware SPI |
| W5500 VDD | 3.3V | Regulated 3.3V only unless the breakout has its own regulator |
| W5500 RSTn | Reset or 3.3V | Optional |
| W5500 INTn | — | Unused |

## Network stack

W5500 over SPI using the standard Arduino `Ethernet` library. DHCP is tried first with a static-IP fallback
(or static only when `static=1`). Services: OSC/UDP (default port 8001), Telnet (23), Web UI (80).
Set a unique `mac[]` per device.

## Libraries

`Ethernet` (2.0.2), `OSC` (CNMAT, 4.0.0), `IRremote` (4.x), `Servo`, `EEPROM`, `SPI`.

## Interfaces and routes

The OSC routes, configuration keys, telemetry, serial/Telnet commands, and web API for this sketch are documented
in the [root README](../../README.md). Routes: `/dowser/{up,down,angle,query,reset}` and
`/dowser/config/{up,down,dnbound,upbound,default,fade,localport,targetport0-2,static,telemetry,heartbeat,queryAll,save}`.

Not in this sketch (Teensy only): `bootmode` / last-position boot, and the `reboot` command.
