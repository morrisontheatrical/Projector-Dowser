# Dowser_Nano-every_5c (Arduino Nano Every, condensed)

Space/cost-optimized variant (`new-features` branch). Sketch: [`Dowser_Nano-every_5c.ino`](Dowser_Nano-every_5c.ino),
with [`WebServer.ino`](WebServer.ino) and [`web_page.h`](web_page.h).

## Target

| Item | Value |
|------|-------|
| MCU | Arduino Nano Every (ATmega4809) |
| Board package | Arduino megaAVR Boards (`arduino:megaavr`), 1.8.8 on the dev machine |
| Network | External W5500 SPI Ethernet module (`Ethernet.h`) |
| Serial | USB, 1,000,000 baud |

## Pinout

The local controls are compiled out in this variant to save resources.

| Function | Pin | Status |
|----------|-----|--------|
| Servo PWM | D3 | Active |
| W5500 CS | D10 | Active |
| W5500 SCK / MOSI / MISO | D13 / D11 / D12 | Active (hardware SPI) |
| Button | D2 | Disabled (commented out) |
| Status LED | D4 | Disabled |
| IR receiver | D5 | Disabled (`IRremote` not included) |

W5500 VDD must be regulated 3.3V; see the wiring table in the [root README](../../README.md).

## Network stack

Same as the Nano R4 build: W5500 over SPI, DHCP with static fallback, OSC/UDP (8001), Telnet (23), Web UI (80).
Set a unique `mac[]` per device.

## Libraries

`Ethernet` (2.0.2), `OSC` (CNMAT, 4.0.0), `Servo`, `EEPROM`, `SPI`.

## Interfaces and routes

OSC routes match the Nano R4 sketch (`/dowser/{up,down,angle,query,reset}`, `/dowser/config/*`); see the
[root README](../../README.md). Physical-button and IR control do not exist in this variant, so those sections
of the root README do not apply.
