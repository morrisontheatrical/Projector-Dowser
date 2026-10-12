# Dowser_Nano-every_5c (Arduino Nano Every, condensed)

Space/cost-optimized variant (`new-features` branch). Sketch: [`Dowser_Nano-every_5c.ino`](Dowser_Nano-every_5c.ino).
The web UI was removed to make room for DMX input, an I2C LCD and a local button.

## Target

| Item | Value |
|------|-------|
| MCU | Arduino Nano Every (ATmega4809) |
| Board package | Arduino megaAVR Boards (`arduino:megaavr`), 1.8.8 on the dev machine |
| Network | External W5500 SPI Ethernet module (`Ethernet.h`) |
| Serial | USB, 1,000,000 baud |

## Pinout


| Function | Pin | Status |
|----------|-----|--------|
| Servo PWM | D3 | Active |
| W5500 CS | D10 | Active |
| W5500 SCK / MOSI / MISO | D13 / D11 / D12 | Active (hardware SPI) |
| Button (to GND) | D2 | Active: short press toggles up/down |
| I2C LCD 16x2 | A4 (SDA) / A5 (SCL) | Active (address auto-detected) |
| MAX485 RO | D0 (RX) | Active (DMX receive, USART1) |
| MAX485 RE+DE | GND, or D4 | D4 is held LOW (receive) |
| Status LED | D4 | Disabled |
| IR receiver | D5 | Disabled (`IRremote` not included) |

W5500 VDD must be regulated 3.3V; see the wiring table in the [root README](../../README.md).

## Network stack

Same as the Nano R4 build: W5500 over SPI, DHCP with static fallback, OSC/UDP (8001), Telnet (23).
Set a unique `mac[]` per device.

## Libraries

`Ethernet` (2.0.2), `OSC` (CNMAT, 4.0.0), `Servo`, `EEPROM`, `SPI`, `Wire`,
`LiquidCrystal I2C` (Frank de Brabander), `DMXSerial` (Matthias Hertel).

## DMX input

Set the address from the serial or Telnet console: `dmx=150`, then `save` to keep it across reboots
(`dmxen=0/1` turns DMX off/on). The channel maps 0-255 onto the down-to-up presets with no fade, so the
console's cue fades control the timing. DMX only acts when the value changes, so OSC, console and button
commands still apply until the next DMX change. With no DMX for 1 s the LCD shows "No DMX signal".

### RDM / position feedback (not implemented)

DMX is one-way, so the console cannot see the dowser's position. RDM would allow it, but was shelved on this board:

- **Libraries:** `DMXSerial2` has RDM but does not support the Nano Every (megaavr). `DMXSerial` supports it but has no RDM.
  `TeensyDMX` 4.2.0 has no RDM controller. RDM would need a custom responder (Nano Every) and controller (Teensy):
  discovery (`DISC_UNIQUE_BRANCH`, mute/un-mute), `DEVICE_INFO`, a sensor value for position, checksums and 2.8 ms response timing.
- **Flash:** the sketch uses about 97% of program space, so RDM would not fit without removing features.
- **Hardware:** the MAX485 needs a software-controlled direction. Today RE and DE are tied to GND (receive only). Both
  the dowser and the transmitter would need RE+DE on a GPIO.
- **Position is commanded, not measured:** a hobby servo has no position output, so any feedback reports the last commanded angle.
- **Workaround today:** OSC position messages (`/dowser/state/angle`, `isUp`, `isDown`) over Ethernet.

Plan: revisit with a more powerful board that has spare flash and a second hardware UART.

## Interfaces and routes

OSC routes match the Nano R4 sketch (`/dowser/{up,down,angle,query,reset}`, `/dowser/config/*`); see the
[root README](../../README.md). Physical-button and IR control do not exist in this variant, so those sections
of the root README do not apply.
