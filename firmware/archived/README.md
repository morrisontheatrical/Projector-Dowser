# Archived Firmware Guide

This directory contains prototypes and earlier firmware variants, not a set of
interchangeable releases. The sketches have different hardware assumptions,
network capabilities, configuration layouts, and levels of completeness.
For the currently documented firmware, see
[`../Nano_Dowser_OSC_4_Companion/Nano_Dowser_OSC_4_Companion.ino`](../Nano_Dowser_OSC_4_Companion/Nano_Dowser_OSC_4_Companion.ino).

## OSC and Ethernet prototypes

| Sketch | Features | Important differences and limitations |
| --- | --- | --- |
| [`Nano_Ethernet.ino`](Nano_Ethernet.ino) | Early W5500/UDP OSC receiver with a `/macro/trigger` route and an OSC status-send helper. | Network-only test: no servo or local controls. Uses fixed IP, ports, and target host. |
| [`Nano_Ethernet_Servo.ino`](Nano_Ethernet_Servo.ino) | Adds `/dowser/up`, `/down`, `/reset`, and `/angle` OSC routes and sends QLab cue name/notes feedback. | Tracks a simulated angle and preset; servo writes are commented out and no Servo library is included. Uses fixed network settings. |
| [`Nano_Ethernet_Servo_2.ino`](Nano_Ethernet_Servo_2.ino) | Adds EEPROM-backed network settings, DHCP with static fallback, three telemetry routing modes, heartbeat, serial configuration, save/reboot, and watchdog reset. | The angle/preset is still software state: this sketch has no servo driver or local button/IR control. Configuration is an early single-target-port design. |

## Local servo prototypes

| Sketch | Features | Important differences and limitations |
| --- | --- | --- |
| [`Servo_Nano.ino`](Servo_Nano.ino) | Local servo control with physical-button toggle, IR remote and presets, pause/detach behavior, and serial input for target angles. Movement is updated in the main loop. | No Ethernet or OSC. LCD support is commented out; pins and movement limits are hard-coded. |
| [`Servo_controller.ino`](Servo_controller.ino) | Builds on the local servo controls with an enabled 16×2 LCD and rotary-encoder input for position adjustments. | No Ethernet or OSC. Assumes the additional LCD and encoder wiring; configuration remains hard-coded. |

## Integrated OSC dowser sketches

| Sketch | Features | Important differences and limitations |
| --- | --- | --- |
| [`Nano_Dowser_OSC.ino`](Nano_Dowser_OSC.ino) | Combines servo, button and IR controls with OSC, EEPROM presets/bounds, serial and Telnet consoles, network configuration, telemetry modes, and heartbeat. | Early integration with a single configured telemetry port and Arduino `String` use in console/logging paths. LCD code is inactive. |
| [`Nano_Dowser_OSC_2.ino`](Nano_Dowser_OSC_2.ino) | Retains the integrated control and configuration feature set, while moving many fixed diagnostic strings to flash (`F()`/`PSTR`) and using a numeric, flash-string message formatter. | Still uses `String` for Telnet input and common console-command parsing; telemetry remains single-port. |
| [`Nano_Dowser_OSC_2_NoStrings.ino`](Nano_Dowser_OSC_2_NoStrings.ino) | Replaces dynamic `String` console buffers/parsing with fixed-size character buffers and C-string parsing; retains the integrated controls and configuration handlers. | Still uses the earlier single-target-port telemetry design. Input buffers are fixed-size. |
| [`Nano_Dowser_OSC_3_NoStrings.ino`](Nano_Dowser_OSC_3_NoStrings.ino) | Continues the fixed-buffer approach and rewrites movement OSC routes as inline handlers. | Unlike `_2_NoStrings`, it removes the OSC configuration/save routes and removes LED output control; its factory boot default also differs (150° rather than 10°). Treat it as an experimental variant, not a drop-in upgrade. |

## Choosing a sketch

- For Companion and QLab multi-destination telemetry and the current
  configuration interface, use the active sketch linked above rather than an
  archived variant.
- The `Nano_Ethernet*` sketches are network/OSC experiments; the first two do
  not drive a physical servo.
- The `Servo_*` sketches are local-control experiments and have no network
  interface.
- The `Nano_Dowser_OSC*` sketches are the integrated single-port predecessors.
  Their features are not strictly cumulative: in particular,
  `Nano_Dowser_OSC_3_NoStrings.ino` omits configuration OSC routes present in
  `Nano_Dowser_OSC_2_NoStrings.ino`.

Do not assume EEPROM data is compatible between sketches. Their configuration
structures and markers vary, and some variants initialize EEPROM with defaults
when the stored marker does not match. Back up settings and verify hardware
limits before testing an archived sketch on a connected dowser.
