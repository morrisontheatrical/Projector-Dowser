# Projector Dowser

Firmware and project notes for an Arduino-based projector dowser. The current
repository sketch is `firmware/Dowser_Nano_5/Dowser_Nano_5.ino`.

## Current Firmware and Installed Hardware

The repository source and the firmware currently installed on the hardware are
not all the same version:

- `Dowser_Nano_5.ino` is loaded on an Arduino Nano R4.
- A separate serial-control firmware version remains loaded on a Nano Every
  connected to the motor through the XLR port.
- The original OSC firmware, created before this repository, remains loaded on
  a second Nano Every connected to the Ethernet port/chip.

The OSC endpoints and configuration documented below describe the tracked
`Dowser_Nano_5.ino` source. Do not assume the two Nano Every boards run this
source or expose the same commands. The original pre-repository OSC firmware
is not included as a separately identified source file here.

## Overview

The Projector Dowser is a custom-built theatrical automation device that:
- **Controls a servo motor** to open/close a projector iris or shutter
- **Communicates via OSC** over a compatible Ethernet interface for remote control and telemetry
- **Supports multiple control interfaces**: physical button, IR remote, serial console, Telnet console, or OSC network commands
- **Streams real-time state** to Companion and QLab for feedback and synchronization
- **Stores configuration in EEPROM** for persistent calibration across reboots

## Firmware Hardware Interfaces

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | Arduino Nano R4 for the current repository sketch; compatible with Nano Every |
| **Motor control** | Servo signal output on pin 3 |
| **Ethernet interface** | SPI Ethernet interface, chip-select pin 10 |
| **IR Receiver** | IR receiver input (pin 5; remote protocol/codes depend on the remote) |
| **Button** | Digital input (pin 2) |
| **LED** | Status indicator (pin 4) |
| **Serial** | USB serial console at 1,000,000 baud |

Pin definitions describe the tracked sketch, not a verified wiring diagram for
each installed board. Confirm the board, Ethernet hardware, power supply, and
wiring against the actual installation before connecting or uploading firmware.

### W5500 Wiring

| W5500 Pin | Function | Target Board / Signal | Wire Color (Your Setup) | Notes & Status |
|-----------|----------|-----------------------|-------------------------|----------------|
| CS / SCSn | Chip Select | D10 | White / Green | Verified (Firmware Default) |
| SCK / SCLK | SPI Clock | D13 | Brown | Verified |
| MOSI / SI | SPI Master Out | D11 | Green | Verified |
| MISO / SO | SPI Master In | D12 | White / Blue | Verified |
| VDD / 3V3 | Power Input | Regulated 3.3V | Orange (Note below) | Caution Required |
| GND | Common Ground | GND | White / Orange | Verified |
| RSTn | Hardware Reset | Reset / 3.3V Pullup | Blue | Optional (Pull High or connect to Reset) |
| INTn | Interrupt | Unconnected | None | Unused in standard firmware |

The W5500 requires regulated 3.3V power. Do not connect 5V directly to VDD
unless the specific breakout board provides its own 5V-to-3.3V regulator.
Check the board's regulator and logic-level specifications before wiring.

## Network Communication

### OSC Endpoints

#### Core Movement
| Endpoint | Args | Description |
|----------|------|-------------|
| `/dowser/up` | optional `int` or `float` fade seconds (0-60) | Move to the "up" preset using the default fade time or supplied one-move duration |
| `/dowser/down` | optional `int` or `float` fade seconds (0-60) | Move to the "down" preset using the default fade time or supplied one-move duration |
| `/dowser/angle` | `int` or `float` angle, optional `int` or `float` fade seconds (0-60) | Move to a specific angle (degrees), optionally overriding the fade time |
| `/dowser/query` | — | Request immediate state broadcast |
| `/dowser/reset` | optional `int` or `float` fade seconds (0-60) | Move to the down preset, optionally overriding the fade time |

#### Configuration (GET/SET REST-style)
| Endpoint | Args | GET Response | SET Behavior |
|----------|------|-----|-----|
| `/dowser/config/up` | `int` | Broadcasts current up position | Sets up position (validated) |
| `/dowser/config/down` | `int` | Broadcasts current down position | Sets down position (validated) |
| `/dowser/config/dnbound` | `int` | Broadcasts lower hard limit | Sets lower hard limit (0-180°) |
| `/dowser/config/upbound` | `int` | Broadcasts upper hard limit | Sets upper hard limit (0-180°) |
| `/dowser/config/default` | `int` | Broadcasts stored default position | Sets stored default position |
| `/dowser/config/localport` | `int` | Broadcasts listening port | Sets listening port |
| `/dowser/config/targetport[0-2]` | `int` | Broadcasts destination port | Sets destination port |
| `/dowser/config/static` | `0\|1` | Broadcasts static IP mode | Enables/disables static IP |
| `/dowser/config/telemetry` | `0\|1\|2` | Broadcasts telemetry mode | Sets telemetry routing mode |
| `/dowser/config/heartbeat` | `0\|1` | Broadcasts heartbeat state | Enables/disables heartbeat |
| `/dowser/config/fade` | `int` or `float` (0-60 seconds) | Broadcasts default fade time | Sets the default move duration |
| `/dowser/config/queryAll` | — | Broadcasts all 13 config fields | N/A (query only) |
| `/dowser/config/save` | — | N/A | Saves config to EEPROM + reboot |

Fade time is the requested duration for the entire move, in seconds. Integer and decimal values are accepted (for example, `/dowser/up 0.4` fades to the up preset over 0.4 seconds, while `/dowser/angle 90 4.25` takes 4.25 seconds to reach 90°). Times are stored and applied internally to the nearest millisecond. A `0`-second move writes the target position immediately. The actual commanded slew rate scales with the distance to the target, so a shorter nonzero move takes the requested duration at a lower rate. The valid range is 0-60 seconds. Overrides do not change the configured default. Motion is updated incrementally without blocking the main loop; elapsed-time catch-up is capped to 20 ms per update after longer delays.

Each move keeps the fade time selected when it started. Changing `/dowser/config/fade` or `fade=` affects subsequent moves, not one already in progress. Configuration changes are held in RAM until `save` writes them to EEPROM and reboots the board. Network settings such as the local listening port take effect after reboot.

Movement endpoints accept no arguments to preserve existing no-argument clients; when supplying a fade time, send exactly one integer or float from 0 to 60 seconds. These arguments replace the former speed percentages and millisecond values, so clients sending either must be updated to send seconds. OSC float angles are checked against the configured bounds and then converted to an integer degree. Invalid OSC fade times (including `/dowser/config/fade`) and angles are rejected and returned to the sender as `/dowser/error/fade` and `/dowser/error/angle`, respectively. The serial console also logs each rejection. `/dowser/reset` retains its existing behavior of moving to the down preset (`config.posDown`), not the stored default position (`config.defaultPos`).

Suggested hardware checks: compare instant (`0`), fractional (`0.4`), and longer (`1`-`60`) fades, repeatedly retarget during a move and confirm it continues from the current reported angle, and verify invalid fade times (below 0, above 60, or non-numeric) and out-of-bounds angles return errors without starting a move. Query/set `/dowser/config/fade` with no argument, valid integer and decimal values, and invalid values. Set `fade=0.4`, save, and reboot to check persistence; on legacy 0x53/0x55 devices, confirm existing settings remain intact.

#### Telemetry (Outbound Broadcasts)
| Endpoint | Value | Update Frequency |
|----------|-------|------------------|
| `/dowser/state/angle` | `int` (0-180°) | On position changes and periodic refresh |
| `/dowser/state/isUp` | `0\|1` | Sent with each state update |
| `/dowser/state/isDown` | `0\|1` | Sent with each state update |
| `/cue/DOWSER_STATUS/notes` | `string` (angle) | Sent with each state update |
| `/cue/DOWSER_STATUS/name` | `string` (angle) | Sent with each state update |

The sketch sends position-change updates and periodic status refreshes. It also
sends a separate configurable heartbeat (4 seconds with factory defaults).
These are not guaranteed fixed-rate streams.

### Telemetry Modes

**Mode 0 (Dynamic Registration / Reply-to-Sender)**  
Targets telemetry at the IP address of the most recent OSC sender and sends it
to the configured target ports. Ideal for:
- Lab troubleshooting
- Mobile tech tables on DHCP networks

Fallback: If device hasn't received any OSC yet, uses `fixedTargetIP`.

**Mode 1 (Fixed Target / Unicast)**  
Locks telemetry to the configured target IP address. Ideal for:
- Permanent installations
- Dedicated Companion Mac Studio
- Eliminating rogue network traffic

**Mode 2 (Subnet Broadcast)**  
Sets the destination to the controller's local IP with the last octet changed
to `.255`. This assumes a subnet where that address is the broadcast address.
Ideal for:
- Dynamic environments
- Multi-subscriber setups (QLab + Companion simultaneously)
- Single-subnet deployments

### Factory Defaults

These values are written when the sketch finds an unrecognized EEPROM marker.
Saved EEPROM settings may differ.

```
Boot Profile       : STATIC IP (10.0.0.100)
Subnet Mask        : 255.255.255.0
Gateway            : 10.0.0.1
Listening Port     : 8001
Target Ports       : [0]=12000 (Companion), [1]=53000 (QLab), [2]=0 (unused)
Telemetry Mode     : 2 (Subnet Broadcast)
Fixed Target IP    : 10.0.0.29 (Companion fallback)
Heartbeat          : Enabled (4000ms interval)
Servo Positions    : Down=10°, Up=170°
Hard Bounds        : 7° - 173°
Default Fade Time  : 3.000s per move
IR Presets         : [0-4]=10°, [5]=125°, [6-9]=170°
```

## Control Interfaces

### 1. **Physical Button (Pin 2)**
- **Press** → Toggle between up and down positions
- **LED feedback**: LOW (pressed), HIGH (released)

### 2. **IR Remote (38kHz, Pin 5)**
All codes specific to your remote model:

| Button | Code | Action |
|--------|------|--------|
| Up Arrow | `0xF609FF00` | Move to local `up` target |
| Down Arrow | `0xF807FF00` | Move to local `down` target |
| Right (Vol+) | `0xB946FF00` | Increment +1° |
| Left (Vol−) | `0xEA15FF00` | Decrement −1° |
| Preset Edit | `0xB847FF00` | Toggle edit mode |
| Number buttons | `0: E916FF00`, `1: F30CFF00`, `2: E718FF00`, `3: A15EFF00`, `4: F708FF00`, `5: E31CFF00`, `6: A55AFF00`, `7: BD42FF00`, `8: AD52FF00`, `9: B54AFF00` | Call preset or save (in edit mode) |
| Play/Pause | `0xBF40FF00` | Pause/resume toggle |
| Power | `0xF20DFF00` | Detach servo (soft-disable) |
| Plus | `0xBC43FF00` | Increment +5° |
| Minus | `0xBB44FF00` | Decrement −5° |

**Edit Mode Workflow**:
1. Press **Preset Edit** to enter edit mode.
2. The first number-button press arms preset editing; press a number button
   again to save the current angle to that button's preset slot.
3. Press **Preset Edit** again to exit edit mode.

In this sketch, local `up` and `down` are initialized from `config.posDown` and
`config.posUp`, respectively. The IR labels and physical-button toggle
therefore use the opposite preset fields from OSC `/dowser/up` and
`/dowser/down`. Verify direction on the installed mechanism before relying on
those local controls.

### 3. **Serial Console (USB, 1Mbaud)**
Connect via USB for direct serial command interface:

```bash
# Example commands
show                # Display network, position, and fade settings
up                  # Move to up position
down                # Move to down position
90                  # Move to 90°
down=10             # Set down position to 10°
up=170              # Set up position to 170°
dnbound=5           # Set lower hard limit to 5°
upbound=175         # Set upper hard limit to 175°
default=90          # Set stored default position to 90°
fade=0.4            # Set default movement fade time (0-60 seconds)
localport=8001      # Set listening port
targetport0=12000   # Set target port 0
targetport1=53000   # Set target port 1
ip=10.0.0.100       # Set static IP
targetip=10.0.0.29  # Set fixed target IP (mode 1/0 fallback)
static=1            # Enable static IP (0 for DHCP)
telemetry=2         # Set telemetry mode (0/1/2)
heartbeat=1         # Enable heartbeat (0/1)
save                # Save config to EEPROM and reboot
help                # Show available commands
?                   # Alias for help
```

### 4. **Telnet Console (Port 23)**
The same command parser as the serial console is available over the network:

```bash
nc 10.0.0.100 23           # macOS
telnet 10.0.0.100 23       # Windows/Linux
# All serial commands work here
```

**Features**:
- Real-time echo with backspace support
- Automatic disconnect after 120 seconds idle
- Some status messages are dispatched to both serial and Telnet

### 5. **OSC (Ethernet, Port 8001)**
Native OSC protocol for QLab, Companion, or any OSC controller:

```python
# Example Python OSC client (requires python-osc)
from pythonosc.udp_client import SimpleUDPClient

client = SimpleUDPClient("10.0.0.100", 8001)
client.send_message("/dowser/angle", 135)   # Move to 135°
client.send_message("/dowser/up", "")        # Move to up
client.send_message("/dowser/query", "")     # Request state
```

### 6. **Web UI (Ethernet, Port 80)**
Browse to the device IP for a single-page control and configuration UI (served from flash by `WebServer.ino` / `web_page.h`). It has no authentication, so use it on trusted networks only.

| Endpoint | Description |
|----------|-------------|
| `GET /` | Web UI |
| `GET /api/status` | Position, target, moving state, IP |
| `GET /api/config` | All settings as JSON |
| `POST /api/move?to=up\|down\|<angle>[&fade=<sec>]` | Move |
| `POST /api/config?key=value[&key=value]` | Set settings (same keys as the serial console; autosaved) |
| `POST /api/reboot` | Save and reboot |

## Software Architecture

### Control Flow

```
loop()
├─ Telemetry check (50ms gated; updateServo also sends on position changes)
│  ├─ Motion detection → broadcast if currentPos changed
│  └─ Static refresh → broadcast about every 1 second
├─ processIncomingOSC() → handle /dowser/*, /cue/* commands
├─ processTelnetConsole() → handle Telnet input
├─ processSerialConsole() → handle USB serial input
├─ processPhysicalButton() → debounced button (pin 2)
├─ processIRInput() → IR remote decoder (pin 5)
└─ updateServo() → write targetPos to servo, broadcast on position change
```

### Key Modules

| Function | Purpose |
|----------|---------|
| `setup()` | Initialize hardware, load EEPROM config, start Ethernet |
| `loop()` | Main event loop with telemetry gating |
| `processIncomingOSC()` | Route incoming OSC messages to handlers |
| `broadcastCompanionState()` | Send telemetry to all configured ports |
| `broadcastConfigItem()` | Broadcast single config parameter |
| `broadcastAllConfigSettings()` | Bulk dump all config (with 20ms pacing) |
| `evaluateTelemetryRouting()` | Determine target IP based on telemetry mode |
| `updateServo()` | Clamp targetPos, write to servo, track changes |
| `handleCommonConsoleCommands()` | Parse and execute serial/Telnet commands |
| `loadConfiguration()` | Load EEPROM or initialize factory defaults |
| `processPhysicalButton()` | Debounce and toggle button press |
| `processIRInput()` | Decode IR remote commands |
| `translateIRCommand()` | Map IR codes to servo actions |
| `handleIRPreset()` | Save/recall preset positions |
| `processTelnetConsole()` | Accept connections, handle Telnet input |
| `initNetwork()` | Configure Ethernet (static or DHCP) |
| `rebootDevice()` | Trigger a software reset |

### Configuration Structure

```c
struct DeviceConfiguration {
  uint8_t magicNumber;          // 0x56 (configuration version)
  uint8_t preferStatic;         // 0=DHCP, 1=static IP
  uint8_t staticIP[4];          // e.g. 10.0.0.100
  uint8_t subnetMask[4];        // e.g. 255.255.255.0
  uint8_t gatewayIP[4];         // e.g. 10.0.0.1
  uint16_t localPort;           // OSC listen port (default 8001)
  uint16_t targetPorts[3];      // Telemetry destinations (default 12000, 53000, 0)
  uint8_t telemetryMode;        // 0=reply-to-sender, 1=fixed, 2=broadcast
  uint8_t fixedTargetIP[4];     // For mode 1 fallback (e.g. 10.0.0.29)
  uint8_t heartbeatEnabled;     // Enable periodic broadcasts
  uint16_t heartbeatInterval;   // Milliseconds (default 4000)
  uint8_t posDown;              // Down preset angle (default 10°)
  uint8_t posUp;                // Up preset angle (default 170°)
  uint8_t dnBound;              // Lower hard limit (default 7°)
  uint8_t upBound;              // Upper hard limit (default 173°)
  uint8_t defaultPos;           // Stored boot-position setting (default 10°)
  uint8_t irPresets[10];        // IR preset angles [0-9]
  uint16_t defaultFadeTimeMs;   // Default movement duration (default 1000 ms)
};
```

Configuration changes are autosaved to EEPROM after 3 seconds without further edits (no reboot); `save` still writes immediately and reboots, which is needed for network changes to take effect. Stored at EEPROM address 0. Marker `0x53` configurations are migrated in place to marker `0x56` with the existing settings preserved and a 1-second default fade time. Marker `0x55` configurations are migrated with their existing speed converted to an equivalent full-range fade time. Unknown markers trigger factory defaults. Fade times are entered in seconds (0-60), and stored to the nearest millisecond. Although `defaultPos` is loaded, the current setup code replaces the startup target before the servo is attached; see the startup sequence below.

## Dependency Libraries

The tracked sketch includes these headers:

- **SPI**
- **Ethernet** (includes `EthernetUdp`)
- **Servo**
- **EEPROM**
- **OSCMessage.h** – OSC message codec compatible with the sketch's API
- **[IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote)** – IR decoder (v4.x)

Install board-core and library versions compatible with the Nano R4 and these
headers. The original OSC firmware on the second Nano Every may have different
build requirements.

## Getting Started

### 1. Hardware Setup
- The tracked sketch uses servo signal pin 3, Ethernet chip-select pin 10,
  IR receiver pin 5, button pin 2, and LED pin 4.
- Use hardware and power wiring compatible with the selected Nano R4 and
  Ethernet interface. The source tree does not define a verified wiring or
  power diagram for the installed boards.

### 2. Flash Firmware
1. Install Arduino IDE 2.0+ and Nano R4 board support
2. Install all required libraries (see above)
3. Update `mac[]` to a unique address on your network (if needed)
4. Update IR remote codes in `translateIRCommand()` to match your remote
5. Open `firmware/Dowser_Nano_5/Dowser_Nano_5.ino` and upload to the Nano R4

### 3. Initial Configuration
- **USB Serial**: Connect and send `show` to inspect current settings
- **Network**: Ping `10.0.0.100` (or your static IP)
- **Servo calibration**: Send `90` to test movement; verify smooth range
- **IR test**: Press remote buttons and watch serial output for codes

### 4. QLab Integration
Historical notes on earlier QLab and macOS serial integration are in
[`docs/project-overview.md`](docs/project-overview.md); they do not describe
the current Nano R4 sketch's complete deployed setup.

### 5. Companion Integration
Historical OSC and Companion troubleshooting notes are in
[`docs/conversation-overview-gemini.md`](docs/conversation-overview-gemini.md);
check the current sketch and installed board before applying them.

## Default Startup Sequence

On power-up or reboot:
1. Serial baud rate set to 1Mbaud
2. EEPROM loaded; legacy markers `0x53` and `0x55` migrate to `0x56`; unknown markers trigger factory reset
3. Ethernet is initialized (static IP or DHCP fallback); UDP and Telnet start only if Ethernet hardware is detected
4. The servo attaches to pin 3 and is commanded to `config.posUp` (170° with factory defaults). Although `config.defaultPos` is loaded earlier, setup overwrites the target before attaching the servo.
5. IR receiver starts on pin 5
6. Telemetry routing is evaluated (mode 2 = subnet broadcast with factory defaults)
7. System ready; listens for OSC on the configured local port (8001 with factory defaults)

## Troubleshooting

### Device not responding on network
- Check Ethernet cable connection
- Verify IP: `Serial.println(Ethernet.localIP());` in serial monitor
- Ping the device: `ping 10.0.0.100`
- Check telemetry mode with `show` (factory default is 2; saved settings may differ)

### Servo moves erratically
- Verify the installed motor power and ground wiring
- Check the pin 3 signal path and connections
- Test servo response time with direct Arduino PWM control

### IR remote not responding
- Verify IR codes match your specific remote model (use IR decoder sketch)
- Update `translateIRCommand()` with correct codes
- Check IR receiver on pin 5 for dust/obstruction
- Enable LED feedback: `IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK)`

### QLab/Companion not receiving telemetry
- Verify telemetry mode: `telemetry=2` for broadcast, `telemetry=1` for fixed target
- Check configured target ports (factory defaults: 12000 and 53000)
- Verify firewall allows UDP on those ports
- Test with `nc -l -u 12000` and send `/dowser/query`

### Configuration lost after power cycle
- Verify `save` command was sent before powering down
- Check EEPROM health: may need chip replacement if corrupted
- `/dowser/config/queryAll` reports settings; it does not reset configuration

## Roadmap / Future Enhancements

- [ ] Auto-save IR presets to EEPROM (currently manual `save` required)
- [ ] Acceleration/deceleration profiles for smoother servo motion
- [ ] Diagnostic telemetry (servo current, temperature, error states)
- [ ] Web UI for configuration (via simple HTTP server)
- [ ] Dual-servo support (for separate iris + shutter)
- [ ] DMX input fallback (via additional module)

## License

[Specify license if applicable]

## Support & Contributing

Created and maintained by Morrison Theatrical.  
For issues, documentation updates, or feature requests, please open an issue or PR on GitHub.

---

## File Structure (This Repository)

```
Projector-Dowser/
├── README.md
├── docs/
│   ├── conversation-overview-gemini.md
│   └── project-overview.md
├── firmware/
│   ├── Dowser_Nano_5/
│   │   └── Dowser_Nano_5.ino           # Current repository sketch
│   ├── Nano_Dowser_OSC_4_Speed-Edits/
│   │   └── Nano_Dowser_OSC_4_Companion_v2-speed.ino
│   └── archived/
│       ├── README.md                   # Archived sketch comparison
│       └── ...                         # Earlier sketches
```

See [`firmware/archived/README.md`](firmware/archived/README.md) for a feature
comparison of archived firmware. The pre-repository OSC firmware currently
installed on the second Nano Every is not identified as a separate tracked file.
