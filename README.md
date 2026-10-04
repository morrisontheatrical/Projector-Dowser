# Projector Dowser

A networked stage automation system for controlling a projector dowser (motorized iris/shutter) via OSC protocol. Built around an Arduino Nano Every with Ethernet connectivity, supporting real-time control from QLab, Bitfocus Companion, or any OSC-capable console.

## Overview

The Projector Dowser is a custom-built theatrical automation device that:
- **Controls a servo motor** to open/close a projector iris or shutter
- **Communicates via OSC** over Ethernet for remote control and telemetry
- **Supports multiple control interfaces**: physical button, IR remote, serial console, Telnet console, or OSC network commands
- **Streams real-time state** to Companion and QLab for feedback and synchronization
- **Stores configuration in EEPROM** for persistent calibration across reboots

## Hardware

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | Arduino Nano Every |
| **Servo Motor** | Standard 180° servo (e.g., SG90) |
| **Ethernet** | W5500 Ethernet shield (CS pin 10) |
| **IR Receiver** | 38kHz IR receiver module (pin 5) |
| **Button** | Tactile pushbutton (pin 2) |
| **LED** | Status indicator (pin 4) |
| **Power** | 5V dedicated supply (servo), isolated from Arduino USB |
| **Umbilical** | 50-foot XLR cable (PWM signal + ground) |

### Power Architecture

Split power design prevents brownouts:
- **Servo power**: dedicated 5V/4A supply
- **Arduino power**: USB or separate 5V supply
- **Common ground**: tied at the Arduino GND pin

The servo's 50-foot umbilical cable carries the PWM signal (pin 3) and ground return to the tech table.

## Network Communication

### OSC Endpoints

#### Core Movement
| Endpoint | Args | Description |
|----------|------|-------------|
| `/dowser/up` | optional `int speed` | Move to the "up" preset at the default speed or the supplied one-move speed |
| `/dowser/down` | optional `int speed` | Move to the "down" preset at the default speed or the supplied one-move speed |
| `/dowser/angle` | `int` or `float` angle, optional `int speed` | Move to a specific angle (degrees), optionally overriding speed for this move |
| `/dowser/query` | — | Request immediate state broadcast |
| `/dowser/reset` | optional `int speed` | Move to default position, optionally overriding speed for this move |

#### Configuration (GET/SET REST-style)
| Endpoint | Args | GET Response | SET Behavior |
|----------|------|-----|-----|
| `/dowser/config/up` | `int` | Broadcasts current up position | Sets up position (validated) |
| `/dowser/config/down` | `int` | Broadcasts current down position | Sets down position (validated) |
| `/dowser/config/dnbound` | `int` | Broadcasts lower hard limit | Sets lower hard limit (0-180°) |
| `/dowser/config/upbound` | `int` | Broadcasts upper hard limit | Sets upper hard limit (0-180°) |
| `/dowser/config/default` | `int` | Broadcasts boot default | Sets boot default position |
| `/dowser/config/localport` | `int` | Broadcasts listening port | Sets listening port |
| `/dowser/config/targetport[0-2]` | `int` | Broadcasts destination port | Sets destination port |
| `/dowser/config/static` | `0\|1` | Broadcasts static IP mode | Enables/disables static IP |
| `/dowser/config/telemetry` | `0\|1\|2` | Broadcasts telemetry mode | Sets telemetry routing mode |
| `/dowser/config/heartbeat` | `0\|1` | Broadcasts heartbeat state | Enables/disables heartbeat |
| `/dowser/config/speed` | `int` (1-100) | Broadcasts default speed | Sets persistent default movement speed (%) |
| `/dowser/config/queryAll` | — | Broadcasts all 13 config fields | N/A (query only) |
| `/dowser/config/save` | — | N/A | Saves config to EEPROM + reboot |

Movement speed is a percentage of the maximum commanded slew rate (180°/s): 100% is the maximum and lower values move more slowly. For example, `/dowser/up 30` uses 30% speed for that move, while `/dowser/angle 90 50` moves to 90° at 50%. Overrides do not change the configured default. Motion is updated incrementally without blocking the main loop; this controls commanded servo position changes, not guaranteed physical speed or mechanical noise.

#### Telemetry (Outbound Broadcasts)
| Endpoint | Value | Update Frequency |
|----------|-------|------------------|
| `/dowser/state/angle` | `int` (0-180°) | 20Hz during motion, 1Hz static |
| `/dowser/state/isUp` | `0\|1` | On state change |
| `/dowser/state/isDown` | `0\|1` | On state change |
| `/cue/DOWSER_STATUS/notes` | `string` (angle) | On position change |
| `/cue/DOWSER_STATUS/name` | `string` (angle) | On position change |

### Telemetry Modes

**Mode 0 (Dynamic Registration / Reply-to-Sender)**  
Automatically detects the last device that sent a command and broadcasts telemetry back to it. Ideal for:
- Lab troubleshooting
- Mobile tech tables on DHCP networks

Fallback: If device hasn't received any OSC yet, uses `fixedTargetIP`.

**Mode 1 (Fixed Target / Unicast)**  
Locks telemetry to a single hardcoded IP address. Ideal for:
- Permanent installations
- Dedicated Companion Mac Studio
- Eliminating rogue network traffic

**Mode 2 (Subnet Broadcast)**  
Broadcasts to all hosts on the local subnet (`.255` address). Ideal for:
- Dynamic environments
- Multi-subscriber setups (QLab + Companion simultaneously)
- Single-subnet deployments

### Default Configuration

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
Default Speed      : 33% of 180°/s commanded slew rate
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
| Up Arrow | `0xF609FF00` | Move to "up" position |
| Down Arrow | `0xF807FF00` | Move to "down" position |
| Right (Vol+) | `0xB946FF00` | Increment +1° |
| Left (Vol−) | `0xEA15FF00` | Decrement −1° |
| Preset Edit | `0xB847FF00` | Toggle edit mode |
| Buttons [0-9] | `0xE916FF00` - `0xB54AFF00` | Call preset [0-9] or save (in edit mode) |
| Play/Pause | `0xBF40FF00` | Pause/resume toggle |
| Power | `0xF20DFF00` | Detach servo (soft-disable) |
| Plus/Minus buttons | Various | Fine adjustments (±5°) |

**Edit Mode Workflow**:
1. Press **Preset Edit** → `editPos=true`, ready to save
2. Press any **[0-9] button** → saves current servo angle to that preset slot
3. Press **Preset Edit** again to exit edit mode

### 3. **Serial Console (USB, 1Mbaud)**
Connect via USB for direct serial command interface:

```bash
# Example commands
show                # Display all settings
up                  # Move to up position
down                # Move to down position
90                  # Move to 90°
down=10             # Set down position to 10°
up=170              # Set up position to 170°
dnbound=5           # Set lower hard limit to 5°
upbound=175         # Set upper hard limit to 175°
default=90          # Set boot default to 90°
speed=33            # Set default movement speed (1-100%)
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
Same commands as serial console, accessible over network:

```bash
nc 10.0.0.100 23           # macOS
telnet 10.0.0.100 23       # Windows/Linux
# All serial commands work here
```

**Features**:
- Real-time echo with backspace support
- Automatic disconnect after 120 seconds idle
- Shared message dispatch to both serial + Telnet simultaneously

### 5. **OSC (Ethernet, Port 8001)**
Native OSC protocol for QLab, Companion, or any OSC controller:

```python
# Example Python OSC client
import pythonosc.osc_message_builder as osm
from pythonosc.udp_client import SimpleUDpClient

client = SimpleUdpClient("10.0.0.100", 8001)
client.send_message("/dowser/angle", 135)   # Move to 135°
client.send_message("/dowser/up", "")        # Move to up
client.send_message("/dowser/query", "")     # Request state
```

## Software Architecture

### Control Flow

```
loop()
├─ Telemetry Engine (20Hz gated)
│  ├─ Motion detection → broadcast if targetPos changed
│  └─ Idle heartbeat → broadcast every 1 second (if static)
├─ processIncomingOSC() → handle /dowser/*, /cue/* commands
├─ processTelnetConsole() → handle Telnet input
├─ processSerialConsole() → handle USB serial input
├─ processPhysicalButton() → debounced button (pin 2)
├─ processIRInput() → IR remote decoder (pin 5)
└─ updateServo() → write targetPos to servo, broadcast on change
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
| `rebootDevice()` | Soft reset via watchdog timer |

### Configuration Structure

```c
struct DeviceConfiguration {
  uint8_t magicNumber;          // 0x53 (factory reset marker)
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
  uint8_t defaultPos;           // Boot power-on position (default 10°)
  uint8_t irPresets[10];        // IR preset angles [0-9]
  uint8_t defaultSpeed;         // Default commanded slew rate (default 33%)
};
```

Stored at EEPROM address 0. Marker `0x53` configurations are migrated in place to marker `0x55` with the existing settings preserved and default speed initialized to 33%. Unknown markers trigger factory defaults. Valid speed values are 1-100%.

## Dependency Libraries

Install via Arduino IDE Library Manager:

- **SPI** (built-in)
- **Ethernet** (built-in, W5500 support)
- **EthernetUdp** (built-in)
- **Servo** (built-in)
- **EEPROM** (built-in)
- **avr/wdt.h** (built-in, watchdog)
- **[ArduinoOSC](https://github.com/h4arts/ArduinoOSC)** – OSC message codec
- **[IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote)** – IR decoder (v4.x)

## Getting Started

### 1. Hardware Setup
- Assemble Arduino Nano Every + W5500 Ethernet shield
- Connect servo to pin 3 (PWM signal)
- Connect IR receiver to pin 5
- Connect button to pin 2
- Connect LED to pin 4 (cathode to GND, anode through 220Ω resistor)
- Power servo from dedicated 5V supply, ground tied to Arduino GND

### 2. Flash Firmware
1. Install Arduino IDE 2.0+
2. Install all required libraries (see above)
3. Update `mac[]` to a unique address on your network (if needed)
4. Update IR remote codes in `translateIRCommand()` to match your remote
5. Open `firmware/Nano_Dowser_OSC_4_Companion/Nano_Dowser_OSC_4_Companion.ino` and upload

### 3. Initial Configuration
- **USB Serial**: Connect, send `show` to verify defaults
- **Network**: Ping `10.0.0.100` (or your static IP)
- **Servo calibration**: Send `90` to test movement; verify smooth range
- **IR test**: Press remote buttons and watch serial output for codes

### 4. QLab Integration
Background notes on the earlier QLab and macOS serial integration are in [`docs/project-overview.md`](docs/project-overview.md).

### 5. Companion Integration
OSC design and Companion-related troubleshooting notes are in [`docs/conversation-overview-gemini.md`](docs/conversation-overview-gemini.md).

## Default Startup Sequence

On power-up or reboot:
1. Serial baud rate set to 1Mbaud
2. EEPROM loaded; legacy marker `0x53` migrates to `0x55`; unknown markers trigger factory reset
3. Ethernet initialized (static IP or DHCP fallback)
4. Telnet server begins listening on port 23
5. Servo attaches to pin 3, moves to `config.defaultPos` (10°)
6. IR receiver starts on pin 5
7. Telemetry routing evaluated (mode 2 = subnet broadcast)
8. System ready; listening for OSC on port 8001

## Troubleshooting

### Device not responding on network
- Check Ethernet cable connection
- Verify IP: `Serial.println(Ethernet.localIP());` in serial monitor
- Ping the device: `ping 10.0.0.100`
- Check telemetry mode: `show` command (should be 2 by default)

### Servo moves erratically
- Verify servo is powered by dedicated 5V supply
- Check for USB power interference (separate power supplies if possible)
- Verify pin 3 signal integrity over the 50-foot XLR cable
- Test servo response time with direct Arduino PWM control

### IR remote not responding
- Verify IR codes match your specific remote model (use IR decoder sketch)
- Update `translateIRCommand()` with correct codes
- Check IR receiver on pin 5 for dust/obstruction
- Enable LED feedback: `IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK)`

### QLab/Companion not receiving telemetry
- Verify telemetry mode: `telemetry=2` for broadcast, `telemetry=1` for fixed target
- Check target port: default is 12000 (Companion), 53000 (QLab)
- Verify firewall allows UDP on those ports
- Test with `nc -l -u 12000` and send `/dowser/query`

### Configuration lost after power cycle
- Verify `save` command was sent before powering down
- Check EEPROM health: may need chip replacement if corrupted
- Force factory reset by sending `/dowser/config/queryAll` immediately after boot

## Roadmap / Future Enhancements

- [ ] Version config struct for non-destructive firmware updates
- [ ] Auto-save IR presets to EEPROM (currently manual `save` required)
- [ ] Velocity/ramping profiles for smoother servo motion
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
│   ├── Nano_Dowser_OSC_4_Companion/
│   │   └── Nano_Dowser_OSC_4_Companion.ino  # Main firmware
│   └── archived/
│       ├── README.md                   # Archived sketch comparison
│       └── ...                         # Earlier and experimental sketches
```

See [`firmware/archived/README.md`](firmware/archived/README.md) for a feature
comparison and notes on the archived firmware variants.
