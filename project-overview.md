# Project Overview
The project is a custom Arduino-based projector dowser controlled via a servo motor. It is designed to be operated locally using a physical button and an IR remote, as well as remotely from a tech table via QLab over USB serial.

## Hardware & Power Architecture
Microcontroller: Arduino Nano Every.

Power Isolation: Uses a split power design to prevent brownouts and noise. A dedicated 5V 4A power supply powers the servo motor, with its ground tied to the Arduino GND pin (common ground). The Arduino itself is powered via USB.

Long-Distance Umbilical Cable: Runs over a 50-foot XLR (microphone) cable to connect the tech table to the stage, carrying the 5V PWM signal (Pin 3) and ground return.

## Arduino Firmware & Logic
Servo Movement & Boundaries: Position updating logic enforces lower and upper safety bounds (dnBound and upBound) to prevent mechanical motor binding.

Serial String Parsing: Upgraded serialToLCD() from single-character/ASCII switch-case mapping to reading full line strings (readStringUntil('\n')) converted to integers (toInt()). This allows QLab to command any direct degree angle (e.g., 15, 90, 135).

Conflict Resolution: Addressed timer conflicts between the Servo and IRremote libraries, and removed blocking delay() calls to ensure responsiveness across button, IR, and serial inputs.

## QLab & macOS Integration
Serial Auto-Reset Prevention: Addressed the issue where opening a serial connection on macOS triggers DTR, resetting the Arduino and dropping QLab commands.

Named Pipe & Python Background Worker: Implemented a virtual named pipe (/tmp/servo_pipe) paired with a persistent Python script. Python opens the USB serial connection once and holds it open in the background, listening for incoming target angles from QLab without resetting the microcontroller.

## QLab AppleScript Formatting:

INIT SERVO Cue: Spawns the background Python worker safely using AppleScript's quoted form of pythonCode to avoid quote-escaping syntax errors.

Movement Cues: Uses lightweight shell commands (do shell script "echo '135' > /tmp/servo_pipe") to trigger position changes with zero latency.

KILL SERVO Cue: Cleans up background processes (killall python; rm -f /tmp/servo_pipe) post-show.

Playback Mac Adaptation (macOS 10.14.6 Mojave):

Adjusted port path configuration to match the playback computer's detected device address (/dev/cu.*).

Updated script references from python3 to Mojave's native system Python (python).
