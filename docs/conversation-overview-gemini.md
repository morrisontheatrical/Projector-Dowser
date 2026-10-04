Conversation Overview
This technical collaboration focused on configuring, troubleshooting, and optimizing the OSC network firmware for an Arduino-based stage dowser. The system communicates concurrently with Bitfocus Companion (for Stream Deck feedback) and QLab (for workspace cue notes).

Key Architectural Decisions & System Evolution
1. Dynamic Multi-Port Telemetry Array
Problem: Changing Companion’s listening port to 53000 caused an OS port conflict because QLab was running on the same host and locked UDP port 53000. However, standard unicast UDP to port 8001 left QLab completely blind to position updates.

Decision: Instead of hardcoding backup pathways or relying on single-port unicast, the system was upgraded to support an array of target ports (config.targetPorts[3]) combined with Subnet Broadcast (Telemetry Mode 2). This allows a single state engine cycle to burst updates to ports 8001 (Companion), 53000 (QLab), and an extra expansion slot simultaneously.

2. Non-Blocking 20Hz Telemetry Engine
Problem: Unthrottled calls to broadcastCompanionState() flooded the network with thousands of packets per second. While QLab handled this, Companion’s Node.js event loop choked, leaving custom variables reading undefined or stuck on static values. Hard delays (delay(2)) inside the network loop also caused motor jitter.

Decision: Moved telemetry execution to a non-blocking millis() clock in loop(). The state engine streams updates at a fluid 20Hz (every 50ms) only while the motor is in transit, falling back to a 1-second heartbeat when idle.

3. REST-Style OSC Configuration API
Problem: The system needed a clean way to both query and live-edit EEPROM settings from a Companion surface without duplicating command paths.

Decision: Implemented a unified OSC pattern (/dowser/config/<parameter>):

No arguments (msg.size() == 0): Treated as a Query (GET); the Arduino broadcasts the requested parameter back.

Has arguments (msg.size() > 0): Treated as a Setter (SET); the Arduino validates boundaries, updates config, and broadcasts the new value as confirmation.

Added /dowser/config/queryAll to execute a bulk dump of all 12 configuration values.

Technical Troubleshooting Details
1. Telemetry Routing & C++ Switch Scope
Issue: evaluateTelemetryRouting() threw compiler errors after deleting config.targetPort and declaring IPAddress loc = Ethernet.localIP() inside a switch case.

Fix: Port handling was stripped from evaluateTelemetryRouting() (delegated entirely to the multi-port loop), and case 2: was wrapped in explicit curly braces { } to enforce proper variable scoping.

2. Inverted Up/Down Position Feedback
Issue: Sending /dowser/up drove the motor to 10° instead of 170°, causing the isUp boolean logic to fail.

Fix: Corrected a scope mismatch inside processIncomingOSC(). The /dowser/up and /dowser/down OSC routes were referencing legacy position variables rather than pulling directly from config.posUp and config.posDown.

3. Companion Event Loop & Packet Race Conditions
Issue 1 (Static Angle Readout): custom_dowser_real_angle stopped updating because Companion’s trigger was watching $(osc_2:latest_received_path). Since /dowser/state/angle was sent last in every burst, the path string never changed, preventing the trigger from firing.

Fix: Changed the Companion trigger event to watch $(osc_2:latest_received_args) or $(osc_2:latest_received_timestamp) so it fires on every incoming network frame.

Issue 2 (queryAll Packet Dropping): Running a bulk query sent 12 configuration packets in under 1ms, overwriting Companion's internal latest_received_path before JavaScript triggers could process individual items.

Fix: Added 20ms pacing delays (delay(20)) between each item inside broadcastAllConfigSettings(), spreading the dump over ~240ms so Companion catches every parameter update.

Final Configuration Map
Companion Trigger Template for Config Readouts
Event: On variable change → osc_2:latest_received_timestamp

Condition: Variable value check → $(osc_2:latest_received_path) == /dowser/config/up

Action: internal: Set custom variable from expression

Target Variable: config_pos_up

Expression: $(osc_2:latest_received_args)
