#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <EEPROM.h>
#include <Servo.h>
#include <avr/wdt.h>

// --- PHYSICAL PIN CONFIGURATION ---
#define BUTTON_PIN 2
#define SERVO_PIN 3
#define ETHERNET_CS_PIN 10
#define IR_RECEIVE_PIN 5
#define LED_PIN 4

#include <IRremote.h>  // after pin definitions for some reason, intentional


// Hardware Drivers
Servo myservo;
EthernetUDP Udp;


// Hardware Network MAC
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// EEPROM Storage Map Structure
struct DeviceConfiguration {
  uint8_t magicNumber;         // Profile Verification (0x54 = 'T')
  uint8_t preferStatic;        // 1 = Static, 0 = DHCP
  uint8_t staticIP[4];         // Local Fallback Static IP
  uint8_t subnetMask[4];       // Subnet Mask
  uint8_t gatewayIP[4];        // Gateway / DNS
  uint16_t localPort;          // Incoming OSC Listening Port
  uint16_t targetPort;         // Outbound Telemetry Port (e.g., 53000)
  uint8_t telemetryMode;       // 0 = Reply to Sender, 1 = Fixed Target, 2 = Broadcast
  uint8_t fixedTargetIP[4];    // Target Host if Mode == 1
  uint8_t heartbeatEnabled;    // 1 = Active, 0 = Deactivated
  uint16_t heartbeatInterval;  // Gap interval in ms

  // --- NEW PERSISTENT HARDWARE & PRESET CONFIGURATIONS ---
  uint8_t posDown;        // User editable Soft Down preset angle
  uint8_t posUp;          // User editable Soft Up preset angle
  uint8_t dnBound;        // Hard physical low limit mechanical boundary
  uint8_t upBound;        // Hard physical high limit mechanical boundary
  uint8_t defaultPos;     // Boot up / Startup angle default (can match up, down, or custom)
  uint8_t irPresets[10];  // 10 User editable Local IR Preset slots
};

// Global Runtime Variables (Dynamic states, not saved to EEPROM)
int targetPos;
int savedTargetPrePause;
bool configLoadedSuccessfully = false;
// Physical Dowser Limits & Presets
int down;
int up;
int dnBound;
int upBound;

DeviceConfiguration config;

// Live Runtime Telemetry Tracking Registers
IPAddress currentTargetIP;
unsigned int currentTargetPort;
unsigned long lastHeartbeatTime = 0;

// Instantiate Telnet Server on standard port 23
EthernetServer telnetServer(23);
EthernetClient telnetClient;
String telnetBuffer = "";
unsigned long lastTelnetActivityTime = 0;
const unsigned long TELNET_TIMEOUT = 120000;  // 2 minutes in milliseconds



//int tar[] = { down, down, down, down, down, 125, up, up, up, up };  // 10 Local IR Presets */

// Volatile Operational Variables
int lastReportedTarget = -1;
const char* currentPresetName = "System Boot";

// Physical Control Variables (Debounce & State)
int ledState = HIGH;
int buttonState;
int lastButtonState = 0;
int buttonPushCounter = 1;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool editPos = false;
bool presetMode = true;
bool pauseMV = false;

bool lcdEnabled = false;  // Toggle to true when you wire the LCD back in
bool topLine = true;      // Tracks alternating lines for your custom LCD buffer

// --- INITIALIZATION ---

void setup() {
  Serial.begin(1000000);

  // Wait up to 3 seconds for Serial Monitor connection, else bypass safely
  unsigned long serialTimeout = millis();
  while (!Serial) {
    if (millis() - serialTimeout > 3000) break;
  }

  Serial.println("\n=== Initializing Projector Dowser Controller ===");

  // Initialize Non-Volatile Memory Profiles
  loadConfiguration();

  // Initialize runtime state engines directly from the retrieved configuration maps
  targetPos = config.defaultPos;
  savedTargetPrePause = config.posDown;
  down = config.posUp;
  up = config.posDown;
  dnBound = config.dnBound;
  upBound = config.upBound;

  // Bring W5500 Core Online
  Ethernet.init(ETHERNET_CS_PIN);
  initNetwork();
  Udp.begin(config.localPort);

  // Start the Telnet Engine
  telnetServer.begin();

  // Initialize Physical Elements
  myservo.attach(SERVO_PIN);
  myservo.write(down);
  targetPos = down;

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Establish base routing topology
  evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);

  Serial.println("System Ready. Listening for Physical and Network triggers.");
  pushStateToTelemetry("System Initialized");
}

// --- CORE EXECUTION LOOP ---

void loop() {
  // 1. Process Network Traffic
  processIncomingOSC();
  processTelnetConsole();

  // 2. Process Interactive Serial Terminal
  processSerialConsole();

  // 3. Process Physical Override Button
  processPhysicalButton();

  // 4. Process Infrared Remote Decodes
  processIRInput();

  // 5. Run Asynchronous Motor Driver
  updateServo();

  // 6. Non-blocking Network Heartbeat Sequence
  if (config.heartbeatEnabled == 1) {
    if (millis() - lastHeartbeatTime >= config.heartbeatInterval) {
      lastHeartbeatTime = millis();
      sendHeartbeat();
    }
  }
}

// --- HARDWARE PROCESSING ENGINE ---

void updateServo() {
  // Mechanical Boundaries Constraints Guardrail
  if (targetPos > upBound) {
    Serial.println("[Warning] Motor hit Software Ceiling bound!");
    targetPos = upBound - 1;
  } else if (targetPos < dnBound) {
    Serial.println("[Warning] Motor hit Software Floor bound!");
    targetPos = dnBound + 1;
  }

  // Direct Non-Blocking Write
  myservo.write(targetPos);

  // Change-Latch Telemetry Dispatcher: Triggers ONLY on target update
  if (targetPos != lastReportedTarget) {
    if (targetPos == up) {
      currentPresetName = "Open (Up)";
    } else if (targetPos == down) {
      currentPresetName = "Closed (Down)";
    } else {
      static char dynamicLabel[24];
      snprintf(dynamicLabel, sizeof(dynamicLabel), "Angle: %d", targetPos);
      currentPresetName = dynamicLabel;
    }

    Serial.print("Servo Moving to Target: ");
    Serial.println(currentPresetName);
    pushStateToTelemetry(currentPresetName);

    lastReportedTarget = targetPos;
  }
}

void processPhysicalButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == 1) {
        Serial.print("Chassis Override Button Triggered #");
        Serial.println(buttonPushCounter++);

        // Toggle the target position non-blockingly
        targetPos = (targetPos == down) ? up : down;
        ledState = LOW;
      } else {
        ledState = HIGH;
      }
      digitalWrite(LED_PIN, ledState);
    }
  }
  lastButtonState = reading;
}

void processIRInput() {
  if (IrReceiver.decode()) {
    long IRData = IrReceiver.decodedIRData.decodedRawData;
    if (IRData != 0) {  // Ignore raw float errors/null anomalies
      translateIRCommand(IRData);
    }
    IrReceiver.resume();
  }
}

// --- OSC NETWORK ROUTER ENGINE ---

void processIncomingOSC() {
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    evaluateTelemetryRouting(Udp.remoteIP(), Udp.remotePort());

    while (packetSize--) {
      msgIn.fill(Udp.read());
    }

    if (!msgIn.hasError()) {
      //Serial.println("Incoming OSC");
      // Core Functional API Endpoints
      msgIn.route("/dowser/up", handleOSCUp);
      msgIn.route("/dowser/down", handleOSCDown);
      msgIn.route("/dowser/angle", handleOSCAngle);
      msgIn.route("/dowser/reset", handleOSCReset);

      // Configuration Remote Engineering Endpoints
      msgIn.route("/config/preferStatic", setOSCPreferStatic);
      msgIn.route("/config/telemetryMode", setOSCTelemetryMode);
      msgIn.route("/config/heartbeat", setOSCHeartbeat);
      msgIn.route("/config/save", executeOSCSave);

    } else {
      Serial.println("OSC Error");
    }
  }
}

// OSC Core Functional Callbacks
void handleOSCUp(OSCMessage& msg, int addrOffset) {
  targetPos = up;
}
void handleOSCDown(OSCMessage& msg, int addrOffset) {
  targetPos = down;
}
void handleOSCReset(OSCMessage& msg, int addrOffset) {
  targetPos = down;
  Serial.println("Remote Reset Issued.");
}
void handleOSCAngle(OSCMessage& msg, int addrOffset) {
  if (msg.size() > 0 && (msg.isInt(0) || msg.isFloat(0))) {
    int angle = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);
    if (angle >= dnBound && angle <= upBound) {
      targetPos = angle;
    }
  }
}

// --- OSC REMOTE CONFIGURATION PROCESSING ---

void setOSCPreferStatic(OSCMessage& msg, int addrOffset) {
  if (msg.size() > 0 && msg.isInt(0)) {
    config.preferStatic = msg.getInt(0) == 1 ? 1 : 0;
    Serial.print("OSC Config Cache Change: preferStatic = ");
    Serial.println(config.preferStatic);
  }
}

void setOSCTelemetryMode(OSCMessage& msg, int addrOffset) {
  if (msg.size() > 0 && msg.isInt(0)) {
    int mode = msg.getInt(0);
    if (mode >= 0 && mode <= 2) {
      config.telemetryMode = mode;
      evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
      Serial.print("OSC Config Cache Change: telemetryMode = ");
      Serial.println(config.telemetryMode);
    }
  }
}

void setOSCHeartbeat(OSCMessage& msg, int addrOffset) {
  if (msg.size() > 0 && msg.isInt(0)) {
    config.heartbeatEnabled = msg.getInt(0) == 1 ? 1 : 0;
    Serial.print("OSC Config Cache Change: Heartbeat Enabled = ");
    Serial.println(config.heartbeatEnabled);
  }
}

void executeOSCSave(OSCMessage& msg, int addrOffset) {
  Serial.println("OSC Remote Commit Order Received. Writing Cache to EEPROM and Restarting System...");
  EEPROM.put(0, config);
  delay(500);
  rebootDevice();
}
void sendOSCLogMessage(const char* source, String actionText) {
  // Build clean container targeted at your custom console workspace log path
  OSCMessage msg("/dowser/log");

  msg.add(source);
  msg.add(actionText.c_str());

  // Dispatch packet using your existing staged configuration targets
  Udp.beginPacket(config.fixedTargetIP, config.targetPort);
  msg.send(Udp);
  Udp.endPacket();

  msg.empty();  // Free memory structures back to stack allocation
}
void dispatchMessage(const char* source, String actionText) {
  // Enforce absolute string consistency: "[SOURCE] Action/Value"
  String formattedConsoleLine = "[" + String(source) + "] " + actionText;

  // Channel 1: Local USB Hardware Serial
  Serial.println(formattedConsoleLine);

  // Channel 2: Live Network Telnet Console
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print("\r\33[2K");  // ANSI: Wipe current user "dowser> " line
    telnetClient.println(formattedConsoleLine);
    telnetClient.print("dowser> ");
  }

  // Channel 3: Outbound OSC Telemetry Log
  // Sends a structured string packet back out to QLab / Control Console
  sendOSCLogMessage(source, actionText);

  // Channel 4: Local Physical LCD Grid Layout
  if (lcdEnabled) {
    //updateLCDScreen(source, actionText);
  }
}

/*void updateLCDScreen(const char* source, String actionText) {
  // Clear layout maps entirely
  lcd.clear();

  // Line 0 (Top): Identity Tracking 
  lcd.setCursor(0, 0);
  lcd.print("SRC: ");
  lcd.print(source);

  // Line 1 (Bottom): Command / Target Metric
  lcd.setCursor(0, 1);
  if (actionText.length() > 16) {
    lcd.print(actionText.substring(0, 16)); // Hard physical truncate safety boundary
  } else {
    lcd.print(actionText);
  }
}*/

// --- INTERACTIVE SERIAL COMMAND MONITOR ---

void processSerialConsole() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      handleCommonConsoleCommands(input, Serial);  // Pass the command and the Serial object
    }
  }
}

// --- TELEMETRY ENGINE & BACKPLANE NETWORK DRIVER ---

void initNetwork() {
  if (config.preferStatic == 1) {
    Serial.println("Assigning Static Profile parameters...");
    IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
    IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
    IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
    Ethernet.begin(mac, ip, gw, gw, sn);
  } else {
    Serial.println("Requesting DHCP Lease Line Configuration...");
    if (Ethernet.begin(mac, 8000, 2000) == 0) {  // Fast 8-sec boot gate timeout
      Serial.println("DHCP Server Absent. Falling back to Safe Static Profile.");
      IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
      IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
      IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
      Ethernet.begin(mac, ip, gw, gw, sn);
    }
  }
  Serial.print("Active Controller Online IP: ");
  Serial.println(Ethernet.localIP());
}

void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort) {
  switch (config.telemetryMode) {
    case 0:  // Reply to Sender Mode
      if (senderIP != IPAddress(0, 0, 0, 0)) {
        currentTargetIP = senderIP;
        currentTargetPort = config.targetPort;
      } else {
        currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
        currentTargetPort = config.targetPort;
      }
      break;
    case 1:  // Fixed Address Matrix Target
      currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
      currentTargetPort = config.targetPort;
      break;
    case 2:  // Network Subnet Wide Broadcast
      IPAddress loc = Ethernet.localIP();
      currentTargetIP = IPAddress(loc[0], loc[1], loc[2], 255);
      currentTargetPort = config.targetPort;
      break;
  }
}

void pushStateToTelemetry(const char* notesMsg) {
  OSCMessage msgOut("/cue/DOWSER_STATUS/notes");
  msgOut.add(notesMsg);

  Udp.beginPacket(currentTargetIP, currentTargetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

void sendHeartbeat() {
  OSCMessage msgOut("/dowser/alive");
  msgOut.add(targetPos);

  Udp.beginPacket(currentTargetIP, currentTargetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

// --- LOCAL PERIPHERAL IR CONTEXTUAL PARSER ---

void translateIRCommand(long IRData) {
  switch (IRData) {
    case 0xBA45FF00: Serial.println("IR: POWER"); break;
    case 0xB847FF00:  // FUNC/STOP
      editPos = !editPos;
      Serial.print("IR: Edit Mode Status = ");
      Serial.println(editPos ? "ON" : "OFF");
      break;
    case 0xB946FF00: targetPos = myservo.read() + 1; break;  // VOL+
    case 0xBB44FF00: targetPos = myservo.read() - 5; break;  // FAST BACK
    case 0xBF40FF00:                                         // PAUSE Button
      if (pauseMV) {
        targetPos = savedTargetPrePause;
        pauseMV = false;
        Serial.println("IR: Drive Resumed");
      } else {
        savedTargetPrePause = targetPos;
        targetPos = myservo.read();
        pauseMV = true;
        Serial.print("IR: Drive Paused at ");
        Serial.println(targetPos);
      }
      break;
    case 0xBC43FF00: targetPos = myservo.read() + 5; break;  // FAST FORWARD
    case 0xF807FF00: targetPos = down; break;                // DOWN Arrow Button
    case 0xEA15FF00: targetPos = myservo.read() - 1; break;  // VOL-
    case 0xF609FF00: targetPos = up; break;                  // UP Arrow Button
    case 0xF20DFF00:
      myservo.detach();
      Serial.println("IR: Servo Safely Detached.");
      break;  // ST/REPT

    // Numeric Array Preset Address Mappings
    case 0xE916FF00: handleIRPreset(0); break;
    case 0xF30CFF00: handleIRPreset(1); break;
    case 0xE718FF00: handleIRPreset(2); break;
    case 0xA15EFF00: handleIRPreset(3); break;
    case 0xF708FF00: handleIRPreset(4); break;
    case 0xE31CFF00: handleIRPreset(5); break;
    case 0xA55AFF00: handleIRPreset(6); break;
    case 0xBD42FF00: handleIRPreset(7); break;
    case 0xAD52FF00: handleIRPreset(8); break;
    case 0xB54AFF00: handleIRPreset(9); break;
    default: break;
  }
}
void handleIRPreset(int num) {
  if (editPos) {
    if (presetMode) {
      presetMode = false;
    } else {
      presetMode = true;
      // Read current physical position and commit directly to the persistent array slot
      config.irPresets[num] = (uint8_t)myservo.read();

      // Standardized Config Format: "[IR_REMOTE] Set Preset X -> Y"
      dispatchMessage("IR_REMOTE", "Set Preset " + String(num) + " -> " + String(config.irPresets[num]));
    }
  } else {
    // Pull the staged position directly out of the persistent array slot
    targetPos = config.irPresets[num];

    // Standardized Execution Format: "[IR_REMOTE] Move -> Y"
    dispatchMessage("IR_REMOTE", "Move -> " + String(targetPos));
  }
}

// --- UTILITIES & STORAGE DATA PROTOCOLS ---

void loadConfiguration() {
  EEPROM.get(0, config);
  if (config.magicNumber != 0x54) {
    Serial.println("Blank Memory Profile Detected. Injecting safe system architecture defaults...");
    config.magicNumber = 0x54;
    config.preferStatic = 1;
    config.staticIP[0] = 10;
    config.staticIP[1] = 0;
    config.staticIP[2] = 0;
    config.staticIP[3] = 100;
    config.subnetMask[0] = 255;
    config.subnetMask[1] = 255;
    config.subnetMask[2] = 255;
    config.subnetMask[3] = 0;
    config.gatewayIP[0] = 10;
    config.gatewayIP[1] = 0;
    config.gatewayIP[2] = 0;
    config.gatewayIP[3] = 1;
    config.localPort = 8000;
    config.targetPort = 53000;
    config.telemetryMode = 0;  // Reply-to-sender tracking standard
    config.fixedTargetIP[0] = 10;
    config.fixedTargetIP[1] = 0;
    config.fixedTargetIP[2] = 0;
    config.fixedTargetIP[3] = 15;
    config.heartbeatEnabled = 1;
    config.heartbeatInterval = 5000;
    config.posDown = 10;
    config.posUp = 170;
    config.dnBound = 7;
    config.upBound = 173;
    config.defaultPos = 10;  // Boots to down position by default
    uint8_t defaultIR[10] = { 10, 10, 10, 10, 10, 125, 170, 170, 170, 170 };
    memcpy(config.irPresets, defaultIR, 10);
    EEPROM.put(0, config);
  } else {
    Serial.println("EEPROM Secure Integrity Check: Verified.");
  }
}

void parseSerialIP(String inputStr, uint8_t* ipArray) {
  int parts[4] = { 0, 0, 0, 0 };
  int counter = 0;
  int lastIndex = 0;
  for (unsigned int i = 0; i < inputStr.length(); i++) {
    if (inputStr.charAt(i) == '.') {
      parts[counter++] = inputStr.substring(lastIndex, i).toInt();
      lastIndex = i + 1;
    }
  }
  parts[counter] = inputStr.substring(lastIndex).toInt();
  for (int x = 0; x < 4; x++) ipArray[x] = (uint8_t)parts[x];
}

void printIPArray(uint8_t* array) {
  Serial.print(array[0]);
  Serial.print(".");
  Serial.print(array[1]);
  Serial.print(".");
  Serial.print(array[2]);
  Serial.print(".");
  Serial.println(array[3]);
}

void processTelnetConsole() {
  // CASE 1: No active engineering session is live. Listen for a new connection.
  if (!telnetClient.connected()) {

    // CHANGE THIS LINE FROM .available() TO .accept()
    EthernetClient incomingClient = telnetServer.accept();

    if (incomingClient) {
      telnetClient = incomingClient;
      telnetClient.println("\r\n=== Connected to Projector Dowser Telnet Console ===");
      telnetClient.println("Type 'show' or '?' for current configuration map.");
      telnetClient.print("dowser> ");
      telnetBuffer = "";
      lastTelnetActivityTime = millis();  // Initialize the activity tracker
    }
  }
  // CASE 2: A session is already active. Service your keystrokes directly.
  else {
    // Guardrail: Enforce the inactivity timeout
    if (millis() - lastTelnetActivityTime > TELNET_TIMEOUT) {
      telnetClient.println("\r\nSession closed due to inactivity timeout.");
      telnetClient.stop();
      Serial.println("Telnet connection dropped due to inactivity timeout.");
      return;
    }

    // Pull data directly from your specific active client socket
    while (telnetClient.available() > 0) {
      char c = telnetClient.read();
      lastTelnetActivityTime = millis();  // Reset timeout on every keystroke

      // Handle Carriage Return / Line Feed execution targets
      if (c == '\r' || c == '\n') {
        telnetBuffer.trim();
        if (telnetBuffer.length() > 0) {
          telnetClient.println();
          handleCommonConsoleCommands(telnetBuffer, telnetClient);
          telnetBuffer = "";
          telnetClient.print("dowser> ");
        }
      }
      // Handle physical Backspace/Delete keys safely
      else if (c == 8 || c == 127) {
        if (telnetBuffer.length() > 0) {
          telnetBuffer.remove(telnetBuffer.length() - 1);
          telnetClient.print("\b \b");
        }
      }
      // Buffer printable characters only
      else if (c >= 32 && c <= 126) {
        telnetBuffer += c;
        telnetClient.print(c);  // Echo character back to your terminal window
      }
    }
  }
}



void handleCommonConsoleCommands(String input, Print& output) {
  const char* source = (&output == &Serial) ? "SERIAL" : "TELNET";

  if (input.equalsIgnoreCase("help") || input.equalsIgnoreCase("?")) {
    output.println("\r\n=== Interface Commands Menu ===");
    output.println("show              - Display entire device profile parameters");
    output.println("[Integer 0-180]   - Input a naked digit to command manual raw servo angle");
    output.println("down=10           - Adjust the soft DOWN position preset");
    output.println("up=170            - Adjust the soft UP position preset");
    output.println("dnbound=7         - Adjust hard mechanical low travel limit");
    output.println("upbound=173       - Adjust hard mechanical high travel limit");
    output.println("default=10        - Adjust boot/startup position default angle");
    output.println("presetX=ANGLE     - Set IR remote slots 0-9 (e.g., preset5=125)");
    output.println("save              - Commit changes to internal EEPROM chip & reboot");
    output.println("--- Network Configs ---");
    output.println("static=1 | ip=10.0.0.100 | targetip=10.0.0.15 | localport=8000 | targetport=53000");
    return;
  }

  if (input.equalsIgnoreCase("show")) {
    output.println("\r\n--- Current Configuration Cache Map ---");
    output.print("Boot Policy Mode:  ");
    output.println(config.preferStatic == 1 ? "STATIC IP" : "DHCP ONLY");
    output.print("Local Input Port:  ");
    output.println(config.localPort);
    output.print("Outbound Base Port:");
    output.println(config.targetPort);
    output.println("-----------------------------------------");
    output.print("Hard Low Limit:    ");
    output.print(config.dnBound);
    output.println(" deg");
    output.print("Hard High Limit:   ");
    output.print(config.upBound);
    output.println(" deg");
    output.print("Soft DOWN Position:");
    output.print(config.posDown);
    output.println(" deg");
    output.print("Soft UP Position:  ");
    output.print(config.posUp);
    output.println(" deg");
    output.print("Boot Default Angle:");
    output.print(config.defaultPos);
    output.println(" deg");

    // Dynamic array parsing block
    output.print("IR Presets (0-9):  [");
    for (int i = 0; i < 10; i++) {
      output.print(config.irPresets[i]);
      if (i < 9) output.print(", ");
    }
    output.println("]");
    output.print("Active Target Pos: ");
    output.print(targetPos);
    output.println(" deg");
    output.println("=========================================");
    return;
  }

  if (input.equalsIgnoreCase("save")) {
    EEPROM.put(0, config);
    
    dispatchMessage(source, "EEPROM Save & Reset");
    delay(500);
    rebootDevice();
    return;
  }

  // Core Mechanical Slicing & Boundary Validations
  if (input.startsWith("down=")) {
    int val = input.substring(5).toInt();
    if (val >= config.dnBound && val <= config.upBound) {
      config.posDown = (uint8_t)val;
      dispatchMessage(source, "Set Down -> " + String(config.posDown));
    } else {
      output.println("Error: Value violates hard travel limits.");
    }
  } else if (input.startsWith("up=")) {
    int val = input.substring(3).toInt();
    if (val >= config.dnBound && val <= config.upBound) {
      config.posUp = (uint8_t)val;
      dispatchMessage(source, "Set Up -> " + String(config.posUp));
    } else {
      output.println("Error: Value violates hard travel limits.");
    }
  } else if (input.startsWith("dnbound=")) {
    int val = input.substring(8).toInt();
    if (val >= 0 && val <= 180) {
      config.dnBound = (uint8_t)val;
      dispatchMessage(source, "Set Lo-Bound -> " + String(config.dnBound));
    } else {
      output.println("Error: Absolute angle bounds range is 0-180.");
    }
  } else if (input.startsWith("upbound=")) {
    int val = input.substring(8).toInt();
    if (val >= 0 && val <= 180) {
      config.upBound = (uint8_t)val;
      dispatchMessage(source, "Set Hi-Bound -> " + String(config.upBound));
    } else {
      output.println("Error: Absolute angle bounds range is 0-180.");
    }
  } else if (input.startsWith("default=")) {
    int val = input.substring(8).toInt();
    if (val >= config.dnBound && val <= config.upBound) {
      config.defaultPos = (uint8_t)val;
      dispatchMessage(source, "Set Boot Default -> " + String(config.defaultPos));
    } else {
      output.println("Error: Boot angle must fall inside mechanical boundaries.");
    }
  }
  // Dynamic Indexing IR Preset Matrix Parser (Slices 'presetX=Y')
  else if (input.startsWith("preset")) {
    int equalsIdx = input.indexOf('=');
    if (equalsIdx > 6) {
      int idx = input.substring(6, equalsIdx).toInt();
      int val = input.substring(equalsIdx + 1).toInt();

      if (idx >= 0 && idx < 10) {
        if (val >= config.dnBound && val <= config.upBound) {
          config.irPresets[idx] = (uint8_t)val;
          dispatchMessage(source, "Set Preset " + String(idx) + " -> " + String(val));
        } else {
          output.println("Error: Value violates hard travel limits.");
        }
      } else {
        output.println("Error: Valid preset slots are index 0-9.");
      }
    }
  }
  // Legacy Network Property Routers
  else if (input.startsWith("ip=")) {
    parseSerialIP(input.substring(3), config.staticIP);
    dispatchMessage(source, "Staged Static IP");
  } else if (input.startsWith("targetip=")) {
    parseSerialIP(input.substring(9), config.fixedTargetIP);
    dispatchMessage(source, "Staged Target IP");
  } else if (input.startsWith("localport=")) {
    config.localPort = (uint16_t)input.substring(10).toInt();
    dispatchMessage(source, "Set Local Port -> " + String(config.localPort));
  } else if (input.startsWith("targetport=")) {
    config.targetPort = (uint16_t)input.substring(11).toInt();
    dispatchMessage(source, "Set Target Port -> " + String(config.targetPort));
  } else if (input.startsWith("static=")) {
    config.preferStatic = input.substring(7).toInt() == 1 ? 1 : 0;
    dispatchMessage(source, "Set PreferStatic -> " + String(config.preferStatic));
  } else if (input.startsWith("telemetry=")) {
    config.telemetryMode = input.substring(10).toInt();
    dispatchMessage(source, "Set Telemetry Mode -> " + String(config.telemetryMode));
  } else if (input.startsWith("heartbeat=")) {
    config.heartbeatEnabled = input.substring(10).toInt() == 1 ? 1 : 0;
    dispatchMessage(source, "Set Heartbeat -> " + String(config.heartbeatEnabled));
  } else {
    // Check for naked numeric motor jump target
    bool isNumeric = true;
    for (uint8_t i = 0; i < input.length(); i++) {
      if (!isDigit(input.charAt(i))) {
        isNumeric = false;
        break;
      }
    }

    if (isNumeric) {
      int parsedVal = input.toInt();
      if (parsedVal >= config.dnBound && parsedVal <= config.upBound) {
        targetPos = parsedVal;
        dispatchMessage(source, "Move -> " + String(targetPos));
      } else {
        output.println("Error: Target angle breaches hardware constraints.");
      }
    } else {
      output.println("Command unrecognized. Type '?' for menu protocol syntax map.");
    }
  }
}

void rebootDevice() {
  Serial.println("Executing Native Software Reset Register Instruction...");
  Serial.flush();
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);  // Open megaAVR safety gates and reset
  while (true) {}
}