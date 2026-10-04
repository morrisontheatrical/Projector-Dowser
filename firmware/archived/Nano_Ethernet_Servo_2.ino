#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <EEPROM.h>
#include <avr/wdt.h>  // Used for clean software reboots

// Unique MAC address for your hardware
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Configuration Structure stored in EEPROM
struct DeviceConfiguration {
  uint8_t magicNumber;         // Verification token (0x53 = 'S' for Seth)
  uint8_t preferStatic;        // 1 = Static IP, 0 = DHCP
  uint8_t staticIP[4];         // Static IP address fallback
  uint8_t subnetMask[4];       // Subnet mask
  uint8_t gatewayIP[4];        // Network gateway
  uint16_t localPort;          // Port Arduino listens on
  uint16_t targetPort;         // Outbound port for feedback
  uint8_t telemetryMode;       // 0 = Reply to Sender, 1 = Fixed Target, 2 = Subnet Broadcast
  uint8_t fixedTargetIP[4];    // Target IP used if telemetryMode == 1
  uint8_t heartbeatEnabled;    // 1 = Enabled, 0 = Disabled
  uint16_t heartbeatInterval;  // Interval in milliseconds
};

DeviceConfiguration config;

// Live Runtime Networking Variables
IPAddress currentTargetIP;
unsigned int currentTargetPort;
unsigned long lastHeartbeatTime = 0;
EthernetUDP Udp;

// Global tracking variables for the physical dowser state
int currentAngle = 0;
const char* currentPreset = "Unknown";

void setup() {
  Serial.begin(1000000);
  // Wait up to 3 seconds for the Serial Monitor to open, otherwise bypass
  unsigned long serialTimeout = millis();
  while (!Serial) {
    if (millis() - serialTimeout > 3000) {
      break;
    }
  }

  Serial.println("\n--- OSC Project Framework Booting ---");
  loadConfiguration();

  // Initialize Network Connection
  initNetwork();

  // Bind UDP port
  Udp.begin(config.localPort);
  Serial.print("Listening for OSC data on port: ");
  Serial.println(config.localPort);

  // Set initial telemetry destinations based on policy
  evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);  // Seed with zero-state
}

void loop() {
  // 1. Process Network Traffic
  processIncomingOSC();

  // 2. Process Serial Commands (Configuration Backup Loop)
  processSerialConsole();

  // 3. Heartbeat Engine (Non-blocking)
  if (config.heartbeatEnabled == 1) {
    if (millis() - lastHeartbeatTime >= config.heartbeatInterval) {
      lastHeartbeatTime = millis();
      sendHeartbeat();
    }
  }
}

// --- NETWORKING LAYER ---

void initNetwork() {
  Ethernet.init(10);

  if (config.preferStatic == 1) {
    Serial.println("Network Policy: Attempting Static IP Assignment...");
    IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
    IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
    IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);

    Ethernet.begin(mac, ip, gw, gw, sn);  // Using gateway as DNS server for simplicity
  } else {
    Serial.println("Network Policy: Requesting DHCP Lease from Network...");
    // Attempt DHCP with a 10-second timeout to prevent absolute grid freezing
    if (Ethernet.begin(mac, 10000, 4000) == 0) {
      Serial.println("DHCP Failed! Dropping back to Static IP safety configuration.");
      IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
      IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
      IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
      Ethernet.begin(mac, ip, gw, gw, sn);
    }
  }

  // Verify hardware connection
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("CRITICAL ERROR: W5500 hardware module unreadable via SPI.");
    while (true) { delay(1); }
  }

  Serial.print("Device Network Online. Assigned IP: ");
  Serial.println(Ethernet.localIP());
}

void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort) {
  switch (config.telemetryMode) {
    case 0:  // Reply to Sender
      if (senderIP != IPAddress(0, 0, 0, 0)) {
        currentTargetIP = senderIP;
        currentTargetPort = config.targetPort;  // Use configuration default port
      } else {
        // Fallback before any message has been received
        currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
        currentTargetPort = config.targetPort;
      }
      break;

    case 1:  // Fixed Target IP
      currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
      currentTargetPort = config.targetPort;
      break;

    case 2:  // Subnet Broadcast
      IPAddress local = Ethernet.localIP();
      currentTargetIP = IPAddress(local[0], local[1], local[2], 255);
      currentTargetPort = config.targetPort;
      break;
  }
}

// --- OSC INPUT PARSING & CONTROL ROUTINES ---

void processIncomingOSC() {
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    // Dynamic routing trigger for "Reply to Sender" mode
    evaluateTelemetryRouting(Udp.remoteIP(), Udp.remotePort());

    while (packetSize--) {
      msgIn.fill(Udp.read());
    }

    if (!msgIn.hasError()) {
      // Functional API Router
      msgIn.route("/dowser/up", handleUp);
      msgIn.route("/dowser/down", handleDown);
      msgIn.route("/dowser/angle", handleAngle);

      // Configuration API Router
      msgIn.route("/config/preferStatic", setOSCPreferStatic);
      msgIn.route("/config/telemetryMode", setOSCTelemetryMode);
      msgIn.route("/config/heartbeat", setOSCHeartbeat);
      msgIn.route("/config/save", executeOSCSave);
    }
  }
}

void handleUp(OSCMessage& msg, int addrOffset) {
  currentAngle = 179;
  currentPreset = "Open (Up)";
  Serial.println("Action: Move Up");
  pushStateToTelemetry();
}

void handleDown(OSCMessage& msg, int addrOffset) {
  currentAngle = 1;
  currentPreset = "Closed (Down)";
  Serial.println("Action: Move Down");
  pushStateToTelemetry();
}

void handleAngle(OSCMessage& msg, int addrOffset) {
  if (msg.size() > 0 && (msg.isInt(0) || msg.isFloat(0))) {
    int targetAngle = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);
    if (targetAngle >= 1 && targetAngle <= 179) {
      currentAngle = targetAngle;
      currentPreset = "Custom Angle";
      Serial.print("Action: Sweeping to ");
      Serial.println(currentAngle);
      pushStateToTelemetry();
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

// --- SERIAL MANAGEMENT CONSOLE ---

void processSerialConsole() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("help") || input.equalsIgnoreCase("?")) {
      printSerialMenu();
      return;
    }

    if (input.startsWith("ip=")) {
      parseSerialIP(input.substring(3), config.staticIP);
      Serial.print("Cached Static IP update: ");
      printIPArray(config.staticIP);
    } else if (input.startsWith("targetip=")) {
      parseSerialIP(input.substring(9), config.fixedTargetIP);
      Serial.print("Cached Fixed Target IP update: ");
      printIPArray(config.fixedTargetIP);
    } else if (input.startsWith("localport=")) {
      // "localport=" is 10 characters long, slice everything after index 10
      config.localPort = (uint16_t)input.substring(10).toInt();
      Serial.print("Cached Local Listening Port update: ");
      Serial.println(config.localPort);
    } else if (input.startsWith("targetport=")) {
      // "targetport=" is 11 characters long, slice everything after index 11
      config.targetPort = (uint16_t)input.substring(11).toInt();
      Serial.print("Cached Target Feedback Port update: ");
      Serial.println(config.targetPort);
    } else if (input.startsWith("static=")) {
      config.preferStatic = input.substring(7).toInt() == 1 ? 1 : 0;
      Serial.print("Cached Policy PreferStatic: ");
      Serial.println(config.preferStatic);
    } else if (input.startsWith("telemetry=")) {
      config.telemetryMode = input.substring(10).toInt();
      Serial.print("Cached Telemetry Mode: ");
      Serial.println(config.telemetryMode);
    } else if (input.startsWith("heartbeat=")) {
      config.heartbeatEnabled = input.substring(10).toInt() == 1 ? 1 : 0;
      Serial.print("Cached Heartbeat Toggle: ");
      Serial.println(config.heartbeatEnabled);
    } else if (input.equalsIgnoreCase("save")) {
      Serial.println("Committing variables to non-volatile EEPROM storage and rebooting...");
      EEPROM.put(0, config);
      delay(500);
      rebootDevice();
    } else if (input.equalsIgnoreCase("show")) {
      printCurrentRuntimeParameters();
    }
  }
}

// --- DATA UTILITIES & PERSISTENCE LAYER ---

void loadConfiguration() {
  EEPROM.get(0, config);

  // Verify magic number signature. If structural mismatch, force recovery profile.
  if (config.magicNumber != 0x53) {
    Serial.println("EEPROM Corrupted or Empty. Injecting Factory Safe Default Profile...");
    config.magicNumber = 0x53;
    config.preferStatic = 1;  // Default to static to clear up unmanaged isolated switches
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
    config.targetPort = 53000;  // Standard QLab internal receiver line
    config.telemetryMode = 0;   // Default: Reply to sender
    config.fixedTargetIP[0] = 10;
    config.fixedTargetIP[1] = 0;
    config.fixedTargetIP[2] = 0;
    config.fixedTargetIP[3] = 15;
    config.heartbeatEnabled = 1;
    config.heartbeatInterval = 5000;  // 5-second cadence default

    EEPROM.put(0, config);
  } else {
    Serial.println("EEPROM Secure Integrity Validation: Confirmed.");
  }
}

void parseSerialIP(String inputStr, uint8_t* ipArray) {
  int parts[4];
  int counter = 0;
  int lastIndex = 0;

  for (int i = 0; i < inputStr.length(); i++) {
    if (inputStr.charAt(i) == '.') {
      parts[counter++] = inputStr.substring(lastIndex, i).toInt();
      lastIndex = i + 1;
    }
  }
  parts[counter] = inputStr.substring(lastIndex).toInt();

  for (int x = 0; x < 4; x++) {
    ipArray[x] = (uint8_t)parts[x];
  }
}

void pushStateToTelemetry() {
  OSCMessage msgOut("/cue/DOWSER_STATUS/name");
  msgOut.add(currentPreset);

  Udp.beginPacket(currentTargetIP, currentTargetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

void sendHeartbeat() {
  OSCMessage msgOut("/dowser/alive");
  msgOut.add(currentAngle);  // Sends current angle metric as data payload

  Udp.beginPacket(currentTargetIP, currentTargetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
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

void printSerialMenu() {
  Serial.println("\n=== Console Configuration Commands ===");
  Serial.println("show              - Display entire live runtime and storage block");
  Serial.println("static=1          - Force Static Allocation (static=0 sets DHCP Mode)");
  Serial.println("ip=10.0.0.105     - Update local static allocation address");
  Serial.println("targetip=10.0.0.5 - Update target telemetry host destination");
  Serial.println("localport=8000    - Update incoming port the Arduino listens on");
  Serial.println("targetport=53000  - Update outbound port for QLab/Companion feedback");
  Serial.println("telemetry=X       - Set feedback structure (0=Reply, 1=Fixed, 2=Broadcast)");
  Serial.println("heartbeat=1       - Enable telemetry heartbeat cycle (0=Disable)");
  Serial.println("save              - Write current staging cache to EEPROM & Reset");
}

void printCurrentRuntimeParameters() {
  Serial.println("\n--- Current Configuration Cache Map ---");
  Serial.print("Boot Policy Mode:  ");
  Serial.println(config.preferStatic == 1 ? "STATIC IP" : "DHCP ONLY");
  Serial.print("Local Static IP:   ");
  printIPArray(config.staticIP);
  Serial.print("Subnet Mask:       ");
  printIPArray(config.subnetMask);
  Serial.print("Gateway / DNS:     ");
  printIPArray(config.gatewayIP);
  Serial.print("Local Input Port:  ");
  Serial.println(config.localPort);
  Serial.print("Outbound Base Port:");
  Serial.println(config.targetPort);
  Serial.print("Telemetry Pattern: ");
  if (config.telemetryMode == 0) Serial.println("0 [Reply to Sender Mode]");
  if (config.telemetryMode == 1) Serial.println("1 [Fixed Remote Client Base Target]");
  if (config.telemetryMode == 2) Serial.println("2 [Subnet Broadcast Optimization]");
  Serial.print("Fixed Target IP:   ");
  printIPArray(config.fixedTargetIP);
  Serial.print("Active Destination:");
  Serial.print(currentTargetIP);
  Serial.print(":");
  Serial.println(currentTargetPort);
  Serial.print("Heartbeat System:  ");
  Serial.println(config.heartbeatEnabled == 1 ? "ENABLED" : "DISABLED");
  Serial.println("=========================================");
}

void rebootDevice() {
  Serial.println("Executing Native Software Reset...");
  Serial.flush();  // Ensure the serial buffer finishes printing before the chip resets

  // Open the configuration lock and set the Software Reset Enable bit (SWRE)
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);

  while (true) {}  // Lock execution here until the hardware forces the system reboot
}