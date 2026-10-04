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


#include <IRremote.h>  // Kept after pin definitions per system design intent

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

  // --- PERSISTENT HARDWARE & PRESET CONFIGURATIONS ---
  uint8_t posDown;        // User editable Soft Down preset angle
  uint8_t posUp;          // User editable Soft Up preset angle
  uint8_t dnBound;        // Hard physical low limit mechanical boundary
  uint8_t upBound;        // Hard physical high limit mechanical boundary
  uint8_t defaultPos;     // Boot up / Startup angle default
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

// Fixed char array buffers replacing heavy String classes
char telnetBuffer[48];
uint8_t telnetBufIdx = 0;
unsigned long lastTelnetActivityTime = 0;
const unsigned long TELNET_TIMEOUT = 120000;  // 2 minutes in milliseconds

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

// Forward declarations
void handleCommonConsoleCommands(char* input, Print& output);
bool parseKeyValue(const char* input, const char* key, int& outValue);
void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort);
void pushStateToTelemetry(const char* notesMsg);
void sendHeartbeat();
void initNetwork();
void loadConfiguration();
void parseSerialIP(char* inputStr, uint8_t* ipArray);
void processTelnetConsole();
void processSerialConsole();
void processPhysicalButton();
void processIRInput();
void updateServo();
void translateIRCommand(long IRData);
void handleIRPreset(int num);
void rebootDevice();

// --- INITIALIZATION ---

void setup() {
  Serial.begin(1000000);

  unsigned long serialTimeout = millis();
  while (!Serial) {
    if (millis() - serialTimeout > 3000) break;
  }

  Serial.println(F("\n=== Initializing Projector Dowser ==="));

  loadConfiguration();

  // Initialize runtime state engines directly from the retrieved configuration maps
  targetPos = config.defaultPos;
  savedTargetPrePause = config.posDown;
  down = config.posUp;
  up = config.posDown;
  dnBound = config.dnBound;
  upBound = config.upBound;

  Ethernet.init(ETHERNET_CS_PIN);
  initNetwork();
  Udp.begin(config.localPort);

  telnetServer.begin();

  myservo.attach(SERVO_PIN);
  myservo.write(down);
  targetPos = down;

  pinMode(BUTTON_PIN, INPUT);


  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);

  Serial.println(F("System Ready."));
  pushStateToTelemetry("System Initialized");
}

// --- CORE EXECUTION LOOP ---

void loop() {
  processIncomingOSC();
  processTelnetConsole();
  processSerialConsole();
  processPhysicalButton();
  processIRInput();
  updateServo();

  if (config.heartbeatEnabled == 1) {
    if (millis() - lastHeartbeatTime >= config.heartbeatInterval) {
      lastHeartbeatTime = millis();
      sendHeartbeat();
    }
  }
}

// --- HARDWARE PROCESSING ENGINE ---

void updateServo() {
  if (targetPos > upBound) {
    Serial.println(F("[Warning] Software Ceiling bound!"));
    targetPos = upBound - 1;
  } else if (targetPos < dnBound) {
    Serial.println(F("[Warning] Software Floor bound!"));
    targetPos = dnBound + 1;
  }

  myservo.write(targetPos);

  if (targetPos != lastReportedTarget) {
    if (targetPos == up) {
      currentPresetName = "Open (Up)";
    } else if (targetPos == down) {
      currentPresetName = "Closed (Down)";
    } else {
      static char dynamicLabel[24];
      snprintf_P(dynamicLabel, sizeof(dynamicLabel), PSTR("Angle: %d"), targetPos);
      currentPresetName = dynamicLabel;
    }

    Serial.print(F("Servo Moving: "));
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
        Serial.print(F("Chassis Button Triggered #"));
        Serial.println(buttonPushCounter++);

        targetPos = (targetPos == down) ? up : down;
        ledState = LOW;
      } else {
        ledState = HIGH;
      }
    }
  }
  lastButtonState = reading;
}

void processIRInput() {
  if (IrReceiver.decode()) {
    long IRData = IrReceiver.decodedIRData.decodedRawData;
    if (IRData != 0) {
      translateIRCommand(IRData);
    }
    IrReceiver.resume();
  }
}

// --- OSC NETWORK ROUTER ENGINE (OPERATIONAL TRACKS ONLY) ---

void processIncomingOSC() {
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    evaluateTelemetryRouting(Udp.remoteIP(), Udp.remotePort());

    while (packetSize--) {
      msgIn.fill(Udp.read());
    }

    if (!msgIn.hasError()) {
      msgIn.route("/dowser/up", [](OSCMessage& msg, int offset) {
        targetPos = up;
      });
      msgIn.route("/dowser/down", [](OSCMessage& msg, int offset) {
        targetPos = down;
      });
      msgIn.route("/dowser/angle", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && (msg.isInt(0) || msg.isFloat(0))) {
          int angle = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);
          if (angle >= dnBound && angle <= upBound) {
            targetPos = angle;
          }
        }
      });
      msgIn.route("/dowser/reset", [](OSCMessage& msg, int offset) {
        targetPos = config.defaultPos;
        Serial.println(F("Remote Reset Issued."));
      });
    } else {
      Serial.println(F("OSC Error"));
    }
  }
}

// --- ULTRA-OPTIMIZED ZERO-STRING DISPATCH SYSTEM ---

void dispatchMessage(const char* source, const __FlashStringHelper* prefix, int32_t val1 = -999, int32_t val2 = -999) {
  Print* outputs[] = { &Serial, (telnetClient && telnetClient.connected()) ? &telnetClient : NULL };
  for (uint8_t i = 0; i < 2; i++) {
    if (outputs[i] == NULL) continue;
    if (i == 1) outputs[i]->print(F("\r\33[2K"));
    outputs[i]->print('[');
    outputs[i]->print(source);
    outputs[i]->print(F("] "));
    outputs[i]->print(prefix);
    if (val1 != -999) outputs[i]->print(val1);
    if (val2 != -999) {
      outputs[i]->print(F(" -> "));
      outputs[i]->print(val2);
    }
    outputs[i]->println();
    if (i == 1) outputs[i]->print(F("dowser> "));
  }

  char buf[32];
  if (val2 != -999) {
    snprintf_P(buf, sizeof(buf), PSTR("%S%ld->%ld"), prefix, (long)val1, (long)val2);
  } else if (val1 != -999) {
    snprintf_P(buf, sizeof(buf), PSTR("%S%ld"), prefix, (long)val1);
  } else {
    snprintf_P(buf, sizeof(buf), PSTR("%S"), prefix);
  }

  OSCMessage msg("/dowser/log");
  msg.add(source);
  msg.add(buf);
  Udp.beginPacket(config.fixedTargetIP, config.targetPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void processSerialConsole() {
  static char serialBuffer[48];
  static uint8_t serialBufIdx = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufIdx > 0) {
        serialBuffer[serialBufIdx] = '\0';
        handleCommonConsoleCommands(serialBuffer, Serial);
        serialBufIdx = 0;
      }
    } else if (c >= 32 && c <= 126 && serialBufIdx < 47) {
      serialBuffer[serialBufIdx++] = c;
    }
  }
}

// --- TELEMETRY ENGINE & BACKPLANE NETWORK DRIVER ---

void initNetwork() {
  if (config.preferStatic == 1) {
    Serial.println(F("Assigning Static Profile parameters..."));
    IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
    IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
    IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
    Ethernet.begin(mac, ip, gw, gw, sn);
  } else {
    Serial.println(F("Requesting DHCP Lease Line Configuration..."));
    if (Ethernet.begin(mac, 8000, 2000) == 0) {
      Serial.println(F("DHCP Absent. Falling back to Safe Static Profile."));
      IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
      IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
      IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
      Ethernet.begin(mac, ip, gw, gw, sn);
    }
  }
  Serial.print(F("Controller IP: "));
  Serial.println(Ethernet.localIP());
}

void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort) {
  switch (config.telemetryMode) {
    case 0:
      if (senderIP != IPAddress(0, 0, 0, 0)) {
        currentTargetIP = senderIP;
        currentTargetPort = config.targetPort;
      } else {
        currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
        currentTargetPort = config.targetPort;
      }
      break;
    case 1:
      currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
      currentTargetPort = config.targetPort;
      break;
    case 2:
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
    case 0xBA45FF00: Serial.println(F("IR: POWER")); break;
    case 0xB847FF00:
      editPos = !editPos;
      Serial.print(F("IR: Edit Mode = "));
      Serial.println(editPos ? F("ON") : F("OFF"));
      break;
    case 0xB946FF00: targetPos = myservo.read() + 1; break;
    case 0xBB44FF00: targetPos = myservo.read() - 5; break;
    case 0xBF40FF00:
      if (pauseMV) {
        targetPos = savedTargetPrePause;
        pauseMV = false;
        Serial.println(F("IR: Drive Resumed"));
      } else {
        savedTargetPrePause = targetPos;
        targetPos = myservo.read();
        pauseMV = true;
        Serial.print(F("IR: Drive Paused at "));
        Serial.println(targetPos);
      }
      break;
    case 0xBC43FF00: targetPos = myservo.read() + 5; break;
    case 0xF807FF00: targetPos = down; break;
    case 0xEA15FF00: targetPos = myservo.read() - 1; break;
    case 0xF609FF00: targetPos = up; break;
    case 0xF20DFF00:
      myservo.detach();
      Serial.println(F("IR: Servo Detached."));
      break;

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
      config.irPresets[num] = (uint8_t)myservo.read();
      dispatchMessage("IR_REMOTE", F("Set Preset "), num, config.irPresets[num]);
    }
  } else {
    targetPos = config.irPresets[num];
    dispatchMessage("IR_REMOTE", F("Move -> "), targetPos);
  }
}

// --- UTILITIES & STORAGE DATA PROTOCOLS ---

void loadConfiguration() {
  EEPROM.get(0, config);
  if (config.magicNumber != 0x54) {
    Serial.println(F("Memory Profile Blank. Injecting defaults..."));
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
    config.telemetryMode = 0;
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
    config.defaultPos = 150;
    uint8_t defaultIR[10] = { 10, 10, 10, 10, 10, 125, 170, 170, 170, 170 };
    memcpy(config.irPresets, defaultIR, 10);
    EEPROM.put(0, config);
  } else {
    Serial.println(F("EEPROM Integrity Verified."));
  }
}

void parseSerialIP(char* inputStr, uint8_t* ipArray) {
  char* token = strtok(inputStr, ".");
  uint8_t i = 0;
  while (token != NULL && i < 4) {
    ipArray[i++] = atoi(token);
    token = strtok(NULL, ".");
  }
}

void processTelnetConsole() {
  if (!telnetClient.connected()) {
    EthernetClient incomingClient = telnetServer.accept();
    if (incomingClient) {
      telnetClient = incomingClient;
      telnetClient.println(F("\r\n=== Projector Dowser Telnet Console ==="));
      telnetClient.print(F("dowser> "));
      telnetBufIdx = 0;
      lastTelnetActivityTime = millis();
    }
  } else {
    if (millis() - lastTelnetActivityTime > TELNET_TIMEOUT) {
      telnetClient.println(F("\r\nSession timeout close."));
      telnetClient.stop();
      Serial.println(F("Telnet dropped via timeout."));
      return;
    }

    while (telnetClient.available() > 0) {
      char c = telnetClient.read();
      lastTelnetActivityTime = millis();

      if (c == '\r' || c == '\n') {
        if (telnetBufIdx > 0) {
          telnetBuffer[telnetBufIdx] = '\0';
          telnetClient.println();
          handleCommonConsoleCommands(telnetBuffer, telnetClient);
          telnetBufIdx = 0;
          telnetClient.print(F("dowser> "));
        }
      } else if (c == 8 || c == 127) {
        if (telnetBufIdx > 0) {
          telnetBufIdx--;
          telnetClient.print(F("\b \b"));
        }
      } else if (c >= 32 && c <= 126 && telnetBufIdx < 47) {
        telnetBuffer[telnetBufIdx++] = c;
        telnetClient.print(c);
      }
    }
  }
}

// --- OPTIMIZED EXTRACTOR UTILITY ---

bool parseKeyValue(const char* input, const char* key, int& outValue) {
  size_t keyLen = strlen(key);
  if (strncasecmp(input, key, keyLen) == 0 && input[keyLen] == '=') {
    outValue = atoi(input + keyLen + 1);
    return true;
  }
  return false;
}

// --- REFACTORED COMMAND ROUTING TERMINAL ENGINE ---

void handleCommonConsoleCommands(char* input, Print& output) {
  const char* source = (&output == &Serial) ? "SERIAL" : "TELNET";
  int val;

  if (strcasecmp(input, "help") == 0 || strcmp(input, "?") == 0) {
    output.println(F("Commands: show, save, [0-180]\nKeys: down, up, dnbound, upbound, default, localport, targetport, static, telemetry, heartbeat, ip, targetip, preset[0-9]"));
    return;
  }

  if (strcasecmp(input, "show") == 0) {
    output.print(F("Boot Profile : "));
    output.println(config.preferStatic == 1 ? F("STATIC") : F("DHCP"));
    output.print(F("Local IP     : "));
    output.print(config.staticIP[0]);
    output.print('.');
    output.print(config.staticIP[1]);
    output.print('.');
    output.print(config.staticIP[2]);
    output.print('.');
    output.println(config.staticIP[3]);
    output.print(F("Subnet Mask  : "));
    output.print(config.subnetMask[0]);
    output.print('.');
    output.print(config.subnetMask[1]);
    output.print('.');
    output.print(config.subnetMask[2]);
    output.print('.');
    output.println(config.subnetMask[3]);
    output.print(F("Gateway IP   : "));
    output.print(config.gatewayIP[0]);
    output.print('.');
    output.print(config.gatewayIP[1]);
    output.print('.');
    output.print(config.gatewayIP[2]);
    output.print('.');
    output.println(config.gatewayIP[3]);
    output.print(F("Local Port   : "));
    output.println(config.localPort);
    output.print(F("Target Port  : "));
    output.println(config.targetPort);
    output.print(F("Telemetry Pat: "));
    output.println(config.telemetryMode);
    output.print(F("Target Host  : "));
    output.print(config.fixedTargetIP[0]);
    output.print('.');
    output.print(config.fixedTargetIP[1]);
    output.print('.');
    output.print(config.fixedTargetIP[2]);
    output.print('.');
    output.println(config.fixedTargetIP[3]);
    output.print(F("Heartbeat    : "));
    output.println(config.heartbeatEnabled == 1 ? F("ACTIVE") : F("MUTED"));
    output.print(F("Hard Bounds  : ["));
    output.print(config.dnBound);
    output.print(F(" - "));
    output.print(config.upBound);
    output.println(F("]"));
    output.print(F("Presets (D/U): ["));
    output.print(config.posDown);
    output.print(F(" / "));
    output.print(config.posUp);
    output.println(F("]"));
    output.print(F("Boot Default : "));
    output.print(config.defaultPos);
    output.println(F(" deg"));
    output.print(F("IR Matrix    : ["));
    for (uint8_t j = 0; j < 10; j++) {
      output.print(config.irPresets[j]);
      if (j < 9) output.print(F(", "));
    }
    output.println(F("]"));
    output.print(F("Active Target: "));
    output.print(targetPos);
    output.println(F(" deg"));
    return;
  }

  if (strcasecmp(input, "save") == 0) {
    EEPROM.put(0, config);
    dispatchMessage(source, F("EEPROM Saved"));
    delay(500);
    rebootDevice();
    return;
  }

  // Value assignment parsing tree via pointer evaluation mappings
  if (parseKeyValue(input, "down", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.posDown = val;
      dispatchMessage(source, F("Set Down -> "), val);
    } else {
      output.println(F("Error: Bound violation."));
    }
  } else if (parseKeyValue(input, "up", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.posUp = val;
      dispatchMessage(source, F("Set Up -> "), val);
    } else {
      output.println(F("Error: Bound violation."));
    }
  } else if (parseKeyValue(input, "dnbound", val)) {
    if (val >= 0 && val <= 180) {
      config.dnBound = val;
      dispatchMessage(source, F("Set Lo-Bound -> "), val);
    } else {
      output.println(F("Error: Range 0-180."));
    }
  } else if (parseKeyValue(input, "upbound", val)) {
    if (val >= 0 && val <= 180) {
      config.upBound = val;
      dispatchMessage(source, F("Set Hi-Bound -> "), val);
    } else {
      output.println(F("Error: Range 0-180."));
    }
  } else if (parseKeyValue(input, "default", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.defaultPos = val;
      dispatchMessage(source, F("Set Boot Default -> "), val);
    } else {
      output.println(F("Error: Out of range."));
    }
  } else if (parseKeyValue(input, "localport", val)) {
    config.localPort = val;
    dispatchMessage(source, F("Set Local Port -> "), val);
  } else if (parseKeyValue(input, "targetport", val)) {
    config.targetPort = val;
    dispatchMessage(source, F("Set Target Port -> "), val);
  } else if (parseKeyValue(input, "static", val)) {
    config.preferStatic = (val == 1) ? 1 : 0;
    dispatchMessage(source, F("Set PreferStatic -> "), config.preferStatic);
  } else if (parseKeyValue(input, "telemetry", val)) {
    config.telemetryMode = val;
    evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
    dispatchMessage(source, F("Set Telemetry Mode -> "), val);
  } else if (parseKeyValue(input, "heartbeat", val)) {
    config.heartbeatEnabled = (val == 1) ? 1 : 0;
    dispatchMessage(source, F("Set Heartbeat -> "), config.heartbeatEnabled);
  } else if (strncasecmp(input, "preset", 6) == 0) {
    char* equalsIdx = strchr(input, '=');
    if (equalsIdx != NULL) {
      *equalsIdx = '\0';
      int idx = atoi(input + 6);
      int targetAngle = atoi(equalsIdx + 1);

      if (idx >= 0 && idx < 10) {
        if (targetAngle >= config.dnBound && targetAngle <= config.upBound) {
          config.irPresets[idx] = targetAngle;
          dispatchMessage(source, F("Set Preset "), idx, targetAngle);
        } else {
          output.println(F("Error: Bound violation."));
        }
      } else {
        output.println(F("Error: Slot index 0-9."));
      }
    }
  } else if (strncmp(input, "ip=", 3) == 0) {
    parseSerialIP(input + 3, config.staticIP);
    dispatchMessage(source, F("Staged Static IP"));
  } else if (strncmp(input, "targetip=", 9) == 0) {
    parseSerialIP(input + 9, config.fixedTargetIP);
    dispatchMessage(source, F("Staged Target IP"));
  } else if (isdigit(input[0])) {
    int parsedVal = atoi(input);
    if (parsedVal >= config.dnBound && parsedVal <= config.upBound) {
      targetPos = parsedVal;
      dispatchMessage(source, F("Move -> "), targetPos);
    } else {
      output.println(F("Error: Constraint breach."));
    }
  } else {
    output.println(F("Unknown command."));
  }
}

void rebootDevice() {
  Serial.println(F("Software Reset..."));
  Serial.flush();
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
  while (true) {}
}