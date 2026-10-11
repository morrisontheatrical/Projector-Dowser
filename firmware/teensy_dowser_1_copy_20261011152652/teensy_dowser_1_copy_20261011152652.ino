#include <SPI.h>
#if defined(ARDUINO_TEENSY41)
// Teensy 4.1 has a native Ethernet MAC (not a W5x00); use QNEthernet (Library Manager).
#include <QNEthernet.h>
using namespace qindesign::network;
#define USE_QNETHERNET 1
#else
#include <Ethernet.h>
#include <EthernetUdp.h>
#endif
#include <OSCMessage.h>
#include <EEPROM.h>
#include <Servo.h>
#include <ctype.h>

// --- PHYSICAL PIN CONFIGURATION ---
#define BUTTON_PIN 2
#define SERVO_PIN 3
#define ETHERNET_CS_PIN 10
#define IR_RECEIVE_PIN 5
#define LED_PIN 4

#include <IRremote.h>

Servo myservo;
EthernetUDP Udp;
EthernetServer webServer(80);

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

struct DeviceConfiguration {
  uint8_t magicNumber;
  uint8_t preferStatic;
  uint8_t staticIP[4];
  uint8_t subnetMask[4];
  uint8_t gatewayIP[4];
  uint16_t localPort;        // Arduino Incoming Listening Port
  uint16_t targetPorts[3];   // Companion Feedback Listening Port
  uint8_t telemetryMode;     // 0 = Reply to Sender, 1 = Fixed Target, 2 = Broadcast
  uint8_t fixedTargetIP[4];  // Companion IP
  uint8_t heartbeatEnabled;
  uint16_t heartbeatInterval;
  uint8_t posDown;
  uint8_t posUp;
  uint8_t dnBound;
  uint8_t upBound;
  uint8_t defaultPos;
  uint8_t irPresets[10];
  // Keep new fields appended here; EEPROM_MAGIC_V1 migration assumes the legacy prefix is unchanged.
  uint16_t defaultFadeTimeMs;
};

int targetPos;
int currentPos;
int savedTargetPrePause;
uint16_t activeMoveFadeTimeMs;
uint8_t activeMoveDistance;
unsigned long lastMoveUpdateMicros = 0;
unsigned long movementCredit = 0;
bool configLoadedSuccessfully = false;
DeviceConfiguration persistedConfig;
unsigned long configDirtySince = 0;
const unsigned long CONFIG_AUTOSAVE_DELAY_MS = 3000;

const uint8_t EEPROM_MAGIC_V1 = 0x53;
const uint8_t EEPROM_MAGIC_V2 = 0x55;
const uint8_t EEPROM_MAGIC_V3 = 0x56;
const uint16_t DEFAULT_FADE_TIME_MS = 1000;
const uint16_t MIN_FADE_TIME_MS = 0;
const uint16_t MAX_FADE_TIME_MS = 60000;
const unsigned long MICROSECONDS_PER_MILLISECOND = 1000UL;
// Covers the common 20 ms loop interval while limiting catch-up after blocking operations.
const unsigned long MAX_MOVEMENT_ELAPSED_US = 20000UL;
const unsigned long MAX_MOVEMENT_DISTANCE_DEGREES = 180UL;
static_assert((unsigned long long)MAX_FADE_TIME_MS * MICROSECONDS_PER_MILLISECOND + (unsigned long long)MAX_MOVEMENT_ELAPSED_US * MAX_MOVEMENT_DISTANCE_DEGREES <= 0xFFFFFFFFULL,
              "Motion credit update must fit in unsigned long");

int down;
int up;
int dnBound;
int upBound;

DeviceConfiguration config;

IPAddress currentTargetIP;
unsigned int currentTargetPort;
unsigned long lastHeartbeatTime = 0;

EthernetServer telnetServer(23);
EthernetClient telnetClient;

char telnetBuffer[48];
uint8_t telnetBufIdx = 0;
unsigned long lastTelnetActivityTime = 0;
const unsigned long TELNET_TIMEOUT = 120000;

unsigned long lastTelemetryTime = 0;
int lastKnownPos = -999;

int lastReportedPos = -1;
const char* currentPresetName = "System Boot";

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
bool isValidFadeTime(int fadeTimeMs);
bool parseFadeTimeSeconds(const char* value, uint16_t& fadeTimeMs);
bool convertFadeTimeSeconds(float seconds, uint16_t& fadeTimeMs);
void broadcastFadeTimeConfig();
void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort);
void broadcastCompanionState();
void pushStateToTelemetry(const char* notesMsg);
void initNetwork();
void loadConfiguration();
void autosaveConfiguration();
void parseSerialIP(char* inputStr, uint8_t* ipArray);
void processTelnetConsole();
void beginWebServer();
void processWebServer();
void processSerialConsole();
void processPhysicalButton();
void processIRInput();
void updateServo();
void setMovementTarget(int target, uint16_t fadeTimeMs);
void setMovementTargetAtDefaultFadeTime(int target);
bool getMovementFadeTime(OSCMessage& msg, int argumentIndex, uint16_t& fadeTimeMs);
void setMovementTargetFromOSC(OSCMessage& msg, int target, int fadeTimeArgumentIndex);
void reportInvalidMovementAngle();
void reportInvalidMovementFadeTime();
void sendOSCError(const char* path, const char* message);
void translateIRCommand(unsigned long IRData);
void handleIRPreset(int num);
void rebootDevice();

bool networkAvailable = false;

void setup() {
  Serial.begin(1000000);
  unsigned long serialTimeout = millis();
  while (!Serial) {
    if (millis() - serialTimeout > 3000) break;
  }

  Serial.println(F("\n=== Initializing Projector Dowser ==="));
  loadConfiguration();

  targetPos = config.defaultPos;
  savedTargetPrePause = config.posDown;
  down = config.posUp;
  up = config.posDown;
  dnBound = config.dnBound;
  upBound = config.upBound;

#if defined(USE_QNETHERNET)
  Ethernet.setMACAddress(mac);
  networkAvailable = true;
#else
  Ethernet.init(ETHERNET_CS_PIN);
  Ethernet.begin(mac, IPAddress(0, 0, 0, 0));
  networkAvailable = (Ethernet.hardwareStatus() != EthernetNoHardware);
#endif
  if (networkAvailable) {
    initNetwork();
    Udp.begin(config.localPort);
    telnetServer.begin();
    beginWebServer();
  } else {
    Serial.println(F("Ethernet hardware not found - network disabled"));
  }

  Serial.println(F("Attaching servo..."));
  myservo.attach(SERVO_PIN);
  currentPos = constrain((int)config.defaultPos, (int)config.dnBound, (int)config.upBound);
  targetPos = currentPos;
  activeMoveFadeTimeMs = config.defaultFadeTimeMs;
  myservo.write(currentPos);
  lastMoveUpdateMicros = micros();

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  Serial.println(F("Starting IR..."));
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
  Serial.println(F("System Ready."));
}

void loop() {
  unsigned long currentMillis = millis();

  // TELEMETRY ENGINE: Gated to a stable 20Hz refresh rate
  if (currentMillis - lastTelemetryTime >= 50) {

    // Condition A: The mechanism is in transit (live update)
    if (currentPos != lastKnownPos) {
      broadcastCompanionState();
      lastKnownPos = currentPos;
      lastTelemetryTime = currentMillis;  // Reset frame clock
    }
    // Condition B: Mechanism is static, send a 1-second background heartbeat
    else if (currentMillis - lastTelemetryTime >= 1000) {
      broadcastCompanionState();
      lastTelemetryTime = currentMillis;  // Reset frame clock
    }
  }

  processIncomingOSC();
  processTelnetConsole();
  processWebServer();
  autosaveConfiguration();
  processSerialConsole();
  processPhysicalButton();
  processIRInput();
  updateServo();

  if (config.heartbeatEnabled == 1) {
    if (millis() - lastHeartbeatTime >= config.heartbeatInterval) {
      lastHeartbeatTime = millis();
      broadcastCompanionState();
    }
  }
}

void updateServo() {
  int previousTargetPos = targetPos;
  if (targetPos > upBound) {
    targetPos = upBound;
  } else if (targetPos < dnBound) {
    targetPos = dnBound;
  }

  unsigned long now = micros();
  if (targetPos != previousTargetPos) {
    activeMoveDistance = (uint8_t)abs(targetPos - currentPos);
    movementCredit = 0;
    lastMoveUpdateMicros = now;
  }
  unsigned long elapsed = now - lastMoveUpdateMicros;
  lastMoveUpdateMicros = now;
  // Discard excess elapsed time after blocking operations rather than catching up in a large jump.
  if (elapsed > MAX_MOVEMENT_ELAPSED_US) elapsed = MAX_MOVEMENT_ELAPSED_US;

  if (activeMoveFadeTimeMs == 0) {
    if (currentPos != targetPos) {
      currentPos = targetPos;
      myservo.write(currentPos);
    }
    movementCredit = 0;
    activeMoveDistance = 0;
  } else if (currentPos != targetPos) {
    unsigned long fadeDurationUs = (unsigned long)activeMoveFadeTimeMs * MICROSECONDS_PER_MILLISECOND;
    movementCredit += elapsed * activeMoveDistance;
    unsigned long steps = movementCredit / fadeDurationUs;
    movementCredit %= fadeDurationUs;
    int distance = abs(targetPos - currentPos);
    if (steps > (unsigned long)distance) {
      steps = distance;
      movementCredit = 0;
    }
    if (steps > 0) {
      currentPos += (targetPos > currentPos) ? (int)steps : -(int)steps;
      myservo.write(currentPos);
    }
  } else {
    movementCredit = 0;
  }

  if (currentPos != lastReportedPos) {
    if (currentPos == up) {
      currentPresetName = "Open (Up)";
    } else if (currentPos == down) {
      currentPresetName = "Closed (Down)";
    } else {
      static char dynamicLabel[24];
      strcpy_P(dynamicLabel, PSTR("Angle: "));
      itoa(currentPos, dynamicLabel + 7, 10);
      currentPresetName = dynamicLabel;
    }

    // Instantly push updates to Companion on mechanical change
    broadcastCompanionState();
    lastReportedPos = currentPos;
  }
}

void setMovementTarget(int target, uint16_t fadeTimeMs) {
  if (target > upBound) {
    targetPos = upBound;
  } else if (target < dnBound) {
    targetPos = dnBound;
  } else {
    targetPos = target;
  }
  activeMoveFadeTimeMs = fadeTimeMs;
  activeMoveDistance = (uint8_t)abs(targetPos - currentPos);
  movementCredit = 0;
  lastMoveUpdateMicros = micros();
}

void setMovementTargetAtDefaultFadeTime(int target) {
  setMovementTarget(target, config.defaultFadeTimeMs);
}

bool getMovementFadeTime(OSCMessage& msg, int argumentIndex, uint16_t& fadeTimeMs) {
  if (msg.size() == argumentIndex) {
    fadeTimeMs = config.defaultFadeTimeMs;
    return true;
  }
  if (msg.size() != argumentIndex + 1) return false;
  if (msg.isInt(argumentIndex)) {
    return convertFadeTimeSeconds(msg.getInt(argumentIndex), fadeTimeMs);
  }
  if (msg.isFloat(argumentIndex)) {
    return convertFadeTimeSeconds(msg.getFloat(argumentIndex), fadeTimeMs);
  }
  return false;
}

void setMovementTargetFromOSC(OSCMessage& msg, int target, int fadeTimeArgumentIndex) {
  uint16_t fadeTimeMs;
  if (getMovementFadeTime(msg, fadeTimeArgumentIndex, fadeTimeMs)) {
    setMovementTarget(target, fadeTimeMs);
  } else {
    reportInvalidMovementFadeTime();
  }
}

void sendOSCError(const char* path, const char* message) {
  Serial.println(message);
  if (!networkAvailable) return;
  IPAddress senderIP = Udp.remoteIP();
  unsigned int senderPort = Udp.remotePort();
  OSCMessage error(path);
  error.add(message);
  Udp.beginPacket(senderIP, senderPort);
  error.send(Udp);
  Udp.endPacket();
}

void reportInvalidMovementFadeTime() {
  sendOSCError("/dowser/error/fade", "Fade time must be a number from 0-60 seconds");
}

void reportInvalidMovementAngle() {
  sendOSCError("/dowser/error/angle", "Angle must be a number within configured bounds");
}

/*/ --- NEW COMPANION OSC FEEDBACK ENGINE ---
void broadcastCompanionState() {
  // 1. Prepare Companion native tracking payloads
  OSCMessage msgAngle("/dowser/state/angle");   msgAngle.add(currentPos);
  OSCMessage msgUp("/dowser/state/isUp");       msgUp.add(currentPos == up ? 1 : 0);
  OSCMessage msgDown("/dowser/state/isDown");   msgDown.add(currentPos == down ? 1 : 0);

  // 2. Prepare QLab note payload (Converts integer position to string)
  char noteBuffer[16];
  itoa(currentPos, noteBuffer, 10); // Converts integer to string character array cleanly
  OSCMessage msgQLab("/cue/DOWSER_STATUS/notes");
  msgQLab.add(noteBuffer);
  OSCMessage msgQLab1("/cue/DOWSER_STATUS/name");
    msgQLab1.add(noteBuffer);


  // 3. Iterate across all configuration port destinations
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t activePort = config.targetPorts[i];
    if (activePort == 0) continue; // Skip unassigned slots

    // Blast Companion Data
    Udp.beginPacket(currentTargetIP, activePort);  msgAngle.send(Udp); Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);  msgUp.send(Udp);    Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);  msgDown.send(Udp);  Udp.endPacket();

    // Blast QLab Workspace Data
    Udp.beginPacket(currentTargetIP, activePort);  msgQLab.send(Udp);  Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);  msgQLab1.send(Udp);  Udp.endPacket();
  }

  // Clear memory registers cleanly
  msgAngle.empty(); msgUp.empty(); msgDown.empty(); msgQLab.empty();
}*/
void broadcastCompanionState() {
  if (!networkAvailable) return;
  // 1. Prepare QLab workspace payloads
  char noteBuffer[16];
  itoa(currentPos, noteBuffer, 10);
  OSCMessage msgQLab("/cue/DOWSER_STATUS/notes");
  msgQLab.add(noteBuffer);
  OSCMessage msgQLab1("/cue/DOWSER_STATUS/name");
  msgQLab1.add(noteBuffer);

  // 2. Prepare distinct Companion telemetry payloads
  OSCMessage msgUp("/dowser/state/isUp");
  msgUp.add(currentPos == config.posUp ? 1 : 0);
  OSCMessage msgDown("/dowser/state/isDown");
  msgDown.add(currentPos == config.posDown ? 1 : 0);
  OSCMessage msgAngle("/dowser/state/angle");
  msgAngle.add(currentPos);

  // 3. Iterate across active port allocations
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t activePort = config.targetPorts[i];
    if (activePort == 0) continue;

    // Deliver to targets clean and fast
    Udp.beginPacket(currentTargetIP, activePort);
    msgQLab.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);
    msgQLab1.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);
    msgUp.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);
    msgDown.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(currentTargetIP, activePort);
    msgAngle.send(Udp);
    Udp.endPacket();
  }

  // Clear memory registers cleanly
  msgQLab.empty();
  msgQLab1.empty();
  msgUp.empty();
  msgDown.empty();
  msgAngle.empty();
}
// Helper to broadcast a single integer configuration item
void broadcastConfigItem(const char* subPath, int value) {
  if (!networkAvailable) return;
  char fullPath[50];
  strcpy_P(fullPath, PSTR("/dowser/config/"));
  strcat(fullPath, subPath);

  OSCMessage msg(fullPath);
  msg.add(value);

  for (uint8_t i = 0; i < 3; i++) {
    uint16_t activePort = config.targetPorts[i];
    if (activePort == 0) continue;
    Udp.beginPacket(currentTargetIP, activePort);
    msg.send(Udp);
    Udp.endPacket();
  }
  msg.empty();
}

// Bulk dump function to sync all Companion configuration fields at once
void broadcastAllConfigSettings() {
  if (!networkAvailable) return;
  broadcastConfigItem("down", config.posDown);
  delay(20);
  broadcastConfigItem("up", config.posUp);
  delay(20);
  broadcastConfigItem("dnbound", config.dnBound);
  delay(20);
  broadcastConfigItem("upbound", config.upBound);
  delay(20);
  broadcastConfigItem("default", config.defaultPos);
  delay(20);
  broadcastConfigItem("localport", config.localPort);
  delay(20);
  broadcastConfigItem("targetport0", config.targetPorts[0]);
  delay(20);
  broadcastConfigItem("targetport1", config.targetPorts[1]);
  delay(20);
  broadcastConfigItem("targetport2", config.targetPorts[2]);
  delay(20);
  broadcastConfigItem("static", config.preferStatic);
  delay(20);
  broadcastConfigItem("telemetry", config.telemetryMode);
  delay(20);
  broadcastConfigItem("heartbeat", config.heartbeatEnabled);
  delay(20);
  broadcastFadeTimeConfig();
}

void broadcastFadeTimeConfig() {
  if (!networkAvailable) return;
  OSCMessage msg("/dowser/config/fade");
  msg.add((float)config.defaultFadeTimeMs / 1000.0f);

  for (uint8_t i = 0; i < 3; i++) {
    uint16_t activePort = config.targetPorts[i];
    if (activePort == 0) continue;
    Udp.beginPacket(currentTargetIP, activePort);
    msg.send(Udp);
    Udp.endPacket();
  }
  msg.empty();
}

void processIncomingOSC() {
  if (!networkAvailable) return;
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    evaluateTelemetryRouting(Udp.remoteIP(), Udp.remotePort());

    while (packetSize--) { msgIn.fill(Udp.read()); }

    if (!msgIn.hasError()) {
      // --- CORE OPERATIONAL ROUTING ---
      msgIn.route("/dowser/up", [](OSCMessage& msg, int offset) {
        setMovementTargetFromOSC(msg, config.posUp, 0);
      });
      msgIn.route("/dowser/down", [](OSCMessage& msg, int offset) {
        setMovementTargetFromOSC(msg, config.posDown, 0);
      });
      msgIn.route("/dowser/angle", [](OSCMessage& msg, int offset) {
        if (msg.size() == 0 || (!msg.isInt(0) && !msg.isFloat(0))) {
          reportInvalidMovementAngle();
          return;
        }
        float requestedAngle = msg.isInt(0) ? msg.getInt(0) : msg.getFloat(0);
        if (!(requestedAngle >= config.dnBound && requestedAngle <= config.upBound)) {
          reportInvalidMovementAngle();
          return;
        }
        int angle = (int)requestedAngle;
        setMovementTargetFromOSC(msg, angle, 1);
      });
      msgIn.route("/dowser/query", [](OSCMessage& msg, int offset) {
        broadcastCompanionState();
      });
      msgIn.route("/dowser/reset", [](OSCMessage& msg, int offset) {
        setMovementTargetFromOSC(msg, config.posDown, 0);
      });

      // --- CONFIGURATION MANAGEMENT LAYER (GET / SET) ---

      // Bulk Query Trigger
      msgIn.route("/dowser/config/queryAll", [](OSCMessage& msg, int offset) {
        broadcastAllConfigSettings();
      });

      // Commit changes permanently to EEPROM
      msgIn.route("/dowser/config/save", [](OSCMessage& msg, int offset) {
        EEPROM.put(0, config);
        delay(200);
        rebootDevice();
      });

      // Target Calibration Positions
      msgIn.route("/dowser/config/down", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          int val = msg.getInt(0);
          if (val >= config.dnBound && val <= config.upBound) { config.posDown = val; }
        }
        broadcastConfigItem("down", config.posDown);
      });

      msgIn.route("/dowser/config/up", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          int val = msg.getInt(0);
          if (val >= config.dnBound && val <= config.upBound) { config.posUp = val; }
        }
        broadcastConfigItem("up", config.posUp);
      });

      // Hard Boundary Limits
      msgIn.route("/dowser/config/dnbound", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          int val = msg.getInt(0);
          if (val >= 0 && val <= 180) { config.dnBound = val; }
        }
        broadcastConfigItem("dnbound", config.dnBound);
      });

      msgIn.route("/dowser/config/upbound", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          int val = msg.getInt(0);
          if (val >= 0 && val <= 180) { config.upBound = val; }
        }
        broadcastConfigItem("upbound", config.upBound);
      });

      // Operational Profiles
      msgIn.route("/dowser/config/default", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          int val = msg.getInt(0);
          if (val >= config.dnBound && val <= config.upBound) { config.defaultPos = val; }
        }
        broadcastConfigItem("default", config.defaultPos);
      });

      msgIn.route("/dowser/config/static", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          config.preferStatic = (msg.getInt(0) == 1) ? 1 : 0;
        }
        broadcastConfigItem("static", config.preferStatic);
      });

      msgIn.route("/dowser/config/telemetry", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          config.telemetryMode = msg.getInt(0);
          evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
        }
        broadcastConfigItem("telemetry", config.telemetryMode);
      });

      msgIn.route("/dowser/config/heartbeat", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) {
          config.heartbeatEnabled = (msg.getInt(0) == 1) ? 1 : 0;
        }
        broadcastConfigItem("heartbeat", config.heartbeatEnabled);
      });

      msgIn.route("/dowser/config/fade", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0) {
          uint16_t fadeTimeMs;
          if (msg.size() == 1 && getMovementFadeTime(msg, 0, fadeTimeMs)) {
            config.defaultFadeTimeMs = fadeTimeMs;
          } else {
            reportInvalidMovementFadeTime();
          }
        }
        broadcastFadeTimeConfig();
      });

      // Network Infrastructure Ports
      msgIn.route("/dowser/config/localport", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) { config.localPort = msg.getInt(0); }
        broadcastConfigItem("localport", config.localPort);
      });

      // Target Destinations Array [0-2]
      msgIn.route("/dowser/config/targetport0", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) { config.targetPorts[0] = (uint16_t)msg.getInt(0); }
        broadcastConfigItem("targetport0", config.targetPorts[0]);
      });

      msgIn.route("/dowser/config/targetport1", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) { config.targetPorts[1] = (uint16_t)msg.getInt(0); }
        broadcastConfigItem("targetport1", config.targetPorts[1]);
      });

      msgIn.route("/dowser/config/targetport2", [](OSCMessage& msg, int offset) {
        if (msg.size() > 0 && msg.isInt(0)) { config.targetPorts[2] = (uint16_t)msg.getInt(0); }
        broadcastConfigItem("targetport2", config.targetPorts[2]);
      });
    }
  }
}

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

void initNetwork() {
#if defined(USE_QNETHERNET)
  IPAddress sIp(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
  IPAddress sSn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
  IPAddress sGw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
  bool ok;
  if (config.preferStatic == 1) {
    ok = Ethernet.begin(sIp, sSn, sGw);
  } else {
    ok = Ethernet.begin() && Ethernet.waitForLocalIP(8000);
    if (!ok) ok = Ethernet.begin(sIp, sSn, sGw);  // DHCP failed: fall back to static
  }
  if (!ok) Serial.println(F("Ethernet start failed"));
#else
  if (config.preferStatic == 1) {
    IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
    IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
    IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
    Ethernet.begin(mac, ip, gw, gw, sn);
  } else {
    if (Ethernet.begin(mac, 8000, 2000) == 0) {
      IPAddress ip(config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3]);
      IPAddress sn(config.subnetMask[0], config.subnetMask[1], config.subnetMask[2], config.subnetMask[3]);
      IPAddress gw(config.gatewayIP[0], config.gatewayIP[1], config.gatewayIP[2], config.gatewayIP[3]);
      Ethernet.begin(mac, ip, gw, gw, sn);
    }
  }
#endif
  Serial.print(F("Controller IP: "));
  Serial.println(Ethernet.localIP());
}

void evaluateTelemetryRouting(IPAddress senderIP, unsigned int senderPort) {
  switch (config.telemetryMode) {

    // =========================================================================
    // MODE 0: DYNAMIC REGISTRATION (REPLY-TO-SENDER)
    // =========================================================================
    // Automatically intercepts the IP address of whichever device talked to the
    // dowser last and targets it for outbound telemetry.
    // Ideal for troubleshooting or temporary tech tables where your laptop
    // might be bouncing around on different DHCP wireless/wired leases.
    case 0:
      if (senderIP != IPAddress(0, 0, 0, 0)) {
        currentTargetIP = senderIP;
      } else {
        // Fallback option: If the unit just booted and hasn't heard an OSC
        // command yet, default to the hardcoded fixed target configuration profile.
        currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
      }
      break;

    // =========================================================================
    // MODE 1: FIXED TARGET (UNICAST)
    // =========================================================================
    // Locks the outbound telemetry stream to one specific, unchanging host IP
    // address (e.g., your primary console or a dedicated Companion Mac Studio).
    // Safest mode for permanent installations to eliminate rogue network traffic.
    case 1:
      currentTargetIP = IPAddress(config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
      break;

    // =========================================================================
    // MODE 2: SUBNET BROADCAST
    // =========================================================================
    // Grabs the device's current local IP address and forces the final octet
    // to .255. This forces the packet to go to every single machine sharing
    // the local subnet simultaneously.
    // Note: Wrapped in explicit curly braces to prevent C++ scope initialization errors.
    case 2:
      {
        IPAddress localIP = Ethernet.localIP();
        currentTargetIP = IPAddress(localIP[0], localIP[1], localIP[2], 255);
        break;
      }
  }
}

void processPhysicalButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) { lastDebounceTime = millis(); }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == 1) {
        setMovementTargetAtDefaultFadeTime((targetPos == down) ? up : down);
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
    // Check if the signal is valid (not a repeat code suppression or error)
    if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
      unsigned long IRData = IrReceiver.decodedIRData.decodedRawData;
      if (IRData != 0) {
        translateIRCommand(IRData);
      }
    }
    // Required in v4 to clear the buffer and ready the receiver for the next press
    IrReceiver.resume();
  }
}
void translateIRCommand(unsigned long IRData) {
  switch (IRData) {
    case 0xBA45FF00: break;
    case 0xB847FF00: editPos = !editPos; break;
    case 0xB946FF00: setMovementTargetAtDefaultFadeTime(currentPos + 1); break;
    case 0xBB44FF00: setMovementTargetAtDefaultFadeTime(currentPos - 5); break;
    case 0xBF40FF00:
      if (pauseMV) {
        setMovementTargetAtDefaultFadeTime(savedTargetPrePause);
        pauseMV = false;
      } else {
        savedTargetPrePause = targetPos;
        setMovementTargetAtDefaultFadeTime(currentPos);
        pauseMV = true;
      }
      break;
    case 0xBC43FF00: setMovementTargetAtDefaultFadeTime(currentPos + 5); break;
    case 0xF807FF00: setMovementTargetAtDefaultFadeTime(down); break;
    case 0xEA15FF00: setMovementTargetAtDefaultFadeTime(currentPos - 1); break;
    case 0xF609FF00: setMovementTargetAtDefaultFadeTime(up); break;
    case 0xF20DFF00: myservo.detach(); break;
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
      config.irPresets[num] = (uint8_t)currentPos;
    }
  } else {
    setMovementTargetAtDefaultFadeTime(config.irPresets[num]);
  }
}

void loadConfiguration() {
  EEPROM.get(0, config);
  if (config.magicNumber == EEPROM_MAGIC_V1) {
    config.magicNumber = EEPROM_MAGIC_V3;
    config.defaultFadeTimeMs = DEFAULT_FADE_TIME_MS;
    EEPROM.put(0, config);
    return;
  }

  if (config.magicNumber == EEPROM_MAGIC_V2) {
    // The V2 field's low byte contains its former one-byte speed setting.
    uint8_t legacySpeed = (uint8_t)config.defaultFadeTimeMs;
    config.magicNumber = EEPROM_MAGIC_V3;
    if (legacySpeed >= 1 && legacySpeed <= 100) {
      unsigned long migratedFadeTimeMs = 100000UL / legacySpeed;
      if (migratedFadeTimeMs < MIN_FADE_TIME_MS) migratedFadeTimeMs = MIN_FADE_TIME_MS;
      if (migratedFadeTimeMs > MAX_FADE_TIME_MS) migratedFadeTimeMs = MAX_FADE_TIME_MS;
      config.defaultFadeTimeMs = (uint16_t)migratedFadeTimeMs;
    } else {
      config.defaultFadeTimeMs = DEFAULT_FADE_TIME_MS;
    }
    EEPROM.put(0, config);
    return;
  }

  if (config.magicNumber != EEPROM_MAGIC_V3) {
    config.magicNumber = EEPROM_MAGIC_V3;
    config.preferStatic = 0;
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
    config.localPort = 8001;        // Matched to Companion config needs
    config.targetPorts[0] = 12000;  // Slot 0 default: Companion Connection
    config.targetPorts[1] = 53000;  // Slot 1 default: QLab Workspace Note Engine
    config.targetPorts[2] = 0;      // Slot 2 default: Empty/Muted
    config.telemetryMode = 2;       // Force Subnet Broadcast out of the gate
    config.fixedTargetIP[0] = 10;
    config.fixedTargetIP[1] = 0;
    config.fixedTargetIP[2] = 0;
    config.fixedTargetIP[3] = 29;  // Companion Host Destination
    config.heartbeatEnabled = 1;
    config.heartbeatInterval = 4000;
    config.posDown = 10;
    config.posUp = 170;
    config.dnBound = 7;
    config.upBound = 173;
    config.defaultPos = 10;
    config.defaultFadeTimeMs = DEFAULT_FADE_TIME_MS;
    uint8_t defaultIR[10] = { 10, 10, 10, 10, 10, 125, 170, 170, 170, 170 };
    memcpy(config.irPresets, defaultIR, 10);
    EEPROM.put(0, config);
  } else if (!isValidFadeTime(config.defaultFadeTimeMs)) {
    config.defaultFadeTimeMs = DEFAULT_FADE_TIME_MS;
    EEPROM.put(0, config);
  }
  persistedConfig = config;
}

// Persists config changes once they have been stable for CONFIG_AUTOSAVE_DELAY_MS,
// so rapid edits cost a single write. Settings like IP/ports still apply after reboot.
void autosaveConfiguration() {
  if (memcmp(&config, &persistedConfig, sizeof(config)) == 0) {
    configDirtySince = 0;
    return;
  }
  unsigned long now = millis();
  if (configDirtySince == 0) {
    configDirtySince = now ? now : 1;
    return;
  }
  if (now - configDirtySince >= CONFIG_AUTOSAVE_DELAY_MS) {
    EEPROM.put(0, config);
    persistedConfig = config;
    configDirtySince = 0;
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
  if (!networkAvailable) return;
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
      telnetClient.stop();
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

bool parseKeyValue(const char* input, const char* key, int& outValue) {
  size_t keyLen = strlen(key);
  if (strncasecmp(input, key, keyLen) == 0 && input[keyLen] == '=') {
    outValue = atoi(input + keyLen + 1);
    return true;
  }
  return false;
}

bool isValidFadeTime(int fadeTimeMs) {
  return fadeTimeMs >= MIN_FADE_TIME_MS && fadeTimeMs <= MAX_FADE_TIME_MS;
}

bool convertFadeTimeSeconds(float seconds, uint16_t& fadeTimeMs) {
  if (!(seconds >= 0.0f && seconds <= 60.0f)) return false;
  fadeTimeMs = (uint16_t)(seconds * 1000.0f + 0.5f);
  return true;
}

bool parseFadeTimeSeconds(const char* value, uint16_t& fadeTimeMs) {
  while (isspace((unsigned char)*value)) value++;
  if (*value == '+') value++;

  unsigned int wholeSeconds = 0;
  bool hasDigit = false;
  while (isdigit((unsigned char)*value)) {
    hasDigit = true;
    unsigned int digit = (unsigned int)(*value++ - '0');
    if (wholeSeconds > 6 || (wholeSeconds == 6 && digit > 0)) return false;
    wholeSeconds = wholeSeconds * 10 + digit;
  }

  unsigned int fractionalMs = 0;
  uint16_t fractionalPlace = 100;
  bool roundUp = false;
  bool roundingDigitSeen = false;
  bool hasNonZeroFraction = false;
  if (*value == '.') {
    value++;
    while (isdigit((unsigned char)*value)) {
      hasDigit = true;
      if (*value != '0') hasNonZeroFraction = true;
      if (fractionalPlace > 0) {
        fractionalMs += (unsigned int)(*value - '0') * fractionalPlace;
        fractionalPlace /= 10;
      } else if (!roundingDigitSeen) {
        roundUp = *value >= '5';
        roundingDigitSeen = true;
      }
      value++;
    }
  }

  while (isspace((unsigned char)*value)) value++;
  if (!hasDigit || *value != '\0' || (wholeSeconds == 60 && hasNonZeroFraction)) return false;

  unsigned int totalMs = wholeSeconds * 1000 + fractionalMs + (roundUp ? 1 : 0);
  if (!isValidFadeTime(totalMs)) return false;
  fadeTimeMs = (uint16_t)totalMs;
  return true;
}

void handleCommonConsoleCommands(char* input, Print& output) {
  const char* source = (&output == &Serial) ? "SERIAL" : "TELNET";
  int val;

  if (strcasecmp(input, "help") == 0 || strcmp(input, "?") == 0) {
#if defined(__AVR__)
    output.println(F("<angle>, up, down, show, save, reboot, key=value. Keys: down, up, dnbound, upbound, default, fade(s), localport, targetport[0-2], static, telemetry, heartbeat, ip, targetip"));
#else
    output.println(F("Commands:"));
    output.println(F("  <angle>        move to angle, e.g. 90"));
    output.println(F("  up / down      move to the saved up/down preset"));
    output.println(F("  show           print current settings"));
    output.println(F("  save           write settings to EEPROM and reboot"));
    output.println(F("  reboot         restart without saving"));
    output.println(F("  key=value      change a setting (staged until save):"));
    output.println(F("    down, up, default      angles, e.g. up=120"));
    output.println(F("    dnbound, upbound       limits 0-180"));
    output.println(F("    fade                   seconds 0-60, e.g. fade=0.4"));
    output.println(F("    localport, targetport[0-2]"));
    output.println(F("    static (0/1), telemetry, heartbeat (0/1)"));
    output.println(F("    ip, targetip           e.g. ip=192.168.1.50"));
#endif
    return;
  }

  if (strcasecmp(input, "up") == 0 || strcasecmp(input, "down") == 0) {
    setMovementTargetAtDefaultFadeTime((input[0] == 'u' || input[0] == 'U') ? config.posUp : config.posDown);
    dispatchMessage(source, F("Move -> "), targetPos);
    return;
  }

  if (strcasecmp(input, "show") == 0) {
    output.print(F("Boot Profile  : "));
    output.println(config.preferStatic == 1 ? F("STATIC") : F("DHCP"));
    output.print(F("Controller IP : "));
    output.println(Ethernet.localIP());
    output.print(F("Static IP     : "));
    output.print(config.staticIP[0]);
    output.print('.');
    output.print(config.staticIP[1]);
    output.print('.');
    output.print(config.staticIP[2]);
    output.print('.');
    output.println(config.staticIP[3]);
    output.print(F("Local Port    : "));
    output.println(config.localPort);
    output.print(F("Target Ports : "));
    for (uint8_t i = 0; i < 3; i++) {
      output.print(F("["));
      output.print(i);
      output.print(F("]:"));
      output.print(config.targetPorts[i]);
      if (i < 2) output.print(F(", "));
    }
    output.println();
    output.print(F("Telemetry Pat : "));
    output.println(config.telemetryMode);
    output.print(F("Target Host   : "));
    output.print(config.fixedTargetIP[0]);
    output.print('.');
    output.print(config.fixedTargetIP[1]);
    output.print('.');
    output.print(config.fixedTargetIP[2]);
    output.print('.');
    output.println(config.fixedTargetIP[3]);
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
    output.print(F("Active Target : "));
    output.print(targetPos);
    output.println(F(" deg"));
    output.print(F("Current Position: "));
    output.print(currentPos);
    output.println(F(" deg"));
    output.print(F("Default Fade  : "));
    output.print(config.defaultFadeTimeMs / 1000);
    output.print('.');
    uint16_t fractionalMs = config.defaultFadeTimeMs % 1000;
    if (fractionalMs < 100) output.print('0');
    if (fractionalMs < 10) output.print('0');
    output.print(fractionalMs);
    output.println(F(" s"));
    return;
  }

  if (strcasecmp(input, "reboot") == 0) {
    dispatchMessage(source, F("Rebooting (unsaved changes discarded)"));
    delay(300);
    rebootDevice();
    return;
  }

  if (strcasecmp(input, "save") == 0) {
    EEPROM.put(0, config);
    dispatchMessage(source, F("EEPROM Saved"));
    delay(500);
    rebootDevice();
    return;
  }

  if (strncasecmp(input, "targetport", 10) == 0) {
    char slotChar = input[10];  // Inspect index 10 for slot identification string
    if (slotChar >= '0' && slotChar <= '2' && input[11] == '=') {
      uint8_t slot = slotChar - '0';  // Convert ASCII char to literal int numerical offset
      int portVal = atoi(input + 12);

      if (portVal >= 0 && portVal <= 65535) {
        config.targetPorts[slot] = (uint16_t)portVal;
        dispatchMessage(source, F("Staged Target Port Array Change"));
      } else {
        output.println(F("Error: Invalid Port Range"));
      }
    } else {
      output.println(F("Syntax: targetport[0-2]=XXXX"));
    }
    return;
  }

  if (parseKeyValue(input, "down", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.posDown = val;
      dispatchMessage(source, F("Set Down -> "), val);
    }
  } else if (parseKeyValue(input, "up", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.posUp = val;
      dispatchMessage(source, F("Set Up -> "), val);
    }
  } else if (parseKeyValue(input, "dnbound", val)) {
    if (val >= 0 && val <= 180) {
      config.dnBound = val;
      dispatchMessage(source, F("Set Lo-Bound -> "), val);
    }
  } else if (parseKeyValue(input, "upbound", val)) {
    if (val >= 0 && val <= 180) {
      config.upBound = val;
      dispatchMessage(source, F("Set Hi-Bound -> "), val);
    }
  } else if (parseKeyValue(input, "default", val)) {
    if (val >= config.dnBound && val <= config.upBound) {
      config.defaultPos = val;
      dispatchMessage(source, F("Set Boot Default -> "), val);
    }
  } else if (strncasecmp(input, "fade=", 5) == 0) {
    uint16_t fadeTimeMs;
    if (parseFadeTimeSeconds(input + 5, fadeTimeMs)) {
      config.defaultFadeTimeMs = fadeTimeMs;
      dispatchMessage(source, F("Set Default Fade (ms) -> "), fadeTimeMs);
    } else {
      dispatchMessage(source, F("Error: Fade must be 0-60 seconds"));
    }
  } else if (parseKeyValue(input, "localport", val)) {
    config.localPort = val;
    dispatchMessage(source, F("Set Local Port -> "), val);
  } /*else if (parseKeyValue(input, "targetport", val)) {
    config.targetPort = val;
    dispatchMessage(source, F("Set Target Port -> "), val);
  }*/
  else if (parseKeyValue(input, "static", val)) {
    config.preferStatic = (val == 1) ? 1 : 0;
    dispatchMessage(source, F("Set PreferStatic -> "), config.preferStatic);
  } else if (parseKeyValue(input, "telemetry", val)) {
    config.telemetryMode = val;
    evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
    dispatchMessage(source, F("Set Telemetry Mode -> "), val);
  } else if (parseKeyValue(input, "heartbeat", val)) {
    config.heartbeatEnabled = (val == 1) ? 1 : 0;
    dispatchMessage(source, F("Set Heartbeat -> "), config.heartbeatEnabled);
  } else if (strncmp(input, "ip=", 3) == 0) {
    parseSerialIP(input + 3, config.staticIP);
    dispatchMessage(source, F("Staged Static IP"));
  } else if (strncmp(input, "targetip=", 9) == 0) {
    parseSerialIP(input + 9, config.fixedTargetIP);
    dispatchMessage(source, F("Staged Target IP"));
  } else if (isdigit(input[0])) {
    int parsedVal = atoi(input);
    if (parsedVal >= config.dnBound && parsedVal <= config.upBound) {
      setMovementTargetAtDefaultFadeTime(parsedVal);
      dispatchMessage(source, F("Move -> "), targetPos);
    } else {
      output.println(F("Error: Angle out of range"));
    }
  } else {
    output.println(F("Unknown command. Type help (use key=value, e.g. up=120)"));
  }
}
void beginWebServer() {
  webServer.begin();
}

static const char PAGE_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Projector Dowser</title>
<style>
body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;background:#121212;color:#e0e0e0;text-align:center;margin:0;padding:12px}
.card{background:#1e1e1e;max-width:420px;margin:14px auto;padding:18px;border-radius:12px}
h1{font-size:1.4rem;margin:0 0 8px;color:#fff}
nav{max-width:420px;margin:0 auto;display:flex;gap:10px}
nav a{flex:1;padding:10px;border-radius:6px;background:#2a2a2a;color:#aaa;text-decoration:none;font-weight:bold}
nav a.on{background:#4CAF50;color:#fff}
.stat{font-size:2.4rem;font-weight:bold;color:#4CAF50;margin:10px 0}.sub{font-size:.9rem;color:#888;margin-bottom:14px}
.bg,.row{display:flex;gap:10px;margin-top:10px}
button{flex:1;padding:12px;font-size:1rem;font-weight:bold;border:0;border-radius:6px;color:#fff;background:#555}
.u{background:#2196F3}.d{background:#f44336}button:active{opacity:.7}
label{display:flex;justify-content:space-between;align-items:center;margin:6px 0;font-size:.9rem;text-align:left}
input{width:150px;padding:6px;background:#2a2a2a;color:#fff;border:1px solid #444;border-radius:4px}
pre{text-align:left;font-size:.8rem;color:#aaa;white-space:pre-wrap}
</style></head><body>
<nav><a href="/" id="n1">Controls</a><a href="/config" id="n2">Config</a></nav>
<script>
function $(i){return document.getElementById(i)}
function c(x){return fetch('/cmd?c='+encodeURIComponent(x)).then(function(r){return r.text()}).then(function(t){if(t.trim()&&$('out'))$('out').textContent=t;poll()})}
</script>
)HTML";

static const char PAGE_CONTROLS[] PROGMEM = R"HTML(<script>$('n1').className='on'</script>
<div class="card"><h1>Projector Dowser</h1>
<div class="stat"><span id="pos">--</span>&deg;</div>
<div class="sub">Status: <span id="pre">-</span> &middot; Target <span id="tgt">-</span>&deg;</div>
<div class="bg"><button class="u" onclick="c('up')">Open (Up)</button><button class="d" onclick="c('down')">Close (Down)</button></div>
<div class="row"><input id="ang" type="number" min="0" max="180" placeholder="Angle" style="flex:1"><button onclick="c(ang.value)">Go</button></div></div>
<script>
function poll(){fetch('/status').then(function(r){return r.json()}).then(function(d){$('pos').textContent=d.pos;$('tgt').textContent=d.tgt;$('pre').textContent=d.pre}).catch(function(){})}
poll();setInterval(poll,400);
</script></body></html>)HTML";

static const char PAGE_CONFIG[] PROGMEM = R"HTML(<script>$('n2').className='on'</script>
<div class="card"><h1>Settings</h1><div id="cfg"></div>
<div class="bg"><button onclick="stage()">Stage</button><button class="d" onclick="save()">Save &amp; Reboot</button></div>
<div class="bg"><button onclick="reboot()">Reboot (discard unsaved)</button></div>
<pre id="out">Stage applies changes live; Save writes them to EEPROM and reboots.</pre></div>
<script>
var F=[["Lower bound","dnbound","dn"],["Upper bound","upbound","upb"],["Up angle","up","up"],["Down angle","down","down"],["Boot default","default","def"],["Fade (s)","fade","fade"],
["Local port","localport","lp"],["Target port 0","targetport0","tp0"],["Target port 1","targetport1","tp1"],["Target port 2","targetport2","tp2"],
["Telemetry (0 reply,1 fixed,2 bcast)","telemetry","tel"],["Heartbeat (0/1)","heartbeat","hb"],["Static IP mode (0/1)","static","st"],["Static IP","ip","ip"],["Target IP","targetip","tip"]];
var orig={};
function poll(){}
fetch('/status').then(function(r){return r.json()}).then(function(d){var h='';F.forEach(function(f){h+='<label>'+f[0]+'<input id="k_'+f[1]+'"></label>'});$('cfg').innerHTML=h;F.forEach(function(f){$('k_'+f[1]).value=d[f[2]];orig[f[1]]=String(d[f[2]])})});
async function stage(){var o='';for(var i=0;i<F.length;i++){var k=F[i][1],v=$('k_'+k).value.trim();if(v!==orig[k]){o+=await fetch('/cmd?c='+encodeURIComponent(k+'='+v)).then(function(r){return r.text()});orig[k]=v}}$('out').textContent=o||'No changes.'}
async function save(){await stage();if(confirm('Save to EEPROM and reboot?')){c('save');$('out').textContent='Saved. Rebooting...'}}
function reboot(){if(confirm('Reboot and discard unsaved changes?')){c('reboot');$('out').textContent='Rebooting...'}}
</script></body></html>)HTML";

static void webSend(EthernetClient& client, const char* data, size_t len) {
#if defined(USE_QNETHERNET)
  client.writeFully((const uint8_t*)data, len);
#else
  client.write((const uint8_t*)data, len);
#endif
}

static void webHeader(EthernetClient& client, const char* type) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: "));
  client.print(type);
  client.print(F("\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
}

static void urlDecode(char* s) {
  char* o = s;
  while (*s) {
    if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char h[3] = { s[1], s[2], 0 };
      *o++ = (char)strtol(h, NULL, 16);
      s += 3;
    } else {
      *o++ = (*s == '+') ? ' ' : *s;
      s++;
    }
  }
  *o = '\0';
}

static void webStatusJson(EthernetClient& client) {
  char buf[400];
  uint16_t ms = config.defaultFadeTimeMs;
  snprintf(buf, sizeof(buf),
           "{\"pos\":%d,\"tgt\":%d,\"pre\":\"%s\",\"up\":%d,\"down\":%d,\"dn\":%d,\"upb\":%d,\"def\":%d,"
           "\"fade\":\"%u.%03u\",\"lp\":%u,\"tp0\":%u,\"tp1\":%u,\"tp2\":%u,\"tel\":%d,\"hb\":%d,\"st\":%d,"
           "\"ip\":\"%d.%d.%d.%d\",\"tip\":\"%d.%d.%d.%d\"}",
           currentPos, targetPos, currentPresetName, config.posUp, config.posDown, config.dnBound, config.upBound,
           config.defaultPos, ms / 1000, ms % 1000, config.localPort, config.targetPorts[0], config.targetPorts[1],
           config.targetPorts[2], config.telemetryMode, config.heartbeatEnabled, config.preferStatic,
           config.staticIP[0], config.staticIP[1], config.staticIP[2], config.staticIP[3],
           config.fixedTargetIP[0], config.fixedTargetIP[1], config.fixedTargetIP[2], config.fixedTargetIP[3]);
  client.print(buf);
}

void processWebServer() {
  if (!networkAvailable) return;

  EthernetClient client = webServer.available();
  if (!client) return;

  // Read only the request line, then drop the remaining headers.
  char req[128];
  size_t n = 0;
  bool lineDone = false;
  unsigned long start = millis();
  while (client.connected() && millis() - start < 250) {
    if (!client.available()) continue;
    char c = client.read();
    if (c == '\n') {
      lineDone = true;
      break;
    }
    if (c != '\r' && n < sizeof(req) - 1) req[n++] = c;
  }
  req[n] = '\0';

  if (lineDone && strncmp(req, "GET ", 4) == 0) {
    char* path = req + 4;
    char* end = strchr(path, ' ');
    if (end) *end = '\0';

    if (strncmp(path, "/status", 7) == 0) {
      webHeader(client, "application/json");
      webStatusJson(client);
    } else if (strncmp(path, "/cmd?c=", 7) == 0) {
      char* cmd = path + 7;
      urlDecode(cmd);
      cmd[47] = '\0';
      webHeader(client, "text/plain");
      handleCommonConsoleCommands(cmd, client);
    } else if (strcmp(path, "/up") == 0 || strcmp(path, "/down") == 0) {
      // Legacy links: act, then send the browser back to the dashboard so a refresh never re-triggers.
      setMovementTargetAtDefaultFadeTime(path[1] == 'u' ? config.posUp : config.posDown);
      client.print(F("HTTP/1.1 303 See Other\r\nLocation: /\r\nConnection: close\r\n\r\n"));
    } else if (strcmp(path, "/") == 0 || strncmp(path, "/?", 2) == 0) {
      webHeader(client, "text/html; charset=utf-8");
      webSend(client, PAGE_HEAD, strlen(PAGE_HEAD));
      webSend(client, PAGE_CONTROLS, strlen(PAGE_CONTROLS));
    } else if (strcmp(path, "/config") == 0) {
      webHeader(client, "text/html; charset=utf-8");
      webSend(client, PAGE_HEAD, strlen(PAGE_HEAD));
      webSend(client, PAGE_CONFIG, strlen(PAGE_CONFIG));
    } else {
      client.print(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n"));
    }
  }
  client.flush();
  client.stop();
}

void rebootDevice() {
  // Teensy 4.1 software reboot trigger
  Serial.flush();
  SCB_AIRCR = 0x05FA0004;
}