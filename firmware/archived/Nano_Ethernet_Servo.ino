#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>

// Network Configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 0, 0, 100);       // Arduino Static IP
IPAddress targetIP(10, 0, 0, 15);  // Mac Studio / QLab Machine IP

const unsigned int localPort = 8000;    // Port Arduino listens to
const unsigned int targetPort = 53000;  // QLab's default incoming OSC port

EthernetUDP Udp;

// Global tracking variables for system state
int currentAngle = 0;
const char* currentPreset = "Unknown";

void setup() {
  Serial.begin(1000000);
  while (!Serial) { ; }

  Ethernet.init(10);
  Serial.println("Initializing W5500 Ethernet...");
  Ethernet.begin(mac, ip);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Error: W5500 not detected.");
    while (true) { delay(1); }
  }

  Udp.begin(localPort);
  Serial.println("Dowser OSC Node Online.");
  
  // Send an initial handshake to QLab to clear the status display
  sendQLabFeedback("/cue/DOWSER_STATUS/notes", "System Ready");
}

void loop() {
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    while (packetSize--) {
      msgIn.fill(Udp.read());
    }

    if (!msgIn.hasError()) {
      // Direct Endpoint Routing
      msgIn.route("/dowser/up", handleUp);
      msgIn.route("/dowser/down", handleDown);
      msgIn.route("/dowser/reset", handleReset);
      msgIn.route("/dowser/angle", handleAngle);
    }
  }
}

// --- CALLBACK HANDLERS ---

void handleUp(OSCMessage& msg, int addrOffset) {
  Serial.println("Command: Open Dowser");
  currentAngle = 179; // Or whatever your physical open limit is
  currentPreset = "Open (Up)";
  
  // Execute physical servo write here: myServo.write(currentAngle);

  pushStateToQLab();
}

void handleDown(OSCMessage& msg, int addrOffset) {
  Serial.println("Command: Close Dowser");
  currentAngle = 1; // Or whatever your physical closed limit is
  currentPreset = "Closed (Down)";

  // Execute physical servo write here: myServo.write(currentAngle);

  pushStateToQLab();
}

void handleReset(OSCMessage& msg, int addrOffset) {
  Serial.println("Command: Resetting Homing Sequence");
  currentAngle = 0;
  currentPreset = "Homing/Reset";

  // Execute physical homing sequence here

  pushStateToQLab();
}

void handleAngle(OSCMessage& msg, int addrOffset) {
  // Ensure an argument exists and is a valid number type (using library-correct .size())
  if (msg.size() > 0 && (msg.isInt(0) || msg.isFloat(0))) {
    // Extract numerical value safely regardless of integer or float formatting
    int targetAngle = msg.isInt(0) ? msg.getInt(0) : (int)msg.getFloat(0);

    // Guard rails: Clamp boundaries between 1 and 179 to protect physical servo limits
    if (targetAngle >= 1 && targetAngle <= 179) {
      currentAngle = targetAngle;
      currentPreset = "Custom Angle";
      
      Serial.print("Command: Moving to explicit angle: ");
      Serial.println(currentAngle);

      // Execute physical servo write here: myServo.write(currentAngle);
      
      pushStateToQLab();
    } else {
      Serial.println("Error: Angle argument out of bounds (1-179).");
      sendQLabFeedback("/cue/DOWSER_STATUS/notes", "Error: Out of Bounds");
    }
  }
}

// --- FEEDBACK ENGINES ---

// Helper to bundle both telemetry components and ship them out
void pushStateToQLab() {
  // Update the Notes field with the textual preset state
  sendQLabFeedback("/cue/DOWSER_STATUS/notes", currentPreset);
  sendQLabFeedback("/cue/DOWSER_STATUS/name", currentPreset);
  
  // Optional: If you want to update a separate cue with the raw integer angle
  // Just create another Memo cue with Custom ID "DOWSER_ANGLE"
  // sendQLabFeedback("/cue/DOWSER_ANGLE/notes", currentAngle);
}

// Overloaded OSC Sender: Handles String Messages
void sendQLabFeedback(const char* address, const char* textPayload) {
  OSCMessage msgOut(address);
  msgOut.add(textPayload);

  Udp.beginPacket(targetIP, targetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

// Overloaded OSC Sender: Handles Integer Data
void sendQLabFeedback(const char* address, int intPayload) {
  OSCMessage msgOut(address);
  msgOut.add(intPayload);

  Udp.beginPacket(targetIP, targetPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}