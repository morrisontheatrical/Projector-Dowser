#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>

// Unique MAC address for your hardware
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Network Topologies (Adjust to match your production subnet)
IPAddress ip(10, 0, 0, 100);       // This Arduino's Static IP
IPAddress targetIP(10, 0, 0, 15);  // Host IP running QLab, Eos, Max, etc.

const unsigned int localPort = 8000;    // Incoming OSC Port
const unsigned int targetPort = 53000;  // Outgoing Port on Host Machine

EthernetUDP Udp;

void setup() {
  Serial.begin(1000000);
  while (!Serial) { ; }  // Hold until serial monitor opens

  // Step 1: Tell the Ethernet library where your Chip Select (CS) pin lives
  Ethernet.init(10);

  // Step 2: Bring up the physical link layer
  Serial.println("Initializing W5500 Ethernet...");
  Ethernet.begin(mac, ip);

  // Verify the hardware is detected
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Error: W5500 module not detected. Check SPI wiring.");
    while (true) { delay(1); }  // Halt execution
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Warning: Ethernet cable unplugged.");
  }

  // Step 3: Bind the UDP socket to the local port
  Udp.begin(localPort);

  Serial.print("OSC Node Active. Listening at: ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(localPort);
}

void loop() {
  OSCMessage msgIn;
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    Serial.print("Raw UDP Packet Received! Size: ");
    Serial.print(packetSize);
    Serial.println(" bytes.");

    while (packetSize--) {
      msgIn.fill(Udp.read());
    }

    if (!msgIn.hasError()) {
      // Sniff out the address string out of the incoming buffer
      char addrBuffer[64];
      msgIn.getAddress(addrBuffer);
      Serial.print("Parsed OSC Address: ");
      Serial.println(addrBuffer);

      // Route normally
      msgIn.route("/macro/trigger", handleMacroTrigger);
    } else {
      Serial.println("Error: Incoming packet was not valid OSC data.");
    }
  }
}

// Callback execution when the routed address match occurs
void handleMacroTrigger(OSCMessage& msg, int addrOffset) {
  // Executes immediately upon matching the address string, regardless of payload
  Serial.println("OSC Match Found! Triggered via address string.");
  
  // Optional: Only read an argument if it happens to be there
  if (msg.isInt(0)) {
    int val = msg.getInt(0);
    Serial.print("With Argument: ");
    Serial.println(val);
  }
}

// Helper function to bundle and ship out clean UDP packets
void sendOSCStatus(const char* address, int argument) {
  OSCMessage msgOut(address);
  msgOut.add(argument);

  Udp.beginPacket(targetIP, targetPort);
  msgOut.send(Udp);
  Udp.endPacket();  // Flushes the buffer out to the network interface
  msgOut.empty();   // Releases the assigned dynamic memory allocation
}