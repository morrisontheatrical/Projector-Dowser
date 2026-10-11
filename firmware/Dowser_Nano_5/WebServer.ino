// Minimal non-blocking HTTP server (port 80) for control and configuration.
// No authentication: intended for trusted show/control networks only.
#include "web_page.h"

EthernetServer webServer(80);
EthernetClient webClient;

const unsigned long WEB_REQUEST_TIMEOUT_MS = 2000;
char webRequestLine[112];
uint8_t webLineLen = 0;
uint8_t webNewlineRun = 0;
bool webLineDone = false;
unsigned long webRequestStart = 0;

void beginWebServer() {
  webServer.begin();
}

static void webReset() {
  webLineLen = 0;
  webNewlineRun = 0;
  webLineDone = false;
}

static void sendHeader(int code, const char* contentType) {
  webClient.print(F("HTTP/1.1 "));
  webClient.print(code);
  webClient.print(code == 200 ? F(" OK") : F(" Error"));
  webClient.print(F("\r\nContent-Type: "));
  webClient.print(contentType);
  webClient.print(F("\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
}

static void sendJsonResult(int code, const char* errorText) {
  sendHeader(code, "application/json");
  if (errorText) {
    webClient.print(F("{\"ok\":false,\"error\":\""));
    webClient.print(errorText);
    webClient.print(F("\"}"));
  } else {
    webClient.print(F("{\"ok\":true}"));
  }
}

static void printFadeSeconds(Print& out, uint16_t ms) {
  out.print(ms / 1000);
  out.print('.');
  uint16_t frac = ms % 1000;
  if (frac < 100) out.print('0');
  if (frac < 10) out.print('0');
  out.print(frac);
}

static void printIpArray(Print& out, const uint8_t* ip) {
  for (uint8_t i = 0; i < 4; i++) {
    if (i) out.print('.');
    out.print(ip[i]);
  }
}

static void sendStatusJson() {
  sendHeader(200, "application/json");
  webClient.print(F("{\"pos\":"));
  webClient.print(currentPos);
  webClient.print(F(",\"target\":"));
  webClient.print(targetPos);
  webClient.print(F(",\"moving\":"));
  webClient.print(currentPos != targetPos ? F("true") : F("false"));
  webClient.print(F(",\"paused\":"));
  webClient.print(pauseMV ? F("true") : F("false"));
  webClient.print(F(",\"dnbound\":"));
  webClient.print(config.dnBound);
  webClient.print(F(",\"upbound\":"));
  webClient.print(config.upBound);
  webClient.print(F(",\"ip\":\""));
  webClient.print(Ethernet.localIP());
  webClient.print(F("\"}"));
}

static void sendConfigJson() {
  sendHeader(200, "application/json");
  webClient.print(F("{\"down\":"));
  webClient.print(config.posDown);
  webClient.print(F(",\"up\":"));
  webClient.print(config.posUp);
  webClient.print(F(",\"dnbound\":"));
  webClient.print(config.dnBound);
  webClient.print(F(",\"upbound\":"));
  webClient.print(config.upBound);
  webClient.print(F(",\"default\":"));
  webClient.print(config.defaultPos);
  webClient.print(F(",\"fade\":"));
  printFadeSeconds(webClient, config.defaultFadeTimeMs);
  webClient.print(F(",\"heartbeat\":"));
  webClient.print(config.heartbeatEnabled);
  webClient.print(F(",\"telemetry\":"));
  webClient.print(config.telemetryMode);
  webClient.print(F(",\"targetip\":\""));
  printIpArray(webClient, config.fixedTargetIP);
  webClient.print(F("\",\"targetport0\":"));
  webClient.print(config.targetPorts[0]);
  webClient.print(F(",\"targetport1\":"));
  webClient.print(config.targetPorts[1]);
  webClient.print(F(",\"targetport2\":"));
  webClient.print(config.targetPorts[2]);
  webClient.print(F(",\"localport\":"));
  webClient.print(config.localPort);
  webClient.print(F(",\"static\":"));
  webClient.print(config.preferStatic);
  webClient.print(F(",\"ip\":\""));
  printIpArray(webClient, config.staticIP);
  webClient.print(F("\",\"mask\":\""));
  printIpArray(webClient, config.subnetMask);
  webClient.print(F("\",\"gw\":\""));
  printIpArray(webClient, config.gatewayIP);
  webClient.print(F("\"}"));
}

static void sendIndexPage() {
  sendHeader(200, "text/html");
  const size_t total = sizeof(WEB_PAGE) - 1;
  char chunk[256];
  for (size_t offset = 0; offset < total; offset += sizeof(chunk)) {
    size_t n = min(sizeof(chunk), total - offset);
    memcpy_P(chunk, WEB_PAGE + offset, n);
    webClient.write((const uint8_t*)chunk, n);
  }
}

static bool parseWebInt(const char* s, long minVal, long maxVal, long& out) {
  if (*s == '\0') return false;
  char* end;
  long v = strtol(s, &end, 10);
  if (*end != '\0' || v < minVal || v > maxVal) return false;
  out = v;
  return true;
}

static bool parseWebIp(const char* s, uint8_t* out) {
  uint8_t tmp[4];
  for (uint8_t i = 0; i < 4; i++) {
    char* end;
    long v = strtol(s, &end, 10);
    if (end == s || v < 0 || v > 255) return false;
    if (i < 3 ? *end != '.' : *end != '\0') return false;
    tmp[i] = (uint8_t)v;
    s = end + 1;
  }
  memcpy(out, tmp, 4);
  return true;
}

// Returns NULL on success or a short error string.
static const char* applyWebConfig(const char* key, const char* val) {
  long n;
  if (!strcmp(key, "down") || !strcmp(key, "up") || !strcmp(key, "default")) {
    if (!parseWebInt(val, config.dnBound, config.upBound, n)) return "Value outside bounds";
    if (key[0] == 'd' && key[1] == 'o') config.posDown = n;
    else if (key[0] == 'u') config.posUp = n;
    else config.defaultPos = n;
  } else if (!strcmp(key, "dnbound") || !strcmp(key, "upbound")) {
    if (!parseWebInt(val, 0, 180, n)) return "Bound must be 0-180";
    if (key[0] == 'd') config.dnBound = n;
    else config.upBound = n;
  } else if (!strcmp(key, "fade")) {
    uint16_t ms;
    if (!parseFadeTimeSeconds(val, ms)) return "Fade must be 0-60 seconds";
    config.defaultFadeTimeMs = ms;
  } else if (!strcmp(key, "heartbeat") || !strcmp(key, "static")) {
    if (!parseWebInt(val, 0, 1, n)) return "Value must be 0 or 1";
    if (key[0] == 'h') config.heartbeatEnabled = n;
    else config.preferStatic = n;
  } else if (!strcmp(key, "telemetry")) {
    if (!parseWebInt(val, 0, 2, n)) return "Telemetry must be 0-2";
    config.telemetryMode = n;
    evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
  } else if (!strcmp(key, "localport")) {
    if (!parseWebInt(val, 1, 65535, n)) return "Invalid port";
    config.localPort = n;
  } else if (!strncmp(key, "targetport", 10) && key[10] >= '0' && key[10] <= '2' && key[11] == '\0') {
    if (!parseWebInt(val, 0, 65535, n)) return "Invalid port";
    config.targetPorts[key[10] - '0'] = n;
  } else if (!strcmp(key, "ip") || !strcmp(key, "mask") || !strcmp(key, "gw") || !strcmp(key, "targetip")) {
    uint8_t* dest = !strcmp(key, "ip") ? config.staticIP
                    : !strcmp(key, "mask") ? config.subnetMask
                    : !strcmp(key, "gw")   ? config.gatewayIP
                                           : config.fixedTargetIP;
    if (!parseWebIp(val, dest)) return "Invalid IP address";
    if (dest == config.fixedTargetIP) evaluateTelemetryRouting(IPAddress(0, 0, 0, 0), 0);
  } else {
    return "Unknown setting";
  }
  return NULL;
}

// Iterates "k=v&k=v" in place. Returns first error or NULL.
static const char* applyWebConfigQuery(char* query) {
  if (*query == '\0') return "No settings supplied";
  char* save = NULL;
  for (char* pair = strtok_r(query, "&", &save); pair; pair = strtok_r(NULL, "&", &save)) {
    char* eq = strchr(pair, '=');
    if (!eq) return "Malformed query";
    *eq = '\0';
    const char* err = applyWebConfig(pair, eq + 1);
    if (err) return err;
  }
  return NULL;
}

static const char* handleWebMove(char* query) {
  const char* to = NULL;
  uint16_t fadeMs = config.defaultFadeTimeMs;
  char* save = NULL;
  for (char* pair = strtok_r(query, "&", &save); pair; pair = strtok_r(NULL, "&", &save)) {
    char* eq = strchr(pair, '=');
    if (!eq) return "Malformed query";
    *eq = '\0';
    if (!strcmp(pair, "to")) to = eq + 1;
    else if (!strcmp(pair, "fade")) {
      if (!parseFadeTimeSeconds(eq + 1, fadeMs)) return "Fade must be 0-60 seconds";
    }
  }
  if (!to) return "Missing 'to'";
  long angle;
  if (!strcmp(to, "up")) angle = config.posUp;
  else if (!strcmp(to, "down")) angle = config.posDown;
  else if (!parseWebInt(to, config.dnBound, config.upBound, angle)) return "Angle outside bounds";
  setMovementTarget(angle, fadeMs);
  return NULL;
}

static void handleWebRequest() {
  char* method = webRequestLine;
  char* path = strchr(method, ' ');
  if (!path) { sendJsonResult(400, "Bad request"); return; }
  *path++ = '\0';
  char* end = strchr(path, ' ');
  if (end) *end = '\0';
  char* query = strchr(path, '?');
  if (query) *query++ = '\0';
  else query = path + strlen(path);

  bool isGet = !strcmp(method, "GET");
  bool isPost = !strcmp(method, "POST");

  if (isGet && !strcmp(path, "/")) { sendIndexPage(); return; }
  if (isGet && !strcmp(path, "/api/status")) { sendStatusJson(); return; }
  if (isGet && !strcmp(path, "/api/config")) { sendConfigJson(); return; }

  if (isPost && !strcmp(path, "/api/move")) {
    const char* err = handleWebMove(query);
    sendJsonResult(err ? 400 : 200, err);
    return;
  }
  if (isPost && !strcmp(path, "/api/config")) {
    const char* err = applyWebConfigQuery(query);
    sendJsonResult(err ? 400 : 200, err);
    return;
  }
  if (isPost && !strcmp(path, "/api/reboot")) {
    sendJsonResult(200, NULL);
    webClient.flush();
    delay(100);
    EEPROM.put(0, config);
    rebootDevice();
    return;
  }
  sendJsonResult(404, "Not found");
}

void processWebServer() {
  if (!networkAvailable) return;

  if (!webClient || !webClient.connected()) {
    webClient.stop();
    EthernetClient incoming = webServer.accept();
    if (!incoming) return;
    webClient = incoming;
    webReset();
    webRequestStart = millis();
  }

  if (millis() - webRequestStart > WEB_REQUEST_TIMEOUT_MS) {
    webClient.stop();
    return;
  }

  // Read the request line, then discard headers until the blank line.
  while (webClient.available() > 0) {
    char c = webClient.read();
    if (c == '\n') {
      webLineDone = true;
      if (++webNewlineRun == 2) {
        webRequestLine[webLineLen] = '\0';
        handleWebRequest();
        webClient.stop();
        return;
      }
    } else if (c != '\r') {
      webNewlineRun = 0;
      if (!webLineDone && webLineLen < sizeof(webRequestLine) - 1) {
        webRequestLine[webLineLen++] = c;
      }
    }
  }
}
