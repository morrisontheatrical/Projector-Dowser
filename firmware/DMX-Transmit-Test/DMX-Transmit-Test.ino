// DMX512 transmitter test for Nano Every + MAX485.
// Wiring: DI->TX (D1), DE+RE tied together->D4 (or straight to 5V), VCC->5V, GND->GND.
// RO is unused. Module A/B -> DMX Data+/Data-.
// Serial monitor (115200, newline): "<channel> <value>" sets one channel (e.g. "1 255"),
//   "<channel> <value> <seconds>" fades to it (e.g. "1 0 3.5"), "off" blacks out all.
// Button on D2 (to GND): toggles channel 1 between 0 and 255 with a 3s fade.

#define DE_PIN 4
#define BUTTON_PIN 2
#define TX_PIN 1
#define TEST_CH 1

uint8_t dmx[513];  // [0] = start code (0)

struct Fade {
  bool active;
  float from;
  uint8_t to;
  unsigned long startMs, durMs;
} fades[513];

void sendFrame() {
  Serial1.end();
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);   // BREAK
  delayMicroseconds(120);
  digitalWrite(TX_PIN, HIGH);  // Mark after break
  delayMicroseconds(12);
  Serial1.begin(250000, SERIAL_8N2);
  Serial1.write(dmx, 513);
  Serial1.flush();
}

void startFade(uint16_t ch, uint8_t to, float seconds) {
  if (ch < 1 || ch > 512) return;
  if (seconds <= 0) { fades[ch].active = false; dmx[ch] = to; return; }
  fades[ch] = { true, (float)dmx[ch], to, millis(), (unsigned long)(seconds * 1000.0f) };
}

void updateFades() {
  unsigned long now = millis();
  for (uint16_t ch = 1; ch <= 512; ch++) {
    if (!fades[ch].active) continue;
    unsigned long el = now - fades[ch].startMs;
    if (el >= fades[ch].durMs) { dmx[ch] = fades[ch].to; fades[ch].active = false; }
    else dmx[ch] = (uint8_t)(fades[ch].from + (fades[ch].to - fades[ch].from) * ((float)el / fades[ch].durMs) + 0.5f);
  }
}

void handleSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0; n = 0;
    if (!buf[0]) continue;
    if (!strcmp(buf, "off")) { memset(dmx, 0, sizeof(dmx)); for (auto &f : fades) f.active = false; continue; }
    int ch, val; float sec = 0;
    int got = sscanf(buf, "%d %d %f", &ch, &val, &sec);
    if (got >= 2 && ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
      startFade(ch, val, sec);
      Serial.print(F("ch ")); Serial.print(ch); Serial.print(F(" -> ")); Serial.println(val);
    } else Serial.println(F("Usage: <ch 1-512> <val 0-255> [fadeSeconds]"));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, HIGH);  // driver enabled
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial1.begin(250000, SERIAL_8N2);
  Serial.println(F("DMX transmit ready"));
}

void loop() {
  static bool lastBtn = HIGH, high = false;
  static unsigned long lastBtnMs = 0;
  bool btn = digitalRead(BUTTON_PIN);
  if (btn != lastBtn && millis() - lastBtnMs > 30) {
    lastBtnMs = millis(); lastBtn = btn;
    if (btn == LOW) { high = !high; startFade(TEST_CH, high ? 255 : 0, 3.0); }
  }
  handleSerial();
  updateFades();
  sendFrame();  // ~44 frames/s at full 512 slots
}
