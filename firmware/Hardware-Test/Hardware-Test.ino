// Bench test: I2C LCD (A4/A5), MAX485 receiver (RO->RX/D0), button (D2->GND)
// Library: "LiquidCrystal I2C" (Frank de Brabander) via Library Manager
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define BUTTON_PIN 2
#define ADDR_EEPROM 0
uint16_t dmxAddr = 1;       // 1..512
bool editMode = false;
unsigned long pressStartMs = 0, lastRepeatMs = 0, lastEditMs = 0;
bool longHandled = false;

LiquidCrystal_I2C *lcd;
uint8_t dmx[513];
int dmxIdx = -1;
unsigned long lastFrameMs = 0, lastDraw = 0;
uint16_t presses = 0;
bool lastBtn = HIGH;
unsigned long lastBtnMs = 0;

uint8_t findLcd() {
  for (uint8_t a = 0x20; a < 0x40; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) return a;
  }
  for (uint8_t a = 0x40; a < 0x80; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) return a;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000);
  uint8_t addr = findLcd();
  Serial.print(F("LCD I2C address: 0x")); Serial.println(addr, HEX);
  if (!addr) { Serial.println(F("No I2C device found - check SDA/SCL/VCC/GND")); }
  lcd = new LiquidCrystal_I2C(addr ? addr : 0x27, 16, 2);  // change 16,2 to 20,4 if needed
  lcd->init();
  lcd->backlight();
  lcd->print(F("Dowser HW test"));
  delay(1000);
  lcd->clear();
  EEPROM.get(ADDR_EEPROM, dmxAddr);
  if (dmxAddr < 1 || dmxAddr > 512) dmxAddr = 1;
  Serial1.begin(250000, SERIAL_8N2);  // DMX512
}

// The core's RX interrupt buffers bytes in Serial1. A DMX byte takes 44us, but the
// break + mark-after-break leaves ~130us before the start code, so a gap of >90us
// between reads marks the start of a frame (the loop must keep up with the UART).
bool dmxSynced = false;
bool frameDone = false;
unsigned long lastByteUs = 0;
// Debug counters, printed to Serial
uint32_t cntBytes = 0, cntBreaks = 0, cntFrames = 0, cntBadStart = 0;

void readDmx() {
  while (Serial1.available()) {
    unsigned long nowUs = micros();
    uint8_t b = Serial1.read();
    cntBytes++;
    if (nowUs - lastByteUs > 90) {
      cntBreaks++;
      dmxIdx = 0;
      dmxSynced = true;
    }
    lastByteUs = nowUs;
    if (!dmxSynced) continue;
    if (dmxIdx == 0 && b != 0) { dmxSynced = false; cntBadStart++; continue; }  // not a DMX frame
    if (dmxIdx >= 0 && dmxIdx < 513) dmx[dmxIdx++] = b;
    if (dmxIdx == 513) { lastFrameMs = millis(); frameDone = true; cntFrames++; dmxIdx = 514; }
  }
}

void loop() {
  readDmx();

  bool btn = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  if (btn != lastBtn && now - lastBtnMs > 30) {
    lastBtnMs = now;
    lastBtn = btn;
    if (btn == LOW) { pressStartMs = now; lastRepeatMs = now; longHandled = false; }
    else if (!longHandled && editMode) {  // short press in edit mode: +1
      dmxAddr = dmxAddr % 512 + 1;
      lastEditMs = now;
    }
  }
  if (lastBtn == LOW) {
    if (!editMode && !longHandled && now - pressStartMs > 1500) {  // long hold: enter edit
      editMode = true; longHandled = true; lastEditMs = now;
    } else if (editMode && now - pressStartMs > 600 && now - lastRepeatMs > 100) {  // hold: auto-repeat
      longHandled = true;
      dmxAddr = dmxAddr % 512 + 1;
      lastRepeatMs = lastEditMs = now;
    }
  }
  if (editMode && lastBtn == HIGH && now - lastEditMs > 3000) {  // idle 3s: save
    EEPROM.put(ADDR_EEPROM, dmxAddr);
    editMode = false;
  }

  // Draw only right after a full frame: the LCD write blocks long enough to overflow the 64-byte RX buffer
  if (now - lastDraw > 250 && (frameDone || now - lastFrameMs > 1000)) {
    frameDone = false;
    lastDraw = now;
    Serial.print(F("bytes ")); Serial.print(cntBytes); Serial.print(F(" breaks ")); Serial.print(cntBreaks);
    Serial.print(F(" badstart ")); Serial.print(cntBadStart);
    Serial.print(F(" frames ")); Serial.println(cntFrames);
    bool live = now - lastFrameMs < 1000;
    lcd->setCursor(0, 0);
    lcd->print(editMode ? F("SET ADDR: ") : F("DMX ADDR: "));
    lcd->print(dmxAddr); lcd->print(F("    "));
    lcd->setCursor(0, 1);
    if (editMode) lcd->print(F("Press=+1 idle=OK"));
    else if (live) {
      lcd->print(F("Value: ")); lcd->print(dmx[dmxAddr]); lcd->print(F("      "));
    } else lcd->print(F("No DMX signal  "));
    // The draw blocks long enough to overflow the buffer: discard stale bytes and wait for a fresh frame start
    while (Serial1.available()) Serial1.read();
    dmxSynced = false;
    lastByteUs = micros();
  }
}
