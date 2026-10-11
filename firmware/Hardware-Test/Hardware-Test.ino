// Bench test: I2C LCD (A4/A5), MAX485 receiver (RO->RX/D0), button (D2->GND)
// Library: "LiquidCrystal I2C" (Frank de Brabander) via Library Manager
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BUTTON_PIN 2
#define DMX_CHANNEL 1

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
  uint8_t addr = findLcd();
  Serial.print(F("LCD I2C address: 0x")); Serial.println(addr, HEX);
  if (!addr) { Serial.println(F("No I2C device found - check SDA/SCL/VCC/GND")); }
  lcd = new LiquidCrystal_I2C(addr ? addr : 0x27, 16, 2);  // change 16,2 to 20,4 if needed
  lcd->init();
  lcd->backlight();
  lcd->print(F("Dowser HW test"));
  delay(1000);
  lcd->clear();
  Serial1.begin(250000, SERIAL_8N2);  // DMX512
}

void readDmx() {
  // A break appears as a framing error/0x00 byte; detect by gap in time instead
  while (Serial1.available()) {
    unsigned long now = micros();
    static unsigned long lastByteUs = 0;
    if (now - lastByteUs > 500) dmxIdx = 0;  // inter-frame gap (>~88us break + MAB) -> new frame
    lastByteUs = now;
    uint8_t b = Serial1.read();
    if (dmxIdx >= 0 && dmxIdx < 513) dmx[dmxIdx++] = b;  // [0]=start code
    lastFrameMs = millis();
  }
}

void loop() {
  readDmx();

  bool btn = digitalRead(BUTTON_PIN);
  if (btn != lastBtn && millis() - lastBtnMs > 30) {
    lastBtnMs = millis();
    lastBtn = btn;
    if (btn == LOW) presses++;
  }

  if (millis() - lastDraw > 200) {
    lastDraw = millis();
    bool live = millis() - lastFrameMs < 1000;
    lcd->setCursor(0, 0);
    lcd->print(F("Btn:")); lcd->print(btn == LOW ? F("DOWN") : F("up  "));
    lcd->print(F(" #")); lcd->print(presses); lcd->print(F("    "));
    lcd->setCursor(0, 1);
    if (live) {
      lcd->print(F("DMX ch")); lcd->print(DMX_CHANNEL); lcd->print(F(":"));
      lcd->print(dmx[DMX_CHANNEL]); lcd->print(F("    "));
    } else {
      lcd->print(F("No DMX signal  "));
    }
  }
}
