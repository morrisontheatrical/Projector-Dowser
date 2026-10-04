#include <Arduino_BuiltIn.h>
#include <Servo.h>
#include <string.h>
#include <LiquidCrystal.h>

#define IR_RECEIVE_PIN 11
#define BUTTON_PIN 2
#define SERVO_PIN 3
#define LED_PIN 13
#define ROT_CLK_PIN 18
#define ROT_DT_PIN 6
#define ROT_SW_PIN 5

#include "IRremote.h"  // after pin definitions for some reason, intentional


volatile boolean TurnDetected;       // need volatile for Interrupts
volatile boolean rotationdirection;  // CW or CCW rotation

int RotaryPosition = 0;  // To store Stepper Motor Position
int PrevPosition;        // Previous Rotary position Value to check accuracy

// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);
bool topLine = false;
bool lcdDisplay = true;

Servo myservo;  // Create Servo object to control a servo

const int down = 15;
const int up = 165;
const int dnBound = 0;  //test motor spins out of control past this
const int upBound = 180;
const int mvdelay = 10;

unsigned int lastStringLength;

int ledState = HIGH;      // The current state of the output pin
int buttonState;          // Variable for reading the pushbutton status
int lastButtonState = 0;  // The previous reading from the input pin
int buttonPushCounter = 1;

unsigned long lastDebounceTime = 0;  // The last time the output pin was toggled
unsigned long debounceDelay = 50;    // The debounce time

//IRrecv irrecv(IR_RECEIVE_PIN);  // Create instance of 'irrecv'
uint32_t last_decodedRawData = 0;
bool editPos = false;
bool presetMode = true;
bool pauseMV = false;

int tar[] = { down, down, down, down, down, 125, up, up, up, up };





// NEW: Global variables for non-blocking servo movement
int targetPos = down;
int tarPos;
unsigned long lastServoMoveTime = 0;


// Interrupt routine runs if CLK goes from HIGH to LOW
void isr() {
  delay(4);  // delay for Debouncing
  //if (digitalRead(ROT_CLK_PIN))
  rotationdirection = digitalRead(ROT_DT_PIN);
  //else
  //  rotationdirection = !digitalRead(ROT_DT_PIN);
  TurnDetected = true;
  //message("Turn Detected");
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  //irrecv.enableIRIn();  // Start the IR_RECEIVE_PIN
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


  myservo.attach(SERVO_PIN);
  myservo.write(down);
  targetPos = down;  // Initialize target to start position

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(ROT_CLK_PIN, INPUT);
  pinMode(ROT_DT_PIN, INPUT);
  pinMode(ROT_SW_PIN, INPUT);
  digitalWrite(ROT_SW_PIN, HIGH);                                     // Pull-Up resistor for switch
  attachInterrupt(digitalPinToInterrupt(ROT_CLK_PIN), isr, FALLING);  // interrupt 0 always connected to pin 2 on Arduino UNO


  lcd.setCursor(0, 1);
  message("Initialized");
  digitalWrite(LED_PIN, ledState);
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  //lcd.setCursor(13, 1);
  //lcd.print((millis() - lastDebounceTime) / 1000);

  serialToLCD();
  IRread();
  updateServo();  // NEW: Continuously moves the servo seamlessly without blocking execution
  rotator();

  // Debounce logic for physical button
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == 1) {
        String strButCount = String(buttonPushCounter);
        message("Button Pressed " + strButCount);

        // Toggle the target position non-blockingly
        targetPos = (myservo.read() == down) ? up : down;

        ledState = LOW;
      } else {
        message("Ready");
        ledState = HIGH;
      }
      delay(20);
    }
  }
  lastButtonState = reading;
}

void message(String text) {
  Serial.println(text);
  lcdcursor(text);
}

/*void lcdscroll(bool scrollOn) {
  if (scrollOn == true) {

    for (int positionCounter = 0; positionCounter < 13; positionCounter++) {
      // scroll one position left:
      lcd.scrollDisplayLeft();
      // wait a bit:
      delay(250);
    }

    // scroll 29 positions (string length + display length) to the right
    // to move it offscreen right:
    for (int positionCounter = 0; positionCounter < 29; positionCounter++) {
      // scroll one position right:
      lcd.scrollDisplayRight();
      // wait a bit:
      delay(250);
    }

    // scroll 16 positions (display length + string length) to the left
    // to move it back to center:
    for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
      // scroll one position left:
      lcd.scrollDisplayLeft();
      // wait a bit:
      delay(250);
    }

    // delay at the end of the full loop:
    delay(1000);
  } else {
    return;
  }
} */

void lcdcursor(String messageText) {
  //if (messageText.length() > 15) {
  //  lcdscroll(true);
  if (topLine == true) {
    lcd.setCursor(0, 1);
    lcd.print("             ");
    lcd.setCursor(0, 1);

    lcd.print(messageText);
    topLine = false;
  } else {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print(messageText);
    topLine = true;
  }
  //}else {
  //  lcdscroll(false);
  //}
  lastStringLength = messageText.length();
}

void serialToLCD() {
  if (Serial.available()) {
    delay(100);
    lcd.clear();
    while (Serial.available() > 0) {
      char c = Serial.read();  // Read once into a variable
      lcd.write(c);
      if (c == '9') {  // Single quote character literal comparison
        message("Activated via serial");
        targetPos = (myservo.read() == down) ? up : down;
      }
    }
  }
}

void IRread() {

  if (IrReceiver.decode()) {
    //Serial.print("IR.");
    translateIR();
    // Duplicated resume call removed from here
  }
}

// NEW: Non-blocking routine called every single loop
void updateServo() {
  int currentPos = myservo.read();

  if (currentPos != targetPos) {
    myservo.attach(SERVO_PIN);
  }

  if (targetPos > upBound) {
    message("Motor Bound");
    targetPos = upBound - 1;
  } else if (targetPos < dnBound) {
    message("Motor Bound");
    targetPos = dnBound + 1;
  }

  if (currentPos != targetPos) {
    if (millis() - lastServoMoveTime >= mvdelay) {
      lastServoMoveTime = millis();

      if (currentPos < targetPos) {
        currentPos += 1;
      } else {
        currentPos -= 1;
      }
      myservo.write(currentPos);

      // Check if destination was reached to trigger messages
      if (currentPos == targetPos) {
        if (targetPos == up) {
          message("to up position");
        } else if (targetPos == down) {
          message("to down position");
        } else {
          topLine = true;
          message("Pos: " + String(currentPos));
        }
        buttonPushCounter++;
      }
    }
  }
  //ddetatch();
}
void tmptranslateIR() {
  // Prints out the modernized format
  Serial.print("Protocol: ");
  Serial.print(IrReceiver.decodedIRData.protocol);
  Serial.print(" | Command: 0x");
  Serial.println(IrReceiver.decodedIRData.command, HEX);

  // Prints out what the library is seeing as raw data
  Serial.print("Raw Hex Data: 0x");
  Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);

  IrReceiver.resume();
}
void ddetatch() {
  int currentPos = myservo.read();
  if (currentPos != targetPos) {
  } else {

    myservo.detach();
  }
}
void translateIR() {
  long IRData = IrReceiver.decodedIRData.decodedRawData;

  switch (IRData) {
    case 0xBA45FF00:
      Serial.println("POWER");
      if (lcdDisplay) {
        lcd.clear();
        message("Display Off");
        lcd.noDisplay();
        lcdDisplay = false;
      } else {
        lcd.display();
        message("Display On");
        lcdDisplay = true;
      }
      break;

    case 0xB847FF00:
      Serial.println("FUNC/STOP");
      topLine = false;
      if (editPos == true) {
        editPos = false;
        message("Edit Mode Off.");

      } else if (editPos == false) {
        editPos = true;
        message("Edit Mode On. Select a preset.");
        lcd.setCursor(13, 0);
        lcd.print("[E]");
      }


      break;

    case 0xB946FF00:
      Serial.println("VOL+");
      targetPos = myservo.read() + 1;
      Serial.println(myservo.read());
      break;

    case 0xBB44FF00:
      Serial.println("FAST BACK");
      targetPos = myservo.read() - 5;
      break;

    case 0xBF40FF00:
      Serial.println("PAUSE");
      ppause();
      break;

    case 0xBC43FF00:
      Serial.println("FAST FORWARD");
      targetPos = myservo.read() + 5;
      break;

    case 0xF807FF00:
      Serial.println("DOWN");
      targetPos = down;
      break;

    case 0xEA15FF00:
      Serial.println("VOL-");
      targetPos = myservo.read() - 1;
      Serial.println(myservo.read());
      break;

    case 0xF609FF00:
      Serial.println("UP");
      targetPos = up;
      break;
    //case 0xE619FF00: Serial.println("EQ"); break;
    case 0xF20DFF00:
      Serial.println("ST/REPT");

      ddetatch();
      break;
    case 0xE916FF00:
      Serial.println("0");
      preset(0);
      break;
    case 0xF30CFF00:
      Serial.println("1");
      preset(1);
      break;
    case 0xE718FF00:
      Serial.println("2");
      preset(2);
      break;
    case 0xA15EFF00:
      Serial.println("3");
      preset(3);
      break;
    case 0xF708FF00:
      Serial.println("4");
      preset(4);
      break;
    case 0xE31CFF00:
      Serial.println("5");
      preset(5);
      break;
    case 0xA55AFF00:
      Serial.println("6");
      preset(6);
      break;
    case 0xBD42FF00:
      Serial.println("7");
      preset(7);
      break;
    case 0xAD52FF00:
      Serial.println("8");
      preset(8);
      break;
    case 0xB54AFF00:
      Serial.println("9");
      preset(9);
      break;
    case 0x0: break;
    default:
      Serial.println(" other button   ");
      discoverIR();
  }

  last_decodedRawData = IrReceiver.decodedIRData.decodedRawData;
  IrReceiver.resume();  // Single, clean point of resumption
}

void discoverIR() {
tmptranslateIR();

}


// Changed to void so it handles the array directly without returning values
void preset(int num) {
  if (editPos == true) {

    if (presetMode == true) {
      presetMode = false;
      topLine = false;
      message("Set Preset " + String(num) + ".");
    } else {
      presetMode = true;
      // Save the current motor position directly into the global array
      tar[num] = myservo.read();
      topLine = false;
      message("Preset " + String(num) + " Saved.");
    }
  } else if (editPos == false) {

    // Recall the position from the array and set it as the new target
    targetPos = tar[num];
    topLine = true;
    message("Go Preset " + String(num));
  }
}
void ppause() {
  int curPos = myservo.read();
  if (pauseMV == true) {
    //press to resume
    message("Resume");
    targetPos = tarPos;
    pauseMV = false;
  } else if (pauseMV == false) {
    //pause activated
    tarPos = targetPos;
    message("Paused @ " + String(curPos));
    message("Saved Target:" + String(tarPos));
    targetPos = curPos;
    pauseMV = true;
  }
}


void rotator() {

  // Encoder button press logic
  if (!(digitalRead(ROT_SW_PIN))) {
    if (RotaryPosition != 0) {
      RotaryPosition = 0;  // Reset position to ZERO
      message("Encoder Reset");
      delay(100);  // Simple debounce for the physical button press
    }
  }

  // Runs if the background ISR flagged a rotation
  if (TurnDetected) {
    // Acknowledge the flag immediately so we don't miss the next click
    TurnDetected = false;

    if (rotationdirection) {
      // Clockwise
      targetPos = myservo.read() - 2;
    } else {
      // Counter-Clockwise
      targetPos = myservo.read() + 2;
    }

    // Optional: Print to verify it's working, though LCD updates might
    // feel a bit sluggish if you spin the wheel super fast.
    // message("Turned");
  }
}
