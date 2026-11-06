/*
  =====================================================
                  LiFi PROJECT
              ARDUINO UNO - RECEIVER
             (ASCII MODE + MORSE MODE)
  =====================================================

  UNO CONNECTIONS
  LDR digital DO       -> D8
  LDR VCC              -> 5V
  LDR GND              -> GND

  I2C LCD SDA          -> A4
  I2C LCD SCL          -> A5
  I2C LCD VCC          -> 5V
  I2C LCD GND          -> GND

  BUTTONS (other switch terminal -> GND)
  Increase speed       -> D2
  Decrease speed       -> D3
  Home screen          -> D4
  Reset/listen again   -> D5
  Mode (ASCII/Morse)   -> D6

  Status LED           -> D7
  Buzzer (+)           -> A0
  Buzzer (-)           -> GND

  UNO D12/TX           -> LEONARDO D0/RX1
  LEONARDO D1/TX1      -> UNO D13/RX
  UNO GND              -> LEONARDO GND

  PROJECT ID: 1241554

  COMMANDS SENT TO THE LEONARDO (one line each):
    S200 / S300 / S500 / S1000   -> speed (bit / unit time in ms)
    M0                           -> ASCII mode
    M1                           -> MORSE mode
  Every speed or mode change is saved in UNO EEPROM and re-sends BOTH
  values to Leonardo. Leonardo requests a settings snapshot after boot.

  IMPORTANT:
  Either board may be powered first. Leonardo requests settings
  until synchronized. Keep transmitter LEDs OFF during LDR
  idle-level calibration.

  MORSE TIMING (T = selected speed):
    dot = ON 1T, dash = ON 3T, start marker = ON 7T
    gap in letter = OFF 1T, between letters = OFF 3T, between words = OFF 7T
    message ends with the prosign ...-.- (SK)
  Pressing any button while a Morse message is being received
  aborts the reception (then the button works as normal).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change to 0x3F if your LCD uses that address.

// ---------------- PINS ----------------
const byte LDR_PIN = 8;
const byte STATUS_LED = 7;
const byte BUZZER_PIN = A0; // Buzzer sounds only after successful final output.
const byte BTN_PLUS = 2;
const byte BTN_MINUS = 3;
const byte BTN_HOME = 4;
const byte BTN_RESET = 5;
const byte BTN_MODE = 6;

// Bidirectional link to Leonardo. USB Serial remains available for debugging.
const byte LINK_RX_PIN = 13; // receives from Leonardo D1/TX1
const byte LINK_TX_PIN = 12; // sends to Leonardo D0/RX1
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);

// ---------------- SPEEDS ----------------
// Index order: slowest -> fastest.
const unsigned long SPEEDS[] = {1000UL, 500UL, 300UL, 200UL};
const byte SPEED_COUNT = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
byte speedIndex = 1;                 // Default = 500 ms
unsigned long bitTimeMs = 500UL;
uint32_t bitIntervalMicros = 500000UL;

// ---------------- MODES ----------------
const byte MODE_ASCII = 0;
const byte MODE_MORSE = 1;
byte currentMode = MODE_ASCII;       // First-boot default = ASCII

// ---------------- PERSISTENT SETTINGS ----------------
const byte EEPROM_MAGIC_ADDRESS = 0;
const byte EEPROM_SPEED_ADDRESS = 1;
const byte EEPROM_MODE_ADDRESS = 2;
const byte EEPROM_MAGIC_VALUE = 0xA7;

void loadSavedSettings() {
  if (EEPROM.read(EEPROM_MAGIC_ADDRESS) == EEPROM_MAGIC_VALUE) {
    byte savedSpeedIndex = EEPROM.read(EEPROM_SPEED_ADDRESS);
    byte savedMode = EEPROM.read(EEPROM_MODE_ADDRESS);

    if (savedSpeedIndex < SPEED_COUNT && savedMode <= MODE_MORSE) {
      speedIndex = savedSpeedIndex;
      currentMode = savedMode;
    } else {
      speedIndex = 1;
      currentMode = MODE_ASCII;
    }
  } else {
    speedIndex = 1; // First-ever startup: 500 ms.
    currentMode = MODE_ASCII;
    EEPROM.update(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_VALUE);
    EEPROM.update(EEPROM_SPEED_ADDRESS, speedIndex);
    EEPROM.update(EEPROM_MODE_ADDRESS, currentMode);
  }

  bitTimeMs = SPEEDS[speedIndex];
  bitIntervalMicros = (uint32_t)(bitTimeMs * 1000UL);
}

void saveSettingsToEEPROM() {
  EEPROM.update(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_VALUE);
  EEPROM.update(EEPROM_SPEED_ADDRESS, speedIndex);
  EEPROM.update(EEPROM_MODE_ADDRESS, currentMode);
}

// ---------------- FRAME PROTOCOL (ASCII) ----------------
// Start marker: optical ON for 3 bit-times, OFF for 1 bit-time,
// then the bytes below. The long marker lets the receiver find frame start.
const byte PREAMBLE = 0xAA;
const byte SYNC_BYTE = 0x55;
const byte SOF = 0x02;
const byte EOF_BYTE = 0x03;
const byte MAX_LENGTH = 32;

// CRC-8/ATM (polynomial 0x07, initial value 0). Both boards use this.
byte crc8Update(byte crc, byte data) {
  crc ^= data;
  for (byte i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (byte)((crc << 1) ^ 0x07) : (byte)(crc << 1);
  }
  return crc;
}

// ---------------- BUZZER ----------------
void buzzerOutput() {
  tone(BUZZER_PIN, 2000); // 2 kHz beep
  delay(500);             // 0.5 second beep after final output
  noTone(BUZZER_PIN);
}

// ---------------- LCD / RECEIVER STATE ----------------
char receivedMessage[MAX_LENGTH + 1] = {0};
bool messageDisplayed = false;
bool settingsDisplayActive = false;   // speed / mode confirmation screen
unsigned long settingsDisplayStarted = 0;
unsigned long lastButtonPressed = 0;
bool resetDuringReception = false;
const unsigned long BUTTON_DEBOUNCE_MS = 220UL;
const unsigned long SETTINGS_SCREEN_MS = 3000UL;

enum ScreenState {
  HOME_SCREEN,
  READY_SCREEN,
  SIGNAL_SCREEN,
  MESSAGE_SCREEN,
  ERROR_SCREEN,
  SPEED_SCREEN,
  MODE_SCREEN
};
ScreenState currentScreen = HOME_SCREEN;

// ---------------- LDR POLARITY CALIBRATION ----------------
int idleRawLevel = HIGH;  // Replaced by measured idle level at startup.

bool lightDetected() {
  // During calibration the LEDs must be OFF. Light is the opposite
  // digital output level from the measured idle/dark level.
  return digitalRead(LDR_PIN) != idleRawLevel;
}

void calibrateLdr() {
  unsigned int highSamples = 0;
  const byte sampleCount = 101;

  Serial.println(F("LDR calibration: keep transmitter LEDs OFF"));
  for (byte i = 0; i < sampleCount; i++) {
    if (digitalRead(LDR_PIN) == HIGH) {
      highSamples++;
    }
    delay(5);
  }

  idleRawLevel = (highSamples > sampleCount / 2) ? HIGH : LOW;
  Serial.print(F("LDR idle DO level: "));
  Serial.println(idleRawLevel == HIGH ? F("HIGH") : F("LOW"));
  Serial.println(F("LDR polarity calibrated"));
}

// ---------------- LCD SCREENS ----------------
void displayHome() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("LiFi Project"));
  lcd.setCursor(0, 1);
  lcd.print(F("ID : 1241554"));
  currentScreen = HOME_SCREEN;
}

void displayReady() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ready: "));
  lcd.print(currentMode == MODE_MORSE ? F("MORSE") : F("ASCII"));
  lcd.setCursor(0, 1);
  lcd.print(F("Waitingforsignal"));
  currentScreen = READY_SCREEN;
}

void displayMessageSignal() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("LiFi Project"));
  lcd.setCursor(0, 1);
  lcd.print(F("Receiving Signal"));
  currentScreen = SIGNAL_SCREEN;
}

void displayMorseReceiving() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Receiving Morse"));
  currentScreen = SIGNAL_SCREEN;
}

// Live preview while a Morse message is arriving: last 16 decoded characters.
void displayMorseLive(byte length) {
  lcd.setCursor(0, 1);
  byte first = (length > 16) ? (length - 16) : 0;
  byte printed = 0;
  for (byte i = first; i < length; i++) {
    lcd.print(receivedMessage[i]);
    printed++;
  }
  while (printed < 16) {
    lcd.print(' ');
    printed++;
  }
}

void displayReceivedMessage() {
  lcd.clear();

  byte messageLength = strlen(receivedMessage);

  // First row: characters 1-16.
  lcd.setCursor(0, 0);
  for (byte i = 0; i < 16 && i < messageLength; i++) {
    lcd.print(receivedMessage[i]);
  }

  // Up to 16 characters: show label on row 2.
  if (messageLength <= 16) {
    lcd.setCursor(0, 1);
    lcd.print(currentMode == MODE_MORSE ? F("Morse Output") : F("Signal Output"));
  } else {
    // 17-32 characters: use both rows for the message.
    lcd.setCursor(0, 1);
    for (byte i = 16; i < 32 && i < messageLength; i++) {
      lcd.print(receivedMessage[i]);
    }
  }

  currentScreen = MESSAGE_SCREEN;
  messageDisplayed = true;
}

void displayError() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("RECEIVE ERROR"));
  lcd.setCursor(0, 1);
  lcd.print(F("CHECK SIGNAL"));
  currentScreen = ERROR_SCREEN;
  messageDisplayed = true;
}

void displaySpeed() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SPEED UPDATED"));
  lcd.setCursor(0, 1);
  lcd.print(bitTimeMs);
  lcd.print(F(" ms"));
  currentScreen = SPEED_SCREEN;
  settingsDisplayActive = true;
  settingsDisplayStarted = millis();
}

void displayMode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("MODE UPDATED"));
  lcd.setCursor(0, 1);
  lcd.print(currentMode == MODE_MORSE ? F("MORSE CODE") : F("ASCII"));
  currentScreen = MODE_SCREEN;
  settingsDisplayActive = true;
  settingsDisplayStarted = millis();
}

// ---------------- SERIAL COMMANDS TO LEONARDO ----------------
// SoftwareSerial sends settings from UNO D12/TX to Leonardo D0/RX1.
// The command lines "S500" and "M0" are the commands; the other
// human-readable lines are ignored by the Leonardo's strict parser.
// Both values are always sent together, so one lost line can never leave
// the two boards out of sync after the next button press.
void transmitSettingsToLeonardo() {
  linkSerial.print('S');
  linkSerial.println(bitTimeMs);
  linkSerial.print('M');
  linkSerial.println(currentMode);
  linkSerial.println(F("SYNC"));
}

void sendSettingsToLeonardo() {
  bitTimeMs = SPEEDS[speedIndex];
  bitIntervalMicros = (uint32_t)(bitTimeMs * 1000UL);

  saveSettingsToEEPROM();
  transmitSettingsToLeonardo();

  Serial.println(F("========== SETTINGS UPDATED =========="));
  Serial.print(F("TRANSMISSION SPEED: "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.print(F("TRANSMISSION MODE: "));
  Serial.println(currentMode == MODE_MORSE ? F("MORSE CODE") : F("ASCII"));
  Serial.println(F("STATUS: SETTINGS SAVED AND SENT"));
  Serial.println(F("======================================"));
}

// Leonardo requests a fresh settings snapshot whenever it boots.
void handleLeonardoRequests() {
  static char requestLine[12];
  static byte requestLength = 0;
  static bool requestOverflow = false;

  while (linkSerial.available() > 0) {
    char c = (char)linkSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      requestLine[requestLength] = '\0';
      if (!requestOverflow && strcmp(requestLine, "REQ") == 0) {
        transmitSettingsToLeonardo();
        Serial.println(F("LEONARDO REQUESTED SYNC - SETTINGS SENT"));
      }
      requestLength = 0;
      requestOverflow = false;
      continue;
    }

    if (requestLength < sizeof(requestLine) - 1) {
      requestLine[requestLength++] = c;
    } else {
      requestOverflow = true;
    }
  }
}

void increaseSpeed() {
  if (speedIndex < SPEED_COUNT - 1) {
    speedIndex++;
  }
  sendSettingsToLeonardo();
  displaySpeed();
}

void decreaseSpeed() {
  if (speedIndex > 0) {
    speedIndex--;
  }
  sendSettingsToLeonardo();
  displaySpeed();
}

void toggleMode() {
  currentMode = (currentMode == MODE_ASCII) ? MODE_MORSE : MODE_ASCII;
  sendSettingsToLeonardo();
  displayMode();
}

void resetReceiver() {
  // Tell Leonardo to abort its current transmission
  linkSerial.println(F("ABORT"));

  // Clear receiver state
  messageDisplayed = false;
  settingsDisplayActive = false;
  receivedMessage[0] = '\0';

  // Reset Morse decoder state
  morseResetState();

  digitalWrite(STATUS_LED, LOW);

  displayReady();

  Serial.println(F("SYSTEM RESET"));
  Serial.println(F("WAITING FOR SIGNAL"));
}

// Checks RESET inside blocking reception loops.
bool handleResetDuringReception() {
  if (digitalRead(BTN_RESET) != LOW) {
    return false;
  }

  resetDuringReception = true;
  resetReceiver();

  while (digitalRead(BTN_RESET) == LOW) {
    delay(5);
  }

  lastButtonPressed = millis();
  return true;
}

void checkSettingsScreenTimer() {
  if (settingsDisplayActive && millis() - settingsDisplayStarted >= SETTINGS_SCREEN_MS) {
    resetReceiver();
  }
}

// ---------------- BUTTON HANDLING ----------------
bool anyButtonPressed() {
  return digitalRead(BTN_PLUS) == LOW ||
         digitalRead(BTN_MINUS) == LOW ||
         digitalRead(BTN_HOME) == LOW ||
         digitalRead(BTN_RESET) == LOW ||
         digitalRead(BTN_MODE) == LOW;
}

void checkButtons() {
  if (millis() - lastButtonPressed < BUTTON_DEBOUNCE_MS) {
    return;
  }

  byte pressedPin = 0;
  if (digitalRead(BTN_PLUS) == LOW) {
    pressedPin = BTN_PLUS;
  } else if (digitalRead(BTN_MINUS) == LOW) {
    pressedPin = BTN_MINUS;
  } else if (digitalRead(BTN_HOME) == LOW) {
    pressedPin = BTN_HOME;
  } else if (digitalRead(BTN_RESET) == LOW) {
    pressedPin = BTN_RESET;
  } else if (digitalRead(BTN_MODE) == LOW) {
    pressedPin = BTN_MODE;
  }

  if (pressedPin == 0) {
    return;
  }

  lastButtonPressed = millis();

  switch (pressedPin) {
    case BTN_PLUS:
      increaseSpeed();
      break;
    case BTN_MINUS:
      decreaseSpeed();
      break;
    case BTN_HOME:
      settingsDisplayActive = false;
      displayHome();
      Serial.println(F("HOME SCREEN"));
      break;
    case BTN_RESET:
      resetReceiver();
      break;
    case BTN_MODE:
      toggleMode();
      break;
  }

  // Wait for release so a held button triggers only once.
  while (digitalRead(pressedPin) == LOW) {
    delay(5);
  }
}

// ======================================================
//                    ASCII RECEIVER
// ======================================================

// ---------------- MICROSECOND SAMPLING ----------------
uint32_t nextSampleMicros = 0;

bool waitUntilSampleTime() {
  while ((int32_t)(micros() - nextSampleMicros) < 0) {
    if (handleResetDuringReception()) {
      return false;
    }
  }

  return true;
}

bool receiveTimedByte(byte &value) {
  value = 0;

  for (byte bitNumber = 0; bitNumber < 8; bitNumber++) {
    if (!waitUntilSampleTime()) {
      return false;
    }

    value <<= 1;

    if (lightDetected()) {
      value |= 1;
    }

    nextSampleMicros += bitIntervalMicros;
  }

  return true;
}

// ---------------- WAIT FOR FRAME START ----------------
// Frame start marker is 3T of light followed by T of darkness.
// This avoids relying on a loop's accumulated polling delay to infer
// where the first data bit begins.
bool waitForDataStartAfterMarker() {
  const uint32_t timeoutUs = bitIntervalMicros * 3UL;
  const uint32_t started = micros();

  while (!lightDetected()) {
    if (handleResetDuringReception()) {
      return false;
    }

    if ((uint32_t)(micros() - started) >= timeoutUs) {
      return false;
    }
  }

  // Current optical transition is the first bit (MSB) of 0xAA.
  // Sample at its centre, then keep one shared schedule for the whole frame.
  nextSampleMicros = micros() + bitIntervalMicros / 2UL;
  return true;
}

bool detectAndValidateStartMarker() {
  // Called only after a transition from idle/dark to light is seen.
  const uint32_t markerStarted = micros();
  const uint32_t maximumMarkerUs = bitIntervalMicros * 5UL;

  while (lightDetected()) {
    if (handleResetDuringReception()) {
      return false;
    }

    if ((uint32_t)(micros() - markerStarted) >= maximumMarkerUs) {
      Serial.println(F("START MARKER TOO LONG"));
      return false;
    }
  }

  const uint32_t markerDuration = (uint32_t)(micros() - markerStarted);

  // Allow broad tolerance for LDR rise/fall response delay.
  if (markerDuration < bitIntervalMicros * 2UL ||
      markerDuration > bitIntervalMicros * 4UL) {
    return false;
  }

  return true;
}

// ---------------- FINAL SERIAL REPORT ----------------
void printFinalOutput(byte length, byte expectedChecksum, byte actualChecksum) {
  Serial.println();
  Serial.println(F("================================="));
  Serial.println(F("          LiFi FINAL OUTPUT"));
  Serial.println(F("================================="));
  Serial.println(F("PROJECT ID       : 1241554"));
  Serial.println(F("DEVICE           : ARDUINO UNO"));
  Serial.println(F("FUNCTION         : RECEIVER"));
  Serial.println(F("MODE             : ASCII"));
  Serial.print(F("BIT TIME         : "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.print(F("MESSAGE LENGTH   : "));
  Serial.println(length);
  Serial.print(F("EXPECTED CHECKSUM: 0x"));
  Serial.println(expectedChecksum, HEX);
  Serial.print(F("RECEIVED CHECKSUM: 0x"));
  Serial.println(actualChecksum, HEX);
  Serial.print(F("FINAL MESSAGE    : "));
  Serial.println(receivedMessage);
  Serial.println(F("CHECKSUM STATUS  : VALID"));
  Serial.println(F("FRAME STATUS     : VALID"));
  Serial.println(F("RECEPTION STATUS : SUCCESS"));
  Serial.println(F("================================="));
}

// ---------------- RECEIVE ONE COMPLETE FRAME ----------------
void receiveFrame() {
  digitalWrite(STATUS_LED, HIGH);

  // No LCD operations or Serial printing during byte reception.
  byte preamble, sync, start, length;

  if (!receiveTimedByte(preamble) ||
      !receiveTimedByte(sync) ||
      !receiveTimedByte(start) ||
      !receiveTimedByte(length)) {
    digitalWrite(STATUS_LED, LOW);
    return; // RESET was pressed; keep the Ready screen.
  }

  if (preamble != PREAMBLE || sync != SYNC_BYTE ||
      start != SOF || length == 0 || length > MAX_LENGTH) {
    digitalWrite(STATUS_LED, LOW);
    Serial.println(F("HEADER ERROR"));
    Serial.print(F("PREAMBLE: 0x")); Serial.println(preamble, HEX);
    Serial.print(F("SYNC: 0x")); Serial.println(sync, HEX);
    Serial.print(F("SOF: 0x")); Serial.println(start, HEX);
    Serial.print(F("LENGTH: ")); Serial.println(length);
    displayError();
    return;
  }

  byte checksum = crc8Update(0, length);
  for (byte i = 0; i < length; i++) {
    byte receivedByte;

    if (!receiveTimedByte(receivedByte)) {
      digitalWrite(STATUS_LED, LOW);
      return; // RESET was pressed; keep the Ready screen.
    }

    receivedMessage[i] = (char)receivedByte;
    checksum = crc8Update(checksum, receivedByte);
  }
  receivedMessage[length] = '\0';

  byte receivedChecksum, ending;

  if (!receiveTimedByte(receivedChecksum) ||
      !receiveTimedByte(ending)) {
    digitalWrite(STATUS_LED, LOW);
    return; // RESET was pressed; keep the Ready screen.
  }

  digitalWrite(STATUS_LED, LOW);

  Serial.println();
  Serial.println(F("========== HEADER RECEIVED =========="));
  Serial.print(F("Preamble: 0x")); Serial.println(preamble, HEX);
  Serial.print(F("Sync: 0x")); Serial.println(sync, HEX);
  Serial.print(F("SOF: 0x")); Serial.println(start, HEX);
  Serial.print(F("Length: ")); Serial.println(length);
  Serial.println(F("HEADER VALID"));
  Serial.println(F("RECEIVING PAYLOAD COMPLETED"));

  if (checksum == receivedChecksum && ending == EOF_BYTE) {
    printFinalOutput(length, checksum, receivedChecksum);
    displayReceivedMessage();
    buzzerOutput();
  } else {
    Serial.println();
    Serial.println(F("========== RECEPTION ERROR =========="));
    Serial.print(F("EXPECTED CHECKSUM: 0x")); Serial.println(checksum, HEX);
    Serial.print(F("RECEIVED CHECKSUM: 0x")); Serial.println(receivedChecksum, HEX);
    Serial.print(F("END BYTE RECEIVED: 0x")); Serial.println(ending, HEX);
    Serial.print(F("EXPECTED END BYTE: 0x")); Serial.println(EOF_BYTE, HEX);
    Serial.print(F("RECEIVED MESSAGE: ")); Serial.println(receivedMessage);
    if (checksum != receivedChecksum) Serial.println(F("ERROR: CHECKSUM MISMATCH"));
    if (ending != EOF_BYTE) Serial.println(F("ERROR: INVALID END BYTE"));
    Serial.println(F("====================================="));
    displayError();
  }
}

// ======================================================
//                    MORSE RECEIVER
// ======================================================
// The Morse receiver measures how long the light stays ON and OFF:
//
//   ON  :  < 2T = dot      >= 2T = dash      5T..9T = start marker
//   OFF :  >= 2T = end of letter    >= 5T = word gap    >= 12T = timeout
//
// With a normal 3T dash / 1T dot the thresholds sit half way between the
// nominal values, so the LDR's slow rise/fall time cannot confuse them.

// Same table as the transmitter. One byte per character: leading 1
// (sentinel) then 0 = dot, 1 = dash.
const char MORSE_CHARS[] PROGMEM = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,?'!/()&:;=+-\"@";
const uint8_t MORSE_CODES[] PROGMEM = {
  0b101,     // A  .-
  0b11000,   // B  -...
  0b11010,   // C  -.-.
  0b1100,    // D  -..
  0b10,      // E  .
  0b10010,   // F  ..-.
  0b1110,    // G  --.
  0b10000,   // H  ....
  0b100,     // I  ..
  0b10111,   // J  .---
  0b1101,    // K  -.-
  0b10100,   // L  .-..
  0b111,     // M  --
  0b110,     // N  -.
  0b1111,    // O  ---
  0b10110,   // P  .--.
  0b11101,   // Q  --.-
  0b1010,    // R  .-.
  0b1000,    // S  ...
  0b11,      // T  -
  0b1001,    // U  ..-
  0b10001,   // V  ...-
  0b1011,    // W  .--
  0b11001,   // X  -..-
  0b11011,   // Y  -.--
  0b11100,   // Z  --..
  0b111111,  // 0  -----
  0b101111,  // 1  .----
  0b100111,  // 2  ..---
  0b100011,  // 3  ...--
  0b100001,  // 4  ....-
  0b100000,  // 5  .....
  0b110000,  // 6  -....
  0b111000,  // 7  --...
  0b111100,  // 8  ---..
  0b111110,  // 9  ----.
  0b1010101, // .  .-.-.-
  0b1110011, // ,  --..--
  0b1001100, // ?  ..--..
  0b1011110, // '  .----.
  0b1101011, // !  -.-.--
  0b110010,  // /  -..-.
  0b110110,  // (  -.--.
  0b1101101, // )  -.--.-
  0b101000,  // &  .-...
  0b1111000, // :  ---...
  0b1101010, // ;  -.-.-.
  0b110001,  // =  -...-
  0b101010,  // +  .-.-.
  0b1100001, // -  -....-
  0b1010010, // "  .-..-.
  0b1011010  // @  .--.-.
};
const byte MORSE_COUNT = sizeof(MORSE_CODES) / sizeof(MORSE_CODES[0]);

// End-of-message prosign  ...-.-  (SK)
const uint8_t MORSE_END_CODE = 0b1000101;
const byte MORSE_MAX_ELEMENTS = 7;

// Decoder state
byte morseLength = 0;            // decoded characters stored in receivedMessage
uint8_t morseCode = 1;           // letter being built (leading 1 = sentinel)
byte morseElements = 0;          // dots/dashes in the current letter
bool morseLetterTooLong = false; // current letter has more than 7 elements
byte morseUnknown = 0;           // symbols that matched no character
bool morseEndSeen = false;       // end prosign received
bool morseOverflow = false;      // more than 32 characters decoded

void morseResetState() {
  morseLength = 0;
  morseCode = 1;
  morseElements = 0;
  morseLetterTooLong = false;
  morseUnknown = 0;
  morseEndSeen = false;
  morseOverflow = false;
  receivedMessage[0] = '\0';
}

char morseCodeToChar(uint8_t code) {
  for (byte i = 0; i < MORSE_COUNT; i++) {
    if (pgm_read_byte(&MORSE_CODES[i]) == code) {
      return (char)pgm_read_byte(&MORSE_CHARS[i]);
    }
  }
  return '\0';
}

void morseAppendChar(char c) {
  if (morseLength >= MAX_LENGTH) {
    morseOverflow = true;
    return;
  }
  receivedMessage[morseLength++] = c;
  receivedMessage[morseLength] = '\0';
  if (c != ' ') {
    displayMorseLive(morseLength);   // happens in the dark gap after a letter
  }
}

void morseAddElement(bool dash) {
  if (morseElements >= MORSE_MAX_ELEMENTS) {
    morseLetterTooLong = true;
    return;
  }
  morseCode = (uint8_t)((morseCode << 1) | (dash ? 1 : 0));
  morseElements++;
}

// Called when the darkness after a pulse is long enough to end a letter.
void morseFlushLetter() {
  if (morseElements == 0) {
    return;
  }

  if (morseLetterTooLong) {
    morseUnknown++;
    morseAppendChar('?');
  } else if (morseCode == MORSE_END_CODE) {
    morseEndSeen = true;
  } else {
    char c = morseCodeToChar(morseCode);
    if (c == '\0') {
      morseUnknown++;
      c = '?';
    }
    morseAppendChar(c);
  }

  morseCode = 1;
  morseElements = 0;
  morseLetterTooLong = false;
}

void morseAppendWordGap() {
  if (morseLength > 0 && receivedMessage[morseLength - 1] != ' ') {
    morseAppendChar(' ');
  }
}

void printMorseOutput(bool success, bool stuck, bool timeout) {
  Serial.println();
  if (success) {
    Serial.println(F("================================="));
    Serial.println(F("          LiFi FINAL OUTPUT"));
    Serial.println(F("================================="));
    Serial.println(F("PROJECT ID       : 1241554"));
    Serial.println(F("DEVICE           : ARDUINO UNO"));
    Serial.println(F("FUNCTION         : RECEIVER"));
    Serial.println(F("MODE             : MORSE CODE"));
    Serial.print(F("UNIT TIME        : "));
    Serial.print(bitTimeMs);
    Serial.println(F(" ms"));
    Serial.print(F("MESSAGE LENGTH   : "));
    Serial.println(morseLength);
    Serial.print(F("FINAL MESSAGE    : "));
    Serial.println(receivedMessage);
    Serial.println(F("END MARKER       : RECEIVED"));
    Serial.println(F("UNKNOWN SYMBOLS  : 0"));
    Serial.println(F("FRAME STATUS     : VALID"));
    Serial.println(F("RECEPTION STATUS : SUCCESS"));
    Serial.println(F("================================="));
  } else {
    Serial.println(F("========== RECEPTION ERROR =========="));
    Serial.println(F("MODE: MORSE CODE"));
    Serial.print(F("RECEIVED MESSAGE: "));
    Serial.println(receivedMessage);
    Serial.print(F("UNKNOWN SYMBOLS: "));
    Serial.println(morseUnknown);
    Serial.print(F("END MARKER: "));
    Serial.println(morseEndSeen ? F("RECEIVED") : F("MISSING"));
    if (!morseEndSeen) Serial.println(F("ERROR: END MARKER NOT RECEIVED"));
    if (morseUnknown > 0) Serial.println(F("ERROR: UNKNOWN MORSE SYMBOL (SHOWN AS ?)"));
    if (morseOverflow) Serial.println(F("ERROR: MORE THAN 32 CHARACTERS"));
    if (stuck) Serial.println(F("ERROR: LIGHT STAYED ON TOO LONG"));
    if (timeout) Serial.println(F("INFO: SIGNAL ENDED (TIMEOUT)"));
    if (morseLength == 0) Serial.println(F("ERROR: EMPTY MESSAGE"));
    Serial.println(F("====================================="));
  }
}

// Called when light is seen while idle and the receiver is in MORSE mode.
void receiveMorseMessage() {
  const uint32_t T = bitIntervalMicros;

  // ---- 1. The first light pulse must be the 7T start marker.
  const uint32_t pulseStart = micros();
  while (lightDetected()) {
    if (anyButtonPressed()) {
      return;
    }
    if ((uint32_t)(micros() - pulseStart) > T * 10UL) {
      Serial.println(F("MORSE: LIGHT ON TOO LONG - IGNORED"));
      while (lightDetected() && !anyButtonPressed()) {
        // wait for darkness again
      }
      return;
    }
  }
  const uint32_t pulseUs = (uint32_t)(micros() - pulseStart);

  if (pulseUs < T * 5UL || pulseUs > T * 9UL) {
    // Short blips are ignored silently. A 2T+ pulse that is not a Morse
    // marker usually means the transmitter is still in ASCII mode.
    static bool hintShown = false;
    static unsigned long lastHint = 0;
    if (pulseUs >= T * 2UL && (!hintShown || millis() - lastHint > 10000UL)) {
      Serial.println(F("MORSE: PULSE IGNORED - NOT A MORSE START MARKER"));
      Serial.println(F("       CHECK THAT BOTH BOARDS ARE IN THE SAME MODE"));
      hintShown = true;
      lastHint = millis();
    }
    return;
  }

  // ---- 2. Valid start marker: start receiving.
  digitalWrite(STATUS_LED, HIGH);
  morseResetState();
  displayMorseReceiving();

  Serial.println();
  Serial.println(F("========== MORSE START MARKER =========="));
  Serial.print(F("UNIT TIME: "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.println(F("RECEIVING MORSE SIGNAL"));

  bool letterDone = false;
  bool wordDone = false;
  bool timeout = false;
  bool aborted = false;
  bool stuck = false;
  uint32_t darkStart = micros();

  for (;;) {
    // ---- 3a. Light is OFF: wait for the next pulse. While waiting,
    //          close the letter (2T) and mark a word gap (5T).
    while (!lightDetected()) {
      if (anyButtonPressed()) {
        aborted = true;
        break;
      }

      const uint32_t dark = (uint32_t)(micros() - darkStart);

      if (!letterDone && dark >= T * 2UL) {
        letterDone = true;
        morseFlushLetter();
        if (morseEndSeen) {
          break;                       // end prosign: message complete
        }
      }
      if (!wordDone && dark >= T * 5UL) {
        wordDone = true;
        morseAppendWordGap();
      }
      if (dark >= T * 12UL) {
        timeout = true;
        break;
      }
    }
    if (aborted || timeout || morseEndSeen) {
      break;
    }

    // ---- 3b. Light is ON: measure how long.
    const uint32_t onStart = micros();
    while (lightDetected()) {
      if (anyButtonPressed()) {
        aborted = true;
        break;
      }
      if ((uint32_t)(micros() - onStart) > T * 5UL) {   // longer than any dash
        stuck = true;
        break;
      }
    }
    if (aborted || stuck) {
      break;
    }

    const uint32_t onUs = (uint32_t)(micros() - onStart);

    if (onUs < T / 4UL) {
      continue;                        // tiny glitch: ignore, keep dark timer
    }

    darkStart = micros();              // falling edge
    letterDone = false;
    wordDone = false;
    morseAddElement(onUs >= T * 2UL);  // 2T or longer = dash
  }

  digitalWrite(STATUS_LED, LOW);

  if (aborted) {
    Serial.println(F("MORSE RECEPTION ABORTED BY BUTTON"));
    resetReceiver();
    return;
  }

  morseFlushLetter();                  // no-op unless a letter is still open

  while (morseLength > 0 && receivedMessage[morseLength - 1] == ' ') {
    morseLength--;
  }
  receivedMessage[morseLength] = '\0';

  const bool success = morseEndSeen && !stuck && !morseOverflow &&
                       morseUnknown == 0 && morseLength > 0;

  printMorseOutput(success, stuck, timeout);

  if (success) {
    displayReceivedMessage();
    buzzerOutput();
  } else {
    displayError();
  }
}

// ---------------- OPTICAL LISTENER ----------------
void checkOpticalSignal() {
  if (messageDisplayed || settingsDisplayActive) {
    return;
  }

  // Wait while the sensor is at its calibrated idle level.
  if (!lightDetected()) {
    return;
  }

  resetDuringReception = false;

  // MORSE mode has its own receiver.
  if (currentMode == MODE_MORSE) {
    receiveMorseMessage();
    return;
  }

  // ---- ASCII mode ----
  // A light transition begins the special 3T start marker.
  if (!detectAndValidateStartMarker()) {
    return;
  }

  // Marker has ended; one-bit dark gap follows. Show the existing
  // signal screen while waiting for the first preamble bit.
  displayMessageSignal();

  if (!waitForDataStartAfterMarker()) {
    if (!resetDuringReception) {
      Serial.println(F("ERROR: PREAMBLE START NOT DETECTED"));
      displayError();
    }
    return;
  }

  // Receive header, payload, checksum and end byte as one timed frame.
  receiveFrame();
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(LDR_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BTN_PLUS, INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);
  pinMode(BTN_HOME, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(9600);
  linkSerial.begin(9600);
  linkSerial.listen();
  loadSavedSettings();

  lcd.init();
  lcd.backlight();

  // Keep transmitter LEDs OFF during the LDR idle-level calibration.
  calibrateLdr();

  displayHome();

  sendSettingsToLeonardo();

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("          LiFi PROJECT"));
  Serial.println(F("          UNO RECEIVER"));
  Serial.println(F("================================"));
  Serial.println(F("PROJECT ID: 1241554"));
  Serial.print(F("RESTORED SPEED: "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.print(F("RESTORED MODE: "));
  Serial.println(currentMode == MODE_MORSE ? F("MORSE CODE") : F("ASCII"));
  Serial.println(F("STATUS: READY"));
}

// ---------------- MAIN LOOP ----------------
void loop() {
  handleLeonardoRequests();
  checkButtons();
  checkSettingsScreenTimer();
  checkOpticalSignal();
}
