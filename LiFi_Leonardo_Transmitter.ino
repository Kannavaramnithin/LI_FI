/*
  =====================================================
                  LiFi PROJECT
          ARDUINO LEONARDO - TRANSMITTER
              (ASCII MODE + MORSE MODE)
  =====================================================

  LED 1 anode (+) through 220 ohm resistor -> D9
  LED 1 cathode (-)                        -> GND
  LED 2 anode (+) through 220 ohm resistor -> D10
  LED 2 cathode (-)                        -> GND

  UNO D12/TX       -> LEONARDO D0/RX1
  LEONARDO D1/TX1  -> UNO D13/RX
  UNO GND          -> LEONARDO GND

  USB Serial Monitor: 9600 baud, send newline.
  Project ID: 1241554
  Maximum message: 32 characters.

  COMMANDS FROM THE UNO (one complete line each):
    S200 / S300 / S500 / S1000   -> bit / unit time in ms
    M0                           -> ASCII mode
    M1                           -> MORSE mode
    SYNC                         -> complete settings snapshot received

  Leonardo sends REQ periodically until it receives SYNC.

  ------------------------------------------------------
  ASCII FRAME
  ------------------------------------------------------
  LEDs OFF for 3T (idle)
  LEDs ON  for 3T (start marker)
  LEDs OFF for 1T (gap)
  0xAA, 0x55, 0x02, length, payload, CRC-8 checksum, 0x03
  (T = bit time, 1 bit per T, MSB first)

  ------------------------------------------------------
  MORSE FRAME   (T = unit time = the selected speed)
  ------------------------------------------------------
  LEDs OFF for 3T (idle)
  LEDs ON  for 7T (start marker)
  LEDs OFF for 3T (gap)
  letters...                        dot  = ON 1T
                                    dash = ON 3T
                                    gap inside a letter = OFF 1T
                                    gap between letters = OFF 3T
                                    gap between words   = OFF 7T
  end prosign  ...-.-  (SK)  -> tells the receiver the message is complete
  LEDs OFF

  Supported Morse characters:
  A-Z  0-9  space  . , ? ' ! / ( ) & : ; = + - " @
  (lower case is converted to upper case, extra spaces are collapsed)
*/

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <avr/pgmspace.h>

const byte LED1_PIN = 9;
const byte LED2_PIN = 10;

unsigned long bitTimeMs = 500UL; // Placeholder until UNO settings arrive.
bool morseMode = false;
bool synchronizedWithUno = false;
unsigned long lastSyncRequest = 0;
const unsigned long SYNC_REQUEST_INTERVAL_MS = 1000UL;

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

// ---------------- MORSE TABLE ----------------
// Each code is one byte: a leading 1 (sentinel) followed by one bit per
// element, 0 = dot and 1 = dash, first element first.
//   A  .-   -> 1 0 1 -> 0b101
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

// ---------------- COMMAND INPUT FROM UNO ----------------
// UNO sends complete lines: S200, S300, S500, S1000, M0 or M1.
char command[12];
byte commandLength = 0;
bool commandOverflow = false;
bool abortRequested = false;

// Non-blocking USB message input.
char messageBuffer[MAX_LENGTH + 1];
byte messageLength = 0;
bool messageOverflow = false;

// ---------------- LED HELPERS ----------------
void ledsOn() {
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
}

void ledsOff() {
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
}

// Light ON / OFF for a number of unit times (Morse timing).
bool lightOn(byte units) {
  ledsOn();
  return waitWithAbort(bitTimeMs * units);
}

bool lightOff(byte units) {
  ledsOff();
  return waitWithAbort(bitTimeMs * units);
}

// ---------------- COMMAND HANDLING ----------------
bool isAllowedSpeed(unsigned long value) {
  return value == 200UL || value == 300UL ||
         value == 500UL || value == 1000UL;
}

// Accept only exact "S<digits>" or "M0"/"M1" lines. All the other UNO
// Serial Monitor status text arriving on RX1 is ignored.
void processCommand() {
    command[commandLength] = '\0';

  if (!commandOverflow && strcmp(command, "ABORT") == 0) {
    abortRequested = true;
    ledsOff();

    commandLength = 0;
    commandOverflow = false;

    Serial.println(F("TRANSMISSION ABORTED"));
    Serial.println(F("STATUS: READY TO TRANSMIT"));
    return;
  }

  if (!commandOverflow && strcmp(command, "SYNC") == 0) {
    synchronizedWithUno = true;
    commandLength = 0;
    commandOverflow = false;
    Serial.println(F("UNO SETTINGS SYNCHRONIZED"));
    Serial.print(F("CURRENT SPEED: "));
    Serial.print(bitTimeMs);
    Serial.println(F(" ms"));
    Serial.print(F("CURRENT MODE: "));
    Serial.println(morseMode ? F("MORSE CODE") : F("ASCII"));
    Serial.println(F("STATUS: READY TO TRANSMIT"));
    return;
  }

  if (commandOverflow || commandLength < 2 ||
      (command[0] != 'S' && command[0] != 'M')) {
    commandLength = 0;
    commandOverflow = false;
    return;
  }

  command[commandLength] = '\0';

  bool digitsOnly = true;
  for (byte i = 1; i < commandLength; i++) {
    if (command[i] < '0' || command[i] > '9') {
      digitsOnly = false;
      break;
    }
  }

  if (digitsOnly) {
    unsigned long value = atol(command + 1);

    if (command[0] == 'S' && isAllowedSpeed(value)) {
      bitTimeMs = value;
      Serial.println();
      Serial.println(F("========== SPEED SYNCHRONIZED =========="));
      Serial.print(F("CURRENT SPEED: "));
      Serial.print(bitTimeMs);
      Serial.println(F(" ms"));
      Serial.println(F("STATUS: READY TO TRANSMIT"));
      Serial.println(F("========================================"));
    } else if (command[0] == 'M' && commandLength == 2 && value <= 1) {
      morseMode = (value == 1);
      Serial.println();
      Serial.println(F("========== MODE SYNCHRONIZED =========="));
      Serial.print(F("CURRENT MODE: "));
      Serial.println(morseMode ? F("MORSE CODE") : F("ASCII"));
      Serial.println(F("STATUS: READY TO TRANSMIT"));
      Serial.println(F("======================================="));
    }
  }

  commandLength = 0;
  commandOverflow = false;
}

void receiveCommands() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();

    if (c == '\n' || c == '\r') {
      if (commandLength > 0 || commandOverflow) {
        processCommand();
      }
      continue;
    }

    if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = c;
    } else {
      commandOverflow = true;
    }
  }
}

// Ask the UNO for its saved settings until a complete SYNC snapshot arrives.
void requestSyncFromUno() {
  if (synchronizedWithUno) return;

  if (millis() - lastSyncRequest >= SYNC_REQUEST_INTERVAL_MS) {
    Serial1.println(F("REQ"));
    lastSyncRequest = millis();
    Serial.println(F("REQUESTING SETTINGS FROM UNO..."));
  }
}

bool waitWithAbort(unsigned long durationMs) {
  unsigned long started = millis();

  while (millis() - started < durationMs) {
    receiveCommands();

    if (abortRequested) {
      ledsOff();
      return false;
    }

    delay(1);
  }

  return true;
}

// ======================================================
//                    ASCII TRANSMITTER
// ======================================================
bool sendBit(byte bitValue) {
  if (bitValue) {
    ledsOn();
  } else {
    ledsOff();
  }

  return waitWithAbort(bitTimeMs);
}

bool sendByte(byte value) {
  for (int bit = 7; bit >= 0; bit--) {
    if (!sendBit((value >> bit) & 0x01)) {
      return false;
    }
  }

  return true;
}
void transmitAsciiMessage(const char *text) {
  byte length = strlen(text);

  byte checksum = crc8Update(0, length);
  for (byte i = 0; i < length; i++) {
    checksum = crc8Update(checksum, (byte)text[i]);
  }

  Serial.println();
  Serial.println(F("========== LiFi TRANSMISSION =========="));
  Serial.println(F("PROJECT ID: 1241554"));
  Serial.println(F("MODE: ASCII"));
  Serial.print(F("MESSAGE: "));
  Serial.println(text);
  Serial.print(F("SPEED: "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.print(F("LENGTH: "));
  Serial.println(length);
  Serial.println(F("STATUS: TRANSMITTING"));

  // Optical frame marker.
  // Optical frame marker.
  ledsOff();
  if (!waitWithAbort(bitTimeMs * 3UL)) return;

  ledsOn();
  if (!waitWithAbort(bitTimeMs * 3UL)) return;

  ledsOff();
  if (!waitWithAbort(bitTimeMs)) return;
  // Data frame begins at a fresh OFF->ON transition (first bit of 0xAA).
  if (!sendByte(PREAMBLE)) return;
  if (!sendByte(SYNC_BYTE)) return;
  if (!sendByte(SOF)) return;
  if (!sendByte(length)) return;

  for (byte i = 0; i < length; i++) {
    if (!sendByte((byte)text[i])) return;
  }

  if (!sendByte(checksum)) return;
  if (!sendByte(EOF_BYTE)) return;

  ledsOff();

  Serial.println(F("STATUS: TRANSMISSION COMPLETED"));
  Serial.println(F("========================================"));
}

// ======================================================
//                    MORSE TRANSMITTER
// ======================================================
bool findMorseCode(char c, uint8_t &code) {
  for (byte i = 0; i < MORSE_COUNT; i++) {
    if ((char)pgm_read_byte(&MORSE_CHARS[i]) == c) {
      code = pgm_read_byte(&MORSE_CODES[i]);
      return true;
    }
  }
  return false;
}

// Position of the sentinel (leading 1) bit.
int8_t morseTopBit(uint8_t code) {
  int8_t top = 7;
  while (top > 0 && !((code >> top) & 1)) {
    top--;
  }
  return top;
}

// Sends one letter. Every element is followed by 1T of darkness, so the
// caller only adds the EXTRA darkness needed for letter/word gaps.
bool sendMorseCode(uint8_t code) {
  for (int8_t bit = morseTopBit(code) - 1; bit >= 0; bit--) {
    bool dash = (code >> bit) & 1;

    if (!lightOn(dash ? 3 : 1)) {
      return false;
    }

    if (!lightOff(1)) {
      return false;
    }
  }

  return true;
}

void printMorsePattern(uint8_t code) {
  for (int8_t bit = morseTopBit(code) - 1; bit >= 0; bit--) {
    Serial.print(((code >> bit) & 1) ? '-' : '.');
  }
}

void transmitMorseMessage(const char *text) {
  // Clean up the text: upper case, no leading/trailing spaces,
  // repeated spaces collapsed to one word gap.
  char clean[MAX_LENGTH + 1];
  byte cleanLength = 0;
  bool pendingSpace = false;

  for (byte i = 0; text[i] != '\0'; i++) {
    char c = (char)toupper((unsigned char)text[i]);

    if (c == ' ') {
      if (cleanLength > 0) {
        pendingSpace = true;
      }
      continue;
    }

    uint8_t code;
    if (!findMorseCode(c, code)) {
      Serial.print(F("ERROR: CHARACTER NOT SUPPORTED IN MORSE: "));
      Serial.println(text[i]);
      Serial.println(F("SUPPORTED: A-Z 0-9 SPACE . , ? ' ! / ( ) & : ; = + - \" @"));
      return;
    }

    if (pendingSpace) {
      clean[cleanLength++] = ' ';
      pendingSpace = false;
    }
    clean[cleanLength++] = c;
  }
  clean[cleanLength] = '\0';

  if (cleanLength == 0) {
    Serial.println(F("ERROR: NOTHING TO SEND (ONLY SPACES)"));
    return;
  }

  Serial.println();
  Serial.println(F("========== LiFi TRANSMISSION =========="));
  Serial.println(F("PROJECT ID: 1241554"));
  Serial.println(F("MODE: MORSE CODE"));
  Serial.print(F("MESSAGE: "));
  Serial.println(clean);
  Serial.print(F("PATTERN: "));
  for (byte i = 0; i < cleanLength; i++) {
    if (clean[i] == ' ') {
      Serial.print(F("/ "));
    } else {
      uint8_t code;
      findMorseCode(clean[i], code);
      printMorsePattern(code);
      Serial.print(' ');
    }
  }
  Serial.println();
  Serial.print(F("UNIT TIME: "));
  Serial.print(bitTimeMs);
  Serial.println(F(" ms"));
  Serial.print(F("LENGTH: "));
  Serial.println(cleanLength);
  Serial.println(F("STATUS: TRANSMITTING"));

  // Start marker: idle 3T dark, 7T light, 3T dark.
  if (!lightOff(3)) return;
  if (!lightOn(7)) return;
  if (!lightOff(3)) return;

  bool firstLetter = true;
  bool wordGapPending = false;

  for (byte i = 0; i < cleanLength; i++) {
    if (clean[i] == ' ') {
      wordGapPending = true;
      continue;
    }

    uint8_t code;
    findMorseCode(clean[i], code);

    // The previous element already left 1T of darkness behind it.
    // Letter gap = 3T in total, word gap = 7T in total.
if (!firstLetter) {
  if (!lightOff(wordGapPending ? 6 : 2)) return;
}

if (!sendMorseCode(code)) return;
    firstLetter = false;
    wordGapPending = false;
  }

  // End prosign, separated from the last letter by a normal letter gap.
if (!lightOff(2)) return;
if (!sendMorseCode(MORSE_END_CODE)) return;

  ledsOff();

  Serial.println(F("STATUS: TRANSMISSION COMPLETED"));
  Serial.println(F("========================================"));
}

// ======================================================
//                    MESSAGE ENTRY
// ======================================================
void transmitMessage(const char *text) {
  abortRequested = false;

  if (!synchronizedWithUno) {
    Serial.println(F("ERROR: NOT SYNCHRONIZED WITH UNO"));
    Serial.println(F("WAITING FOR UNO SETTINGS..."));
    return;
  }

  byte length = strlen(text);

  if (length == 0 || length > MAX_LENGTH) {
    Serial.println(F("ERROR: MESSAGE MUST BE 1 TO 32 CHARACTERS"));
    return;
  }

  if (morseMode) {
    transmitMorseMessage(text);
  } else {
    transmitAsciiMessage(text);
  }
}

// Read a line from the USB Serial Monitor without blocking the loop,
// so commands from the UNO can still be processed while typing.
void readUsbMessage() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      messageBuffer[messageLength] = '\0';

      if (messageOverflow) {
        Serial.println(F("ERROR: MAXIMUM 32 CHARACTERS"));
      } else if (messageLength > 0) {
        transmitMessage(messageBuffer);
        if (abortRequested) {
  abortRequested = false;
  messageLength = 0;
  messageOverflow = false;
  messageBuffer[0] = '\0';
  ledsOff();
  return;
}
      }

      messageLength = 0;
      messageOverflow = false;
      messageBuffer[0] = '\0';
      continue;
    }

    if (messageLength < MAX_LENGTH) {
      messageBuffer[messageLength++] = c;
    } else {
      messageOverflow = true;
    }
  }
}

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  ledsOff();

  Serial.begin(9600);   // USB Serial Monitor
  Serial1.begin(9600);  // Hardware RX1 on D0 from UNO D12/TX

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("          LiFi PROJECT"));
  Serial.println(F("       LEONARDO TRANSMITTER"));
  Serial.println(F("================================"));
  Serial.println(F("PROJECT ID: 1241554"));
  Serial.println(F("MAXIMUM MESSAGE: 32 CHARACTERS"));
  Serial.println(F("STATUS: WAITING FOR UNO SYNCHRONIZATION"));
}

void loop() {
  receiveCommands();
  requestSyncFromUno();
  readUsbMessage();
}
