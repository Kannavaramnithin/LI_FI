# Li-Fi Communication System (ASCII & Morse Code)

A dual-mode optical wireless communication system using light (Li-Fi) built with **Arduino Leonardo** (Transmitter) and **Arduino Uno** (Receiver). The system supports both **binary-framed ASCII data transmission** (with CRC-8 verification) and **Morse code optical transmission**.

---

## 📌 Features

- **Dual Mode Transmission**:
  - **ASCII Mode**: Packets framed with Preamble (`0xAA`), Sync (`0x55`), Start-of-Frame (`0x02`), length, payload, CRC-8/ATM checksum, and End-of-Frame (`0x03`).
  - **Morse Code Mode**: Timing-standard Morse optical signalling (dot, dash, element/character/word gaps) ending with the `...-.-` (SK) prosign.
- **Configurable Speed Levels**:
  - 1000 ms, 500 ms (default), 300 ms, 200 ms per bit / timing unit.
- **EEPROM Storage**:
  - Speed and mode choices are persistently stored on the Arduino Uno and synced to Leonardo on boot.
- **Interactive UI**:
  - 16x2 I2C LCD displaying live status, mode, speed, and received messages.
  - 5 hardware pushbuttons (Speed +, Speed -, Home, Reset, Mode Toggle).
  - Status LED and piezo buzzer tone indicator.
- **Companion Mobile App**:
  - `LI_FI.apk` included for Android integration.

---

## 🔌 Circuit Connection Guide

### 1. Arduino Leonardo (Transmitter Unit)

The transmitter uses high-brightness LEDs to emit optical pulses based on text sent via the USB Serial Monitor.

| Component | Pin on Leonardo | Description / Notes |
| :--- | :--- | :--- |
| **LED 1 (Anode +)** | **D9** | Through a **220 Ω** current-limiting resistor |
| **LED 1 (Cathode -)** | **GND** | Ground |
| **LED 2 (Anode +)** | **D10** | Through a **220 Ω** current-limiting resistor |
| **LED 2 (Cathode -)** | **GND** | Ground |
| **Hardware Serial RX1** | **D0 (RX1)** | Connects to Arduino Uno **D12 (TX)** |
| **Hardware Serial TX1** | **D1 (TX1)** | Connects to Arduino Uno **D13 (RX)** |
| **Ground** | **GND** | Connects to Arduino Uno **GND** (Common Ground) |

---

### 2. Arduino Uno (Receiver Unit)

The receiver monitors light transitions using an LDR sensor module, decodes incoming ASCII packets or Morse signals, displays information on the LCD, and controls communication speed and modes.

| Component / Function | Pin on Arduino Uno | Connection Notes |
| :--- | :--- | :--- |
| **LDR Sensor DO (Digital Out)** | **D8** | LDR digital threshold output pin |
| **LDR Sensor VCC** | **5V** | Power (5V) |
| **LDR Sensor GND** | **GND** | Ground |
| **16x2 I2C LCD SDA** | **A4** | I2C Data line |
| **16x2 I2C LCD SCL** | **A5** | I2C Clock line |
| **16x2 I2C LCD VCC** | **5V** | Power (5V) |
| **16x2 I2C LCD GND** | **GND** | Ground |
| **Pushbutton: Speed Up (+)** | **D2** | Other terminal to **GND** (internal pull-up enabled) |
| **Pushbutton: Speed Down (-)**| **D3** | Other terminal to **GND** (internal pull-up enabled) |
| **Pushbutton: Home Screen** | **D4** | Other terminal to **GND** (internal pull-up enabled) |
| **Pushbutton: Reset / Listen**| **D5** | Other terminal to **GND** (internal pull-up enabled) |
| **Pushbutton: Mode Toggle** | **D6** | Other terminal to **GND** (internal pull-up enabled) |
| **Status Indicator LED** | **D7** | Anode (+) through 220 Ω resistor, Cathode (-) to GND |
| **Buzzer (+)** | **A0** | Piezo buzzer positive terminal |
| **Buzzer (-)** | **GND** | Piezo buzzer negative terminal |
| **SoftwareSerial TX** | **D12** | Connects to Leonardo **D0 (RX1)** |
| **SoftwareSerial RX** | **D13** | Connects to Leonardo **D1 (TX1)** |
| **Ground** | **GND** | Connects to Leonardo **GND** (Common Ground) |

---

### 3. Board-to-Board Interconnect (Uno ↔ Leonardo)

A 3-wire serial link synchronizes speed, mode, and reset states between both boards:

```text
  Arduino Uno                    Arduino Leonardo
+--------------+               +------------------+
|      D12(TX) | ------------> | D0(RX1)          |
|      D13(RX) | <------------ | D1(TX1)          |
|          GND | ------------- | GND (Common GND) |
+--------------+               +------------------+
```

> [!IMPORTANT]
> Always ensure a **Common Ground (GND)** connection between both Arduino boards to prevent signal drift or serial communication issues.

---

## 🛠️ Hardware Requirements

- 1x Arduino Leonardo
- 1x Arduino Uno
- 1x LDR Sensor Module with Digital Output (DO) and sensitivity potentiometer
- 1x 16x2 Character LCD with I2C Backback (Default address `0x27` or `0x3F`)
- 2x High-brightness white or red LEDs
- 5x Pushbuttons
- 3x 220 Ω Resistors (2 for transmitter LEDs, 1 for status LED)
- 1x 5V Piezo Buzzer
- Breadboards & Jumper Wires
- Android Phone (to install `LI_FI.apk`)

---

## 🚀 How to Run the Project

### Step 1: Upload Firmware
1. Open [LiFi_Leonardo_Transmitter.ino](file:///c:/Users/Nithin%20Kannavaram/Downloads/LI_FI%20PROJECT/LiFi_Leonardo_Transmitter.ino) in Arduino IDE:
   - Select Board: **Arduino Leonardo**
   - Select the respective COM Port.
   - Click **Upload**.
2. Open [LiFi_Receiver.ino](file:///c:/Users/Nithin%20Kannavaram/Downloads/LI_FI%20PROJECT/LiFi_Receiver.ino) in Arduino IDE:
   - Select Board: **Arduino Uno**
   - Select the respective COM Port.
   - Install required library: `LiquidCrystal_I2C` by Frank de Brabander.
   - Click **Upload**.

### Step 2: Physical Alignment & Calibration
1. Align the transmitter LEDs directly facing the receiver LDR sensor (distance: 2 cm to 10 cm recommended for testing).
2. Ensure ambient light is stable.
3. Adjust the onboard potentiometer on the LDR module so that the DO output changes state cleanly when the LEDs switch ON and OFF.

### Step 3: Sending & Receiving Data
1. Open the Serial Monitor for the **Arduino Leonardo** at **9600 baud** (configured to send *Newline*).
2. Use the buttons on the Arduino Uno to set the desired speed and mode (ASCII or Morse).
3. Type any message (up to 32 characters) into the Leonardo Serial Monitor and press Enter.
4. The LEDs will blink out the data frame or Morse timing.
5. The Uno receiver reads the optical signals, validates the integrity (CRC-8 in ASCII mode or prosign in Morse mode), triggers the buzzer on completion, and displays the decoded message on the 16x2 LCD!
