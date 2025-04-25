# Beginner's Guide to Setting Up and Using the 6-Axis Robot Arm Control with ESP32

This guide will walk you through the setup and operation of your 6-axis robot arm, controlled via potentiometers or a web interface.

---

## Overview

This program allows you to control a 6-axis robotic arm using:
- **Manual Mode**: Adjust servo positions with potentiometers.
- **Web Mode**: Use a web interface to control the arm and save movements.
- **Playback**: Record and replay sequences of movements.

The ESP32 microcontroller manages the servos, buttons, and web interface.

---

## Required Components

1. ESP32 microcontroller.
2. A 6-axis robotic arm with servos.
3. Potentiometers (6 for manual control).
4. Buttons (4 for controlling modes and playback).
5. Adafruit PCA9685 PWM driver (for servo control).
6. NeoPixel LED (for status indication).
7. Breadboard, power supply, and connecting wires.

---

## Features

- **Manual Control**: Adjust servo positions with potentiometers.
- **Web Control**: Use a web page to move servos and save positions.
- **Recording**: Save sequences of movements to EEPROM.
- **Playback**: Replay saved sequences in various modes (manual, semi-auto, full-auto).
- **Status LED**: Indicates the current mode or errors.

---

## Wiring Diagram

### Inputs
- **Potentiometers**: Connect to analog pins:
  - A0: Base
  - A1: Shoulder
  - A2: Elbow
  - A3: Wrist Rotate
  - A4: Wrist Up/Down
  - A5: Gripper

- **Buttons**: Connect to digital pins (using pull-up resistors):
  - Pin 5: Clear Memory
  - Pin 18: Record/Play
  - Pin 19: Stop
  - Pin 23: Start Playback

### Outputs
- **Servos**: Connected to PCA9685 PWM pins.
- **NeoPixel LED**: Connect to pin 26.

---
## Step-by-Step Setup

### 1. Flash the Program
1. Download the `.ino` file from the repository.
2. Open it in the Arduino IDE.
3. Install the required libraries:
   - `WiFi`
   - `Adafruit_PWMServoDriver`
   - `Adafruit_NeoPixel`
   - `Bounce2`
   - `ArduinoJson`
4. Replace the placeholders in the code with your WiFi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";

---
## Instructions for Setting Up and Using the Robot Arm

## Step 1: Connect and Upload
- Connect your ESP32 to your computer and upload the program from the repository.

## Step 2: Hardware Assembly
- Connect the potentiometers, buttons, servos, and NeoPixel as per the wiring diagram provided.
- Power the ESP32 and servos with a sufficient power supply.

## Step 3: Using the Robot Arm
---
### Modes of Operation

#### Manual Control (Potentiometers)
- Default mode if WiFi is not connected.
- Use potentiometers to adjust servo positions.

#### Web Control
- Connect to the ESP32's IP address in your browser.
- Use the web interface to control the servos.

#### Playback
- Record movements with the "Record" button.
- Replay with "Playback" buttons (manual, semi-auto, or full-auto).

### Button Functions
- **Clear Memory**: Hold for 1 second to clear all recorded movements.
- **Record/Play**: Short press to record the current position.
- **Stop**: Short press stops all movement and return to idle mode.  Double press returns to step 0
- **Start Playback**: Single press moves through steps manually. Double press is Semi Automatic and will cycle all steps once.  Short followed by long press continiously cycles through all steps.

## Troubleshooting

### WiFi Not Connecting
- Double-check your WiFi credentials in the code.
- Ensure the ESP32 is within range of your WiFi router.

### Servos Not Moving
- Verify servo connections to the PCA9685 driver.
- Ensure the power supply provides enough current for all servos.

### Web Interface Not Loading
- Check the ESP32's IP address in the Serial Monitor.
- Ensure your browser is on the same network as the ESP32.

### Playback Errors
- Clear EEPROM if movements are not being recorded or replayed correctly.

## Additional Notes
- The NeoPixel LED indicates the current status:
  - **Blue**: Idle
  - **Cyan**: Manual (Potentiometer) Mode
  - **Magenta**: Web Control
  - **Green**: Full Auto Playback
  - **Red**: Error or Fully Automatic
- The servo positions are saved in EEPROM, allowing sequences to persist after a restart.
