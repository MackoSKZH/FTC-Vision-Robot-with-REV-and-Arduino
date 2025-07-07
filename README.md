# FTC Color-Sorting Robot with Arduino LED Feedback

**Autonomous FTC robot for color-based object sorting using AprilTag navigation and LED status indicators.**

---

## Overview

This project is a mobile FTC robot that sorts objects by color into designated containers. It uses a webcam with color detection, AprilTag markers for container recognition, and visual feedback via WS2812 LEDs controlled by an Arduino Nano ESP32. Powered by the REV Control Hub.

Ideal for FTC teams, robotics enthusiasts, and anyone interested in vision-based sorting systems with real-time LED feedback!

---

## Hardware Components

- **REV Control Hub** – robot power & motor control
- **2× HD Hex Motors** – drivetrain (ports 0: Right, 1: Left)
- **1× Core Hex Motor** – lift mechanism (port 2)
- **2× Servo Motors** – grip mechanism (CatchL & CatchR)
- **Logitech C270 Webcam** – color & AprilTag detection
- **Arduino Nano ESP32** – drives WS2812 LEDs based on digital signal
- **2× DFROBOT Gravity WS2812 LEDs** – status indication:
    - LED 0 → Green: correct placement
    - LED 1 → Red: incorrect placement

---

## Software & Technologies

- **FTC SDK** (Java / Android)
- **Vision Portal** with:
    - `PredominantColorProcessor` – detects object colors
    - `AprilTagProcessor` – identifies container tags
- **Arduino C++** with `Adafruit_NeoPixel` library
- Communication: digital output from REV → Arduino input → LED control

---

## Features & Workflow

1. **Drive and grip control** via gamepad
2. **Color detection** of held object (history-based filtering)
3. **AprilTag detection** to recognize container type (IDs 1–3)
4. **Logic: score if object-color matches container type**
5. **Arduino signals: Green LED for correct, Red LED for incorrect**

---

## Gamepad Controls

- **Left/Right sticks**: drive left/right
- **D-pad Up/Down**: control lift
- **Right trigger**: grip/release object
- **Button A**: toggle telemetry display

---

## Code Structure

```text
alphaBot_OpMode_v02.java   ← FTC Java TeleOp program
led_control.ino               ← Arduino Nano ESP32 LED control sketch