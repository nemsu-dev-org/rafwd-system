<div align="center">

<p align="center">
  <img width="1186" height="386" alt="hero" src="https://github.com/user-attachments/assets/38582a61-27ab-4a89-a11c-f1b4ab30b355" />
</p>

# Radar-Assisted Flood and Floating Waste Detection System

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](#)
[![Platform](https://img.shields.io/badge/platform-ESP32-lightgrey.svg)](#)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](#)

**ESP32-based radar and ultrasonic sensor system for real-time canal flood and floating waste detection.**

</div>

---

## Table of Contents

- [Features](#features)
- [Hardware Setup](#hardware-setup)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

---

## Features

* **Real-time Monitoring:** Continuous scanning using an SG90 servo and HC-SR04 ultrasonic sensor to detect water depth and surface obstructions.
* **Responsive Web Dashboard:** Live radar visualizer, historical data graphs, and system metrics accessible from any browser on the same network.
* **Asynchronous Processing:** Built on ESPAsyncWebServer for non-blocking WebSocket communication and client updates.
* **Automated Alerts:** Integrated active buzzer and LED status indicators for on-site elevated and critical risk warnings.

---

## Hardware Setup

Full wiring diagrams, GPIO pinout tables, hardware calibration constants (sweep angles, sensor range, water depth thresholds, buzzer frequencies), and assembly notes are in the dedicated guide:

**[View Wiring Guide](code_esp32/WIRING_GUIDE.md)**

Key pin assignments at a glance:

| Component | ESP32 Pin |
|---|---|
| SG90 Servo (Signal) | D13 / GPIO 13 |
| HC-SR04 Trigger | D14 / GPIO 14 |
| HC-SR04 Echo | D12 / GPIO 12 |
| HW-038 Water Level (Power) | D33 / GPIO 33 |
| HW-038 Water Level (Signal) | D34 / GPIO 34 |
| Passive Buzzer | D32 / GPIO 32 |
| Green LED | D25 / GPIO 25 |
| Yellow LED | D26 / GPIO 26 |
| Red LED | D27 / GPIO 27 |

Power all sensors via the **VIN pin** (5V passthrough), not the 3.3V rail.

---

## Installation

### Prerequisites

* Arduino IDE or PlatformIO
* ESP32 Board Support Package installed
* Libraries: `AsyncTCP`, `ESPAsyncWebServer`, `ArduinoJson`

### Steps

**1. Clone the repository**

```bash
git clone https://github.com/Huerte/rafwd-system.git
cd rafwd-system
```

**2. Wire the hardware**

Follow the [Wiring Guide](code_esp32/WIRING_GUIDE.md) before proceeding.

**3. Flash the firmware**

Open `code_esp32/code_esp32.ino` in your IDE and upload it to the ESP32. No separate build step is needed. The web interface is embedded directly in the firmware.

---

## Usage

1. Power on the ESP32 via a **5V/2A** charger connected to the Micro-USB port.
2. On your phone or laptop, connect to the Wi-Fi network `CanalMonitor` (Password: `12345678`).
3. Open a browser and go to `http://192.168.4.1`. The captive portal may open it automatically.
4. The radar display will begin live scanning. Click **Enable Alerts** to activate audio notifications in the browser.

---

## Project Structure

```text
rafwd-system/
├── code_esp32/
│   ├── code_esp32.ino    # ESP32 firmware with embedded web interface
│   ├── minify.py         # Script used to compress HTML/CSS/JS into the firmware
│   └── WIRING_GUIDE.md   # Hardware pinout, calibration constants, and wiring diagrams
└── README.md             # This file
```

---

## Contributing

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit your changes (`git commit -m "Add your feature"`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a Pull Request.

---

## License

See the LICENSE file for details.

Copyright (c) 2026 Research Group 3 Team
