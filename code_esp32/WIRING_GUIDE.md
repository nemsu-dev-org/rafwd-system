# Canal Flood & Waste Detection System - Wiring Guide

<p align="center">
  <img width="1192" height="386" alt="wiring1" src="https://github.com/user-attachments/assets/cd65332c-2096-4fd3-849b-55f82ffb8267" />
</p>

**Board:** ESP32 DevKitV1 (ESP32-WROOM-32)  
**Power Supply:** Micro USB Phone Charger (5V / 2A, 10W) -> AC Outlet (220-240V) -> ESP32 Micro-USB port

---

## Pin Configuration Summary

| Component | Component Pin | ESP32 Board Label | GPIO # | Wire Color (Suggested) |
|---|---|---|---|---|
| **SG90 Servo** | Signal (Orange) | D13 | GPIO 13 | Orange |
| | VCC (Red) | VIN | 5V passthrough | Red |
| | GND (Brown) | GND | Ground | Black/Brown |
| **HC-SR04 Ultrasonic** | VCC | VIN | 5V passthrough | Red |
| | TRIG | D14 | GPIO 14 | Yellow |
| | ECHO | D12 | GPIO 12 | Green |
| | GND | GND | Ground | Black |
| **HW-038 Water Level** | VCC (+) | D33 | GPIO 33 | Red |
| | GND (-) | GND | Ground | Black |
| | Signal (S/AO) | D34 | GPIO 34 (ADC1) | Blue |
| **Green LED** | Anode (+) long leg | D25 | GPIO 25 | Green |
| | Cathode (-) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Yellow LED** | Anode (+) long leg | D26 | GPIO 26 | Yellow |
| | Cathode (-) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Red LED** | Anode (+) long leg | D27 | GPIO 27 | Red |
| | Cathode (-) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Active Buzzer** | Signal (+) | D32 | GPIO 32 | Orange |
| | GND (-) | GND | Ground | Black |

---

## Hardware Calibration & Range

All calibration constants are defined at the top of `code_esp32.ino` (lines 16-36). Changing a value there propagates updates across the system and web interface automatically. No other file needs editing.

---

### Servo Sweep (Radar Arc)

| Constant | Value | Web Label | Notes |
|---|---|---|---|
| `SWEEP_MIN` | `47°` | `ANGLE` | Init position and lower sweep boundary. The servo moves to this angle at startup. |
| `SWEEP_MAX` | `133°` | `ANGLE` | Upper sweep boundary. Total arc = `133 - 47 = 86°`. |
| `SWEEP_STEP` | `1°` | - | Angular increment per step. Smaller = finer resolution, slower full sweep. |
| `SWEEP_MARGIN` | `5°` | - | Dead zone at both ends of the arc. Angles within this margin are excluded from obstruction and clog checks to prevent false positives from sensor edge noise. |

The `ANGLE` label in the web interface shows the **live current angle** of the servo, updated in real time via WebSocket. It does not show a fixed configured value.

To widen or narrow the arc, edit `SWEEP_MIN` and `SWEEP_MAX`. Also update the `SM` and `SX` JavaScript variables inside the raw HTML string (they mirror `SWEEP_MIN` and `SWEEP_MAX` for the browser-side radar renderer):

```js
// Inside INDEX_HTML - line ~73 of code_esp32.ino
var MD=9.0, SM=47, SX=133, ...
//          ↑ SWEEP_MIN  ↑ SWEEP_MAX
```

---

### Ultrasonic Sensor Range

Instead of a static range limit, the system dynamically calculates the detection range in correlation with the water level sensor. This filters out the water surface and ensures that only solid waste is detected.

| Constant | Value | Notes |
|---|---|---|
| `SENSOR_HEIGHT_CM` | `9.0 cm` | Physical sensor mounting height above container floor. Used as the baseline validation ceiling. |
| `WATER_SAFETY_MARGIN` | `0.5 cm` | Buffer zone kept above the water surface to filter out waves and ripples. Reduced for controlled demo environments. |
| `MIN_EFFECTIVE_RANGE` | `2.5 cm` | Minimum allowed range ceiling to ensure proximity scanning remains active. |

The dynamic effective range is computed per-angle using beam geometry:
`effectiveRange = max(2.5, min(beam, (SENSOR_HEIGHT_CM - waterDepth) / cosA - WATER_SAFETY_MARGIN))`

*   **Dry State (0.0 cm depth)**: `effectiveRange = min(beam, 9.0)`. Range is capped by sensor height.
*   **High Water (3.0 cm depth)**: `effectiveRange = max(2.5, min(beam, 4.85))`. The surface is masked.

The `DISTANCE` label in the web interface shows the **live measured distance** from the HC-SR04 at the current sweep angle. It displays `NO ECHO` when no valid return is detected within range. The radar interface auto-scales dynamically based on `effectiveRange` sent via the `maxDist` WebSocket field.

---

### Water Level Sensor

| Constant | Value | Web Label | Notes |
|---|---|---|---|
| `DEPTH_ELEVATED` | `3.0 cm` | `DEPTH` | Water depth at which the system escalates to **Elevated** status. |
| `DEPTH_CRITICAL` | `4.0 cm` | `DEPTH` | Water depth at which the system escalates to **Critical Flood Risk**. |
| `WL_EMA_ALPHA` | `0.8` | - | Exponential Moving Average smoothing factor for depth readings. Higher = more responsive, lower = smoother. |

The `DEPTH` label shows the **live EMA-filtered water depth** in centimeters. The raw ADC reading from GPIO 34 is mapped to depth using: `depth = raw * 5.5 / 4095.0`, giving a 0–5 cm effective scale (HW-038 calibrated).

---

### Active Buzzer Alert Patterns

The buzzer **must be an active type** (has a built-in oscillator). The firmware drives it using digital high/low transitions on a non-blocking `millis()` schedule instead of LEDC PWM.

| Status | Pattern | Cycle and Durations |
|---|---|---|
| **Elevated** | Silent | No audio alert |
| **Waste Detected** | Periodic beep every 1 second | 300 ms ON, 700 ms OFF (synchronized with Yellow LED) |
| **Waste Detected (Clogged)** | Rapid beep every 600 ms | 150 ms ON, 450 ms OFF (synchronized with Red LED) |
| **Critical Flood Risk** | Continuous solid tone | Permanently ON |

These patterns are handled inside the `setOutput()` function.

---

## Detailed Wiring Per Component

### 1. SG90 9g Micro Servo Motor
The servo has a 3-wire ribbon cable:

```
Servo Wire        ->  ESP32 Pin
─────────────────────────────────
Orange (Signal)   ->  D13
Red    (VCC)      ->  VIN (5V from USB)
Brown  (GND)      ->  GND
```

> ⚠️ **Important:** Connect the servo's red VCC wire to the **VIN** pin, NOT to 3.3V. The SG90 requires 5V to operate properly. The VIN pin passes the raw 5V directly from the USB charger without going through the ESP32's internal regulator, so the servo's current spikes (up to 500mA) won't affect the ESP32. Adding a 100µF capacitor across VCC and GND close to the servo is highly recommended to suppress back-EMF spikes.

---

### 2. HC-SR04 Ultrasonic Distance Sensor
The sensor has 4 pins in a row:

```
HC-SR04 Pin       ->  ESP32 Pin
─────────────────────────────────
VCC               ->  VIN (5V from USB)
TRIG              ->  D14
ECHO              ->  D12
GND               ->  GND
```

> ⚠️ **Important:** The HC-SR04 requires 5V on VCC to function. Its ECHO pin outputs a 5V signal. To protect the ESP32 inputs, a voltage divider (e.g. 1kΩ and 2kΩ resistors) must be placed on the ECHO line to step down the output voltage from 5V to 3.3V.

---

### 3. HW-038 Water Level Detection Sensor
The sensor module has 3 pins:

```
HW-038 Pin        ->  ESP32 Pin
─────────────────────────────────
VCC  (+)          ->  D33  (GPIO 33, used as power pin)
GND  (-)          ->  GND
Signal (S or AO)  ->  D34  (GPIO 34, analog input)
```

> ⚠️ **Why D33 for VCC?** The code deliberately powers the water level sensor from a GPIO pin (`WL_VCC_PIN = 33`) instead of the 3.3V rail. This allows the firmware to turn the sensor ON only when taking a reading, which reduces electrochemical corrosion of the sensor traces over time. GPIO 33 outputs 3.3V when set HIGH; this is enough for the HW-038.

> ⚠️ **Why D34 for Signal?** GPIO 34 is on the **ADC1** bus. When Wi-Fi is active on the ESP32, the **ADC2** bus is completely blocked by the Wi-Fi radio. Pins on ADC2 (e.g., GPIO 2, 4, 15, 25-27) cannot be used for analog reads. GPIO 34 (ADC1) works reliably even during Wi-Fi transmission.

---

### 4. LED Indicators (Green, Yellow, Red)
Each LED requires a **220Ω resistor** between its cathode (short leg) and GND to limit current:

```
Green LED:
  Anode  (+, long leg)   ->  D25
  Cathode (-, short leg) ->  220Ω resistor -> GND

Yellow LED:
  Anode  (+, long leg)   ->  D26
  Cathode (-, short leg) ->  220Ω resistor -> GND

Red LED:
  Anode  (+, long leg)   ->  D27
  Cathode (-, short leg) ->  220Ω resistor -> GND
```

*   **Normal**: Solid Green LED ON
*   **Elevated**: Solid Yellow LED ON, Buzzer OFF
*   **Waste Detected**: Blinking Yellow LED ON (300ms ON / 700ms OFF), Buzzer synchronized
*   **Waste Detected (Clogged)**: Blinking Red LED ON (150ms ON / 450ms OFF), Buzzer synchronized
*   **Critical**: Solid Red LED ON, Constant Buzzer

---

### 5. Active Buzzer Module
```
Buzzer Pin        ->  ESP32 Pin
─────────────────────────────────
Signal (+)        ->  D32
GND    (-)        ->  GND
```

> ⚠️ **Must be an ACTIVE buzzer**, not a passive buzzer. The code drives the pin directly with digitalWrite HIGH/LOW, which activates the buzzer's internal sound-generating circuit. For louder warning signals in outdoor environments, use an NPN transistor (like BC547 or 2N2222) to drive multiple buzzers from the 5V rail, switching the base through a 1kΩ resistor from GPIO 32.

---

## Power Distribution Diagram

```
  AC Outlet (220-240V)
        │
  ┌─────┴──────┐
  │ Micro USB  │
  │ Charger    │
  │ 5V / 2A    │
  └─────┬──────┘
        │
    Micro-USB Cable
        │
  ┌─────┴─────────────┐
  │     ESP32         │
  │    DevKitV1       │
  │                   │
  │   VIN ────────────┼──── 5V to Breadboard (+) Rail
  │                   │         │
  │   GND ────────────┼──── Breadboard (-) Rail
  │                   │         │
  │   3.3V (internal) │    ┌────┴──────────────────┐
  │     │             │    │    Shared GND Rail     │
  └─────┼─────────────┘    │                       │
        │                  │  Servo GND (Brown)    │
    ┌───┴───┐              │  HC-SR04 GND          │
    │       │              │  HW-038 GND           │
  HW-038  LEDs &           │  LED Cathodes (x3)    │
  VCC via Buzzer           │  Buzzer GND           │
  D33    (3.3V             └───────────────────────┘
         logic)
                    (+) Rail provides 5V to:
                      • Servo VCC (Red wire)
                      • HC-SR04 VCC
```

---

## GND Rail
All component GND wires connect to the same **ground rail** on your breadboard. Run one jumper wire from any ESP32 GND pin to the breadboard's negative (-) rail, then connect all component grounds to that rail. All ESP32 GND pins are internally connected, so any one will work.
