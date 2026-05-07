# Canal Flood & Waste Detection System - Wiring Guide

**Board:** ESP32 DevKitV1 (ESP32-WROOM-32)  
**Power Supply:** Micro USB Phone Charger (5V / 2A, 10W) → AC Outlet (220–240V) → ESP32 Micro-USB port

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
| | GND (−) | GND | Ground | Black |
| | Signal (S/AO) | D34 | GPIO 34 (ADC1) | Blue |
| **Green LED** | Anode (+) long leg | D25 | GPIO 25 | Green |
| | Cathode (−) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Yellow LED** | Anode (+) long leg | D26 | GPIO 26 | Yellow |
| | Cathode (−) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Red LED** | Anode (+) long leg | D27 | GPIO 27 | Red |
| | Cathode (−) short leg | GND | Ground | Black (via 220Ω resistor) |
| **Passive Buzzer** | Signal (+) | D32 | GPIO 32 | Orange |
| | GND (−) | GND | Ground | Black |

---

## Hardware Calibration & Range

All calibration constants are defined at the top of `code_esp32.ino` (lines 17–35). Change a value there and the entire system, including the web interface, reflects the update automatically. No other file needs editing.

---

### Servo Sweep (Radar Arc)

| Constant | Value | Web Label | Notes |
|---|---|---|---|
| `SWEEP_MIN` | `25°` | `ANGLE` | Init position and lower sweep boundary. The servo moves to this angle at startup. |
| `SWEEP_MAX` | `155°` | `ANGLE` | Upper sweep boundary. Total arc = `155 − 25 = 130°`. |
| `SWEEP_STEP` | `1°` | — | Angular increment per step. Smaller = finer resolution, slower full sweep. |
| `SWEEP_MARGIN` | `3°` | — | Dead zone at both ends of the arc. Angles within this margin are excluded from obstruction and clog checks to prevent false positives from sensor edge noise. |

The `ANGLE` label in the web interface shows the **live current angle** of the servo, updated in real time via WebSocket. It does not show a fixed configured value.

To widen or narrow the arc, edit `SWEEP_MIN` and `SWEEP_MAX`. Also update the `SM` and `SX` JavaScript variables inside the `R"rawliteral(..."` block (they mirror `SWEEP_MIN` and `SWEEP_MAX` for the browser-side radar renderer):

```js
// Inside INDEX_HTML — line ~69 of code_esp32.ino
var MD=20, SM=25, SX=155, ...
//          ↑ SWEEP_MIN  ↑ SWEEP_MAX
```

---

### Ultrasonic Sensor Range

| Constant | Value | Web Label | Notes |
|---|---|---|---|
| `MAX_DETECTION_RANGE_CM` | `20.0 cm` | `DISTANCE` | Readings beyond this value are discarded as out-of-range. |

The `DISTANCE` label in the web interface shows the **live measured distance** from the HC-SR04 at the current sweep angle. It displays `NO ECHO` when no valid return is detected within range.

The JavaScript variable `MD=20` inside the raw literal mirrors this constant for the radar ring scaling. If you change `MAX_DETECTION_RANGE_CM`, update `MD` in the same `var MD=20, SM=25...` line to keep the radar display consistent.

---

### Water Level Sensor

| Constant | Value | Web Label | Notes |
|---|---|---|---|
| `DEPTH_ELEVATED` | `10.0 cm` | `DEPTH` | Water depth at which the system escalates to **Elevated** status. |
| `DEPTH_CRITICAL` | `15.0 cm` | `DEPTH` | Water depth at which the system escalates to **Critical Flood Risk**. |
| `WL_EMA_ALPHA` | `0.8` | — | Exponential Moving Average smoothing factor for depth readings. Higher = more responsive, lower = smoother. |

The `DEPTH` label shows the **live EMA-filtered water depth** in centimetres. The raw ADC reading from GPIO 34 is mapped to depth using: `depth = raw × 30.0 / 4095.0`, giving a 0–30 cm effective range.

---

### Passive Buzzer Alert Frequencies

The buzzer **must be a passive type**. The firmware drives it with `ledcWriteTone()` / `ledcChangeFrequency()`, which outputs a square wave at a specific frequency. An active buzzer ignores the frequency and plays only its fixed internal tone.

| Status | Pattern | Frequency |
|---|---|---|
| **Elevated** | Short beep every 3 s | `3000 Hz` |
| **Waste Detected** | Double beep every 2 s | `3000 Hz` |
| **Waste Detected (Clogged)** | Rising sweep every 1.5 s | `2500–3500 Hz` (swept) |
| **Critical Flood Risk** | Continuous alternating | `4000 Hz / 3000 Hz` (150 ms toggle) |

These frequencies are hardcoded in `setOutput()`. To change them, locate the `buzzerTone(...)` calls inside that function.

---

## Detailed Wiring Per Component

### 1. SG90 9g Micro Servo Motor
The servo has a 3-wire ribbon cable:

```
Servo Wire        →  ESP32 Pin
─────────────────────────────────
Orange (Signal)   →  D13
Red    (VCC)      →  VIN (5V from USB)
Brown  (GND)      →  GND
```

> ⚠️ **Important:** Connect the servo's red VCC wire to the **VIN** pin, NOT to 3.3V. The SG90 requires 5V to operate properly. The VIN pin passes the raw 5V directly from the USB charger without going through the ESP32's internal regulator, so the servo's current spikes (up to 500mA) won't affect the ESP32.

---

### 2. HC-SR04 Ultrasonic Distance Sensor
The sensor has 4 pins in a row:

```
HC-SR04 Pin       →  ESP32 Pin
─────────────────────────────────
VCC               →  VIN (5V from USB)
TRIG              →  D14
ECHO              →  D12
GND               →  GND
```

> ⚠️ **Important:** The HC-SR04 requires 5V on VCC to function. Its ECHO pin outputs a 5V signal, but ESP32 GPIO pins are 5V-tolerant in practice for digital input. If you want extra safety, you can add a voltage divider (two resistors) on the ECHO line, but for a prototype this is not strictly necessary.

---

### 3. HW-038 Water Level Detection Sensor
The sensor module has 3 pins:

```
HW-038 Pin        →  ESP32 Pin
─────────────────────────────────
VCC  (+)          →  D33  (GPIO 33, used as power pin)
GND  (−)          →  GND
Signal (S or AO)  →  D34  (GPIO 34, analog input)
```

> ⚠️ **Why D33 for VCC?** The code deliberately powers the water level sensor from a GPIO pin (`WL_VCC_PIN = 33`) instead of the 3.3V rail. This allows the firmware to turn the sensor ON only when taking a reading, which reduces electrochemical corrosion of the sensor traces over time. GPIO 33 outputs 3.3V when set HIGH — this is enough for the HW-038.

> ⚠️ **Why D34 for Signal?** GPIO 34 is on the **ADC1** bus. When Wi-Fi is active on the ESP32, the **ADC2** bus is completely blocked by the Wi-Fi radio. Pins on ADC2 (e.g., GPIO 2, 4, 15, 25-27) cannot be used for analog reads. GPIO 34 (ADC1) works reliably even during Wi-Fi transmission.

---

### 4. LED Indicators (Green, Yellow, Red)
Each LED requires a **220Ω resistor** between its cathode (short leg) and GND to limit current:

```
Green LED:
  Anode  (+, long leg)   →  D25
  Cathode (−, short leg) →  220Ω resistor → GND

Yellow LED:
  Anode  (+, long leg)   →  D26
  Cathode (−, short leg) →  220Ω resistor → GND

Red LED:
  Anode  (+, long leg)   →  D27
  Cathode (−, short leg) →  220Ω resistor → GND
```

> 💡 **Tip:** If you don't have 220Ω resistors, anything between 150Ω and 470Ω will work. Lower = brighter (but don't go below 100Ω or you risk burning out the LED or the GPIO pin).

---

### 5. Passive Buzzer Module
```
Buzzer Pin        →  ESP32 Pin
─────────────────────────────────
Signal (+)        →  D32
GND    (−)        →  GND
```

> ⚠️ **Must be a PASSIVE buzzer**, not an active buzzer. The code uses `ledcWriteTone()` to generate specific frequencies (3000Hz / 4000Hz). An active buzzer has a built-in oscillator and will only make one fixed pitch regardless of the frequency you send it.

---

## Power Distribution Diagram

```
  AC Outlet (220–240V)
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
  │   GND ────────────┼──── Breadboard (−) Rail
  │                   │         │
  │   3.3V (internal) │    ┌────┴──────────────────┐
  │     │             │    │    Shared GND Rail     │
  └─────┼─────────────┘    │                       │
        │                  │  Servo GND (Brown)    │
    ┌───┴───┐              │  HC-SR04 GND          │
    │       │              │  HW-038 GND           │
  HW-038  LEDs &           │  LED Cathodes (×3)    │
  VCC via Buzzer           │  Buzzer GND           │
  D33    (3.3V             └───────────────────────┘
         logic)
                    (+) Rail provides 5V to:
                      • Servo VCC (Red wire)
                      • HC-SR04 VCC
```

---

## GND Rail
All component GND wires connect to the same **ground rail** on your breadboard. Run one jumper wire from any ESP32 GND pin to the breadboard's negative (−) rail, then connect all component grounds to that rail. All ESP32 GND pins are internally connected, so any one will work.
