# Canal Flood & Waste Detection System — Wiring Guide

**Board:** ESP32 DevKitV1 (ESP32-WROOM-32)  
**Power Supply:** USB Powerbank (5V / 2.4A) → ESP32 Micro-USB port

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

> ⚠️ **Important:** Connect the servo's red VCC wire to the **VIN** pin, NOT to 3.3V. The SG90 requires 5V to operate properly. The VIN pin passes the raw 5V directly from the USB powerbank without going through the ESP32's internal regulator, so the servo's current spikes (up to 500mA) won't affect the ESP32.

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
                    USB Powerbank (5V / 2.4A)
                           │
                      Micro-USB Cable
                           │
                    ┌──────┴──────┐
                    │   ESP32     │
                    │  DevKitV1  │
                    │             │
              VIN ──┤  (5V out)   ├── 3.3V (internal regulator)
                │   │             │        │
                │   └──────┬──────┘        │
                │          │               │
           ┌────┴────┐    GND          ┌───┴───┐
           │         │     │           │       │
       Servo VCC  HC-SR04  │       HW-038   LEDs &
       (Red)      VCC      │       VCC via  Buzzer
                           │       D33      (3.3V
                     All GND wires         logic)
                     connect here
                     (shared rail)
```

---

## GND Rail
All component GND wires should connect to the same **ground rail** on your breadboard. You can use any of the ESP32's GND pins — they are all internally connected. For a clean layout, run one jumper wire from any ESP32 GND pin to the breadboard's negative (−) rail, then connect all component grounds to that rail.

---

## Quick Checklist Before Power On

- [ ] Servo VCC → **VIN**, not 3.3V
- [ ] HC-SR04 VCC → **VIN**, not 3.3V
- [ ] HW-038 Signal → **D34** (ADC1 pin, Wi-Fi safe)
- [ ] All 3 LEDs have a **220Ω resistor** on the cathode side
- [ ] Buzzer is **passive** (not active)
- [ ] All GND wires go to the **same ground rail**
- [ ] No wires touching each other / no shorts on the breadboard