# Canal Flood & Waste Monitor - Project Context

This document provides a comprehensive summary of the project state, hardware configuration, physical geometry, and the history of firmware debugging. It serves as context for future development.

## 1. Physical Geometry & Measurements

The system operates in a controlled prototype environment (a plastic container). The physical measurements are critical to the firmware's geometric beam calculations.

- **Ultrasonic Mount Height:** `9.0 cm` (from the face of the HC-SR04 straight down to the floor of the container).
- **Container Total Depth:** `5.7 cm`
- **Max Safe Water Depth:** `5.0 cm` (exceeding this will reach the resistors on the water level sensor, potentially damaging it).
- **Water Level Sensor Physical Height:** `6.0 cm` total. Base sits perfectly on the floor of the container.
- **Water Level Sensor Traces:** `4.0 cm` of exposed silver lining, starting from the container floor.
- **Container Width:** `17.0 cm` total (`8.5 cm` half-width). The ultrasonic sensor is mounted exactly in the center.
- **HC-SR04 Minimum Detection:** Verified to detect objects as close as `1.0 cm`.

## 2. Hardware Architecture

- **Microcontroller:** ESP32 DEVKIT V1
- **Power Supply:** 5V / 2A phone charger connected to the ESP32.
- **Sensors:** 
  - HC-SR04 Ultrasonic Distance Sensor (mounted on a servo)
  - HW-038 Analog Water Level Sensor
- **Actuator:** SG90 Micro Servo (powered via an isolated breadboard power rail to prevent electrical noise)
- **Indicators:** Green, Yellow, and Red LEDs + Active Buzzer
- **Power Stability (Brownout Prevention):** 5x 100µF capacitors installed in parallel (400µF total on the servo's power rail, 100µF on the ESP32 VIN/GND pins) to absorb the SG90's transient current spikes and prevent ESP32 WiFi brownout crashes.

## 3. Firmware Configuration & Thresholds

The firmware operates a WebSocket server, captive portal, and an HTML5 canvas radar UI.

**Key Constants:**
- `SENSOR_HEIGHT_CM = 9.0`: The baseline distance to the floor at 90° (center).
- `CONTAINER_HALF_WIDTH = 8.5`: Used to compute the maximum valid beam distance at steep angles (`calcAngleRange`).
- `SWEEP_MIN = 47` / `SWEEP_MAX = 133`: The precise angle limits where the ultrasonic beam strikes the container's physical edges.
- `WATER_SAFETY_MARGIN = 1.0`: Additional padding added to the water depth to compensate for the HW-038's tendency to under-report water levels.
- `OBSTRUCT_THRESH = 1.2`: A delta of 1.2 cm or greater from the baseline is required to flag an obstruction.
- `VARIANCE_THRESH = 2.5` / `MEAN_DELTA_THRESH = 1.5`: The variance buffer thresholds to trigger a `WASTE` state.