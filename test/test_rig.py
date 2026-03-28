import serial
import time
import random

# ── Configuration ────────────────────────────────────────────────
PORT = 'COM3'  # Change to your Arduino port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD = 115200  # High speed for V2
TIMEOUT = 1

def send_data(ser, data_type, value, wait_ack=True):
    """Sends simulation data to Arduino and waits for exact [ACK] or [SYSTEM]."""
    cmd = f"{data_type}:{value}\n"
    ser.write(cmd.encode())
    
    if not wait_ack:
        return True

    # Construct the exact expected ACK (e.g., "[ACK] W:12.0")
    expected_ack = f"[ACK] {data_type}:{value}"
    if type(value) is float:
        # Avoid float formatting mismatches by using startswith/in
        expected_ack = f"[ACK] {data_type}:"

    start = time.time()
    while time.time() - start < 1.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if (data_type == 'R' and "[SYSTEM]" in line) or (expected_ack in line):
                return True
    return False

def read_status(ser):
    """Drain the entire buffer to always catch up to real-time status."""
    latest_status = None
    latest_data = None
    try:
        # Loop until buffer is completely empty to prevent backlog
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line.startswith("STAT:"):
                parts = line.split('|')
                latest_status = parts[0].split(':')[1]
                latest_data = {p.split(':')[0]: p.split(':')[1] for p in parts[1:]}
    except Exception:
        pass
        
    return latest_status, latest_data

def run_test_case(ser, name, water_depth, ultrasonic_values, expected_status, duration=4):
    """Runs a specific test scenario and validates the result."""
    print(f"\n[TEST] {name}")
    
    # FLUSH any old data to guarantee we start clean
    ser.reset_input_buffer()
    
    # 1. RESET STATE BEFORE TEST
    if not send_data(ser, 'R', 1):
        print("      Warning: Reset command not acknowledged.")
    
    print(f"      Setting Water Depth: {water_depth}cm")
    print(f"      Expected Status:     {expected_status}")
    
    start_time = time.time()
    results = []
    
    # Pre-feed the water depth
    send_data(ser, 'W', water_depth)

    while time.time() - start_time < duration:
        # Feed ultrasonic data
        u_val = random.choice(ultrasonic_values) if isinstance(ultrasonic_values, list) else ultrasonic_values
        send_data(ser, 'U', u_val, wait_ack=False) # Don't wait ACK in hot loop
        
        status, data = read_status(ser)
        if status and data is not None:
            results.append(status)
            w = data.get('W', '?')
            v = data.get('V', '?')
            u = data.get('U', '?')
            print(f"      Current: {status} | W:{w} | V:{v} | U:{u} ", end='\r')
        
        time.sleep(0.05)
    
    # Analyze the result from the last 2 seconds (post-debounce)
    recent_results = results[len(results)//2:] if (results and len(results) > 0) else []
    final_status = max(set(recent_results), key=recent_results.count) if recent_results else "UNKNOWN"
    
    if final_status == expected_status:
        print(f"\n[PASS] System correctly identified: {final_status}")
        return True
    else:
        print(f"\n[FAIL] Predicted: {expected_status} | Received: {final_status}")
        return False

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print(f"Connecting to {PORT}...")
        time.sleep(3) # Wait for Arduino reboot
        ser.reset_input_buffer()
        print("Connected.")
    except Exception as e:
        print(f"Error connecting to serial port: {e}")
        return

    test_results = []

    try:
        # 1. Baseline / Normal
        test_results.append(run_test_case(ser, "NORMAL CONDITION", 0.0, 35.0, "NORMAL"))

        # 2. Elevated Water
        test_results.append(run_test_case(ser, "ELEVATED WATER", 12.0, 35.0, "ELEVATED"))

        # 3. Critical Flood
        test_results.append(run_test_case(ser, "CRITICAL FLOOD", 22.0, 35.0, "CRITICAL_FLOOD"))

        # 4. Waste Detection (Moving - high variance)
        waste_readings = [30.0, 35.0, 28.0, 38.0, 25.0, 40.0]
        test_results.append(run_test_case(ser, "WASTE (VARIANCE)", 0.0, waste_readings, "WASTE_DETECTED", duration=6))

        # 5. Waste Detection (Stationary - obstruction)
        test_results.append(run_test_case(ser, "WASTE (OBSTRUCTION)", 0.0, 20.0, "WASTE_DETECTED"))

        # 6. Critical Flood Overrides Waste
        test_results.append(run_test_case(ser, "CRITICAL OVERRIDES WASTE", 25.0, waste_readings, "CRITICAL_FLOOD"))

        # Summary
        passed = sum(1 for r in test_results if r)
        total = len(test_results)
        print("\n" + "="*40)
        print(f"TEST SUMMARY: {passed}/{total} Passed")
        print("="*40)
        if passed == total:
            print("SUCCESS: All system logic verified (Simulation V2).")
        else:
            print(f"ATTENTION: {total - passed} test case(s) failed validation.")

    except KeyboardInterrupt:
        print("\nTesting stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        ser.close()
        print("\nSerial connection closed.")

if __name__ == "__main__":
    main()
