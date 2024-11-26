import serial
import struct

# MSP Command IDs
MSP_STATUS = 101  # Example MSP command to request the status

def calculate_checksum(payload):
    """Calculate MSP checksum (XOR of all payload bytes)."""
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum

def build_msp_command(command, payload=[]):
    """Build an MSP command frame."""
    header = b'$M<'  # MSP header
    size = len(payload)
    checksum = calculate_checksum([size, command] + payload)
    frame = header + bytes([size, command] + payload + [checksum])
    return frame

def send_msp_command(ser, command, payload=[]):
    """Send an MSP command and return the response."""
    frame = build_msp_command(command, payload)
    ser.write(frame)
    ser.flush()

    # Read the response header and size
    response_header = ser.read(3)
    if response_header != b'$M>':
        print("Invalid response header:", response_header)
        return None
    
    size = ord(ser.read(1))
    command = ord(ser.read(1))
    response_payload = ser.read(size)
    checksum = ord(ser.read(1))
    
    # Verify the checksum
    if checksum != calculate_checksum([size, command] + list(response_payload)):
        print("Checksum mismatch!")
        return None
    
    return response_payload

def main():
    # Open serial port connected to the flight controller
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

    try:
        # Request MSP_STATUS
        send_msp_command(ser, MSP_STATUS)
        response = send_msp_command(ser, MSP_STATUS)

        if response:
            if len(response) == 24:
                # Corrected unpacking format string for 24 bytes
                (
                    cycle_time,            # uint16_t (2 bytes)
                    i2c_error_counter,     # uint16_t (2 bytes)
                    sensor,                # uint16_t (2 bytes)
                    flight_mode_flags,     # uint32_t (4 bytes)
                    config_profile_index,  # uint8_t (1 byte)
                    system_load,           # uint8_t (1 byte)
                    gyro_cycle_time,       # uint16_t (2 bytes)
                    accel_cycle_time,      # uint16_t (2 bytes)
                    baro_cycle_time,       # uint16_t (2 bytes)
                    mag_cycle_time,        # uint16_t (2 bytes)
                    sonar_cycle_time,      # uint16_t (2 bytes)
                    main_loop_cycle_time   # uint16_t (2 bytes)
                ) = struct.unpack('<HHHIBBHHHHHH', response)

                print(f"Cycle Time: {cycle_time} µs")
                print(f"I2C Error Counter: {i2c_error_counter}")
                print(f"Sensor Present: {sensor}")
                print(f"Flight Mode Flags: {flight_mode_flags}")
                print(f"Config Profile Index: {config_profile_index}")
                print(f"System Load: {system_load} %")
                print(f"Gyro Cycle Time: {gyro_cycle_time} µs")
                print(f"Accel Cycle Time: {accel_cycle_time} µs")
                print(f"Baro Cycle Time: {baro_cycle_time} µs")
                print(f"Mag Cycle Time: {mag_cycle_time} µs")
                print(f"Sonar Cycle Time: {sonar_cycle_time} µs")
                print(f"Main Loop Cycle Time: {main_loop_cycle_time} µs")
            else:
                print(f"Unexpected response length: {len(response)} bytes.")
        
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
