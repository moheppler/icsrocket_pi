import serial
import struct
import time

# Define MSP commands and constants
MSP_HEADER = b'$M<'
MSP_ATTITUDE = 108  # MSP code for attitude information (gyro data)
MSP_RAW_IMU = 102   # MSP code for raw IMU (accelerometer, gyro, and magnetometer)
MSP_DEBUG_ARRAY = 116  # MSP code for debug array (for additional debug data)
MSP_GET_CONFIG = 135  # MSP code for getting controller configuration

# Function to calculate checksum for MSP message
def calculate_checksum(message):
    checksum = 0
    for byte in message:
        checksum ^= byte
    return checksum

# Function to construct MSP request
def construct_msp_request(msp_command):
    message = struct.pack('<BB', 0, msp_command)  # Length is 0 for requests
    checksum = calculate_checksum(message)
    return MSP_HEADER + message + struct.pack('<B', checksum)

# Function to parse the MSP attitude response
def parse_msp_attitude(response):
    roll, pitch, yaw = struct.unpack('<hhh', response[5:11])
    return roll / 10, pitch / 10, yaw / 10  # MSP Attitude returns data in tenths of a degree

# Function to parse the MSP debug array response (25 doubles = 200 bytes)
def parse_msp_debug_array(response):
    if len(response) != 200:
        raise ValueError(f"Expected 200 bytes for 25 doubles, got {len(response)}")
    return struct.unpack('<25d', response)  # Skip header and length

def parse_msp_controller_config(response):
    # Function to parse the MSP get controller config response (25 doubles = 200 bytes)
    if len(response) != 200:
        raise ValueError(f"Expected 200 bytes for 25 doubles, got {len(response)}")
    return struct.unpack('<25d', response)  # Skip header and length


# Function to send an MSP request and read the response
def send_msp_request(ser, msp_command):
    request = construct_msp_request(msp_command)
    ser.write(request)

    # Read the header
    header = ser.read(3)
    if header != b'$M>':
        raise Exception("Invalid MSP response header")

    # Read payload size
    payload_size = ser.read(1)[0]
    
    # Read command byte (this was missing!)
    cmd_byte = ser.read(1)[0]

    # Read the payload
    payload = ser.read(payload_size)

    # Read the checksum
    checksum = ser.read(1)[0]

    # Correct checksum calculation
    expected_checksum = calculate_checksum(bytes([payload_size, cmd_byte]) + payload)
    
    if expected_checksum != checksum:
        print(f"Payload size: {payload_size} bytes")
        print(f"Payload (hex): {payload.hex()}")
        print(f"Checksum (received): {checksum}")
        print(f"Checksum (expected): {expected_checksum}")
        raise Exception("Checksum mismatch")

    return payload


def main():
    # Open serial port for the Betaflight flight controller (adjust port as needed)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
        # Get controller configuration
    config_response = send_msp_request(ser, MSP_GET_CONFIG)
    config_array = parse_msp_controller_config(config_response)
    print("Controller Configuration:")
    for i, val in enumerate(config_array):
        print(f"  [{i:02}] {val:.6f}")
        
    try:
        while True:
            # Request gyro/attitude data
            # response = send_msp_request(ser, MSP_ATTITUDE)
            # roll, pitch, yaw = parse_msp_attitude(response)
            # print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            
            # Debug array (24 doubles)
            debug_response = send_msp_request(ser, MSP_DEBUG_ARRAY)
            debug_array = parse_msp_debug_array(debug_response)
            print("Debug Array:")
            for i, val in enumerate(debug_array):
                print(f"  [{i:02}] {val:.6f}")
                
            # Get controller configuration
            # config_response = send_msp_request(ser, MSP_GET_CONFIG)
            # config_array = parse_msp_controller_config(config_response)
            # print("Controller Configuration:")
            # for i, val in enumerate(config_array):
            #     print(f"  [{i:02}] {val:.6f}")
            
            # Wait before sending the next request
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("Exiting...")
    
    finally:
        ser.close()

if __name__ == "__main__":
    main()
