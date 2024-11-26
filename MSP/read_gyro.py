import serial
import struct
import time

# Define MSP commands and constants
MSP_HEADER = b'$M<'
MSP_ATTITUDE = 108  # MSP code for attitude information (gyro data)
MSP_RAW_IMU = 102   # MSP code for raw IMU (accelerometer, gyro, and magnetometer)

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
    
    # Read the payload
    payload = ser.read(payload_size)
    
    # Read the checksum
    checksum = ser.read(1)[0]
    
    # Validate checksum
    if calculate_checksum(struct.pack('<B', payload_size) + payload) != checksum:
        raise Exception("Checksum mismatch")
    
    return payload

def main():
    # Open serial port for the Betaflight flight controller (adjust port as needed)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    
    try:
        while True:
            # Request gyro/attitude data
            response = send_msp_request(ser, MSP_ATTITUDE)
            roll, pitch, yaw = parse_msp_attitude(response)
            print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            
            # Wait before sending the next request
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("Exiting...")
    
    finally:
        ser.close()

if __name__ == "__main__":
    main()
