"""
python3 read_GPS.py

https://github.com/betaflight/betaflight/blob/254da8f4605ef2b7b75e7054b1c9f907e1941c20/src/main/msp/msp.c#L1512
"""

import serial
import struct

# MSP Command ID for RAW GPS
MSP_RAW_GPS = 106

def calculate_checksum(payload):
    """Calculate MSP checksum (XOR of all payload bytes)."""
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum

def build_msp_command(command):
    """Build an MSP command frame to send."""
    header = b'$M<'  # MSP header
    payload = []  # No additional payload for GPS request
    size = len(payload)
    checksum = calculate_checksum([size, command] + payload)
    frame = header + bytes([size, command] + payload + [checksum])
    return frame

def send_msp_command(ser, command):
    """Send an MSP command and receive a response."""
    # Send the MSP command
    frame = build_msp_command(command)
    ser.write(frame)
    ser.flush()

    # Read the response header
    response_header = ser.read(3)
    if response_header[0:2] != b'$M':
        raise ValueError("Invalid response header")

    # Read the size and command
    size = ser.read(1)[0]
    command = ser.read(1)[0]

    # Read the payload
    response_payload = ser.read(size)

    # Read and verify checksum
    checksum = ser.read(1)[0]
    if checksum != calculate_checksum([size, command] + list(response_payload)):
        raise ValueError("Checksum mismatch")

    return response_payload

def parse_msp_raw_gps(payload):
    """Parse the MSP_RAW_GPS payload (message ID 106)."""
    # The payload structure is as follows:
    # 1 byte: GPS Fix (0 or 1)
    # 1 byte: Number of Satellites
    # 4 bytes: Latitude (signed 32-bit integer)
    # 4 bytes: Longitude (signed 32-bit integer)
    # 2 bytes: Altitude in meters (unsigned 16-bit integer)
    # 2 bytes: Ground Speed in cm/s (unsigned 16-bit integer)
    # 2 bytes: Ground Course in degrees (unsigned 16-bit integer)
    # 2 bytes: PDOP (unsigned 16-bit integer)

    if len(payload) < 16:
        raise ValueError("Invalid payload size for MSP_RAW_GPS")

    gps_fix = payload[0]
    num_satellites = payload[1]
    latitude, longitude = struct.unpack('<ii', payload[2:10])
    altitude, ground_speed, ground_course = struct.unpack('<HHH', payload[10:16])

    # If available, parse PDOP (added in API version 1.44)
    pdop = None
    if len(payload) >= 18:
        pdop = struct.unpack('<H', payload[16:18])[0]

    return {
        "gps_fix": gps_fix,
        "num_satellites": num_satellites,
        "latitude": latitude / 10000000,  # Latitude is scaled by 10^7
        "longitude": longitude / 10000000,  # Longitude is scaled by 10^7
        "altitude": altitude,  # Altitude in meters
        "ground_speed": ground_speed / 100,  # Ground speed in m/s (from cm/s)
        "ground_course": ground_course / 10,  # Ground course in degrees
        "pdop": pdop  # PDOP (optional, may be None if not available)
    }

def main():
    # Open the serial connection
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

    try:
        # Send the MSP_RAW_GPS command and receive the response
        response_payload = send_msp_command(ser, MSP_RAW_GPS)

        # Parse the GPS data
        gps_data = parse_msp_raw_gps(response_payload)

        # Print the GPS data
        print("GPS Data:")
        print(f"  GPS Fix: {'Yes' if gps_data['gps_fix'] else 'No'}")
        print(f"  Number of Satellites: {gps_data['num_satellites']}")
        print(f"  Latitude: {gps_data['latitude']}°")
        print(f"  Longitude: {gps_data['longitude']}°")
        print(f"  Altitude: {gps_data['altitude']} meters")
        print(f"  Ground Speed: {gps_data['ground_speed']} m/s")
        print(f"  Ground Course: {gps_data['ground_course']}°")
        if gps_data['pdop'] is not None:
            print(f"  PDOP: {gps_data['pdop']}")

    finally:
        # Always close the serial connection
        ser.close()

if __name__ == "__main__":
    main()
