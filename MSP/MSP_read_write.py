import serial
import struct
import time

# MSP Command IDs
MSP_SET_RAW_RC = 200  # Command to set RC channels
MSP_RAW_GPS = 106     # Command for RAW GPS data
MSP_RAW_IMU = 102     # Command for RAW IMU data (9 DOF)

def calculate_checksum(payload):
    """Calculate MSP checksum (XOR of all payload bytes)."""
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum

def build_msp_command(command, payload=[]):
    """Build an MSP command frame to send."""
    header = b'$M<'  # MSP header
    size = len(payload)
    checksum = calculate_checksum([size, command] + payload)
    frame = header + bytes([size, command] + payload + [checksum])
    return frame

def send_msp_command(ser, command, payload=[]):
    """Send an MSP command and receive a response."""
    # Send the MSP command
    frame = build_msp_command(command, payload)
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

def parse_msp_raw_imu(payload):
    """Parse the MSP_RAW_IMU payload (message ID 102)."""
    if len(payload) < 18:
        raise ValueError("Invalid payload size for MSP_RAW_IMU")

    # Unpack the raw IMU data (9 degrees of freedom)
    acc_x, acc_y, acc_z = struct.unpack('<hhh', payload[0:6])  # Acceleration in m/s^2 (scaled)
    gyro_x, gyro_y, gyro_z = struct.unpack('<hhh', payload[6:12])  # Gyroscope in degrees/s (scaled)
    mag_x, mag_y, mag_z = struct.unpack('<hhh', payload[12:18])  # Magnetometer in microtesla (scaled)

    return {
        "acc_x": acc_x,
        "acc_y": acc_y,
        "acc_z": acc_z,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
    }

def set_raw_rc(ser, channels):
    """Set the raw RC channels. Channels should be a list of 8 integers."""
    assert len(channels) == 8, "There should be exactly 8 channels"

    # Prepare payload for MSP_SET_RAW_RC command
    payload = []
    for ch in channels:
        payload += [ch & 0xFF, (ch >> 8) & 0xFF]  # Convert to little-endian 16-bit values
    
    # Send the command and read response
    return send_msp_command(ser, MSP_SET_RAW_RC, payload)

def arm(ser):
    """Arms the flight controller by setting the RC commands accordingly."""
    channels = [1500, 1500, 2000, 900, 1500, 1500, 1500, 1500]  # Roll, Pitch, Yaw, Throttle, AUX1-4
    for _ in range(50):  # Send arming command for a short duration
        set_raw_rc(ser, channels)
        time.sleep(0.02)
    print("Flight controller armed")

def disarm(ser):
    """Disarms the flight controller by setting the RC commands accordingly."""
    channels = [1500, 1500, 1500, 900, 1500, 1500, 1500, 1500]  # Roll, Pitch, Yaw, Throttle, AUX1-4
    set_raw_rc(ser, channels)
    print("Flight controller disarmed")

def main():
    # Open the serial connection
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

    try:
        # Arm the flight controller
        arm(ser)

        # Example channel values (1500 is mid-point for most RC channels)
        channels = [1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500]  # AERT + AUX1-4

        while True:
            # Set throttle value for testing
            throttle = 1090
            channels[3] = throttle

            # Send the raw RC values
            set_raw_rc(ser, channels)

            # Send the MSP_RAW_GPS command and receive the response
            gps_response_payload = send_msp_command(ser, MSP_RAW_GPS)

            # Parse the GPS data
            gps_data = parse_msp_raw_gps(gps_response_payload)

            # Print the GPS data
            # print("GPS Data:")
            # print(f"  GPS Fix: {'Yes' if gps_data['gps_fix'] else 'No'}")
            # print(f"  Number of Satellites: {gps_data['num_satellites']}")
            # print(f"  Latitude: {gps_data['latitude']}°")
            # print(f"  Longitude: {gps_data['longitude']}°")
            # print(f"  Altitude: {gps_data['altitude']} meters")
            # print(f"  Ground Speed: {gps_data['ground_speed']} m/s")
            # print(f"  Ground Course: {gps_data['ground_course']}°")
            # if gps_data['pdop'] is not None:
            #     print(f"  PDOP: {gps_data['pdop']}")

            # Send the MSP_RAW_IMU command and receive the response
            imu_response_payload = send_msp_command(ser, MSP_RAW_IMU)

            # Parse the IMU data
            imu_data = parse_msp_raw_imu(imu_response_payload)

            # Print the IMU data
            print("IMU Data:")
            print(f"  Accelerometer X: {imu_data['acc_x']} m/s²")
            print(f"  Accelerometer Y: {imu_data['acc_y']} m/s²")
            print(f"  Accelerometer Z: {imu_data['acc_z']} m/s²")
            print(f"  Gyroscope X: {imu_data['gyro_x']}°/s")
            print(f"  Gyroscope Y: {imu_data['gyro_y']}°/s")
            print(f"  Gyroscope Z: {imu_data['gyro_z']}°/s")
            print(f"  Magnetometer X: {imu_data['mag_x']} μT")
            print(f"  Magnetometer Y: {imu_data['mag_y']} μT")
            print(f"  Magnetometer Z: {imu_data['mag_z']} μT")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        # Disarm and close the serial connection
        disarm(ser)
        ser.close()

if __name__ == "__main__":
    main()
