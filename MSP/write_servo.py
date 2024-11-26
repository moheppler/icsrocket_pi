"""
python3 write_servo.py

Arm the motor, run 1050 throttle, read from DSHOT

check the CLI config for: (lines starting with # are the commands, starting with " are comments)
# status
" check for <Arming disable flags>. Allowed: RXLOSS CLI
" Info on other flags: https://betaflight.com/docs/wiki/guides/current/arming-sequence-and-safety#description-of-arming-prevention-flags (and from there I search in the https://github.com/betaflight/betaflight for code definition of the flags)

# get enable_stick_arming
enable_stick_arming = ON
" this enables arming the motors by RPYT position: 1500, 1500, 2000, 900

# get dshot_bitbang
dshot_bitbang = ON
" this is needed to turn on dshot_bidir

# get dshot_bidir
dshot_bidir = ON
" this enables dshot telemetry data

" mixer doc: https://betaflight.com/docs/development/mixer
# mixer
Mixer: CUSTOM
" needed for custom config of motor and servos

# mmix
mmix 0  1.000  0.000  0.000  0.000
" example for single motor. Note also: the motors have to start in order, i.e. you can't say you just want to use motor 3&4
"""

import serial
import struct
import time

# MSP Command IDs
MSP_SET_RAW_RC = 200  # Command to set RC channels
MSP_MOTOR_TELEMETRY = 139

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
    """Send an MSP command and receive the response."""
    # Build and send the MSP command
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

def read_motor_telemetry(ser):
    """Request and print the motor telemetry data."""
    response = send_msp_command(ser, MSP_MOTOR_TELEMETRY)

    # First byte is the number of motors
    motor_count = response[0]
    telemetry_data = response[1:]

    print(f"Motor Count: {motor_count}")

    for i in range(motor_count):
        offset = i * 13  # Each motor's data is 13 bytes
        (
            rpm,
            invalid_pct,
            esc_temperature,
            esc_voltage,
            esc_current,
            esc_consumption
        ) = struct.unpack('<I H B H H H', telemetry_data[offset:offset + 13])

        print(f"Motor {i + 1} Telemetry:")
        print(f"  RPM: {rpm}")
        print(f"  Invalid Packet %: {invalid_pct / 100:.2f}%")
        print(f"  ESC Temperature: {esc_temperature} °C")
        print(f"  ESC Voltage: {esc_voltage / 100:.2f} V")
        print(f"  ESC Current: {esc_current / 100:.2f} A")
        print(f"  ESC Consumption: {esc_consumption} mAh")

def rpyt_to_aetr(channels):
    return [channels[0], channels[1], channels[3], channels[2]] + channels[4:]

def set_raw_rc(ser, channels):
    """
    Set the raw RC channels. Channels should be a list of 8 integers.
    """
    assert len(channels) == 8, "There should be exactly 8 channels"

    # test code
    channels = rpyt_to_aetr(channels)
    print(channels)
    
    # Prepare payload for MSP_SET_RAW_RC command
    payload = []
    for ch in channels:
        payload += [ch & 0xFF, (ch >> 8) & 0xFF]  # Convert to little-endian 16-bit values
    
    # Send the command and read response
    return send_msp_command(ser, MSP_SET_RAW_RC, payload)

def arm(ser):
    """
    Arms the flight controller by setting the RC commands accordingly.
    Typically, this involves setting throttle to minimum and yaw to maximum.
    """
    read_motor_telemetry(ser)
    # Set channels for arming (e.g., throttle at minimum, yaw to the right)
    channels = [1500, 1500, 2000, 900, 1500, 1500, 1500, 1500]  # Roll, Pitch, Yaw, Throttle, AUX1-4
    for i in range(50):
        # read_motor_telemetry(ser)
        set_raw_rc(ser, channels)
        time.sleep(0.02)
    print("Flight controller armed")

def disarm(ser):
    """
    Disarms the flight controller by setting the RC commands accordingly.
    Typically, this involves setting throttle to minimum and yaw to center or opposite direction.
    """
    # Set channels for disarming (e.g., throttle at minimum, yaw centered)
    channels = [1500, 1500, 1500, 900, 1500, 1500, 1500, 1500]  # Roll, Pitch, Yaw, Throttle, AUX1-4
    set_raw_rc(ser, channels)
    print("Flight controller disarmed")

def main():
    # Open serial port connected to the flight controller
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

    try:
        # Arm
        # arm(ser)

        # Example channel values (1500 is mid-point for most RC channels)
        channels = [1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500]  # AERT + AUX1-4

        while True:
            ### Servo test: the corresponding servo slot and AUX channel have to be configured 
            # # Set AUX1 to a specific value (e.g., 1000, 1500, or 2000)
            # aux1_value = 1200  # Adjust this value to see servo change 
            # channels[4] = aux1_value  # Set AUX1 value (5th channel)

            ### Motor test
            thrott = 1090
            channels[3] = thrott

            # read DSHOT telemetry, esp. RPM
            # read_motor_telemetry(ser)
            # Send the raw RC values
            resp = set_raw_rc(ser, channels)

            # print(f"AUX1 set to {aux1_value}, resp is {resp}")
            # print(f"Thrott set to {thrott}, resp is {resp}")

            # the 
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        disarm(ser)
        ser.close()

if __name__ == "__main__":
    main()
