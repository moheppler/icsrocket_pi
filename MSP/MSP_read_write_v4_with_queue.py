import serial
import time
import struct
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import threading
import queue

# MSP Command Constants
MSP_SET_RAW_RC = 200  # Set RC Channels
MSP_RAW_IMU = 102  # Get RAW IMU Data
MSP_DEBUG = 254  # Command to get debug data

# Serial Port Settings
SERIAL_PORT = "/dev/ttyACM0"  # Update based on your setup
BAUD_RATE = 115200

# Initialize RC Channels (4 Channels: Roll, Pitch, Throttle, Yaw)
rc_channels = [1500, 1500, 1500, 1500]
send_rc = True

# Command Queue for MSP Commands
command_queue = queue.Queue()

# Lock for serial access synchronization
serial_lock = threading.Lock()


def send_msp_command(serial_conn, cmd, payload=[]):
    """Send an MSP command with optional payload."""
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b

    packet = (
        b"$M<"  # Header
        + bytes([size])  # Payload size
        + bytes([cmd])  # Command
        + bytes(payload)  # Payload
        + bytes([checksum])  # Checksum
    )

    # Use the lock to ensure only one thread writes to the serial port at a time
    with serial_lock:
        serial_conn.write(packet)


def read_msp_response(serial_conn):
    """Read and parse MSP response."""
    try:
        with serial_lock:
            # Read until we get the header
            header = serial_conn.read(3)
            if header != b"$M>":
                print(f"Invalid header: {header}")
                return None

            size = serial_conn.read(1)
            if not size:
                print("Size byte missing")
                return None
            size = size[0]

            cmd = serial_conn.read(1)
            if not cmd:
                print("Command byte missing")
                return None
            cmd = cmd[0]

            payload = serial_conn.read(size)
            if len(payload) != size:
                print(f"Payload size mismatch: expected {size}, got {len(payload)}")
                return None

            checksum = serial_conn.read(1)
            if not checksum:
                print("Checksum byte missing")
                return None
            checksum = checksum[0]

        # Verify checksum
        computed_checksum = size ^ cmd
        for b in payload:
            computed_checksum ^= b
        if computed_checksum != checksum:
            print(f"Checksum mismatch: {computed_checksum} != {checksum}")
            return None

        return cmd, payload
    except Exception as e:
        print(f"Error reading response: {e}")
        return None


def send_combined_msp(serial_conn):
    """Send combined MSP commands in a single message."""
    while True:
        try:
            # Prepare payloads for each MSP command
            imu_payload = []  # Payload for MSP_RAW_IMU (No payload required)
            debug_payload = []  # Payload for MSP_DEBUG (No payload required)

            # MSP_SET_RAW_RC: Build the payload based on RC channels
            rc_payload = []
            for ch in rc_channels:
                rc_payload += list(struct.pack("<H", ch))  # Convert to little-endian 16-bit

            # Combine all commands and their payloads into one payload
            combined_payload = []

            # Add MSP_RAW_IMU
            combined_payload.append(MSP_RAW_IMU)
            combined_payload.append(len(imu_payload))  # Payload size for MSP_RAW_IMU
            combined_payload += imu_payload

            # Add MSP_DEBUG
            combined_payload.append(MSP_DEBUG)
            combined_payload.append(len(debug_payload))  # Payload size for MSP_DEBUG
            combined_payload += debug_payload

            # Add MSP_SET_RAW_RC
            combined_payload.append(MSP_SET_RAW_RC)
            combined_payload.append(len(rc_payload))  # Payload size for MSP_SET_RAW_RC
            combined_payload += rc_payload

            # Compute checksum for the entire payload
            checksum = 0
            for b in combined_payload:
                checksum ^= b

            # Build the final packet
            packet = (
                b"$M<"  # Header
                + bytes([len(combined_payload)])  # Total size of combined payload
                + bytes(combined_payload)  # Combined payload
                + bytes([checksum])  # Checksum
            )

            # Send the combined MSP message
            with serial_lock:
                serial_conn.write(packet)
            print(f"Sent combined MSP packet: {packet}")

            # Handle the response
            while serial_conn.in_waiting > 0:
                response = read_msp_response(serial_conn)
                if response:
                    cmd, payload = response
                    if cmd == MSP_RAW_IMU:
                        imu_data = struct.unpack("<hhhhhhhhh", payload)
                        print(f"IMU Data: {imu_data}")
                    elif cmd == MSP_DEBUG:
                        debug_values = struct.unpack("<hhhh", payload)
                        print(f"Debug Values: {debug_values}")
                    elif cmd == MSP_SET_RAW_RC:
                        print("RC Channels set successfully")
                else:
                    print("Error or no response received")

            time.sleep(0.1)  # Adjust the loop interval as necessary

        except Exception as e:
            print(f"Error sending combined MSP commands: {e}")
            time.sleep(0.5)



# Dash App for Control
app = dash.Dash(__name__)

app.layout = html.Div([
    html.H1("RC Channel Control"),
    html.Div([
        html.Label("Roll"),
        dcc.Slider(1000, 2000, 1, value=1500, id="roll-slider"),
        html.Label("Pitch"),
        dcc.Slider(1000, 2000, 1, value=1500, id="pitch-slider"),
        html.Label("Throttle"),
        dcc.Slider(1000, 2000, 1, value=1500, id="throttle-slider"),
        html.Label("Yaw"),
        dcc.Slider(1000, 2000, 1, value=1500, id="yaw-slider"),
    ])
])


@app.callback(
    Output("roll-slider", "value"),
    Output("pitch-slider", "value"),
    Output("throttle-slider", "value"),
    Output("yaw-slider", "value"),
    [Input("roll-slider", "value"),
     Input("pitch-slider", "value"),
     Input("throttle-slider", "value"),
     Input("yaw-slider", "value")]
)
def update_channels(roll, pitch, throttle, yaw):
    """Update RC channel values from sliders."""
    rc_channels[0] = roll
    rc_channels[1] = pitch
    rc_channels[2] = throttle
    rc_channels[3] = yaw
    print(f"Updating rc_channels: {rc_channels}")
    return roll, pitch, throttle, yaw


if __name__ == "__main__":
    # Open Serial Connection
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        exit()

    # Start Combined MSP Command Thread
    combined_thread = threading.Thread(target=send_combined_msp, args=(serial_conn,))
    combined_thread.daemon = True
    combined_thread.start()

    # Start Dash Server
    try:
        app.run_server(debug=True, host="0.0.0.0", use_reloader=False)
    finally:
        send_rc = False
        # Close serial connection on exit
        serial_conn.close()